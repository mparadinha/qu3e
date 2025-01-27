/**
@file	q3Scene.h

@author	Randy Gaul
@date	10/10/2014

        Copyright (c) 2014 Randy Gaul http://www.randygaul.net

        This software is provided 'as-is', without any express or implied
        warranty. In no event will the authors be held liable for any damages
        arising from the use of this software.

        Permission is granted to anyone to use this software for any purpose,
        including commercial applications, and to alter it and redistribute it
        freely, subject to the following restrictions:
          1. The origin of this software must not be misrepresented; you must
not claim that you wrote the original software. If you use this software in a
product, an acknowledgment in the product documentation would be appreciated but
is not required.
          2. Altered source versions must be plainly marked as such, and must
not be misrepresented as being the original software.
          3. This notice may not be removed or altered from any source
distribution.
*/

#include <stdlib.h>

#include "q3Scene.h"
#include "../collision/q3Box.h"
#include "../dynamics/q3Body.h"
#include "../dynamics/q3Contact.h"
#include "../dynamics/q3Island.h"
#include "../debug/q3Render.h"

q3Scene::q3Scene(r32 dt, const q3Vec3& gravity, usize iterations) :
    allocator(),
    contact_manager(allocator),
    bodies(LinkedList<q3Body>::init(allocator)),
    gravity(gravity),
    dt(dt),
    new_box(false),
    enable_friction(true),
    iterations(iterations) {}

q3Scene::~q3Scene() {
    RemoveAllBodies();
}

void q3Scene::BuildIsland(q3Island* island, q3Body* seed) {
    seed->flags.Island = true; // Mark seed as apart of island

    auto stack = ArrayList<q3Body*>::init(allocator);
    defer(stack.deinit());
    stack.append(seed).unwrap();

    island->bodies.shrinkRetainingCapacity(0);
    island->contacts.shrinkRetainingCapacity(0);

    // Perform DFS on constraint graph
    while (stack.items.len > 0) {
        // Decrement stack to implement iterative backtracking
        q3Body* body = stack.pop();
        island->Add(body);

        // Do not search across static bodies to keep island
        // formations as small as possible, however the static
        // body itself should be apart of the island in order
        // to properly represent a full contact
        if (body->flags.Static) continue;

        // Search all contacts connected to this body
        q3ContactEdge* contacts = body->contact_edge_list;
        for (q3ContactEdge* edge = contacts; edge; edge = edge->next) {
            q3ContactConstraint* contact = edge->constraint;

            // Skip contacts that have been added to an island already
            if (contact->flags.Island) continue;
            // Can safely skip this contact if it didn't actually collide
            // with anything
            if (!(contact->flags.Colliding)) continue;
            // Skip sensors
            if (contact->A->sensor || contact->B->sensor) continue;

            // Mark island flag and add to island
            contact->flags.Island = true;
            island->Add(contact);

            // Attempt to add the other body in the contact to the island
            // to simulate contact awakening propogation
            q3Body* other = edge->other;
            if (other->flags.Island) continue;

            stack.append(other).unwrap();
            other->flags.Island = true;
        }
    }
}

void q3Scene::Step() {
    contact_manager.TestCollisions();

    for (q3Body* body : bodies.ptrIter()) body->flags.Island = false;

    q3Island island = q3Island::init(allocator, dt, gravity, iterations, enable_friction);
    defer(island.deinit());
    island.bodies.ensureTotalCapacity(bodies.len).unwrap();
    island.velocities.ensureTotalCapacity(bodies.len).unwrap();
    island.contacts.ensureTotalCapacity(contact_manager.contacts.len).unwrap();
    island.contact_states.ensureTotalCapacity(contact_manager.contacts.len).unwrap();

    // Build each active island and then solve each built island
    for (q3Body* seed : bodies.ptrIter()) {
        if (seed->flags.Island) continue; // Seed can't be part of an island already
        // Seed cannot be a static body in order to keep islands as small as possible
        if (seed->flags.Static) continue;

        BuildIsland(&island, seed);
        debug::assert(island.bodies.items.len != 0);

        island.Initialize();
        island.Solve();

        // Reset all static island flags
        // This allows static bodies to participate in other island formations
        for (auto body : island.bodies.items) {
            if (body->flags.Static) body->flags.Island = false;
        }
    }

    // Update the broadphase AABBs
    for (q3Body* body : bodies.ptrIter()) {
        if (body->flags.Static) continue;
        body->SynchronizeProxies();
    }

    // Look for new contacts
    contact_manager.FindNewContacts();

    // Clear all forces
    for (q3Body* body : bodies.ptrIter()) {
        q3Identity(body->m_force);
        q3Identity(body->m_torque);
    }
}

q3Body* q3Scene::CreateBody(const q3BodyDef& def) {
    q3Body* body = &bodies.prepend(q3Body(def, this)).unwrap()->data;
    return body;
}

void q3Scene::RemoveBody(q3Body* body) {
    debug::assert(bodies.len > 0);
    contact_manager.RemoveContactsFromBody(body);
    body->RemoveAllBoxes();
    bodies.remove(body);
}

void q3Scene::RemoveAllBodies() {
    auto opt_node = bodies.head;
    while (opt_node.is_not_null()) {
        auto opt_next = opt_node.unwrap()->next;
        q3Body* body = &opt_node.unwrap()->data;
        body->RemoveAllBoxes();
        bodies.remove(body);
        opt_node = opt_next;
    }
}

void q3Scene::QueryAABB(q3QueryCallback* cb, const q3AABB& aabb) {
    struct SceneQueryWrapper {
        bool TreeCallBack(i32 id) {
            q3AABB aabb;
            q3Box* box = broadPhase->GetBoxInfo(id).box;

            box->ComputeAABB(box->body->m_tx, &aabb);

            if (q3AABBtoAABB(m_aabb, aabb)) { return cb->ReportShape(box); }

            return true;
        }

        q3QueryCallback* cb;
        q3BroadPhase* broadPhase;
        q3AABB m_aabb;
    };

    SceneQueryWrapper wrapper;
    wrapper.m_aabb = aabb;
    wrapper.broadPhase = &contact_manager.m_broadphase;
    wrapper.cb = cb;
    contact_manager.m_broadphase.Query(&wrapper, aabb);
}

void q3Scene::QueryPoint(q3QueryCallback* cb, const q3Vec3& point) {
    struct SceneQueryWrapper {
        bool TreeCallBack(i32 id) {
            q3Box* box = broadPhase->GetBoxInfo(id).box;
            if (box->TestPoint(box->body->m_tx, m_point)) { cb->ReportShape(box); }
            return true;
        }

        q3QueryCallback* cb;
        q3BroadPhase* broadPhase;
        q3Vec3 m_point;
    };

    SceneQueryWrapper wrapper;
    wrapper.m_point = point;
    wrapper.broadPhase = &contact_manager.m_broadphase;
    wrapper.cb = cb;
    const r32 k_fattener = r32(0.5);
    q3Vec3 v(k_fattener, k_fattener, k_fattener);
    q3AABB aabb;
    aabb.min = point - v;
    aabb.max = point + v;
    contact_manager.m_broadphase.Query(&wrapper, aabb);
}

void q3Scene::RayCast(q3QueryCallback* cb, q3RaycastData& rayCast) {
    struct SceneQueryWrapper {
        bool TreeCallBack(i32 id) {
            q3Box* box = broadPhase->GetBoxInfo(id).box;

            if (box->Raycast(box->body->m_tx, m_rayCast)) { return cb->ReportShape(box); }

            return true;
        }

        q3QueryCallback* cb;
        q3BroadPhase* broadPhase;
        q3RaycastData* m_rayCast;
    };

    SceneQueryWrapper wrapper;
    wrapper.m_rayCast = &rayCast;
    wrapper.broadPhase = &contact_manager.m_broadphase;
    wrapper.cb = cb;
    contact_manager.m_broadphase.Query(&wrapper, rayCast);
}

void q3Scene::Render(q3Render* render) {
    // clang-format off
    const i32 box_indices[36] = {
        1, 7, 5,     1, 3, 7,     1, 4, 3,     1, 2, 4,     3, 8, 7,     3, 4, 8,
        5, 7, 8,     5, 8, 6,     1, 5, 6,     1, 6, 2,     2, 6, 8,     2, 8, 4,
    };
    // clang-format on
    for (q3Body* body : bodies.ptrIter()) {
        const auto box = &body->box;
        q3Transform world = q3Mul(body->m_tx, box->local);
        const auto e = box->e;
        const q3Vec3 vertices[8] = {q3Vec3(-e.x, -e.y, -e.z), q3Vec3(-e.x, -e.y, e.z),
                                    q3Vec3(-e.x, e.y, -e.z),  q3Vec3(-e.x, e.y, e.z),
                                    q3Vec3(e.x, -e.y, -e.z),  q3Vec3(e.x, -e.y, e.z),
                                    q3Vec3(e.x, e.y, -e.z),   q3Vec3(e.x, e.y, e.z)};
        for (i32 i = 0; i < 36; i += 3) {
            q3Vec3 a = q3Mul(world, vertices[box_indices[i + 0] - 1]);
            q3Vec3 b = q3Mul(world, vertices[box_indices[i + 1] - 1]);
            q3Vec3 c = q3Mul(world, vertices[box_indices[i + 2] - 1]);
            q3Vec3 n = q3Normalize(q3Cross(b - a, c - a));
            render->SetTriNormal(n.x, n.y, n.z);
            render->Triangle(a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z);
        }
    }

    for (auto contact : contact_manager.contacts.iter()) {
        if (!contact.flags.Colliding) continue;
        q3Manifold m = contact.manifold;
        for (i32 j = 0; j < m.contactCount; ++j) {
            q3Contact c = m.contacts[j];
            render->SetScale(10.0f, 10.0f, 10.0f);

            render->SetPenColor(1, 0, 0);
            render->SetPenPosition(c.position.x, c.position.y, c.position.z);
            render->Point();

            render->SetPenColor(1, 1, 1);
            render->SetPenPosition(c.position.x, c.position.y, c.position.z);
            render->Line(
                c.position.x + m.normal.x * 0.5f, c.position.y + m.normal.y * 0.5f,
                c.position.z + m.normal.z * 0.5f
            );
        }
    }

    render->SetScale(1.0f, 1.0f, 1.0f);
}
