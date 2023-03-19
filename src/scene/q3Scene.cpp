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
#include "../dynamics/q3ContactSolver.h"
#include "../dynamics/q3Island.h"
#include "../debug/q3Render.h"

q3Scene::q3Scene(r32 dt, const q3Vec3& gravity, usize iterations) :
    contact_manager(),
    box_allocator(sizeof(q3Box), 256),
    body_count(0),
    body_list(NULL),
    gravity(gravity),
    dt(dt),
    iterations(iterations),
    new_box(false),
    allow_sleep(true),
    enable_friction(true) {}

q3Scene::~q3Scene() {
    Shutdown();
}

void q3Scene::BuildIsland(q3Island* island, q3Body* seed, q3Body** stack, i32 stackSize) {
    seed->SetFlag(q3Body::eIsland); // Mark seed as apart of island

    i32 stackCount = 0;
    stack[stackCount++] = seed;
    island->bodies.shrinkRetainingCapacity(0);
    island->contacts.shrinkRetainingCapacity(0);

    // Perform DFS on constraint graph
    while (stackCount > 0) {
        // Decrement stack to implement iterative backtracking
        q3Body* body = stack[--stackCount];
        island->Add(body);

        // Awaken all bodies connected to the island
        body->SetToAwake();

        // Do not search across static bodies to keep island
        // formations as small as possible, however the static
        // body itself should be apart of the island in order
        // to properly represent a full contact
        if (body->HasFlag(q3Body::eStatic)) continue;

        // Search all contacts connected to this body
        q3ContactEdge* contacts = body->m_contactList;
        for (q3ContactEdge* edge = contacts; edge; edge = edge->next) {
            q3ContactConstraint* contact = edge->constraint;

            // Skip contacts that have been added to an island already
            if (contact->m_flags & q3ContactConstraint::eIsland) continue;
            // Can safely skip this contact if it didn't actually collide
            // with anything
            if (!(contact->m_flags & q3ContactConstraint::eColliding)) continue;
            // Skip sensors
            if (contact->A->sensor || contact->B->sensor) continue;

            // Mark island flag and add to island
            contact->m_flags |= q3ContactConstraint::eIsland;
            island->Add(contact);

            // Attempt to add the other body in the contact to the island
            // to simulate contact awakening propogation
            q3Body* other = edge->other;
            if (other->HasFlag(q3Body::eIsland)) continue;

            debug::assert(stackCount < stackSize);

            stack[stackCount++] = other;
            other->m_flags |= q3Body::eIsland;
        }
    }
}

void q3Scene::Step() {
    if (new_box) {
        contact_manager.m_broadphase.UpdatePairs();
        new_box = false;
    }

    contact_manager.TestCollisions();

    for (q3Body* body = body_list; body; body = body->m_next) body->m_flags &= ~q3Body::eIsland;

    q3Island island =
        q3Island::init(allocator, dt, gravity, iterations, allow_sleep, enable_friction);
    defer(island.deinit());
    island.bodies.ensureTotalCapacity(body_count).unwrap();
    island.velocities.ensureTotalCapacity(body_count).unwrap();
    island.contacts.ensureTotalCapacity(contact_manager.m_contactCount).unwrap();
    island.contact_states.ensureTotalCapacity(contact_manager.m_contactCount).unwrap();

    // Build each active island and then solve each built island
    i32 stackSize = body_count;
    auto stack_slice = allocator.alloc<q3Body*>(stackSize).value;
    defer(allocator.free(stack_slice));
    for (q3Body* seed = body_list; seed; seed = seed->m_next) {
        if (seed->HasFlag(q3Body::eIsland)) continue; // Seed can't be part of an island already
        if (!seed->HasFlag(q3Body::eAwake)) continue; // Seed must be awake
        // Seed cannot be a static body in order to keep islands as small as possible
        if (seed->HasFlag(q3Body::eStatic)) continue;

        BuildIsland(&island, seed, stack_slice.ptr, stackSize);
        debug::assert(island.bodies.items.len != 0);

        island.Initialize();
        island.Solve();

        // Reset all static island flags
        // This allows static bodies to participate in other island formations
        for (auto body : island.bodies.items) {
            if (body->HasFlag(q3Body::eStatic)) body->UnsetFlag(q3Body::eIsland);
        }
    }

    // Update the broadphase AABBs
    for (q3Body* body = body_list; body; body = body->m_next) {
        if (body->m_flags & q3Body::eStatic) continue;
        body->SynchronizeProxies();
    }

    // Look for new contacts
    contact_manager.FindNewContacts();

    // Clear all forces
    for (q3Body* body = body_list; body; body = body->m_next) {
        q3Identity(body->m_force);
        q3Identity(body->m_torque);
    }
}

q3Body* q3Scene::CreateBody(const q3BodyDef& def) {
    q3Body* body = this->allocator.create<q3Body>().value;
    *body = q3Body(def, this);

    // Add body to scene bodyList
    body->m_prev = NULL;
    body->m_next = body_list;

    if (body_list) body_list->m_prev = body;

    body_list = body;
    ++body_count;

    return body;
}

void q3Scene::RemoveBody(q3Body* body) {
    debug::assert(body_count > 0);

    contact_manager.RemoveContactsFromBody(body);

    body->RemoveAllBoxes();

    // Remove body from scene bodyList
    if (body->m_next) body->m_next->m_prev = body->m_prev;
    if (body->m_prev) body->m_prev->m_next = body->m_next;
    if (body == body_list) body_list = body->m_next;
    --body_count;

    this->allocator.destroy(body);
}

void q3Scene::RemoveAllBodies() {
    q3Body* body = body_list;
    while (body) {
        q3Body* next = body->m_next;
        body->RemoveAllBoxes();
        this->allocator.destroy(body);
        body = next;
    }
    body_list = NULL;
}

void q3Scene::SetAllowSleep(bool allowSleep) {
    allow_sleep = allowSleep;
    if (!allowSleep) {
        for (q3Body* body = body_list; body; body = body->m_next) body->SetToAwake();
    }
}

void q3Scene::Shutdown() {
    RemoveAllBodies();
    box_allocator.Clear();
}

void q3Scene::QueryAABB(q3QueryCallback* cb, const q3AABB& aabb) const {
    struct SceneQueryWrapper {
        bool TreeCallBack(i32 id) {
            q3AABB aabb;
            q3Box* box = (q3Box*)broadPhase->m_tree.GetUserData(id);

            box->ComputeAABB(box->body->GetTransform(), &aabb);

            if (q3AABBtoAABB(m_aabb, aabb)) { return cb->ReportShape(box); }

            return true;
        }

        q3QueryCallback* cb;
        const q3BroadPhase* broadPhase;
        q3AABB m_aabb;
    };

    SceneQueryWrapper wrapper;
    wrapper.m_aabb = aabb;
    wrapper.broadPhase = &contact_manager.m_broadphase;
    wrapper.cb = cb;
    contact_manager.m_broadphase.m_tree.Query(&wrapper, aabb);
}

void q3Scene::QueryPoint(q3QueryCallback* cb, const q3Vec3& point) const {
    struct SceneQueryWrapper {
        bool TreeCallBack(i32 id) {
            q3Box* box = (q3Box*)broadPhase->m_tree.GetUserData(id);

            if (box->TestPoint(box->body->GetTransform(), m_point)) { cb->ReportShape(box); }

            return true;
        }

        q3QueryCallback* cb;
        const q3BroadPhase* broadPhase;
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
    contact_manager.m_broadphase.m_tree.Query(&wrapper, aabb);
}

void q3Scene::RayCast(q3QueryCallback* cb, q3RaycastData& rayCast) const {
    struct SceneQueryWrapper {
        bool TreeCallBack(i32 id) {
            q3Box* box = (q3Box*)broadPhase->m_tree.GetUserData(id);

            if (box->Raycast(box->body->GetTransform(), m_rayCast)) { return cb->ReportShape(box); }

            return true;
        }

        q3QueryCallback* cb;
        const q3BroadPhase* broadPhase;
        q3RaycastData* m_rayCast;
    };

    SceneQueryWrapper wrapper;
    wrapper.m_rayCast = &rayCast;
    wrapper.broadPhase = &contact_manager.m_broadphase;
    wrapper.cb = cb;
    contact_manager.m_broadphase.m_tree.Query(&wrapper, rayCast);
}

void q3Scene::Render(q3Render* render) const {
    // clang-format off
    const i32 box_indices[36] = {
        1, 7, 5,     1, 3, 7,     1, 4, 3,     1, 2, 4,     3, 8, 7,     3, 4, 8,
        5, 7, 8,     5, 8, 6,     1, 5, 6,     1, 6, 2,     2, 6, 8,     2, 8, 4,
    };
    // clang-format on
    for (q3Body* body = body_list; body; body = body->m_next) {
        bool awake = body->IsAwake();
        for (q3Box* box = body->m_boxes; box; box = box->next) {
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
    }

    for (auto contact = contact_manager.m_contactList; contact; contact = contact->next) {
        if (!(contact->m_flags & q3ContactConstraint::eColliding)) continue;
        q3Manifold* m = &contact->manifold;
        for (i32 j = 0; j < m->contactCount; ++j) {
            const q3Contact* c = m->contacts + j;
            f32 blue = (f32)(255 - c->warmStarted) / 255.0f;
            f32 red = 1.0f - blue;
            render->SetScale(10.0f, 10.0f, 10.0f);
            render->SetPenColor(red, blue, blue);
            render->SetPenPosition(c->position.x, c->position.y, c->position.z);
            render->Point();

            auto color = m->A->body->IsAwake() ? q3Vec3(1, 1, 1) : q3Vec3(0.2, 0.2, 0.2);
            render->SetPenColor(color.x, color.y, color.z);

            render->SetPenPosition(c->position.x, c->position.y, c->position.z);
            render->Line(
                c->position.x + m->normal.x * 0.5f, c->position.y + m->normal.y * 0.5f,
                c->position.z + m->normal.z * 0.5f
            );
        }
    }

    render->SetScale(1.0f, 1.0f, 1.0f);
}
