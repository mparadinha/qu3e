/**
@file	q3ContactManager.cpp

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

#include "../collision/q3Box.h"
#include "../debug/q3Render.h"
#include "../scene/q3Scene.h"
#include "q3Body.h"
#include "q3Contact.h"
#include "q3ContactManager.h"

q3ContactManager::q3ContactManager(Allocator allocator) :
    contacts(LinkedList<q3ContactConstraint>::init(allocator)),
    m_broadphase(allocator) {}

void q3ContactManager::AddContact(q3Box* A, q3Box* B) {
    q3Body* bodyA = A->body;
    q3Body* bodyB = B->body;
    if (!bodyA->CanCollide(bodyB)) return;

    // Search for existing matching contact
    // Return if found duplicate to avoid duplicate constraints
    // Mark pre-existing duplicates as active
    for (auto edge = A->body->contact_edge_list; edge; edge = edge->next) {
        if (edge->other == bodyB) {
            q3Box* shapeA = edge->constraint->A;
            q3Box* shapeB = edge->constraint->B;
            // @TODO: Verify this against Box2D; not sure if this is all we need here
            if ((A == shapeA) && (B == shapeB)) return;
        }
    }

    // Create new contact
    q3ContactConstraint* contact = &contacts.prepend({}).unwrap()->data;
    contact->A = A;
    contact->B = B;
    contact->bodyA = A->body;
    contact->bodyB = B->body;
    contact->friction = q3MixFriction(A, B);
    contact->restitution = q3MixRestitution(A, B);
    contact->m_flags = 0;
    contact->manifold.SetPair(A, B);
    contact->manifold.contactCount = 0;
    for (i32 i = 0; i < 8; ++i) contact->manifold.contacts[i].warmStarted = 0;

    // Connect A
    contact->edgeA.constraint = contact;
    contact->edgeA.other = bodyB;
    bodyA->linkEdgeIntoList(&contact->edgeA);
    // Connect B
    contact->edgeB.constraint = contact;
    contact->edgeB.other = bodyA;
    bodyB->linkEdgeIntoList(&contact->edgeB);

    bodyA->SetToAwake();
    bodyB->SetToAwake();
}

void q3ContactManager::FindNewContacts() {
    m_broadphase.UpdatePairs(this);
}

void q3ContactManager::RemoveContact(q3ContactConstraint* contact) {
    q3Body* A = contact->bodyA;
    q3Body* B = contact->bodyB;

    A->unlinkEdgeFromList(&contact->edgeA);
    B->unlinkEdgeFromList(&contact->edgeB);

    A->SetToAwake();
    B->SetToAwake();

    contacts.remove(contact);
}

void q3ContactManager::RemoveContactsFromBody(q3Body* body) {
    // note: the `RemoveContact` frees a q3ContactConstraint which holds the
    // edge pointed to in that iteration, so we can't use a normal for loop
    q3ContactEdge* edge = body->contact_edge_list;
    while (edge) {
        q3ContactEdge* next = edge->next;
        RemoveContact(edge->constraint);
        edge = next;
    }
}

void q3ContactManager::RemoveFromBroadphase(q3Body* body) {
    q3Box* box = body->m_boxes;

    while (box) {
        m_broadphase.RemoveBox(box);
        box = box->next;
    }
}

void q3ContactManager::TestCollisions(void) {
    auto opt_node = contacts.head;
    while (opt_node.is_not_null()) {
        auto opt_next = opt_node.unwrap()->next;
        q3ContactConstraint* constraint = &opt_node.unwrap()->data;

        q3Box* A = constraint->A;
        q3Box* B = constraint->B;
        q3Body* bodyA = A->body;
        q3Body* bodyB = B->body;

        constraint->m_flags &= ~q3ContactConstraint::eIsland;

        if (!bodyA->HasFlag(q3Body::eAwake) && !bodyB->HasFlag(q3Body::eAwake)) {
            opt_node = opt_next;
            continue;
        }

        if (!bodyA->CanCollide(bodyB)) {
            RemoveContact(constraint);
            opt_node = opt_next;
            continue;
        }

        // Check if contact should persist
        if (!m_broadphase.TestOverlap(A->broadPhaseIndex, B->broadPhaseIndex)) {
            RemoveContact(constraint);
            opt_node = opt_next;
            continue;
        }
        q3Manifold* manifold = &constraint->manifold;
        q3Manifold oldManifold = constraint->manifold;
        q3Vec3 ot0 = oldManifold.tangentVectors[0];
        q3Vec3 ot1 = oldManifold.tangentVectors[1];
        constraint->SolveCollision();
        q3ComputeBasis(manifold->normal, manifold->tangentVectors, manifold->tangentVectors + 1);

        for (i32 i = 0; i < manifold->contactCount; ++i) {
            q3Contact* c = manifold->contacts + i;
            c->tangentImpulse[0] = c->tangentImpulse[1] = c->normalImpulse = r32(0.0);
            u8 oldWarmStart = c->warmStarted;
            c->warmStarted = u8(0);

            for (i32 j = 0; j < oldManifold.contactCount; ++j) {
                q3Contact* oc = oldManifold.contacts + j;
                if (c->fp.key == oc->fp.key) {
                    c->normalImpulse = oc->normalImpulse;

                    // Attempt to re-project old friction solutions
                    q3Vec3 friction = ot0 * oc->tangentImpulse[0] + ot1 * oc->tangentImpulse[1];
                    c->tangentImpulse[0] = q3Dot(friction, manifold->tangentVectors[0]);
                    c->tangentImpulse[1] = q3Dot(friction, manifold->tangentVectors[1]);
                    c->warmStarted = q3Max(oldWarmStart, u8(oldWarmStart + 1));
                    break;
                }
            }
        }

        opt_node = opt_next;
    }
}
