/**
@file	q3Island.cpp

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

#include "../broadphase/q3BroadPhase.h"
#include "../common/q3Settings.h"
#include "q3Body.h"
#include "q3Contact.h"
#include "q3ContactSolver.h"
#include "q3Island.h"

void q3Island::Solve() {
    // Apply gravity
    // Integrate velocities and create state buffers, calculate world inertia
    for (auto [body, i] : bodies.items.iter()) {
        q3VelocityState* v = &velocities.items[i];

        if (body->flags.Dynamic) {
            body->ApplyLinearForce(gravity * body->m_gravityScale);

            // Calculate world space inertia tensor
            q3Mat3 r = body->m_tx.rotation;
            body->m_invInertiaWorld = r * body->m_invInertiaModel * q3Transpose(r);

            // Integrate velocity
            body->m_linearVelocity += (body->m_force * body->m_invMass) * dt;
            body->m_angularVelocity += (body->m_invInertiaWorld * body->m_torque) * dt;

            // From Box2D!
            // Apply damping.
            // ODE: dv/dt + c * v = 0
            // Solution: v(t) = v0 * exp(-c * t)
            // Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t)
            // * exp(-c * dt) = v * exp(-c * dt) v2 = exp(-c * dt) * v1 Pade
            // approximation: v2 = v1 * 1 / (1 + c * dt)
            body->m_linearVelocity *= r32(1.0) / (r32(1.0) + dt * body->m_linearDamping);
            body->m_angularVelocity *= r32(1.0) / (r32(1.0) + dt * body->m_angularDamping);
        }

        v->v = body->m_linearVelocity;
        v->w = body->m_angularVelocity;
    }

    // Create contact solver, pass in state buffers, create buffers for contacts
    // Initialize velocity constraint for normal + friction and warm start
    q3ContactSolver contactSolver;
    contactSolver.Initialize(this);
    contactSolver.PreSolve(dt);

    // Solve contacts
    for (i32 i = 0; i < iterations; ++i) contactSolver.Solve();

    contactSolver.ShutDown();

    // Copy back state buffers
    // Integrate positions
    for (auto [body, i] : bodies.items.iter()) {
        q3VelocityState* v = &velocities.items[i];

        if (body->flags.Static) continue;

        body->m_linearVelocity = v->v;
        body->m_angularVelocity = v->w;

        // Integrate position
        body->m_worldCenter += body->m_linearVelocity * dt;
        body->m_q.Integrate(body->m_angularVelocity, dt);
        body->m_q = q3Normalize(body->m_q);
        body->m_tx.rotation = body->m_q.ToMat3();
    }
}

void q3Island::Add(q3Body* body) {
    body->m_islandIndex = bodies.items.len;
    bodies.append(body).unwrap();
    velocities.append(q3VelocityState{}).unwrap();
}

void q3Island::Add(q3ContactConstraint* contact) {
    contacts.append(contact).unwrap();
    contact_states.append(q3ContactConstraintState{}).unwrap();
}

void q3Island::Initialize() {
    for (auto [cc, i] : contacts.items.iter()) {
        q3ContactConstraintState* c = &contact_states.items[i];
        c->centerA = cc->bodyA->m_worldCenter;
        c->centerB = cc->bodyB->m_worldCenter;
        c->iA = cc->bodyA->m_invInertiaWorld;
        c->iB = cc->bodyB->m_invInertiaWorld;
        c->mA = cc->bodyA->m_invMass;
        c->mB = cc->bodyB->m_invMass;
        c->restitution = cc->restitution;
        c->friction = cc->friction;
        c->indexA = cc->bodyA->m_islandIndex;
        c->indexB = cc->bodyB->m_islandIndex;
        c->normal = cc->manifold.normal;
        c->tangentVectors[0] = cc->manifold.tangentVectors[0];
        c->tangentVectors[1] = cc->manifold.tangentVectors[1];
        c->contactCount = cc->manifold.contactCount;

        for (i32 j = 0; j < c->contactCount; ++j) {
            q3ContactState* s = c->contacts + j;
            q3Contact* cp = cc->manifold.contacts + j;
            s->ra = cp->position - c->centerA;
            s->rb = cp->position - c->centerB;
            s->penetration = cp->penetration;
            s->normalImpulse = cp->normalImpulse;
            s->tangentImpulse[0] = cp->tangentImpulse[0];
            s->tangentImpulse[1] = cp->tangentImpulse[1];
        }
    }
}
