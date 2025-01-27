/**
@file	q3Body.cpp

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

#include "q3Body.h"
#include "../collision/q3Box.h"
#include "../scene/q3Scene.h"
#include "../dynamics/q3Contact.h"
#include "../broadphase/q3BroadPhase.h"

q3Body::q3Body(const q3BodyDef& def, q3Scene* scene) {
    m_linearVelocity = def.linearVelocity;
    m_angularVelocity = def.angularVelocity;
    q3Identity(m_force);
    q3Identity(m_torque);
    m_q.Set(q3Normalize(def.axis), def.angle);
    m_tx.rotation = m_q.ToMat3();
    m_tx.position = def.position;
    m_gravityScale = def.gravityScale;
    m_scene = scene;
    flags = {};
    m_linearDamping = def.linearDamping;
    m_angularDamping = def.angularDamping;

    if (def.bodyType == eDynamicBody) {
        flags.Dynamic = true;
    } else {
        if (def.bodyType == eStaticBody) {
            flags.Static = true;
            q3Identity(m_linearVelocity);
            q3Identity(m_angularVelocity);
            q3Identity(m_force);
            q3Identity(m_torque);
        } else if (def.bodyType == eKinematicBody) {
            flags.Kinematic = true;
        }
    }

    contact_edge_list = NULL;
}

const q3Box* q3Body::SetBox(const q3BoxDef& def) {
    q3AABB aabb;
    box.local = def.m_tx;
    box.e = def.m_e;
    box.ComputeAABB(m_tx, &aabb);
    box.body = this;
    box.friction = def.m_friction;
    box.restitution = def.m_restitution;
    box.density = def.m_density;
    box.sensor = def.m_sensor;

    CalculateMassData();

    m_scene->contact_manager.m_broadphase.InsertBox(&box, aabb);
    m_scene->new_box = true;

    return &box;
}

void q3Body::RemoveBox() {
    // Remove all contacts associated with this shape
    // note: the `RemoveContact` frees a q3ContactConstraint which hold the
    // edge pointed to in that iteration, so we can't use a normal for loop
    q3ContactEdge* edge = contact_edge_list;
    while (edge) {
        q3ContactConstraint* contact = edge->constraint;
        m_scene->contact_manager.RemoveContact(contact);
    }

    m_scene->contact_manager.m_broadphase.RemoveBox(&box);

    CalculateMassData();
}

void q3Body::RemoveAllBoxes() {
    m_scene->contact_manager.m_broadphase.RemoveBox(&box);
    m_scene->contact_manager.RemoveContactsFromBody(this);
}

void q3Body::ApplyLinearForce(const q3Vec3& force) {
    m_force += force * m_mass;
}

void q3Body::ApplyForceAtWorldPoint(const q3Vec3& force, const q3Vec3& point) {
    m_force += force * m_mass;
    m_torque += q3Cross(point - m_worldCenter, force);
}

void q3Body::ApplyLinearImpulse(const q3Vec3& impulse) {
    m_linearVelocity += impulse * m_invMass;
}

void q3Body::ApplyLinearImpulseAtWorldPoint(const q3Vec3& impulse, const q3Vec3& point) {
    m_linearVelocity += impulse * m_invMass;
    m_angularVelocity += m_invInertiaWorld * q3Cross(point - m_worldCenter, impulse);
}

void q3Body::ApplyTorque(const q3Vec3& torque) {
    m_torque += torque;
}

const q3Vec3 q3Body::GetLocalPoint(const q3Vec3& p) const {
    return q3MulT(m_tx, p);
}

const q3Vec3 q3Body::GetLocalVector(const q3Vec3& v) const {
    return q3MulT(m_tx.rotation, v);
}

const q3Vec3 q3Body::GetWorldPoint(const q3Vec3& p) const {
    return q3Mul(m_tx, p);
}

const q3Vec3 q3Body::GetWorldVector(const q3Vec3& v) const {
    return q3Mul(m_tx.rotation, v);
}

const q3Vec3 q3Body::GetVelocityAtWorldPoint(const q3Vec3& p) const {
    q3Vec3 directionToPoint = p - m_worldCenter;
    q3Vec3 relativeAngularVel = q3Cross(m_angularVelocity, directionToPoint);
    return m_linearVelocity + relativeAngularVel;
}

void q3Body::SetLinearVelocity(const q3Vec3& v) {
    // Velocity of static bodies cannot be adjusted
    debug::assert(!flags.Static);
    m_linearVelocity = v;
}

void q3Body::SetAngularVelocity(const q3Vec3 v) {
    // Velocity of static bodies cannot be adjusted
    debug::assert(!flags.Static);
    m_angularVelocity = v;
}

bool q3Body::CanCollide(const q3Body* other) const {
    if (this == other) return false;

    // Every collision must have at least one dynamic body involved
    if (!flags.Dynamic && !other->flags.Dynamic) return false;

    return true;
}

void q3Body::SetTransform(const q3Vec3& position) {
    m_worldCenter = position;
    SynchronizeProxies();
}

void q3Body::SetTransform(const q3Vec3& position, const q3Vec3& axis, r32 angle) {
    m_worldCenter = position;
    m_q.Set(axis, angle);
    m_tx.rotation = m_q.ToMat3();
    SynchronizeProxies();
}

void q3Body::CalculateMassData() {
    q3Mat3 inertia = q3Diagonal(r32(0.0));
    m_invInertiaModel = q3Diagonal(r32(0.0));
    m_invInertiaWorld = q3Diagonal(r32(0.0));
    m_invMass = r32(0.0);
    m_mass = r32(0.0);
    r32 mass = r32(0.0);

    if (flags.Static || flags.Kinematic) {
        q3Identity(m_localCenter);
        m_worldCenter = m_tx.position;
        return;
    }

    q3Vec3 lc;
    q3Identity(lc);

    if (box.density != r32(0.0)) {
        q3MassData md;
        box.ComputeMass(&md);
        mass += md.mass;
        inertia += md.inertia;
        lc += md.center * md.mass;
    }

    if (mass > r32(0.0)) {
        m_mass = mass;
        m_invMass = r32(1.0) / mass;
        lc *= m_invMass;
        q3Mat3 identity;
        q3Identity(identity);
        inertia -= (identity * q3Dot(lc, lc) - q3OuterProduct(lc, lc)) * mass;
        m_invInertiaModel = q3Inverse(inertia);
    } else {
        // Force all dynamic bodies to have some mass
        m_invMass = r32(1.0);
        m_invInertiaModel = q3Diagonal(r32(0.0));
        m_invInertiaWorld = q3Diagonal(r32(0.0));
    }

    m_localCenter = lc;
    m_worldCenter = q3Mul(m_tx, lc);
}

void q3Body::SynchronizeProxies() {
    q3BroadPhase* broadphase = &m_scene->contact_manager.m_broadphase;

    m_tx.position = m_worldCenter - q3Mul(m_tx.rotation, m_localCenter);

    q3AABB aabb;
    q3Transform tx = m_tx;

    box.ComputeAABB(tx, &aabb);
    broadphase->Update(box.broadPhaseIndex, aabb);
}
