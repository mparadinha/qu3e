/**
@file	q3Body.h

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

#pragma once

#include <stdio.h>

#include "../common/q3Types.h"
#include "../math/q3Math.h"
#include "../math/q3Transform.h"
#include "../dynamics/q3Contact.h"

enum q3BodyType { eStaticBody, eDynamicBody, eKinematicBody };

struct q3BodyDef {
    // Initial world transformation
    q3Vec3 axis = q3Vec3(0, 0, 0);
    r32 angle = 0;
    q3Vec3 position = q3Vec3(0, 0, 0);

    // Initial velocity in world space
    q3Vec3 linearVelocity = q3Vec3(0, 0, 0);
    q3Vec3 angularVelocity = q3Vec3(0, 0, 0);

    r32 gravityScale = 1;

    r32 linearDamping = 0;
    r32 angularDamping = 0.1;

    // Static bodies never move or integrate (CPU efficient) and have infinite mass.
    // Dynamic bodies with zero mass are defaulted to a mass of 1.
    // Kinematic bodies have infinite mass, but *do* integrate and move around.
    // Kinematic bodies do not resolve any collisions.
    q3BodyType bodyType = eStaticBody;
};

struct q3Body {
    struct Flags {
        bool Island = false;
        bool Static = false;
        bool Dynamic = false;
        bool Kinematic = false;
    };

    q3Mat3 m_invInertiaModel;
    q3Mat3 m_invInertiaWorld;
    r32 m_mass;
    r32 m_invMass;
    q3Vec3 m_linearVelocity;
    q3Vec3 m_angularVelocity;
    q3Vec3 m_force;
    q3Vec3 m_torque;
    q3Transform m_tx;
    q3Quaternion m_q;
    q3Vec3 m_localCenter;
    q3Vec3 m_worldCenter;
    r32 m_gravityScale;
    Flags flags;

    q3Box box;
    q3Scene* m_scene;
    q3Body* m_next;
    q3Body* m_prev;
    i32 m_islandIndex;

    r32 m_linearDamping;
    r32 m_angularDamping;

    // contact manager owns the actual memory
    q3ContactEdge* contact_edge_list;

    q3Body(const q3BodyDef& def, q3Scene* scene);

    void CalculateMassData();
    void SynchronizeProxies();

    // After setting the edge data, call this to link it into the body's list
    void linkEdgeIntoList(q3ContactEdge* edge) {
        edge->prev = NULL;
        edge->next = this->contact_edge_list;
        if (edge->next) edge->next->prev = edge;
        this->contact_edge_list = edge;
    }

    // note this does not free any memory. only changes the links in the list
    void unlinkEdgeFromList(q3ContactEdge* edge) {
        if (edge->prev) edge->prev->next = edge->next;
        if (edge->next) edge->next->prev = edge->prev;
        if (this->contact_edge_list == edge) this->contact_edge_list = edge->next;
    }

    // Boxes are all defined in local space of their owning body.
    // Boxes cannot be defined relative to one another.
    // The body will recalculate its mass values.
    // No contacts will be created until the next q3Scene::Step() call.
    const q3Box* SetBox(const q3BoxDef& def);
    // Removes this box from the body and broadphase.
    // Forces the body to recompute its mass if the body is dynamic.
    void RemoveBox();
    // Removes all boxes from this body and the broadphase.
    void RemoveAllBoxes();

    void ApplyLinearForce(const q3Vec3& force);
    void ApplyForceAtWorldPoint(const q3Vec3& force, const q3Vec3& point);
    void ApplyLinearImpulse(const q3Vec3& impulse);
    void ApplyLinearImpulseAtWorldPoint(const q3Vec3& impulse, const q3Vec3& point);
    void ApplyTorque(const q3Vec3& torque);
    const q3Vec3 GetLocalPoint(const q3Vec3& p) const;
    const q3Vec3 GetLocalVector(const q3Vec3& v) const;
    const q3Vec3 GetWorldPoint(const q3Vec3& p) const;
    const q3Vec3 GetWorldVector(const q3Vec3& v) const;
    const q3Vec3 GetVelocityAtWorldPoint(const q3Vec3& p) const;
    void SetLinearVelocity(const q3Vec3& v);
    void SetAngularVelocity(const q3Vec3 v);
    bool CanCollide(const q3Body* other) const;

    // Manipulating the transformation of a body manually will result in
    // non-physical behavior. Contacts are updated upon the next call to
    // q3Scene::Step(). Parameters are in world space. All body types
    // can be updated.
    void SetTransform(const q3Vec3& position);
    void SetTransform(const q3Vec3& position, const q3Vec3& axis, r32 angle);
};
