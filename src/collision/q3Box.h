/**
@file	q3Box.h

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

#include "../math/q3Mat3.h"
#include "../math/q3Transform.h"
#include "../math/q3Vec3.h"

struct q3MassData {
    q3Mat3 inertia;
    q3Vec3 center;
    r32 mass;
};

struct q3Box {
    q3Transform local;
    q3Vec3 e; // extent, as in the extent of each OBB axis

    class q3Body* body;
    r32 friction;
    r32 restitution;
    r32 density;
    i32 broadPhaseIndex;
    mutable void* userData;
    mutable bool sensor;

    bool TestPoint(const q3Transform& tx, const q3Vec3& p) const;
    bool Raycast(const q3Transform& tx, q3RaycastData* raycast) const;
    void ComputeAABB(const q3Transform& tx, q3AABB* aabb) const;
    void ComputeMass(q3MassData* md) const;
};

struct q3BoxDef {
    q3Transform m_tx;
    q3Vec3 m_e;

    r32 m_friction;
    r32 m_restitution;
    r32 m_density;
    bool m_sensor;

    q3BoxDef() {
        // Common default values
        m_friction = r32(0.4);
        m_restitution = r32(0.2);
        m_density = r32(1.0);
        m_sensor = false;
    }

    void Set(const q3Transform& tx, const q3Vec3& extents) {
        m_tx = tx;
        m_e = extents * r32(0.5);
    }
};
