/**
@file	q3Contact.h

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

#include "../collision/q3Box.h"
#include "../collision/q3Collide.h"
#include "../common/q3Settings.h"
#include "../math/q3Math.h"

r32 q3MixRestitution(const q3Box* A, const q3Box* B);
r32 q3MixFriction(const q3Box* A, const q3Box* B);

union q3FeaturePair {
    struct {
        u8 inR;
        u8 outR;
        u8 inI;
        u8 outI;
    };

    i32 key;
};

struct q3Contact {
    q3Vec3 position;       // World coordinate of contact
    r32 penetration;       // Depth of penetration from collision
    r32 normalImpulse;     // Accumulated normal impulse
    r32 tangentImpulse[2]; // Accumulated friction impulse
    r32 bias;              // Restitution + baumgarte
    r32 normalMass;        // Normal constraint mass
    r32 tangentMass[2];    // Tangent constraint mass
    q3FeaturePair fp;      // Features on A and B for this contact
};

struct q3Manifold {
    void SetPair(q3Box* a, q3Box* b);

    q3Box* A;
    q3Box* B;

    q3Vec3 normal;            // From A to B
    q3Vec3 tangentVectors[2]; // Tangent vectors
    q3Contact contacts[8];
    i32 contactCount;

    q3Manifold* next;
    q3Manifold* prev;

    bool sensor;
};

struct q3ContactConstraint; // what a mess

struct q3ContactEdge {
    q3Body* other;
    q3ContactConstraint* constraint;
    q3ContactEdge* next;
    q3ContactEdge* prev;
};

struct q3ContactConstraint {
    struct Flags {
        bool Colliding = false;    // Set when contact collides during a step
        bool WasColliding = false; // Set when two objects stop colliding
        bool Island = false;       // For internal marking during island forming
    };

    q3Box *A, *B;
    q3Body *bodyA, *bodyB;
    q3ContactEdge edgeA;
    q3ContactEdge edgeB;

    r32 friction;
    r32 restitution;
    q3Manifold manifold;

    Flags flags;

    void SolveCollision() {
        manifold.contactCount = 0;
        q3BoxtoBox(&manifold, A, B);
        if (manifold.contactCount > 0) {
            if (flags.Colliding) flags.WasColliding = true;
            flags.Colliding = true;
        } else {
            flags.WasColliding = flags.Colliding;
            flags.Colliding = false;
        }
    }
};
