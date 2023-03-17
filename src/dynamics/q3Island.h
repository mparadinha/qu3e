/**
@file	q3Island.h

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

#ifndef Q3ISLAND_H
#define Q3ISLAND_H

#include "../common/q3Geometry.h"
#include "../common/q3Settings.h"
#include "../dynamics/q3Body.h"
#include "../dynamics/q3ContactManager.h"
#include "../dynamics/q3ContactSolver.h"
#include "../math/q3Math.h"

struct q3VelocityState {
    q3Vec3 w;
    q3Vec3 v;
};

struct q3Island {
    ArrayList<q3Body*> bodies;
    ArrayList<q3VelocityState> velocities;
    ArrayList<q3ContactConstraint*> contacts;
    ArrayList<q3ContactConstraintState> contact_states;
    f32 dt;
    q3Vec3 gravity;
    usize iterations;
    bool allow_sleep;
    bool enable_friction;

    static q3Island init(
        Allocator allocator, f32 dt, q3Vec3 gravity, usize iterations, bool allow_sleep,
        bool enable_friction
    ) {
        return q3Island{
            .bodies = ArrayList<q3Body*>::init(allocator),
            .velocities = ArrayList<q3VelocityState>::init(allocator),
            .contacts = ArrayList<q3ContactConstraint*>::init(allocator),
            .contact_states = ArrayList<q3ContactConstraintState>::init(allocator),
            .dt = dt,
            .gravity = gravity,
            .iterations = iterations,
            .allow_sleep = allow_sleep,
            .enable_friction = enable_friction,
        };
    }

    void deinit() {
        this->bodies.deinit();
        this->velocities.deinit();
        this->contacts.deinit();
        this->contact_states.deinit();
    }

    void Solve();
    void Add(q3Body* body);
    void Add(q3ContactConstraint* contact);
    void Initialize();
};

#endif // Q3ISLAND_H
