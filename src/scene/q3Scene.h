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

#pragma once

#include <stdio.h>

#include "../common/q3Types.h"
#include "../math/q3Math.h"
#include "../dynamics/q3ContactManager.h"

struct q3QueryCallback {
    virtual ~q3QueryCallback() {}

    virtual bool ReportShape(q3Box* box) = 0;
};

struct q3Scene {
    Allocator allocator;
    r32 dt;
    q3Vec3 gravity;
    bool new_box;
    bool allow_sleep;
    // Friction occurs when two rigid bodies have shapes that slide along one
    // another. The friction force resists this sliding motion.
    bool enable_friction;
    // Increasing the iteration count increases the CPU cost of simulating
    // Scene.Step(). Decreasing the iterations makes the simulation less
    // realistic (convergent). A good iteration number range is 5 to 20.
    usize iterations;
    q3ContactManager contact_manager;
    LinkedList<q3Body> bodies;

    q3Scene(
        r32 dt, const q3Vec3& gravity = q3Vec3(r32(0.0), r32(-9.8), r32(0.0)), usize iterations = 20
    );
    ~q3Scene();

    // Run the simulation forward in time by dt (fixed timestep). Variable
    // timestep is not supported.
    void Step();
    // helper for `q3Scene::Step`
    void BuildIsland(q3Island* island, q3Body* seed);

    // Construct a new rigid body. The BodyDef can be reused at the user's
    // discretion, as no reference to the BodyDef is kept.
    q3Body* CreateBody(const q3BodyDef& def);

    void RemoveBody(q3Body* body);

    void RemoveAllBodies();

    // Enables or disables rigid body sleeping. Sleeping is an effective CPU
    // optimization where bodies are put to sleep if they don't move much.
    // Sleeping bodies sit in memory without being updated, until the are
    // touched by something that wakes them up. The default is enabled.
    void SetAllowSleep(bool allowSleep);

    // Query the world to find any shapes that can potentially intersect
    // the provided AABB. This works by querying the broadphase with an
    // AAABB -- only *potential* intersections are reported. Perhaps the
    // user might use lmDistance as fine-grained collision detection.
    void QueryAABB(q3QueryCallback* cb, const q3AABB& aabb);

    // Query the world to find any shapes intersecting a world space point.
    void QueryPoint(q3QueryCallback* cb, const q3Vec3& point);

    // Query the world to find any shapes intersecting a ray.
    void RayCast(q3QueryCallback* cb, q3RaycastData& rayCast);

    // Render the scene with an interpolated time between the last frame and
    // the current simulation step.
    void Render(q3Render* render);
};
