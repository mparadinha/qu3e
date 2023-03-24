/**
@file	q3DynamicAABBTree.h

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

#include "../common/q3Types.h"
#include "../common/q3Geometry.h"
#include "../math/q3Math.h"

struct q3DynamicAABBTree {
    struct Node {
        q3AABB aabb;
        void* userdata;
    };

    ArrayList<Node> nodes;
    ArrayList<usize> empty_slots;

    q3DynamicAABBTree(Allocator allocator);
    ~q3DynamicAABBTree();

    // Provide tight-AABB
    i32 Insert(const q3AABB& aabb, void* userData);
    void Remove(i32 id);
    bool Update(i32 id, const q3AABB& aabb);

    void* GetUserData(i32 id) const;
    const q3AABB& GetAABB(i32 id) const;

    template <typename T>
    inline void Query(T* cb, const q3AABB& aabb) const {
        for (auto [node, idx] : nodes.items.iter()) {
            if (mem::indexOfScalar(empty_slots.items, idx).is_not_null()) continue;
            if (q3AABBtoAABB(aabb, node.aabb)) {
                if (!cb->TreeCallBack(idx)) return;
            }
        }
    }

    template <typename T>
    void Query(T* cb, q3RaycastData& rayCast) const {
        const r32 k_epsilon = r32(1.0e-6);
        q3Vec3 p0 = rayCast.start;
        q3Vec3 p1 = p0 + rayCast.dir * rayCast.t;

        for (auto [node, idx] : nodes.items.iter()) {
            if (mem::indexOfScalar(empty_slots.items, idx).is_not_null()) continue;

            q3Vec3 e = node.aabb.max - node.aabb.min;
            q3Vec3 d = p1 - p0;
            q3Vec3 m = p0 + p1 - node.aabb.min - node.aabb.max;

            r32 adx = q3Abs(d.x);
            r32 ady = q3Abs(d.y);
            r32 adz = q3Abs(d.z);
            if (q3Abs(m.x) > e.x + adx) continue;
            if (q3Abs(m.y) > e.y + ady) continue;
            if (q3Abs(m.z) > e.z + adz) continue;

            adx += k_epsilon;
            ady += k_epsilon;
            adz += k_epsilon;

            if (q3Abs(m.y * d.z - m.z * d.y) > e.y * adz + e.z * ady) continue;
            if (q3Abs(m.z * d.x - m.x * d.z) > e.x * adz + e.z * adx) continue;
            if (q3Abs(m.x * d.y - m.y * d.x) > e.x * ady + e.y * adx) continue;

            if (!cb->TreeCallBack(idx)) return;
        }
    }
};
