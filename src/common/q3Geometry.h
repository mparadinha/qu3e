/**
@file	q3Geometry.h

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

#include "../math/q3Math.h"

struct q3AABB {
    q3Vec3 min;
    q3Vec3 max;

    bool Contains(const q3AABB& other) const;
    bool Contains(const q3Vec3& point) const;
    r32 SurfaceArea() const;
};

const q3AABB q3Combine(const q3AABB& a, const q3AABB& b);

struct q3HalfSpace {
    q3HalfSpace();
    q3HalfSpace(const q3Vec3& normal, r32 distance);

    void Set(const q3Vec3& a, const q3Vec3& b, const q3Vec3& c);
    void Set(const q3Vec3& n, const q3Vec3& p);
    const q3Vec3 Origin() const;
    r32 Distance(const q3Vec3& p) const;
    const q3Vec3 Projected(const q3Vec3& p) const;

    q3Vec3 normal;
    r32 distance;
};

struct q3RaycastData {
    q3Vec3 start; // Beginning point of the ray
    q3Vec3 dir;   // Direction of the ray (normalized)
    r32 t;        // Time specifying ray endpoint

    r32 toi;       // Solved time of impact
    q3Vec3 normal; // Surface normal at impact

    void Set(const q3Vec3& startPoint, const q3Vec3& direction, r32 endPointTime);

    // Uses toi, start and dir to compute the point at toi. Should
    // only be called after a raycast has been conducted with a
    // return value of true.
    const q3Vec3 GetImpactPoint() const;
};

inline void q3ComputeBasis(const q3Vec3& a, q3Vec3* __restrict b, q3Vec3* __restrict c) {

    if (q3Abs(a.x) >= r32(0.57735027))
        b->Set(a.y, -a.x, r32(0.0));
    else
        b->Set(r32(0.0), a.z, -a.y);

    *b = q3Normalize(*b);
    *c = q3Cross(a, *b);
}

inline bool q3AABBtoAABB(const q3AABB& a, const q3AABB& b) {
    if (a.max.x < b.min.x || a.min.x > b.max.x) return false;

    if (a.max.y < b.min.y || a.min.y > b.max.y) return false;

    if (a.max.z < b.min.z || a.min.z > b.max.z) return false;

    return true;
}

inline bool q3AABB::Contains(const q3AABB& other) const {
    return min.x <= other.min.x && min.y <= other.min.y && min.z <= other.min.z &&
           max.x >= other.max.x && max.y >= other.max.y && max.z >= other.max.z;
}

inline bool q3AABB::Contains(const q3Vec3& point) const {
    return min.x <= point.x && min.y <= point.y && min.z <= point.z && max.x >= point.x &&
           max.y >= point.y && max.z >= point.z;
}

inline r32 q3AABB::SurfaceArea() const {
    r32 x = max.x - min.x;
    r32 y = max.y - min.y;
    r32 z = max.z - min.z;

    return r32(2.0) * (x * y + x * z + y * z);
}

inline const q3AABB q3Combine(const q3AABB& a, const q3AABB& b) {
    q3AABB c;

    c.min = q3Min(a.min, b.min);
    c.max = q3Max(a.max, b.max);

    return c;
}

inline void q3RaycastData::Set(
    const q3Vec3& startPoint, const q3Vec3& direction, r32 endPointTime
) {
    start = startPoint;
    dir = direction;
    t = endPointTime;
}

inline const q3Vec3 q3RaycastData::GetImpactPoint() const {
    return q3Vec3(start + dir * toi);
}
