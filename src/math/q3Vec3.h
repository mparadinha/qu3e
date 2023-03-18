/**
@file	q3Vec3.h

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

#include <cmath>

#include "../common/q3Types.h"

r32 q3Abs(r32 a);
r32 q3Min(r32 a, r32 b);
r32 q3Max(r32 a, r32 b);

struct q3Vec3 {
    union {
        r32 v[3];

        struct {
            r32 x;
            r32 y;
            r32 z;
        };
    };

    q3Vec3();
    q3Vec3(r32 _x, r32 _y, r32 _z);

    void Set(r32 _x, r32 _y, r32 _z);
    void SetAll(r32 a);
    q3Vec3& operator+=(const q3Vec3& rhs);
    q3Vec3& operator-=(const q3Vec3& rhs);
    q3Vec3& operator*=(r32 f);
    q3Vec3& operator/=(r32 f);

    r32& operator[](u32 i);
    r32 operator[](u32 i) const;

    q3Vec3 operator-(void) const;

    const q3Vec3 operator+(const q3Vec3& rhs) const;
    const q3Vec3 operator-(const q3Vec3& rhs) const;
    const q3Vec3 operator*(r32 f) const;
    const q3Vec3 operator/(r32 f) const;
};

inline void q3Identity(q3Vec3& v) {
    v.Set(r32(0.0), r32(0.0), r32(0.0));
}

inline const q3Vec3 q3Mul(const q3Vec3& a, const q3Vec3& b) {
    return q3Vec3(a.x * b.x, a.y * b.y, a.z * b.z);
}

inline r32 q3Dot(const q3Vec3& a, const q3Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline const q3Vec3 q3Cross(const q3Vec3& a, const q3Vec3& b) {
    return q3Vec3((a.y * b.z) - (b.y * a.z), (b.x * a.z) - (a.x * b.z), (a.x * b.y) - (b.x * a.y));
}

inline r32 q3Length(const q3Vec3& v) {
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

inline r32 q3LengthSq(const q3Vec3& v) {
    return v.x * v.x + v.y * v.y + v.z * v.z;
}

inline const q3Vec3 q3Normalize(const q3Vec3& v) {
    r32 l = q3Length(v);

    if (l != r32(0.0)) {
        r32 inv = r32(1.0) / l;
        return v * inv;
    }

    return v;
}

inline r32 q3Distance(const q3Vec3& a, const q3Vec3& b) {
    r32 xp = a.x - b.x;
    r32 yp = a.y - b.y;
    r32 zp = a.z - b.z;

    return std::sqrt(xp * xp + yp * yp + zp * zp);
}

inline r32 q3DistanceSq(const q3Vec3& a, const q3Vec3& b) {
    r32 xp = a.x - b.x;
    r32 yp = a.y - b.y;
    r32 zp = a.z - b.z;

    return xp * xp + yp * yp + zp * zp;
}

inline const q3Vec3 q3Abs(const q3Vec3& v) {
    return q3Vec3(q3Abs(v.x), q3Abs(v.y), q3Abs(v.z));
}

inline const q3Vec3 q3Min(const q3Vec3& a, const q3Vec3& b) {
    return q3Vec3(q3Min(a.x, b.x), q3Min(a.y, b.y), q3Min(a.z, b.z));
}

inline const q3Vec3 q3Max(const q3Vec3& a, const q3Vec3& b) {
    return q3Vec3(q3Max(a.x, b.x), q3Max(a.y, b.y), q3Max(a.z, b.z));
}

inline const r32 q3MinPerElem(const q3Vec3& a) {
    return q3Min(a.x, q3Min(a.y, a.z));
}

inline const r32 q3MaxPerElem(const q3Vec3& a) {
    return q3Max(a.x, q3Max(a.y, a.z));
}
