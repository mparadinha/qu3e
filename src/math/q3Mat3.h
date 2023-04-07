/**
@file	q3Mat3.h

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

#include <cstring> // memset

#include "../common/q3Types.h"
#include "q3Vec3.h"

struct q3Mat3 {
    union {
        struct {
            q3Vec3 x;
            q3Vec3 y;
            q3Vec3 z;
        } e;

        f32 cels[9];
    };

    q3Mat3();
    q3Mat3(r32 a, r32 b, r32 c, r32 d, r32 e, r32 f, r32 g, r32 h, r32 i);
    q3Mat3(const q3Vec3& _x, const q3Vec3& _y, const q3Vec3& _z);

    void Set(r32 a, r32 b, r32 c, r32 d, r32 e, r32 f, r32 g, r32 h, r32 i);
    void Set(const q3Vec3& axis, r32 angle);
    void SetRows(const q3Vec3& x, const q3Vec3& y, const q3Vec3& z);

    q3Mat3& operator=(const q3Mat3& rhs);
    q3Mat3& operator*=(const q3Mat3& rhs);
    q3Mat3& operator*=(r32 f);
    q3Mat3& operator+=(const q3Mat3& rhs);
    q3Mat3& operator-=(const q3Mat3& rhs);

    q3Vec3& operator[](u32 index);
    const q3Vec3& operator[](u32 index) const;

    q3Vec3 col(u32 i) {
        if (i == 0) return q3Vec3(e.x.x, e.y.x, e.z.x);
        if (i == 1) return q3Vec3(e.x.y, e.y.y, e.z.y);
        if (i == 2) return q3Vec3(e.x.z, e.y.z, e.z.z);
        unreachable();
    }

    const q3Vec3 operator*(const q3Vec3& rhs) const;
    const q3Mat3 operator*(const q3Mat3& rhs) const;
    const q3Mat3 operator*(r32 f) const;
    const q3Mat3 operator+(const q3Mat3& rhs) const;
    const q3Mat3 operator-(const q3Mat3& rhs) const;
};

inline void q3Identity(q3Mat3& m) {
    m.Set(r32(1.0), r32(0.0), r32(0.0), r32(0.0), r32(1.0), r32(0.0), r32(0.0), r32(0.0), r32(1.0));
}

inline const q3Mat3 q3Rotate(const q3Vec3& x, const q3Vec3& y, const q3Vec3& z) {
    return q3Mat3(x, y, z);
}

inline const q3Mat3 q3Transpose(const q3Mat3& m) {
    return q3Mat3(
        m.e.x.x, m.e.y.x, m.e.z.x,
        m.e.x.y, m.e.y.y, m.e.z.y,
        m.e.x.z, m.e.y.z, m.e.z.z
    );
}

inline void q3Zero(q3Mat3& m) {
    memset(&m, 0, sizeof(r32) * 9);
}

inline const q3Mat3 q3Diagonal(r32 a) {
    return q3Mat3(
        r32(a), r32(0.0), r32(0.0), r32(0.0), r32(a), r32(0.0), r32(0.0), r32(0.0), r32(a)
    );
}

inline const q3Mat3 q3Diagonal(r32 a, r32 b, r32 c) {
    return q3Mat3(
        r32(a), r32(0.0), r32(0.0), r32(0.0), r32(b), r32(0.0), r32(0.0), r32(0.0), r32(c)
    );
}

inline const q3Mat3 q3OuterProduct(const q3Vec3& u, const q3Vec3& v) {
    q3Vec3 a = v * u.x;
    q3Vec3 b = v * u.y;
    q3Vec3 c = v * u.z;

    return q3Mat3(a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z);
}

inline const q3Mat3 q3Covariance(q3Vec3* points, u32 numPoints) {
    r32 invNumPoints = r32(1.0) / r32(numPoints);
    q3Vec3 c = q3Vec3(r32(0.0), r32(0.0), r32(0.0));

    for (u32 i = 0; i < numPoints; ++i) c += points[i];

    c /= r32(numPoints);

    r32 m00, m11, m22, m01, m02, m12;
    m00 = m11 = m22 = m01 = m02 = m12 = r32(0.0);

    for (u32 i = 0; i < numPoints; ++i) {
        q3Vec3 p = points[i] - c;

        m00 += p.x * p.x;
        m11 += p.y * p.y;
        m22 += p.z * p.z;
        m01 += p.x * p.y;
        m02 += p.x * p.z;
        m12 += p.y * p.z;
    }

    r32 m01inv = m01 * invNumPoints;
    r32 m02inv = m02 * invNumPoints;
    r32 m12inv = m12 * invNumPoints;

    return q3Mat3(
        m00 * invNumPoints, m01inv, m02inv, m01inv, m11 * invNumPoints, m12inv, m02inv, m12inv,
        m22 * invNumPoints
    );
};

inline const q3Mat3 q3Inverse(const q3Mat3& m) {
    q3Vec3 tmp0, tmp1, tmp2;
    r32 detinv;

    tmp0 = q3Cross(m.e.y, m.e.z);
    tmp1 = q3Cross(m.e.z, m.e.x);
    tmp2 = q3Cross(m.e.x, m.e.y);

    detinv = r32(1.0) / q3Dot(m.e.z, tmp2);

    return q3Mat3(
        tmp0.x * detinv, tmp1.x * detinv, tmp2.x * detinv, tmp0.y * detinv, tmp1.y * detinv,
        tmp2.y * detinv, tmp0.z * detinv, tmp1.z * detinv, tmp2.z * detinv
    );
}
