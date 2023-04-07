/**
@file	q3Mat3.cpp

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

#include "q3Mat3.h"

q3Mat3::q3Mat3() {}

q3Mat3::q3Mat3(r32 a, r32 b, r32 c, r32 d, r32 e, r32 f, r32 g, r32 h, r32 i) {
    this->e.x = q3Vec3(a, b, c);
    this->e.y = q3Vec3(d, e, f);
    this->e.z = q3Vec3(g, h, i);
}

q3Mat3::q3Mat3(const q3Vec3& _x, const q3Vec3& _y, const q3Vec3& _z) {
    e.x = _x;
    e.y = _y;
    e.z = _z;
}

void q3Mat3::Set(r32 a, r32 b, r32 c, r32 d, r32 e, r32 f, r32 g, r32 h, r32 i) {
    this->e.x.Set(a, b, c);
    this->e.y.Set(d, e, f);
    this->e.z.Set(g, h, i);
}

void q3Mat3::Set(const q3Vec3& axis, r32 angle) {
    r32 s = std::sin(angle);
    r32 c = std::cos(angle);
    r32 x = axis.x;
    r32 y = axis.y;
    r32 z = axis.z;
    r32 xy = x * y;
    r32 yz = y * z;
    r32 zx = z * x;
    r32 t = r32(1.0) - c;

    Set(x * x * t + c, xy * t + z * s, zx * t - y * s, xy * t - z * s, y * y * t + c,
        yz * t + x * s, zx * t + y * s, yz * t - x * s, z * z * t + c);
}

void q3Mat3::SetRows(const q3Vec3& x, const q3Vec3& y, const q3Vec3& z) {
    e.x = x;
    e.y = y;
    e.z = z;
}

q3Mat3& q3Mat3::operator=(const q3Mat3& rhs) {
    e.x = rhs.e.x;
    e.y = rhs.e.y;
    e.z = rhs.e.z;

    return *this;
}

q3Mat3& q3Mat3::operator*=(const q3Mat3& rhs) {
    *this = *this * rhs;

    return *this;
}

q3Mat3& q3Mat3::operator*=(r32 f) {
    e.x *= f;
    e.y *= f;
    e.z *= f;

    return *this;
}

q3Mat3& q3Mat3::operator+=(const q3Mat3& rhs) {
    e.x += rhs.e.x;
    e.y += rhs.e.y;
    e.z += rhs.e.z;

    return *this;
}

q3Mat3& q3Mat3::operator-=(const q3Mat3& rhs) {
    e.x -= rhs.e.x;
    e.y -= rhs.e.y;
    e.z -= rhs.e.z;

    return *this;
}

q3Vec3& q3Mat3::operator[](u32 index) {
    switch (index) {
        case 0: return e.x;
        case 1: return e.y;
        case 2: return e.z;
        default: debug::assert(false); return e.x;
    }
}

const q3Vec3& q3Mat3::operator[](u32 index) const {
    switch (index) {
        case 0: return e.x;
        case 1: return e.y;
        case 2: return e.z;
        default: debug::assert(false); return e.x;
    }
}

const q3Vec3 q3Mat3::operator*(const q3Vec3& rhs) const {
    return q3Vec3(
        e.x.x * rhs.x + e.y.x * rhs.y + e.z.x * rhs.z,
        e.x.y * rhs.x + e.y.y * rhs.y + e.z.y * rhs.z,
        e.x.z * rhs.x + e.y.z * rhs.y + e.z.z * rhs.z
    );
}

const q3Mat3 q3Mat3::operator*(const q3Mat3& rhs) const {
    return q3Mat3((*this * rhs.e.x), (*this * rhs.e.y), (*this * rhs.e.z));
}

const q3Mat3 q3Mat3::operator*(r32 f) const {
    return q3Mat3(e.x * f, e.y * f, e.z * f);
}

const q3Mat3 q3Mat3::operator+(const q3Mat3& rhs) const {
    return q3Mat3(e.x + rhs.e.x, e.y + rhs.e.y, e.z + rhs.e.z);
}

const q3Mat3 q3Mat3::operator-(const q3Mat3& rhs) const {
    return q3Mat3(e.x - rhs.e.x, e.y - rhs.e.y, e.z - rhs.e.z);
}
