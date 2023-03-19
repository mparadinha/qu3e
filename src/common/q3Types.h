/**
@file	q3Types.h

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

typedef float r32;
typedef double r64;
typedef float f32;
typedef double f64;
typedef signed char i8;
typedef signed short i16;
typedef signed int i32;
typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;

#define Q3_UNUSED(A) (void)A

// forward declare all the types here because this is a mess
struct q3AABB;
struct q3Body;
struct q3BodyDef;
struct q3Box;
struct q3BoxDef;
struct q3BroadPhase;
struct q3ClipVertex;
struct q3Contact;
struct q3ContactConstraint;
struct q3ContactConstraintState;
struct q3ContactEdge;
struct q3ContactListener;
struct q3ContactManager;
struct q3ContactPair;
struct q3ContactSolver;
struct q3ContactState;
struct q3DynamicAABBTree;
struct q3HalfSpace;
struct q3Island;
struct q3Manifold;
struct q3MassData;
struct q3Mat3;
struct q3PagedAllocator;
struct q3Quaternion;
struct q3QueryCallback;
struct q3RaycastData;
struct q3Render;
struct q3Scene;
struct q3Transform;
struct q3Vec3;
struct q3VelocityState;

// below is the zig style stuff

#ifdef assert
#undef assert
#define assert_was_already_defined
#endif

#include "../zig_style/allocator.cpp"
#include "../zig_style/array_list.cpp"
#include "../zig_style/base.cpp"
#include "../zig_style/builtin.cpp"
#include "../zig_style/debug.cpp"
#include "../zig_style/linked_list.cpp"
#include "../zig_style/mem.cpp"

#ifdef assert_was_already_defined
constexpr auto assert = debug::assert;
#endif
