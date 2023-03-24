/**
@file	q3DynamicAABBTree.cpp

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

#include "q3DynamicAABBTree.h"

q3DynamicAABBTree::q3DynamicAABBTree(Allocator allocator) {
    nodes = ArrayList<Node>::init(allocator);
    empty_slots = ArrayList<usize>::init(allocator);
}

q3DynamicAABBTree::~q3DynamicAABBTree() {
    nodes.deinit();
    empty_slots.deinit();
}

i32 q3DynamicAABBTree::Insert(const q3AABB& aabb, void* userData) {
    if (empty_slots.items.len > 0) {
        i32 id = empty_slots.pop();
        nodes.items[id] = {.aabb = aabb, .userdata = userData};
        return id;
    } else {
        nodes.append({.aabb = aabb, .userdata = userData}).unwrap();
        return nodes.items.len - 1;
    }
}

void q3DynamicAABBTree::Remove(i32 id) {
    empty_slots.append(id).unwrap();
}

bool q3DynamicAABBTree::Update(i32 id, const q3AABB& aabb) {
    if (nodes.items[id].aabb.Contains(aabb)) return false;
    nodes.items[id].aabb = aabb;
    return true;
}

void* q3DynamicAABBTree::GetUserData(i32 id) const {
    return nodes.items[id].userdata;
}

const q3AABB& q3DynamicAABBTree::GetAABB(i32 id) const {
    return nodes.items[id].aabb;
}