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
    q3DynamicAABBTree();
    ~q3DynamicAABBTree();

    // Provide tight-AABB
    i32 Insert(const q3AABB& aabb, void* userData);
    void Remove(i32 id);
    bool Update(i32 id, const q3AABB& aabb);

    void* GetUserData(i32 id) const;
    const q3AABB& GetFatAABB(i32 id) const;

    template <typename T>
    void Query(T* cb, const q3AABB& aabb) const;
    template <typename T>
    void Query(T* cb, q3RaycastData& rayCast) const;

    // For testing
    void Validate() const;

    struct Node {
        bool IsLeaf(void) const {
            // The right leaf does not use the same memory as the userdata,
            // and will always be Null (no children)
            return right == Null;
        }

        // Fat AABB for leafs, bounding AABB for branches
        q3AABB aabb;

        union {
            i32 parent;
            i32 next; // free list
        };

        // Child indices
        struct {
            i32 left;
            i32 right;
        };

        // Since only leaf nodes hold userdata, we can use the
        // same memory used for left/right indices to store
        // the userdata void pointer
        void* userData;

        // leaf = 0, free nodes = -1
        i32 height;

        static const i32 Null = -1;
    };

    inline i32 AllocateNode();
    inline void DeallocateNode(i32 index);
    i32 Balance(i32 index);
    void InsertLeaf(i32 index);
    void RemoveLeaf(i32 index);
    void ValidateStructure(i32 index) const;

    // Correct AABB hierarchy heights and AABBs starting at supplied
    // index traversing up the heirarchy
    void SyncHeirarchy(i32 index);

    // Insert nodes at a given index until m_capacity into the free list
    void AddToFreeList(i32 index);

    i32 m_root;
    Node* m_nodes;
    i32 m_count;    // Number of active nodes
    i32 m_capacity; // Max capacity of nodes
    i32 m_freeList;
};

inline void q3DynamicAABBTree::AddToFreeList(i32 index) {
    for (i32 i = index; i < m_capacity - 1; ++i) {
        m_nodes[i].next = i + 1;
        m_nodes[i].height = Node::Null;
    }

    m_nodes[m_capacity - 1].next = Node::Null;
    m_nodes[m_capacity - 1].height = Node::Null;
    m_freeList = index;
}

inline void q3DynamicAABBTree::DeallocateNode(i32 index) {
    debug::assert(index >= 0 && index < m_capacity);

    m_nodes[index].next = m_freeList;
    m_nodes[index].height = Node::Null;
    m_freeList = index;

    --m_count;
}

template <typename T>
inline void q3DynamicAABBTree::Query(T* cb, const q3AABB& aabb) const {
    const i32 k_stackCapacity = 256;
    i32 stack[k_stackCapacity];
    i32 sp = 1;

    *stack = m_root;

    while (sp) {
        debug::assert(sp < k_stackCapacity);

        i32 id = stack[--sp];

        const Node* n = m_nodes + id;
        if (q3AABBtoAABB(aabb, n->aabb)) {
            if (n->IsLeaf()) {
                if (!cb->TreeCallBack(id)) return;
            } else {
                stack[sp++] = n->left;
                stack[sp++] = n->right;
            }
        }
    }
}

template <typename T>
void q3DynamicAABBTree::Query(T* cb, q3RaycastData& rayCast) const {
    const r32 k_epsilon = r32(1.0e-6);
    const i32 k_stackCapacity = 256;
    i32 stack[k_stackCapacity];
    i32 sp = 1;

    *stack = m_root;

    q3Vec3 p0 = rayCast.start;
    q3Vec3 p1 = p0 + rayCast.dir * rayCast.t;

    while (sp) {
        debug::assert(sp < k_stackCapacity);

        i32 id = stack[--sp];

        if (id == Node::Null) continue;

        const Node* n = m_nodes + id;

        q3Vec3 e = n->aabb.max - n->aabb.min;
        q3Vec3 d = p1 - p0;
        q3Vec3 m = p0 + p1 - n->aabb.min - n->aabb.max;

        r32 adx = q3Abs(d.x);

        if (q3Abs(m.x) > e.x + adx) continue;

        r32 ady = q3Abs(d.y);

        if (q3Abs(m.y) > e.y + ady) continue;

        r32 adz = q3Abs(d.z);

        if (q3Abs(m.z) > e.z + adz) continue;

        adx += k_epsilon;
        ady += k_epsilon;
        adz += k_epsilon;

        if (q3Abs(m.y * d.z - m.z * d.y) > e.y * adz + e.z * ady) continue;

        if (q3Abs(m.z * d.x - m.x * d.z) > e.x * adz + e.z * adx) continue;

        if (q3Abs(m.x * d.y - m.y * d.x) > e.x * ady + e.y * adx) continue;

        if (n->IsLeaf()) {
            if (!cb->TreeCallBack(id)) return;
        }

        else {
            stack[sp++] = n->left;
            stack[sp++] = n->right;
        }
    }
}
