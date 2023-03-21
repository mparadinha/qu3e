/**
@file	q3BroadPhase.h

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
#include "q3DynamicAABBTree.h"

struct q3ContactPair {
    i32 A;
    i32 B;
};

struct q3BroadPhase {
    q3BroadPhase(Allocator allocator, q3ContactManager* manager);
    ~q3BroadPhase();

    void InsertBox(q3Box* shape, const q3AABB& aabb);
    void RemoveBox(const q3Box* shape);

    // Generates the contact list. All previous contacts are returned to the
    // allocator before generation occurs.
    void UpdatePairs(void);

    void Update(i32 id, const q3AABB& aabb);

    bool TestOverlap(i32 A, i32 B) const;

    q3ContactManager* m_manager;

    q3ContactPair* m_pairBuffer;
    i32 m_pairCount;
    i32 m_pairCapacity;

    i32* m_moveBuffer;
    i32 m_moveCount;
    i32 m_moveCapacity;

    q3DynamicAABBTree m_tree;
    i32 m_currentIndex;

    void BufferMove(i32 id);
    bool TreeCallBack(i32 index);
};

inline bool q3BroadPhase::TreeCallBack(i32 index) {
    // Cannot collide with self
    if (index == m_currentIndex) return true;

    if (m_pairCount == m_pairCapacity) {
        auto old_slice = Slice<q3ContactPair>(m_pairBuffer, m_pairCapacity);
        m_pairCapacity *= 2;
        auto allocator = Allocator();
        m_pairBuffer = allocator.alloc<q3ContactPair>(m_pairCapacity).unwrap().ptr;
        memcpy(m_pairBuffer, old_slice.ptr, m_pairCount * sizeof(q3ContactPair));
        allocator.free(old_slice);
    }

    i32 iA = q3Min(index, m_currentIndex);
    i32 iB = q3Max(index, m_currentIndex);

    m_pairBuffer[m_pairCount].A = iA;
    m_pairBuffer[m_pairCount].B = iB;
    ++m_pairCount;

    return true;
}
