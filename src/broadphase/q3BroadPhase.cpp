/**
@file	q3BroadPhase.cpp

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

#include "q3BroadPhase.h"
#include "../collision/q3Box.h"
#include "../common/q3Geometry.h"
#include "../dynamics/q3ContactManager.h"

q3BroadPhase::q3BroadPhase(Allocator allocator, q3ContactManager* manager) :
    m_tree(q3DynamicAABBTree(allocator)) {

    m_manager = manager;
    pairs = ArrayList<q3ContactPair>::initCapacity(allocator, 64).unwrap();
    moving_boxes = ArrayList<i32>::initCapacity(allocator, 64).unwrap();
    // aabb_list = ArrayList<Opt<q3AABB>>::init(allocator);
}

q3BroadPhase::~q3BroadPhase() {
    pairs.deinit();
    moving_boxes.deinit();
}

// void q3Broadphase::Query(q3C) {}

void q3BroadPhase::InsertBox(q3Box* box, const q3AABB& aabb) {
    i32 id = m_tree.Insert(aabb, box);
    // i32 id = aabb_list.items.len;
    // aabb_list.append(aabb).unwrap();
    box->broadPhaseIndex = id;
    moving_boxes.append(id).unwrap();
}

void q3BroadPhase::RemoveBox(const q3Box* box) {
    m_tree.Remove(box->broadPhaseIndex);
    // aabb_list.remove(box->broadPhaseIndex);
}

void q3BroadPhase::UpdatePairs() {
    pairs.shrinkRetainingCapacity(0);

    // Query the tree with all moving boxes
    for (auto [val, idx] : moving_boxes.items.iter()) {
        m_currentIndex = val;
        q3AABB aabb = m_tree.GetAABB(m_currentIndex);
        // @TODO: Use a static and non-static tree and query one against the other.
        // This will potentially prevent (gotta think about this more)
        // time wasted with queries of static bodies against static
        // bodies, and kinematic to kinematic.
        m_tree.Query(this, aabb);
    }

    // Reset the move buffer
    moving_boxes.shrinkRetainingCapacity(0);

    // Sort pairs to expose duplicates
    std::sort(
        pairs.items.ptr, pairs.items.ptr + pairs.items.len,
        [](const auto& lhs, const auto& rhs) -> bool {
            if (lhs.A < rhs.A) return true;
            if (lhs.A == rhs.A) return lhs.B < rhs.B;
            return false;
        }
    );

    // Queue manifolds for solving
    i32 i = 0;
    while (i < pairs.items.len) {
        // Add contact to manager
        q3ContactPair* pair = &pairs.items[i];
        q3Box* A = (q3Box*)m_tree.GetUserData(pair->A);
        q3Box* B = (q3Box*)m_tree.GetUserData(pair->B);
        m_manager->AddContact(A, B);
        ++i;

        // Skip duplicate pairs by iterating i until we find a unique pair
        while (i < pairs.items.len) {
            q3ContactPair* potentialDup = &pairs.items[i];
            if (pair->A != potentialDup->A || pair->B != potentialDup->B) break;
            ++i;
        }
    }
}

void q3BroadPhase::Update(i32 id, const q3AABB& aabb) {
    if (m_tree.Update(id, aabb)) moving_boxes.append(id).unwrap();
}

bool q3BroadPhase::TestOverlap(i32 A, i32 B) const {
    return q3AABBtoAABB(m_tree.GetAABB(A), m_tree.GetAABB(B));
}

inline bool q3BroadPhase::TreeCallBack(i32 index) {
    // Cannot collide with self
    if (index == m_currentIndex) return true;

    i32 iA = q3Min(index, m_currentIndex);
    i32 iB = q3Max(index, m_currentIndex);
    pairs.append({.A = iA, .B = iB}).unwrap();
    return true;
}
