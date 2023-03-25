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

inline q3AABB FatAABB(q3AABB aabb) {
    const r32 k_fattener = r32(0.5);
    q3Vec3 v(k_fattener, k_fattener, k_fattener);
    aabb.min -= v;
    aabb.max += v;
    return aabb;
}

q3BroadPhase::q3BroadPhase(Allocator allocator) {
    pairs = ArrayList<q3ContactPair>::initCapacity(allocator, 64).unwrap();
    moving_boxes = ArrayList<i32>::initCapacity(allocator, 64).unwrap();
    boxes = ArrayList<BoxInfo>::init(allocator);
    unused_boxes = ArrayList<usize>::init(allocator);
}

q3BroadPhase::~q3BroadPhase() {
    pairs.deinit();
    moving_boxes.deinit();
    boxes.deinit();
    unused_boxes.deinit();
}

void q3BroadPhase::InsertBox(q3Box* box, const q3AABB& aabb) {
    i32 id = -1;
    if (opt_capture(unused_boxes.popOrNull(), idx)) {
        id = intCast<i32>(idx);
    } else {
        id = intCast<i32>(boxes.items.len);
        boxes.append({}).unwrap();
    }

    boxes.items[id] = {.box = box, .aabb = aabb};
    box->broadPhaseIndex = id;
    moving_boxes.append(id).unwrap();
}

BoxInfo q3BroadPhase::GetBoxInfo(i32 id) { return boxes.items[id]; }

void q3BroadPhase::RemoveBox(const q3Box* box) {
    i32 id = box->broadPhaseIndex;
    boxes.items[id] = undefined;
    unused_boxes.append(intCast<usize>(id)).unwrap();
}

void q3BroadPhase::UpdatePairs(q3ContactManager* manager) {
    pairs.shrinkRetainingCapacity(0);

    // Query the tree with all moving boxes
    for (auto moving_box_idx : moving_boxes.items) {
        q3AABB aabb = GetBoxInfo(moving_box_idx).aabb;
        for (auto [node, index] : boxes.items.iter()) {
            if (q3AABBtoAABB(aabb, node.aabb)) {
                if (index == moving_box_idx) break; // Cannot collide with self
                i32 iA = q3Min(index, moving_box_idx);
                i32 iB = q3Max(index, moving_box_idx);
                pairs.append({.A = iA, .B = iB}).unwrap();
                break;
            }
        }
    }

    moving_boxes.shrinkRetainingCapacity(0);
}

void q3BroadPhase::Update(i32 id, const q3AABB& aabb) {
    if (!boxes.items[id].aabb.Contains(aabb)) {
        boxes.items[id].aabb = FatAABB(aabb);
        moving_boxes.append(id).unwrap();
    }
}

bool q3BroadPhase::TestOverlap(i32 A, i32 B) {
    return q3AABBtoAABB(GetBoxInfo(A).aabb, GetBoxInfo(B).aabb);
}
