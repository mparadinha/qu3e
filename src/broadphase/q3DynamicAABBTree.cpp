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

inline void FattenAABB(q3AABB& aabb) {
    const r32 k_fattener = r32(0.5);
    q3Vec3 v(k_fattener, k_fattener, k_fattener);
    aabb.min -= v;
    aabb.max += v;
}

q3DynamicAABBTree::q3DynamicAABBTree(Allocator allocator) {
    m_root = Node::Null;

    this->allocator = allocator;
    m_capacity = 1024;
    m_count = 0;
    nodes = allocator.alloc<Node>(m_capacity).unwrap();

    AddToFreeList(0);
}

q3DynamicAABBTree::~q3DynamicAABBTree() {
    allocator.free(nodes);
}

i32 q3DynamicAABBTree::Insert(const q3AABB& aabb, void* userData) {
    i32 id = AllocateNode();

    // Fatten AABB and set height/userdata
    nodes[id].aabb = aabb;
    FattenAABB(nodes[id].aabb);
    nodes[id].userData = userData;
    nodes[id].height = 0;

    InsertLeaf(id);

    return id;
}

void q3DynamicAABBTree::Remove(i32 id) {
    debug::assert(nodes[id].IsLeaf());

    RemoveLeaf(id);
    DeallocateNode(id);
}

bool q3DynamicAABBTree::Update(i32 id, const q3AABB& aabb) {
    debug::assert(nodes[id].IsLeaf());

    if (nodes[id].aabb.Contains(aabb)) return false;

    RemoveLeaf(id);

    nodes[id].aabb = aabb;
    FattenAABB(nodes[id].aabb);

    InsertLeaf(id);

    return true;
}

void* q3DynamicAABBTree::GetUserData(i32 id) const {
    return nodes[id].userData;
}

const q3AABB& q3DynamicAABBTree::GetFatAABB(i32 id) const {
    return nodes[id].aabb;
}

i32 q3DynamicAABBTree::AllocateNode() {
    if (m_freeList == Node::Null) {
        m_capacity *= 2;
        auto new_nodes = allocator.alloc<Node>(m_capacity).unwrap();
        mem::copy(new_nodes, nodes);
        allocator.free(nodes);
        nodes = new_nodes;

        AddToFreeList(m_count);
    }

    i32 freeNode = m_freeList;
    m_freeList = nodes[m_freeList].next;
    nodes[freeNode].height = 0;
    nodes[freeNode].left = Node::Null;
    nodes[freeNode].right = Node::Null;
    nodes[freeNode].parent = Node::Null;
    nodes[freeNode].userData = NULL;
    ++m_count;
    return freeNode;
}

i32 q3DynamicAABBTree::Balance(i32 iA) {
    Node* A = &nodes[iA];

    if (A->IsLeaf() || A->height == 1) return iA;

    /*      A
          /   \
         B     C
        / \   / \
       D   E F   G
    */

    i32 iB = A->left;
    i32 iC = A->right;
    Node* B = &nodes[iB];
    Node* C = &nodes[iC];

    i32 balance = C->height - B->height;

    // C is higher, promote C
    if (balance > 1) {
        i32 iF = C->left;
        i32 iG = C->right;
        Node* F = &nodes[iF];
        Node* G = &nodes[iG];

        // grandParent point to C
        if (A->parent != Node::Null) {
            if (nodes[A->parent].left == iA) {
                nodes[A->parent].left = iC;
            } else {
                nodes[A->parent].right = iC;
            }
        } else {
            m_root = iC;
        }

        // Swap A and C
        C->left = iA;
        C->parent = A->parent;
        A->parent = iC;

        // Finish rotation
        if (F->height > G->height) {
            C->right = iF;
            A->right = iG;
            G->parent = iA;
            A->aabb = q3Combine(B->aabb, G->aabb);
            C->aabb = q3Combine(A->aabb, F->aabb);

            A->height = 1 + q3Max(B->height, G->height);
            C->height = 1 + q3Max(A->height, F->height);
        } else {
            C->right = iG;
            A->right = iF;
            F->parent = iA;
            A->aabb = q3Combine(B->aabb, F->aabb);
            C->aabb = q3Combine(A->aabb, G->aabb);

            A->height = 1 + q3Max(B->height, F->height);
            C->height = 1 + q3Max(A->height, G->height);
        }

        return iC;
    } else if (balance < -1) { // B is higher, promote B
        i32 iD = B->left;
        i32 iE = B->right;
        Node* D = &nodes[iD];
        Node* E = &nodes[iE];

        // grandParent point to B
        if (A->parent != Node::Null) {
            if (nodes[A->parent].left == iA) {
                nodes[A->parent].left = iB;
            } else {
                nodes[A->parent].right = iB;
            }
        } else {
            m_root = iB;
        }

        // Swap A and B
        B->right = iA;
        B->parent = A->parent;
        A->parent = iB;

        // Finish rotation
        if (D->height > E->height) {
            B->left = iD;
            A->left = iE;
            E->parent = iA;
            A->aabb = q3Combine(C->aabb, E->aabb);
            B->aabb = q3Combine(A->aabb, D->aabb);

            A->height = 1 + q3Max(C->height, E->height);
            B->height = 1 + q3Max(A->height, D->height);
        } else {
            B->left = iE;
            A->left = iD;
            D->parent = iA;
            A->aabb = q3Combine(C->aabb, D->aabb);
            B->aabb = q3Combine(A->aabb, E->aabb);

            A->height = 1 + q3Max(C->height, D->height);
            B->height = 1 + q3Max(A->height, E->height);
        }

        return iB;
    }

    return iA;
}

void q3DynamicAABBTree::InsertLeaf(i32 id) {
    if (m_root == Node::Null) {
        m_root = id;
        nodes[m_root].parent = Node::Null;
        return;
    }

    // Search for sibling
    i32 searchIndex = m_root;
    q3AABB leafAABB = nodes[id].aabb;
    while (!nodes[searchIndex].IsLeaf()) {
        // Cost for insertion at index (branch node), involves creation
        // of new branch to contain this index and the new leaf
        q3AABB combined = q3Combine(leafAABB, nodes[searchIndex].aabb);
        r32 combinedArea = combined.SurfaceArea();
        r32 branchCost = r32(2.0) * combinedArea;

        // Inherited cost (surface area growth from heirarchy update after
        // descent)
        r32 inheritedCost = r32(2.0) * (combinedArea - nodes[searchIndex].aabb.SurfaceArea());

        i32 left = nodes[searchIndex].left;
        i32 right = nodes[searchIndex].right;

        // Calculate costs for left/right descents. If traversal is to a leaf,
        // then the cost of the combind AABB represents a new branch node.
        // Otherwise the cost is only the inflation of the pre-existing branch.
        r32 leftDescentCost;
        if (nodes[left].IsLeaf())
            leftDescentCost = q3Combine(leafAABB, nodes[left].aabb).SurfaceArea() + inheritedCost;
        else {
            r32 inflated = q3Combine(leafAABB, nodes[left].aabb).SurfaceArea();
            r32 branchArea = nodes[left].aabb.SurfaceArea();
            leftDescentCost = inflated - branchArea + inheritedCost;
        }

        // Cost for right descent
        r32 rightDescentCost;
        if (nodes[right].IsLeaf())
            rightDescentCost = q3Combine(leafAABB, nodes[right].aabb).SurfaceArea() + inheritedCost;
        else {
            r32 inflated = q3Combine(leafAABB, nodes[right].aabb).SurfaceArea();
            r32 branchArea = nodes[right].aabb.SurfaceArea();
            rightDescentCost = inflated - branchArea + inheritedCost;
        }

        // Determine traversal direction, or early out on a branch index
        if (branchCost < leftDescentCost && branchCost < rightDescentCost) break;

        if (leftDescentCost < rightDescentCost)
            searchIndex = left;

        else
            searchIndex = right;
    }

    i32 sibling = searchIndex;

    // Create new parent
    i32 oldParent = nodes[sibling].parent;
    i32 newParent = AllocateNode();
    nodes[newParent].parent = oldParent;
    nodes[newParent].userData = NULL;
    nodes[newParent].aabb = q3Combine(leafAABB, nodes[sibling].aabb);
    nodes[newParent].height = nodes[sibling].height + 1;

    // Sibling was root
    if (oldParent == Node::Null) {
        nodes[newParent].left = sibling;
        nodes[newParent].right = id;
        nodes[sibling].parent = newParent;
        nodes[id].parent = newParent;
        m_root = newParent;
    }

    else {
        if (nodes[oldParent].left == sibling)
            nodes[oldParent].left = newParent;

        else
            nodes[oldParent].right = newParent;

        nodes[newParent].left = sibling;
        nodes[newParent].right = id;
        nodes[sibling].parent = newParent;
        nodes[id].parent = newParent;
    }

    SyncHeirarchy(nodes[id].parent);
}

void q3DynamicAABBTree::RemoveLeaf(i32 id) {
    if (id == m_root) {
        m_root = Node::Null;
        return;
    }

    // Setup parent, grandParent and sibling
    i32 parent = nodes[id].parent;
    i32 grandParent = nodes[parent].parent;
    i32 sibling;

    if (nodes[parent].left == id)
        sibling = nodes[parent].right;

    else
        sibling = nodes[parent].left;

    // Remove parent and replace with sibling
    if (grandParent != Node::Null) {
        // Connect grandParent to sibling
        if (nodes[grandParent].left == parent)
            nodes[grandParent].left = sibling;

        else
            nodes[grandParent].right = sibling;

        // Connect sibling to grandParent
        nodes[sibling].parent = grandParent;
    }

    // Parent was root
    else {
        m_root = sibling;
        nodes[sibling].parent = Node::Null;
    }

    DeallocateNode(parent);
    SyncHeirarchy(grandParent);
}

void q3DynamicAABBTree::SyncHeirarchy(i32 index) {
    while (index != Node::Null) {
        index = Balance(index);

        i32 left = nodes[index].left;
        i32 right = nodes[index].right;

        nodes[index].height = 1 + q3Max(nodes[left].height, nodes[right].height);
        nodes[index].aabb = q3Combine(nodes[left].aabb, nodes[right].aabb);

        index = nodes[index].parent;
    }
}

inline void q3DynamicAABBTree::AddToFreeList(i32 index) {
    for (i32 i = index; i < m_capacity - 1; ++i) {
        nodes[i].next = i + 1;
        nodes[i].height = Node::Null;
    }

    nodes[m_capacity - 1].next = Node::Null;
    nodes[m_capacity - 1].height = Node::Null;
    m_freeList = index;
}

inline void q3DynamicAABBTree::DeallocateNode(i32 index) {
    nodes[index].next = m_freeList;
    nodes[index].height = Node::Null;
    m_freeList = index;
    --m_count;
}
