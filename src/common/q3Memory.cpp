/**
@file	q3Memory.cpp

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

#include <cstdio>

#include "../math/q3Math.h"
#include "q3Memory.h"

q3PagedAllocator::q3PagedAllocator(i32 elementSize, i32 elementsPerPage) {
    m_blockSize = elementSize;
    m_blocksPerPage = elementsPerPage;

    m_pages = NULL;
    m_pageCount = 0;

    m_freeList = NULL;
}

q3PagedAllocator::~q3PagedAllocator() {
    Clear();
}

void* q3PagedAllocator::Allocate() {
    if (m_freeList) {
        q3Block* data = m_freeList;
        m_freeList = data->next;

        return data;
    }

    else {
        q3Page* page = (q3Page*)q3Alloc(m_blockSize * m_blocksPerPage + sizeof(q3Page));
        ++m_pageCount;

        page->next = m_pages;
        page->data = (q3Block*)Q3_PTR_ADD(page, sizeof(q3Page));
        m_pages = page;

        i32 blocksPerPageMinusOne = m_blocksPerPage - 1;
        for (i32 i = 0; i < blocksPerPageMinusOne; ++i) {
            q3Block* node = Q3_PTR_ADD(page->data, m_blockSize * i);
            q3Block* next = Q3_PTR_ADD(page->data, m_blockSize * (i + 1));
            node->next = next;
        }

        q3Block* last = Q3_PTR_ADD(page->data, m_blockSize * (blocksPerPageMinusOne));
        last->next = NULL;

        m_freeList = page->data->next;

        return page->data;
    }
}

void q3PagedAllocator::Free(void* data) {
#ifdef _DEBUG
    bool found = false;

    for (q3Page* page = m_pages; page; page = page->next) {
        if (data >= page->data && data < Q3_PTR_ADD(page->data, m_blockSize * m_blocksPerPage)) {
            found = true;
            break;
        }
    }

    // Address of data does not lie within any pages of this allocator.
    debug::assert(found);
#endif // DEBUG

    ((q3Block*)data)->next = m_freeList;
    m_freeList = ((q3Block*)data);
}

void q3PagedAllocator::Clear() {
    q3Page* page = m_pages;

    for (i32 i = 0; i < m_pageCount; ++i) {
        q3Page* next = page->next;
        q3Free(page);
        page = next;
    }

    m_freeList = NULL;
    m_pageCount = 0;
}
