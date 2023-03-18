/**
@file	q3Memory.h

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

#include <stdlib.h>

#include "q3Types.h"

inline void* q3Alloc(i32 bytes) {
    return malloc(bytes);
}

inline void q3Free(void* memory) {
    free(memory);
}

#define Q3_PTR_ADD(P, BYTES) ((decltype(P))(((u8*)P) + (BYTES)))

const i32 q3k_heapSize = 1024 * 1024 * 20;
const i32 q3k_heapInitialCapacity = 1024;

struct q3PagedAllocator {
    struct q3Block {
        q3Block* next;
    };

    struct q3Page {
        q3Page* next;
        q3Block* data;
    };

    q3PagedAllocator(i32 elementSize, i32 elementsPerPage);
    ~q3PagedAllocator();

    void* Allocate();
    void Free(void* data);

    void Clear();

    i32 m_blockSize;
    i32 m_blocksPerPage;

    q3Page* m_pages;
    i32 m_pageCount;

    q3Block* m_freeList;
};
