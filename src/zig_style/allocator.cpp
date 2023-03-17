#pragma once

#include <stdlib.h>

#include "base.cpp"
#include "debug.cpp"

struct Allocator {
    template <typename T>
    ErrOr<Slice<T>> alloc(usize n) {
        T* ptr = (T*)malloc(sizeof(T) * n);
        debug::assert(ptr != nullptr);
        // debug::print("allocated new slice: {.ptr=%p, .len=%lu}\n", ptr, n);
        return Slice<T>(ptr, n);
    }

    template <typename T, T sentinel>
    ErrOr<SliceS<T, sentinel>> allocSentinel(usize n) {
        const auto bytes = try_expr(this->alloc<T>(n + 1));
        const auto slice_s = SliceS<T, sentinel>(bytes.ptr, bytes.len - 1);
        // debug::print("allocated new slice (w/ sentinel) {.ptr=%p, .len=%lu}\n", slice_s.ptr,
        // slice_s.len);
        slice_s[n] = sentinel;
        return slice_s;
    }

    template <typename T>
    ErrOr<T*> create() {
        T* ptr = (T*)malloc(sizeof(T));
        debug::assert(ptr != nullptr);
        // debug::print("allocated new ptr: .ptr=%p\n", ptr);
        return ptr;
    }

    template <typename T>
    void free(Slice<T> memory) {
        // debug::print("freeing slice: {.ptr=%p, .len=%lu}\n", memory.ptr, memory.len);
        ::free(memory.ptr);
    }

    template <typename T, T sentinel>
    void free(SliceS<T, sentinel> memory) {
        this->free(Slice<T>(memory));
    }

    template <typename T>
    void destroy(T* ptr) {
        // debug::print("freeing slice: {.ptr=%p, .len=%lu}\n", memory.ptr, memory.len);
        ::free((void*)ptr);
    }

    template <typename T>
    ErrOr<Slice<T>> realloc(Slice<T> old_mem, usize new_len) {
        // debug::print("old_mem={.ptr=%p, .len=%lu}, new_len=%lu\n", old_mem.ptr, old_mem.len,
        // new_len); debug::print("new byte size: %lu\n", sizeof(T) * new_len);
        T* ptr = (T*)::realloc(old_mem.ptr, sizeof(T) * new_len);
        debug::assert(ptr != nullptr);
        // debug::print("new_ptr=%p\n", ptr);
        return Slice<T>(ptr, new_len);
    }

    template <typename T>
    Slice<T> shrink(Slice<T> old_mem, usize new_len) {
        debug::assert(new_len <= old_mem.len);
        // TODO: other allocators should call the vtable resize, but c allocator
        // doesnt need to do anything
        return Slice<T>(old_mem.ptr, new_len);
    }
};
