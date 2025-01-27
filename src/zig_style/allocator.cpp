#pragma once

#include <stdlib.h>

#include "base.cpp"
#include "debug.cpp"

struct Allocator {
    template <typename T>
    ErrOr<Slice<T>> alloc(usize n) {
        T* ptr = (T*)malloc(sizeof(T) * n);
        debug::assert(ptr != nullptr);
        return Slice<T>(ptr, n);
    }

    template <typename T, T sentinel>
    ErrOr<SliceS<T, sentinel>> allocSentinel(usize n) {
        const auto bytes = try_expr(this->alloc<T>(n + 1));
        const auto slice_s = SliceS<T, sentinel>(bytes.ptr, bytes.len - 1);
        slice_s[n] = sentinel;
        return slice_s;
    }

    template <typename T>
    ErrOr<T*> create() {
        T* ptr = (T*)malloc(sizeof(T));
        debug::assert(ptr != nullptr);
        // debug::print("created %p\n", ptr);
        // debug::printStackTrace(stderr);
        *ptr = undefined;
        return ptr;
    }

    template <typename T>
    void free(Slice<T> memory) {
        ::free(memory.ptr);
    }

    template <typename T, T sentinel>
    void free(SliceS<T, sentinel> memory) {
        this->free(Slice<T>(memory));
    }

    template <typename T>
    void destroy(T* ptr) {
        *ptr = undefined;
        ::free((void*)ptr);
        // debug::print("destroyed %p\n", ptr);
        // debug::printStackTrace(stderr);
    }

    template <typename T>
    ErrOr<Slice<T>> realloc(Slice<T> old_mem, usize new_len) {
        T* ptr = (T*)::realloc(old_mem.ptr, sizeof(T) * new_len);
        debug::assert(ptr != nullptr);
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
