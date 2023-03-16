/**
@file	q3Types.h

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

#ifndef Q3TYPES_H
#define Q3TYPES_H

typedef float r32;
typedef double r64;
typedef float f32;
typedef double f64;
typedef signed char i8;
typedef signed short i16;
typedef signed int i32;
typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;

#define Q3_UNUSED(A) (void)A

// ===============
// zig style stuff
// ===============
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef size_t usize; // TODO(targets)

typedef struct {
} Void;

/// global error set
enum class [[nodiscard]] Error{
    None = 0,
    OutOfMemory,
};

#define discard_var(x) (x) = (x)

constexpr u8 undefined_bytes_pool[] = {
    0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
    0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
    0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
    0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
};

struct Undefined {
    template <typename T>
    operator T() const {
        // debug::assert(sizeof(T) <= sizeof(undefined_bytes_pool));
        return *ptrCast<const T*>(&undefined_bytes_pool[0]);
    }
};

constexpr Undefined undefined;

using NullType = struct {};

constexpr NullType Null = {};

template <typename T>
struct Opt {
    bool maybe;
    T value;

    using Inner = T;

    Opt() : maybe(undefined), value(undefined) {}

    constexpr Opt(T value) : maybe(true), value(value) {}

    Opt(NullType null) : maybe(false) { discard_var(null); }

    bool is_null() const { return !this->maybe; }

    bool is_not_null() const { return this->maybe; }

    T unwrap() const {
        // debug::assert(this->maybe);
        return this->value;
    }
};

// if you want `orelse_case` to be a whole block put it between `({` and `})`
// see `try_expr` for an example
#define orelse(opt, orelse_case) (opt.maybe) ? opt.value : orelse_case
#define orelse_return(opt, ret_val)                                                                \
    ({                                                                                             \
        const auto _tmp_result = (opt);                                                            \
        if (_tmp_result.is_null()) return ret_val;                                                 \
        _tmp_result.unwrap();                                                                      \
    })

// intended to use in `if` statements like: if (opt_capture(my_opt, value))
#define opt_capture(opt, capture_name)                                                             \
    auto [_opt_maybe, capture_name] = (opt);                                                       \
    _opt_maybe

template <typename T>
struct Slice {
    T* ptr;
    usize len;

    Slice() : ptr(nullptr), len(0) {}

    Slice(T* ptr, usize len) : ptr(ptr), len(len) {}

    Slice(const char* str) : ptr((T*)str), len(strlen(str)) {}

    T& operator[](usize idx) {
        // debug::assert(idx < this->len);
        return this->ptr[idx];
    }

    const T& operator[](usize idx) const {
        // debug::assert(idx < this->len);
        return this->ptr[idx];
    }

    Slice<T> slice(usize start, usize end) const {
        // debug::assert(start < end);
        // debug::assert(end <= this->len);
        return Slice(this->ptr + start, end - start);
    }

    Slice<T> sliceToEnd(usize start) const { return this->slice(start, this->len); }

    auto begin() { return this->ptr; }

    auto end() { return this->ptr + this->len; }
};

template <typename T, T sentinel_val>
struct SliceS {
    T* ptr;
    usize len;

    SliceS() : ptr(nullptr), len(0) {}

    SliceS(T* ptr, usize len) : ptr(ptr), len(len) {}

    SliceS(const char* str) : ptr((T*)str), len(strlen(str)) {}

    operator Slice<T>() const { return Slice<T>(this->ptr, this->len); }

    T& operator[](usize idx) const {
        // debug::assert(idx <= this->len);
        return this->ptr[idx];
    }

    Slice<T> slice(usize start, usize end) const {
        // debug::assert(start < end);
        // debug::assert(end <= this->len);
        return Slice(this->ptr + start, end - start);
    }

    Slice<T> sliceToEnd(usize start) const { return this->slice(start, this->len); }

    auto begin() { return this->ptr; }

    auto end() { return this->ptr + this->len; }
};

template <typename T>
struct [[nodiscard]] ErrOr {
    Error err;
    T value;

    // ErrOr(Error err) : err(err) { debug::assert(err != Error::None); }

    ErrOr(T value) : err(Error::None), value(value) {}

    template <typename U>
    ErrOr(ErrOr<U> other_err) : err(other_err.err) {
        // debug::assert(other_err.is_err());
    }

    bool is_err() const { return this->err != Error::None; }

    T unwrap() const {
        // debug::assert(!this->is_err());
        return this->value;
    }
};

// special case for error unions that have no payload
struct [[nodiscard]] ErrOrVoid {
    Error err;

    // ErrOrVoid(Error err) : err(err) { debug::assert(err != Error::None); }

    ErrOrVoid() : err(Error::None) {}

    bool is_err() const { return this->err != Error::None; }

    // void unwrap() const { debug::assert(!this->is_err()); }
};

// inspired by SerenityOS's AK/Try.h
#define try_expr(expr)                                                                             \
    ({                                                                                             \
        const auto _tmp_result = (expr);                                                           \
        if (_tmp_result.is_err()) return _tmp_result.err;                                          \
        _tmp_result.unwrap();                                                                      \
    })

#define catch_expr(expr, catch_expr)                                                               \
    ({                                                                                             \
        const auto _tmp_result = (expr);                                                           \
        (_tmp_result.is_err()) ? catch_expr : _tmp_result.unwrap();                                \
    })
#define catch_expr_return(expr, ret_val)                                                           \
    ({                                                                                             \
        const auto _tmp_result = (expr);                                                           \
        if (_tmp_result.is_err()) return ret_val;                                                  \
        _tmp_result.unwrap();                                                                      \
    })

// for the `if (err_union) |value|` type zig syntax. usage is like `opt_capture`
#define err_val_capture(err_un, capture_name)                                                      \
    auto [_err_un_err, capture_name] = (err_un);                                                   \
    _err_un_err != Error::None
// to emulate the `maybe_err() catch |err| {}` zig syntax. usage is like `opt_capture`
#define err_err_capture(err_un, capture_name)                                                      \
    auto capture_name = (err_un).err;                                                              \
    capture_name != Error::None

namespace mem {

template <typename T>
bool eql(Slice<T> a, Slice<T> b) {
    if (a.len != b.len) return false;
    if (a.ptr == b.ptr) return true;
    for (usize i = 0; i < a.len; i += 1)
        if (a[i] != b[i]) return false;
    return true;
}

template <typename T>
void copy(Slice<T> dest, Slice<T> source) {
    // debug::assert(dest.len >= source.len);
    for (usize i = 0; i < source.len; i += 1) dest[i] = source[i];
};

template <typename T>
bool startsWith(Slice<T> haystack, Slice<T> needle) {
    if (needle.len > haystack.len) return false;
    return mem::eql<T>(haystack.slice(0, needle.len), needle);
}

template <typename T>
Slice<u8> sliceAsBytes(Slice<T> slice) {
    return Slice<u8>(ptrCast<u8*>(slice.ptr), slice.len * sizeof(T));
}

template <typename T>
Opt<usize> indexOfScalarPos(Slice<T> slice, usize start_index, T value) {
    for (usize i = start_index; i < slice.len; i += 1) {
        if (slice[i] == value) return i;
    }
    return Null;
}

template <typename T>
Opt<usize> indexOfScalar(Slice<T> slice, T value) {
    return mem::indexOfScalarPos<T>(slice, 0, value);
}

template <typename T>
Slice<T> trimRight(Slice<T> slice, Slice<T> values_to_strip) {
    auto end = slice.len;
    while (end > 0 and indexOfScalar<T>(values_to_strip, slice[end - 1]).is_not_null()) {
        end -= 1;
    }
    return slice.slice(0, end);
}

}; // namespace mem

struct Allocator {
    template <typename T>
    ErrOr<Slice<T>> alloc(usize n) {
        T* ptr = (T*)malloc(sizeof(T) * n);
        // debug::assert(ptr != nullptr);
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
        // debug::assert(ptr != nullptr);
        // debug::print("allocated new slice: {.ptr=%p, .len=%lu}\n", ptr, n);
        return ptr;
    }

    template <typename T>
    void free(Slice<T> memory) {
        // debug::print("freeing slice: {.ptr=%p, .len=%lu}\n", memory.ptr, memory.len);
        ::free(memory.ptr);
    }

    template <typename T>
    void free(T* ptr) {
        ::free((void*)ptr);
    }

    template <typename T, T sentinel>
    void free(SliceS<T, sentinel> memory) {
        this->free(Slice<T>(memory));
    }

    template <typename T>
    ErrOr<Slice<T>> realloc(Slice<T> old_mem, usize new_len) {
        // debug::print("old_mem={.ptr=%p, .len=%lu}, new_len=%lu\n", old_mem.ptr, old_mem.len,
        // new_len); debug::print("new byte size: %lu\n", sizeof(T) * new_len);
        T* ptr = (T*)::realloc(old_mem.ptr, sizeof(T) * new_len);
        // debug::assert(ptr != nullptr);
        // debug::print("new_ptr=%p\n", ptr);
        return Slice<T>(ptr, new_len);
    }

    template <typename T>
    Slice<T> shrink(Slice<T> old_mem, usize new_len) {
        // debug::assert(new_len <= old_mem.len);
        // TODO: other allocators should call the vtable resize, but c allocator
        // doesnt need to do anything
        return Slice<T>(old_mem.ptr, new_len);
    }
};

template <typename T>
struct ArrayList {
    Slice<T> items;
    usize capacity;
    Allocator allocator;

    static ArrayList<T> init(Allocator allocator) {
        return ArrayList<T>{.items = Slice<T>(nullptr, 0), .capacity = 0, .allocator = allocator};
    }

    void deinit() {
        this->allocator.free(this->allocatedSlice());
        *this = undefined;
    }

    Slice<T> toOwnedSlice() {
        const auto result = this->allocator.shrink(this->allocatedSlice(), this->items.len);
        this->items = Slice<T>(nullptr, 0);
        this->capacity = 0;
        return result;
    }

    ErrOrVoid append(T item) {
        try_expr(this->ensureTotalCapacity(this->items.len + 1));
        this->items.len += 1;
        this->items[this->items.len - 1] = item;
        return {};
    }

    ErrOrVoid appendSlice(Slice<T> items) {
        try_expr(this->ensureUnusedCapacity(items.len));
        const auto old_len = this->items.len;
        const auto new_len = old_len + items.len;
        // debug::assert(new_len <= this->capacity);
        this->items.len = new_len;
        mem::copy<T>(this->items.sliceToEnd(old_len), items);
        return {};
    }

    ErrOrVoid ensureTotalCapacity(usize new_capacity) {
        if (new_capacity <= this->capacity) return {};
        const auto new_mem =
            try_expr(this->allocator.realloc(this->allocatedSlice(), new_capacity));
        this->items.ptr = new_mem.ptr;
        this->capacity = new_mem.len;
        return {};
    }

    ErrOrVoid ensureUnusedCapacity(usize additional_count) {
        return this->ensureTotalCapacity(this->items.len + additional_count);
    }

    void shrinkRetainingCapacity(usize new_len) {
        // debug::assert(new_len <= this->items.len);
        this->items.len = new_len;
    }

    Slice<T> allocatedSlice() { return Slice<T>(this->items.ptr, this->capacity); }
};

#endif // Q3TYPES_H
