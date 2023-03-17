#pragma once

#include "base.cpp"

namespace mem {
#include "allocator.cpp"

template <typename T>
static bool eql(Slice<T> a, Slice<T> b) {
    if (a.len != b.len) return false;
    if (a.ptr == b.ptr) return true;
    for (usize i = 0; i < a.len; i += 1)
        if (a[i] != b[i]) return false;
    return true;
}

template <typename T>
static void copy(Slice<T> dest, Slice<T> source) {
    debug::assert(dest.len >= source.len);
    for (usize i = 0; i < source.len; i += 1) dest[i] = source[i];
};

template <typename T>
static bool startsWith(Slice<T> haystack, Slice<T> needle) {
    if (needle.len > haystack.len) return false;
    return mem::eql<T>(haystack.slice(0, needle.len), needle);
}

template <typename T>
static Slice<u8> sliceAsBytes(Slice<T> slice) {
    return Slice<u8>(ptrCast<u8*>(slice.ptr), slice.len * sizeof(T));
}

template <typename T>
static Opt<usize> indexOfScalarPos(Slice<T> slice, usize start_index, T value) {
    for (usize i = start_index; i < slice.len; i += 1) {
        if (slice[i] == value) return i;
    }
    return Null;
}

template <typename T>
static Opt<usize> indexOfScalar(Slice<T> slice, T value) {
    return mem::indexOfScalarPos<T>(slice, 0, value);
}

template <typename T>
static Slice<T> trimRight(Slice<T> slice, Slice<T> values_to_strip) {
    auto end = slice.len;
    while (end > 0 and indexOfScalar<T>(values_to_strip, slice[end - 1]).is_not_null()) {
        end -= 1;
    }
    return slice.slice(0, end);
}

} // namespace mem
