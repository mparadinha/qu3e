#pragma once

#include "allocator.cpp"
#include "base.cpp"
#include "debug.cpp"
#include "mem.cpp"

template <typename T>
struct ArrayList {
    Slice<T> items;
    usize capacity;
    Allocator allocator;

    static ArrayList<T> init(Allocator allocator) {
        return ArrayList<T>{.items = Slice<T>(nullptr, 0), .capacity = 0, .allocator = allocator};
    }

    static ErrOr<ArrayList<T>> initCapacity(Allocator allocator, usize size) {
        auto list =
            ArrayList<T>{.items = Slice<T>(nullptr, 0), .capacity = 0, .allocator = allocator};
        try_expr(list.ensureTotalCapacity(size));
        return list;
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
        debug::assert(new_len <= this->capacity);
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
        debug::assert(new_len <= this->items.len);
        this->items.len = new_len;
    }

    ErrOrVoid resize(usize new_len) {
        this->ensureTotalCapacity(new_len);
        this->items.len = new_len;
        return {};
    }

    Slice<T> allocatedSlice() { return Slice<T>(this->items.ptr, this->capacity); }
};
