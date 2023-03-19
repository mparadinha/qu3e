#pragma once

#include "allocator.cpp"
#include "base.cpp"
#include "debug.cpp"
#include "mem.cpp"

template <typename T>
struct LinkedList {
    struct Node {
        Opt<Node*> prev;
        Opt<Node*> next;
        T data;
    };

    Opt<Node*> head;
    Allocator allocator;
    usize len;

    static LinkedList<T> init(Allocator allocator) {
        return LinkedList<T>{.head = Null, .allocator = allocator, .len = 0};
    }

    void deinit() {
        Opt<Node*> opt_node = this->head;
        while (opt_node.is_not_null()) {
            auto node = opt_node.unwrap();
            Opt<Node*> next = node->next;
            this->allocator.destroy(node);
            opt_node = next;
        }
        *this = undefined;
    }

    ErrOr<Node*> prepend(T item) {
        Node* node = try_expr(this->allocator.template create<Node>());
        node->prev = Null;
        node->next = this->head;
        node->data = item;
        if (opt_capture(node->next, next)) { next->prev = node; }
        this->len += 1;
        return node;
    }

    // if `node` does not belong to this list this will hopefully crash, but it might
    // invoke undefined behavior instead, who knows (good luck)
    void remove(Node* node) {
        if (node->prev.is_not_null()) { node->prev.unwrap()->next = node->next; }
        if (node->next.is_not_null()) { node->next.unwrap()->prev = node->prev; }
        this->allocator.destroy(node);
        this->len -= 1;
    }

    // assumes that this is a valid pointer to the `data` member of a `Node` which
    // belongs to this list
    void remove(T* data) {
        constexpr usize data_offset = offsetof(Node, data);
        auto data_ptr = reinterpret_cast<usize>(data);
        auto node_ptr = data_ptr - data_offset;
        Node* node = reinterpret_cast<Node*>(node_ptr);
        this->remove(node);
    }

    struct Iterator {
        Opt<Node*> cur_node;

        Iterator begin() { return *this; }

        Iterator end() { return Iterator{.cur_node = Null}; }

        bool operator==(Iterator other) { return this->cur_node == other.cur_node; }

        Iterator& operator++() {
            if (opt_capture(this->cur_node, node)) {
                this->cur_node = node->next;
            } else {
                this->cur_node = Null;
            }
            return *this;
        }

        T operator*() const { return this->cur_node.unwrap()->data; }
    };

    Iterator iter() const { return Iterator{.cur_node = this->head}; }

    struct PtrIterator : Iterator {
        T* operator*() const { return &this->cur_node.unwrap()->data; }
    };

    PtrIterator ptrIter() { return PtrIterator{.cur_node = this->head}; }
};
