#pragma once

#include "allocator.cpp"
#include "base.cpp"
#include "debug.cpp"
#include "mem.cpp"

template <typename T>
struct DoublyLinkedList {
    struct Node {
        Opt<Node*> prev;
        Opt<Node*> next;
        T data;
    };

    Opt<Node*> head;
    Allocator allocator;

    static DoublyLinkedList<T> init(Allocator allocator) {
        return DoublyLinkedList<T>{.head = Null, .allocator = allocator};
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
};
