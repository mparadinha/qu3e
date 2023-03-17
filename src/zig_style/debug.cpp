#pragma once

#include <stdarg.h>
#include <stdio.h>

#include "base.cpp"
#include "builtin.cpp"

namespace debug {

template <typename... ArgTypes>
static void print(const char* fmt, ArgTypes... args) {
    fprintf(stderr, fmt, args...);
}

static void printStackTrace() {
    // TODO
}

template <typename... ArgTypes>
[[noreturn]] static void panic(const char* fmt, ArgTypes... args) {
    print(fmt, args...);
    printStackTrace();
    builtin::breakpoint();
    exit(1);
}

static void assert(bool ok) {
    if (!ok) panic("assertion failed\n");
}

} // namespace debug

#define stub_crash() debug::panic("stub @ %s, line %d\n", __PRETTY_FUNCTION__, __LINE__);
