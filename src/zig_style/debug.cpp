#pragma once

#include <cxxabi.h>
#include <elfutils/libdwfl.h>
#include <execinfo.h>
#include <stacktrace>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#include "base.cpp"
#include "builtin.cpp"

namespace debug {

template <typename... ArgTypes>
static void print(const char* fmt, ArgTypes... args) {
    fprintf(stderr, fmt, args...);
}

static void printStackTrace(FILE* file) {
    // get return address for stack trace
    constexpr size_t addr_buf_size = 100;
    void* addr_buf[addr_buf_size];
    int32_t addrs_got = backtrace(addr_buf, addr_buf_size);
    char** symbols = backtrace_symbols(addr_buf, addrs_got);

    // backtrace_symbols(3) can only has the function name and a hex offset it.
    // instead we use libdwfl which gives us line, source file information,
    // and the function name as well

    // init libdwfl session
    char* debuginfo_path = NULL;
    Dwfl_Callbacks callbacks = {
        .find_elf = dwfl_linux_proc_find_elf,
        .find_debuginfo = dwfl_standard_find_debuginfo,
        .debuginfo_path = &debuginfo_path,
    };
    Dwfl* dwfl = dwfl_begin(&callbacks);
    dwfl_linux_proc_report(dwfl, getpid());
    dwfl_report_end(dwfl, NULL, NULL);

    // the first function in the backtrace is this one we're in
    // right now, so start printing after that first symbol
    for (uint32_t i = 1; i < (uint32_t)addrs_got; i++) {
        uint64_t addr = (uint64_t)addr_buf[i];

        Dwfl_Module* module = dwfl_addrmodule(dwfl, addr);
        // function name
        const char* fn_name = dwfl_module_addrname(module, addr); // NULL if unavailable
        // source filename and line number
        int line = -1, column = -1;
        const char* filename = NULL;
        Dwfl_Line* module_line = dwfl_module_getsrc(module, addr);
        if (module_line) {
            filename = dwfl_lineinfo(module_line, &addr, &line, &column, NULL, NULL);
            // TODO: free this ^
        }

        // turn filename into a canonical path (resolve symlinks, remove ../ etc.)
        filename = realpath(filename, NULL);
        // TODO: free this ^

        // try to demangle c++ function names if needed
        int demangle_status;
        char* demangled_name = abi::__cxa_demangle(fn_name, NULL, NULL, &demangle_status);
        // TODO: free this ^
        fn_name = demangle_status == 0 ? demangled_name : fn_name;

        if (filename && line != -1 && column != -1) {
            // fprintf(file, "called from: %s @ %s:%d:%d\n", fn_name, filename, line, column);
            const char* bold_code = "\033[1m";
            const char* reset_code = "\033[0m";
            const char* green_code = "\033[92;1m";
            fprintf(file, "%s%s:%d:%d: %s", bold_code, filename, line, column, reset_code);
            fprintf(file, "%s\n", fn_name);
            FILE* current_file = fopen(filename, "r");
            for (size_t line_idx = 0; line_idx < line; line_idx++) {
                char linebuf[0x1000];
                char* line_content = fgets(&linebuf[0], 0x1000, current_file);
                if (*line_content == EOF) break;
                if (line_idx + 1 == line) {
                    fprintf(file, "%s", line_content);
                    fprintf(file, "%s%*c%s\n", green_code, column, '^', reset_code);
                }
            }
            fclose(current_file);
        } else {
            fprintf(file, "called from: %s\n", fn_name);
        }
    }

    free(symbols);
    dwfl_end(dwfl);
}

template <typename... ArgTypes>
[[noreturn]] static void panic(const char* fmt, ArgTypes... args) {
    print(fmt, args...);
    printStackTrace(stderr);
    builtin::breakpoint();
    exit(1);
}

static void assert(bool ok) {
    if (!ok) panic("assertion failed\n");
}

} // namespace debug

#define stub_crash() debug::panic("stub @ %s, line %d\n", __PRETTY_FUNCTION__, __LINE__);
