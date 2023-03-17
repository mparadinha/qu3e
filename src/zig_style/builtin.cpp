#pragma once

namespace builtin {

static void breakpoint() {
#if (COMP_VARS_CPU_ARCH == x86_64)
    __asm__("int3");
#else
#error "TODO: implement `breakpoint` for this arch"
#endif
}

} // namespace builtin
