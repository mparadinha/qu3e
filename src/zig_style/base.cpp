#pragma once

#include <bit>              // for std::bit_cast
#include <initializer_list> // for std::initiazlier_list
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>

#include "debug.cpp"

/// basic types
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
typedef size_t usize; // TODO(targets)
typedef int8_t i8;
typedef int16_t i16;
typedef int32_t i32;
typedef int64_t i64;
typedef float f32;
typedef double f64;

typedef struct {
} Void;

/// global error set
enum class [[nodiscard]] Error{
    None = 0,
    AccessDenied,
    BadPathName,
    DeviceBusy,
    FileBusy,
    FileLocksNotSupported,
    FileNotFound,
    FileTooBig,
    InvalidHandle,
    InvalidUtf8,
    IsDir,
    NameTooLong,
    NoDevice,
    NoSpaceLeft,
    NotDir,
    OutOfMemory,
    PathAlreadyExists,
    PipeBusy,
    ProcessFdQuotaExceeded,
    SharingViolation,
    SymLinkLoop,
    SystemFdQuotaExceeded,
    SystemResources,
    Unexpected,
    Unseekable,
    WouldBlock,
    Utf8InvalidStartByte,
    Utf8ExpectedContinuation,
    Utf8OverlongEncoding,
    Utf8CodepointTooLarge,
    Utf8EncodesSurrogateHalf,
    UnexpectedEndOfFile,
    ParseError,
};

// TODO: check if we're discarding an Error and don't allow that
#define discard_var(x) (x) = (x)

/// zig builtin functions
template <typename Dst, typename Src>
static Dst bitCast(Src src_int) {
    return std::bit_cast<Dst, Src>(src_int);
}

template <typename Dst, typename Src>
static Dst intCast(Src src_int) {
    const auto dst_int = (Dst)src_int;
    debug::assert((Src)dst_int == src_int);
    return dst_int;
}

template <typename Dst, typename Src>
static Dst ptrCast(Src src_ptr) {
    return reinterpret_cast<Dst>(src_ptr);
}

[[noreturn]] static void unreachable() {
    debug::panic("reached unreachable code\n");
}

struct Undefined {
    template <typename T>
    struct UndefinedBytes {
        u8 bytes[sizeof(T)];

        constexpr UndefinedBytes() {
            for (usize i = 0; i < sizeof(T); i++) bytes[i] = 0xaa;
        }
    };

    template <typename T>
    operator T() const {
        auto undef_bytes = UndefinedBytes<T>();
        return *ptrCast<const T*>(&undef_bytes);
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
        debug::assert(this->maybe);
        return this->value;
    }

    bool operator==(Opt<T> other) {
        if (!this->maybe and !other.maybe) return true; // both Null
        if (!this->maybe or !other.maybe) return false; // one of them is Null
        return this->value == other.value;
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
// note: this does *not* work with `while`!
#define opt_capture(opt, capture_name)                                                             \
    auto [_opt_maybe, capture_name] = (opt);                                                       \
    _opt_maybe

// to allow for iterating with index information: `for (auto [val, idx] : array.iter())`
template <typename T>
struct IdxIter {
    T* ptr;
    usize len;
    usize idx;

    IdxIter(T* ptr, usize len) : ptr(ptr), len(len), idx(0) {}

    IdxIter(T* ptr, usize len, usize idx) : ptr(ptr), len(len), idx(idx) {}

    struct Pair {
        T val;
        usize idx;
    };

    Pair operator*() { return Pair{.val = this->ptr[this->idx], .idx = this->idx}; }

    IdxIter& operator++() {
        this->idx += 1;
        return *this;
    }

    bool operator==(IdxIter iter) {
        return (this->ptr == iter.ptr) and (this->len == iter.len) and (this->idx == iter.idx);
    }

    auto begin() { return *this; }

    auto end() { return IdxIter<T>(this->ptr, this->len, this->len); }
};

template <typename T>
struct Slice {
    T* ptr;
    usize len;

    Slice() : ptr(nullptr), len(0) {}

    Slice(T* ptr, usize len) : ptr(ptr), len(len) {}

    Slice(const char* str) : ptr((T*)str), len(strlen(str)) {}

    T& operator[](usize idx) {
        debug::assert(idx < this->len);
        return this->ptr[idx];
    }

    const T& operator[](usize idx) const {
        debug::assert(idx < this->len);
        return this->ptr[idx];
    }

    Slice<T> slice(usize start, usize end) const {
        debug::assert(start < end);
        debug::assert(end <= this->len);
        return Slice(this->ptr + start, end - start);
    }

    Slice<T> sliceToEnd(usize start) const { return this->slice(start, this->len); }

    auto begin() const { return this->ptr; }

    auto end() const { return this->ptr + this->len; }

    IdxIter<T> iter() const { return IdxIter<T>(this->ptr, this->len); }
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
        debug::assert(idx <= this->len);
        return this->ptr[idx];
    }

    Slice<T> slice(usize start, usize end) const {
        debug::assert(start < end);
        debug::assert(end <= this->len);
        return Slice(this->ptr + start, end - start);
    }

    Slice<T> sliceToEnd(usize start) const { return this->slice(start, this->len); }

    auto begin() { return this->ptr; }

    auto end() { return this->ptr + this->len; }

    IdxIter<T> iter() { return IdxIter<T>(this->ptr, this->len); }
};

template <typename T, usize N>
struct Array {
    T* ptr;

    const usize len = N;

    Array() { this->ptr = this->buf; }

    Array(T arr[N]) { this->ptr = this->buf = arr; }

    // Array<T, N>(std::initializer_list<T> list) {
    //     for (usize i = 0; const auto elem : list) this->buf[i++] = elem;
    // }

    T& operator[](usize idx) {
        debug::assert(idx < N);
        return this->ptr[idx];
    }

    Slice<T> operator&() const { return this->slice(); }

    Slice<T> slice() const { return Slice(this->ptr, N); }

    Slice<T> slice(usize start, usize end) const {
        debug::assert(start < end);
        debug::assert(end <= N);
        return Slice(this->ptr + start, end - start);
    }

    Slice<T> sliceToEnd(usize start) const { return this->slice(start, this->len); }

    auto begin() { return this->ptr; }

    auto end() { return this->ptr + N; }

    IdxIter<T> iter() { return IdxIter<T>(this->ptr, N); }

private:
    T buf[N];
};

template <typename T>
struct [[nodiscard]] ErrOr {
    Error err;
    T value;

    ErrOr(Error err) : err(err) { debug::assert(err != Error::None); }

    ErrOr(T value) : err(Error::None), value(value) {}

    template <typename U>
    ErrOr(ErrOr<U> other_err) : err(other_err.err) {
        debug::assert(other_err.is_err());
    }

    bool is_err() const { return this->err != Error::None; }

    T unwrap() const {
        debug::assert(!this->is_err());
        return this->value;
    }
};

// special case for error unions that have no payload
struct [[nodiscard]] ErrOrVoid {
    Error err;

    ErrOrVoid(Error err) : err(err) { debug::assert(err != Error::None); }

    ErrOrVoid() : err(Error::None) {}

    bool is_err() const { return this->err != Error::None; }

    void unwrap() const { debug::assert(!this->is_err()); }
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

// we have tagged unions at home
template <typename E, typename U>
struct TaggedUnion {
    using Tag = E;
    using RawUnion = U;
    Tag tag;
    RawUnion data;
};

#define union_set(TU, union_var, field_name, field_var)                                            \
    ({                                                                                             \
        union_var.tag = TU::Tag::field_name;                                                       \
        union_var.data.field_name = field_var;                                                     \
    })
#define union_get(TU, union_var, field_name)                                                       \
    ({                                                                                             \
        debug::assert(union_var.tag == TU::Tag::field_name);                                       \
        union_var.data.field_name;                                                                 \
    })
#define union_make(TU, field_name, field_var)                                                      \
    TU {                                                                                           \
        .tag = TU::Tag::field_name, .data = TU::RawUnion {                                         \
            .field_name = field_var                                                                \
        }                                                                                          \
    }

/// defer implementation from https://www.gingerbill.org/article/2015/08/19/defer-in-cpp/
template <typename F>
struct privDefer {
    F f;

    privDefer(F f) : f(f) {}

    ~privDefer() { f(); }
};

template <typename F>
privDefer<F> defer_func(F f) {
    return privDefer<F>(f);
}

#define DEFER_1(x, y) x##y
#define DEFER_2(x, y) DEFER_1(x, y)
#define DEFER_3(x)    DEFER_2(x, __COUNTER__)
#define defer(code)   auto DEFER_3(_defer_) = defer_func([&]() { code; })

/// string table for global error set
static const char* errorName(Error err) {
    switch (err) {
        case Error::None: return "None";
        case Error::AccessDenied: return "AccessDenied";
        case Error::BadPathName: return "BadPathName";
        case Error::DeviceBusy: return "DeviceBusy";
        case Error::FileBusy: return "FileBusy";
        case Error::FileLocksNotSupported: return "FileLocksNotSupported";
        case Error::FileNotFound: return "FileNotFound";
        case Error::FileTooBig: return "FileTooBig";
        case Error::InvalidHandle: return "InvalidHandle";
        case Error::InvalidUtf8: return "InvalidUtf8";
        case Error::IsDir: return "IsDir";
        case Error::NameTooLong: return "NameTooLong";
        case Error::NoDevice: return "NoDevice";
        case Error::NoSpaceLeft: return "NoSpaceLeft";
        case Error::NotDir: return "NotDir";
        case Error::OutOfMemory: return "OutOfMemory";
        case Error::PathAlreadyExists: return "PathAlreadyExists";
        case Error::PipeBusy: return "PipeBusy";
        case Error::ProcessFdQuotaExceeded: return "ProcessFdQuotaExceeded";
        case Error::SharingViolation: return "SharingViolation";
        case Error::SymLinkLoop: return "SymLinkLoop";
        case Error::SystemFdQuotaExceeded: return "SystemFdQuotaExceeded";
        case Error::SystemResources: return "SystemResources";
        case Error::Unexpected: return "Unexpected";
        case Error::Unseekable: return "Unseekable";
        case Error::WouldBlock: return "WouldBlock";
        case Error::Utf8InvalidStartByte: return "Utf8InvalidStartByte";
        case Error::Utf8ExpectedContinuation: return "Utf8ExpectedContinuation";
        case Error::Utf8OverlongEncoding: return "Utf8OverlongEncoding";
        case Error::Utf8CodepointTooLarge: return "Utf8CodepointTooLarge";
        case Error::Utf8EncodesSurrogateHalf: return "Utf8EncodesSurrogateHalf";
        case Error::UnexpectedEndOfFile: return "UnexpectedEndOfFile";
        case Error::ParseError: return "ParseError";
    }
    unreachable();
}

template <typename ErrUnT>
static const char* errorName(ErrUnT err_un) {
    return errorName(err_un.err);
}
