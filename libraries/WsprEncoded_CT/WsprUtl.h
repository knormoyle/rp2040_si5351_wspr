#ifndef WSPR_UTL_H
#define WSPR_UTL_H

#include <cctype>
#include <cstdint>
#include <cstring>

// Note: The namespace is PascalCase to preserve the project's existing public
// API. Strict Google style would rename it to `wspr_utl`, but doing so would
// break every caller of this library.
namespace WsprUtl {
class CString {
public:
    CString() {}

    // The buffer must be writable since this class mutates it through
    // Set(), ToUpper(), Trim*(), etc. Accepting `char*` here makes that
    // requirement visible at the type level (the previous overload took
    // `const char*` and silently `const_cast`'d it).
    CString(char* buf, size_t buf_capacity) {
        Target(buf, buf_capacity);
    }

    void Target(char* buf, size_t buf_capacity) {
        if (buf && buf_capacity) {
            buf_ = buf;
            buf_capacity_ = buf_capacity;
        }
    }

    void Clear() {
        if (buf_capacity_) {
            memset(buf_, 0, buf_capacity_);
        }
    }

    void Set(const char* str) {
        if (str && buf_capacity_) {
            // Clear() zero-fills the entire buffer, so we can copy up to
            // buf_capacity_ - 1 bytes without worrying about a separate
            // null terminator: byte [buf_capacity_ - 1] is already '\0',
            // and any unused trailing bytes are too. Using memcpy with an
            // explicit length avoids the -Wstringop-truncation warning
            // that strncpy(buf, str, capacity - 1) emits when callers pass
            // a string of exactly capacity - 1 characters (a common case
            // here: "0A0AAA" into a 7-byte buffer).
            Clear();
            size_t copy_max = buf_capacity_ - 1;
            size_t src_len  = strlen(str);
            size_t copy_len = src_len < copy_max ? src_len : copy_max;
            memcpy(buf_, str, copy_len);
        }
    }

    void ToUpper() {
        if (buf_capacity_) {
            char* p = buf_;
            while (*p != '\0') {
                *p = static_cast<char>(toupper(static_cast<unsigned char>(*p)));
                ++p;
            }
        }
    }

    bool IsPaddedLeft() const {
        return buf_capacity_ && buf_[0] == ' ';
    }

    bool IsPaddedRight() const {
        bool ret_val = false;
        size_t len = Len();
        if (len) {
            ret_val = buf_[len - 1] == ' ';
        }
        return ret_val;
    }

    bool IsUppercase() const {
        // empty string considered uppercase
        // non-alpha chars are considered uppercase
        bool ret_val = false;
        if (buf_capacity_) {
            if (buf_[0] == '\0') {
                ret_val = true;
            } else {
                const char* p = buf_;
                while (*p != '\0' &&
                       *p == static_cast<char>(
                                 toupper(static_cast<unsigned char>(*p)))) {
                    ++p;
                }
                ret_val = *p == '\0';
            }
        }
        return ret_val;
    }

    bool IsEqual(const char* str) const {
        bool ret_val = false;
        if (str && buf_capacity_) {
            ret_val = strcmp(buf_, str) == 0;
        }
        return ret_val;
    }

    size_t Len() const {
        size_t ret_val = 0;
        if (buf_capacity_) {
            ret_val = strlen(buf_);
        }
        return ret_val;
    }

    // Shift left any spaces.
    void TrimLeft() {
        if (buf_capacity_) {
            // find first non-space char
            size_t idx_first = 0;
            while (buf_[idx_first] == ' ') {
                ++idx_first;
            }
            if (buf_[idx_first] == '\0') {
                // whole string is whitespace; null-terminate at start
                Clear();
            } else if (idx_first == 0) {
                // nothing to do
            } else {
                // there was some whitespace, shift
                size_t len = Len();
                memmove(buf_, &buf_[idx_first], len - idx_first);
                buf_[len - idx_first] = '\0';
            }
        }
    }

    void TrimRight() {
        if (buf_capacity_) {
            size_t len = Len();
            for (int i = static_cast<int>(len) - 1; i >= 0; --i) {
                if (buf_[i] == ' ') {
                    buf_[i] = '\0';
                } else {
                    break;
                }
            }
        }
    }

    void Trim() {
        TrimRight();
        TrimLeft();
    }

    const char* Get() const {
        return buf_;
    }

private:
    char* buf_ = nullptr;
    size_t buf_capacity_ = 0;
};

// Rotate a uint8_t array of length 5 by `count` positions.
// Positive values rotate right, negative values rotate left, zero is no-op.
// This is the only rotation use site in the codebase (minute lists). It is
// intentionally not generic since templates are not used here.
inline void Rotate5(uint8_t val_list[5], int count) {
    const int kSize = 5;
    // normalize count into [0, kSize) representing a right rotation
    int right = count % kSize;
    if (right < 0) {
        right += kSize;
    }
    if (right == 0) {
        return;
    }
    uint8_t tmp[5];
    for (int i = 0; i < kSize; ++i) {
        tmp[(i + right) % kSize] = val_list[i];
    }
    for (int i = 0; i < kSize; ++i) {
        val_list[i] = tmp[i];
    }
}
}  // namespace WsprUtl

#endif  // WSPR_UTL_H
