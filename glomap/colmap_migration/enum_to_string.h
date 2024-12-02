#ifndef GLOMAP_ENUM_TO_STRING_H
#define GLOMAP_ENUM_TO_STRING_H

#include <string_view>
#include <ostream>
#include <glog/logging.h>

namespace glomap {

// Helper macros for counting arguments
#define _ENUM_ARG_N( \
     _1, _2, _3, _4, _5, _6, _7, _8, _9,_10, \
    _11,_12,_13,_14,_15, N,...) N

#define _ENUM_RSEQ_N() 15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0
#define _ENUM_NUM_ARGS(...) _ENUM_ARG_N(__VA_ARGS__)
#define NUM_ARGS(...) _ENUM_NUM_ARGS(__VA_ARGS__, _ENUM_RSEQ_N())

// Base case and recursive enum value definition
#define ENUM_VALUE_0(start_idx)
#define ENUM_VALUE_1(start_idx, x1)          x1 = (start_idx),
#define ENUM_VALUE_2(start_idx, x1, x2)      x1 = (start_idx), x2 = ((start_idx) + 1),
#define ENUM_VALUE_3(start_idx, x1, x2, x3)  x1 = (start_idx), x2 = ((start_idx) + 1), x3 = ((start_idx) + 2),
#define ENUM_VALUE_4(start_idx, x1, x2, x3, x4) \
    x1 = (start_idx), x2 = ((start_idx) + 1), x3 = ((start_idx) + 2), x4 = ((start_idx) + 3),
#define ENUM_VALUE_5(start_idx, x1, x2, x3, x4, x5) \
    ENUM_VALUE_4(start_idx, x1, x2, x3, x4) x5 = ((start_idx) + 4),
#define ENUM_VALUE_6(start_idx, x1, x2, x3, x4, x5, x6) \
    ENUM_VALUE_5(start_idx, x1, x2, x3, x4, x5) x6 = ((start_idx) + 5),
#define ENUM_VALUE_7(start_idx, x1, x2, x3, x4, x5, x6, x7) \
    ENUM_VALUE_6(start_idx, x1, x2, x3, x4, x5, x6) x7 = ((start_idx) + 6),
#define ENUM_VALUE_8(start_idx, x1, x2, x3, x4, x5, x6, x7, x8) \
    ENUM_VALUE_7(start_idx, x1, x2, x3, x4, x5, x6, x7) x8 = ((start_idx) + 7),
#define ENUM_VALUE_9(start_idx, x1, x2, x3, x4, x5, x6, x7, x8, x9) \
    ENUM_VALUE_8(start_idx, x1, x2, x3, x4, x5, x6, x7, x8) x9 = ((start_idx) + 8),
#define ENUM_VALUE_10(start_idx, x1, x2, x3, x4, x5, x6, x7, x8, x9, x10) \
    ENUM_VALUE_9(start_idx, x1, x2, x3, x4, x5, x6, x7, x8, x9) x10 = ((start_idx) + 9),
#define ENUM_VALUE_11(start_idx, x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11) \
    ENUM_VALUE_10(start_idx, x1, x2, x3, x4, x5, x6, x7, x8, x9, x10) x11 = ((start_idx) + 10),
#define ENUM_VALUE_12(start_idx, x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12) \
    ENUM_VALUE_11(start_idx, x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11) x12 = ((start_idx) + 11),
#define ENUM_VALUE_13(start_idx, x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12, x13) \
    ENUM_VALUE_12(start_idx, x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12) x13 = ((start_idx) + 12),

// String conversion cases
#define ENUM_STRING_0(start_idx)
#define ENUM_STRING_1(start_idx, x1)          case (start_idx): return #x1;
#define ENUM_STRING_2(start_idx, x1, x2)      ENUM_STRING_1(start_idx, x1) case ((start_idx) + 1): return #x2;
#define ENUM_STRING_3(start_idx, x1, x2, x3)  ENUM_STRING_2(start_idx, x1, x2) case ((start_idx) + 2): return #x3;
#define ENUM_STRING_4(start_idx, x1, x2, x3, x4) \
    ENUM_STRING_3(start_idx, x1, x2, x3) case ((start_idx) + 3): return #x4;
#define ENUM_STRING_5(start_idx, x1, x2, x3, x4, x5) \
    ENUM_STRING_4(start_idx, x1, x2, x3, x4) case ((start_idx) + 4): return #x5;
#define ENUM_STRING_6(start_idx, x1, x2, x3, x4, x5, x6) \
    ENUM_STRING_5(start_idx, x1, x2, x3, x4, x5) case ((start_idx) + 5): return #x6;
#define ENUM_STRING_7(start_idx, x1, x2, x3, x4, x5, x6, x7) \
    ENUM_STRING_6(start_idx, x1, x2, x3, x4, x5, x6) case ((start_idx) + 6): return #x7;
#define ENUM_STRING_8(start_idx, x1, x2, x3, x4, x5, x6, x7, x8) \
    ENUM_STRING_7(start_idx, x1, x2, x3, x4, x5, x6, x7) case ((start_idx) + 7): return #x8;
#define ENUM_STRING_9(start_idx, x1, x2, x3, x4, x5, x6, x7, x8, x9) \
    ENUM_STRING_8(start_idx, x1, x2, x3, x4, x5, x6, x7, x8) case ((start_idx) + 8): return #x9;
#define ENUM_STRING_10(start_idx, x1, x2, x3, x4, x5, x6, x7, x8, x9, x10) \
    ENUM_STRING_9(start_idx, x1, x2, x3, x4, x5, x6, x7, x8, x9) case ((start_idx) + 9): return #x10;
#define ENUM_STRING_11(start_idx, x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11) \
    ENUM_STRING_10(start_idx, x1, x2, x3, x4, x5, x6, x7, x8, x9, x10) case ((start_idx) + 10): return #x11;
#define ENUM_STRING_12(start_idx, x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12) \
    ENUM_STRING_11(start_idx, x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11) case ((start_idx) + 11): return #x12;
#define ENUM_STRING_13(start_idx, x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12, x13) \
    ENUM_STRING_12(start_idx, x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12) case ((start_idx) + 12): return #x13;

// Helper macros for macro selection
#define _ENUM_VALUES_(N, start_idx, ...) ENUM_VALUE_##N(start_idx, __VA_ARGS__)
#define _ENUM_VALUES(N, start_idx, ...) _ENUM_VALUES_(N, start_idx, __VA_ARGS__)
#define ENUM_VALUES_GLOMAP(start_idx, ...) _ENUM_VALUES(NUM_ARGS(__VA_ARGS__), start_idx, __VA_ARGS__)

#define _ENUM_STRINGS_(N, start_idx, ...) ENUM_STRING_##N(start_idx, __VA_ARGS__)
#define _ENUM_STRINGS(N, start_idx, ...) _ENUM_STRINGS_(N, start_idx, __VA_ARGS__)
#define ENUM_STRINGS_GLOMAP(start_idx, ...) _ENUM_STRINGS(NUM_ARGS(__VA_ARGS__), start_idx, __VA_ARGS__)

// Define the enum to string conversion function
#define DEFINE_ENUM_TO_STRING_GLOMAP(name, start_idx, ...) \
    static std::string_view name##ToString(int value) { \
        switch (value) { \
            ENUM_STRINGS_GLOMAP(start_idx, __VA_ARGS__) \
            default: \
                LOG(ERROR) << "Invalid input value: " << value; \
                return "None"; \
        } \
    } \
    static std::string_view name##ToString(name value) { \
        return name##ToString(static_cast<int>(value)); \
    }

// Main macros for enum creation
#define MAKE_ENUM_GLOMAP(name, start_idx, ...) \
    enum name { ENUM_VALUES_GLOMAP(start_idx, __VA_ARGS__) }; \
    DEFINE_ENUM_TO_STRING_GLOMAP(name, start_idx, __VA_ARGS__)

#define MAKE_ENUM_CLASS_GLOMAP(name, start_idx, ...) \
    enum class name { ENUM_VALUES_GLOMAP(start_idx, __VA_ARGS__) }; \
    DEFINE_ENUM_TO_STRING_GLOMAP(name, start_idx, __VA_ARGS__)

#define MAKE_ENUM_CLASS_OVERLOAD_STREAM_GLOMAP(name, start_idx, ...) \
    MAKE_ENUM_CLASS_GLOMAP(name, start_idx, __VA_ARGS__); \
    inline std::ostream& operator<<(std::ostream& os, name value) { \
        return os << name##ToString(value); \
    }

}  // namespace glomap

#endif  // GLOMAP_ENUM_TO_STRING_H