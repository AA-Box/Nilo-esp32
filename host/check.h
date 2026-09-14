// A test framework in forty lines, because the alternative is a dependency.
//
// The firmware has no test framework and adding one would mean a package manager, a
// version to pin, and a second way to build this code. `CHECK` prints the file, the line
// and the expression, which is the whole of what a failing assertion has to do.

#pragma once

#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace check {

inline int& Failures() {
    static int failures = 0;
    return failures;
}

inline void Fail(const char* file, int line, const char* expression, const char* detail) {
    std::fprintf(stderr, "FAIL %s:%d  %s%s%s\n", file, line, expression, detail ? "  -- " : "",
                 detail ? detail : "");
    Failures()++;
}

inline int Report(const char* suite) {
    if (Failures() == 0) {
        std::printf("ok   %s\n", suite);
        return 0;
    }
    std::printf("FAIL %s (%d failure%s)\n", suite, Failures(), Failures() == 1 ? "" : "s");
    return 1;
}

}  // namespace check

#define CHECK(expression)                                            \
    do {                                                             \
        if (!(expression)) {                                         \
            ::check::Fail(__FILE__, __LINE__, #expression, nullptr); \
        }                                                            \
    } while (0)

#define CHECK_EQ(actual, expected)                                               \
    do {                                                                         \
        const auto actual_value = (actual);                                      \
        const auto expected_value = (expected);                                  \
        if (!(actual_value == expected_value)) {                                 \
            char detail[160];                                                    \
            std::snprintf(detail, sizeof(detail), "got %lld, wanted %lld",       \
                          static_cast<long long>(actual_value),                  \
                          static_cast<long long>(expected_value));               \
            ::check::Fail(__FILE__, __LINE__, #actual " == " #expected, detail); \
        }                                                                        \
    } while (0)

#define CHECK_NEAR(actual, expected, tolerance)                                                   \
    do {                                                                                          \
        const long long actual_value = static_cast<long long>(actual);                            \
        const long long expected_value = static_cast<long long>(expected);                        \
        const long long allowed = static_cast<long long>(tolerance);                              \
        const long long difference = actual_value > expected_value                                \
                                         ? actual_value - expected_value                          \
                                         : expected_value - actual_value;                         \
        if (difference > allowed) {                                                               \
            char detail[160];                                                                     \
            std::snprintf(detail, sizeof(detail), "got %lld, wanted %lld +/- %lld", actual_value, \
                          expected_value, allowed);                                               \
            ::check::Fail(__FILE__, __LINE__, #actual " ~= " #expected, detail);                  \
        }                                                                                         \
    } while (0)

#define CHECK_STREQ(actual, expected)                                              \
    do {                                                                           \
        if (std::strcmp((actual), (expected)) != 0) {                              \
            ::check::Fail(__FILE__, __LINE__, #actual " == " #expected, (actual)); \
        }                                                                          \
    } while (0)
