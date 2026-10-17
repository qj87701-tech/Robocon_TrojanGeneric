#ifndef TESTS_HOST_TEST_ASSERT_H
#define TESTS_HOST_TEST_ASSERT_H

#include <cstdio>
#include <cstdlib>

#undef assert
#define assert(condition)                                                          \
    do                                                                             \
    {                                                                              \
        if (!(condition))                                                          \
        {                                                                          \
            std::fprintf(stderr, "Assertion failed: %s (%s:%d)\n", #condition,   \
                         __FILE__, __LINE__);                                       \
            std::exit(EXIT_FAILURE);                                               \
        }                                                                          \
    } while (false)

#endif
