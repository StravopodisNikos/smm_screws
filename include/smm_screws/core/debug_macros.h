#ifndef DEBUG_MACROS_H
#define DEBUG_MACROS_H

#include <cstdio>    // std::printf
#include <iostream>  // std::cout

#define DEBUG_PRINTF(cond, fmt, ...)                    \
    do {                                                \
        if (cond) {                                     \
            std::printf(fmt, ##__VA_ARGS__);            \
            std::printf("\n");                          \
        }                                               \
    } while (0)

#define DEBUG_STREAM(cond, expr)                        \
    do {                                                \
        if (cond) {                                     \
            std::cout << expr << std::endl;             \
        }                                               \
    } while (0)

#endif // DEBUG_MACROS_H