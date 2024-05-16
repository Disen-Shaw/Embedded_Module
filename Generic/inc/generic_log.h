/**
 *******************************************************************************
 * @file    generic_log.h
 * @brief   Log Definations
 * @author  Disen-Shaw <DisenShaw@gmail.com>
 * @date    2023-06-20
 * @version v1.0.2
 *******************************************************************************
 */

#ifndef __GENERIC_LOG_H__
#define __GENERIC_LOG_H__

/**< Header Files **/
#include "gd32f10x.h"
#include <stdio.h>

/**
 *  @defgroup Generic_Log
 *      @brief Log Function Defination
 *      @{
 */

#define LOG_DBG_TRIGGER (0U)

#define LOG(fmt, ...)                                                                                                  \
    (void)({                                                                                                           \
        printf("\t" fmt, ##__VA_ARGS__);                                                                               \
        printf("\r\n");                                                                                                \
    })

#define LOG_ERROR(fmt, ...)                                                                                            \
    (void)({                                                                                                           \
        printf("\t-------------------------------------------------------------------------\r\n");                     \
        printf("\tFILE:\t%s\r\n", __FILE__);                                                                           \
        printf("\tLINE:\t%d\r\n", __LINE__);                                                                           \
        printf("\tError\t" fmt "\r\n", ##__VA_ARGS__);                                                                 \
        printf("\t-------------------------------------------------------------------------\r\n");                     \
    })

#define LOG_LINE_LIGHT                                                                                                 \
    (void)({ printf("\t-------------------------------------------------------------------------\r\n"); })

#define LOG_LINE_MIDDLE                                                                                                \
    (void)({ printf("\t=========================================================================\r\n"); })

#define LOG_LINE_HEAVY                                                                                                 \
    (void)({ printf("\t#########################################################################\r\n"); })

#define LOG_U32_BIT_SHOW(X)                                                                                            \
    (void)({                                                                                                           \
        uint32_t __value = (uint32_t)X;                                                                                \
        printf("\t");                                                                                                  \
        for (int i = 0; i < 32; i++)                                                                                   \
        {                                                                                                              \
            if (__value >> (32 - 1 - i) & 0x01)                                                                        \
            {                                                                                                          \
                printf("%d", 1);                                                                                       \
            }                                                                                                          \
            else                                                                                                       \
            {                                                                                                          \
                printf("%d", 0);                                                                                       \
            }                                                                                                          \
            if ((i + 1) % 4 == 0)                                                                                      \
            {                                                                                                          \
                printf(" ");                                                                                           \
            }                                                                                                          \
        }                                                                                                              \
        printf("\r\n");                                                                                                \
    })

#define LOG_U16_BIT_SHOW(X)                                                                                            \
    (void)({                                                                                                           \
        uint16_t __value = (uint16_t)X;                                                                                \
        printf("\t");                                                                                                  \
        for (int i = 0; i < 16; i++)                                                                                   \
        {                                                                                                              \
            if (__value >> (16 - 1 - i) & 0x01)                                                                        \
            {                                                                                                          \
                printf("%d", 1);                                                                                       \
            }                                                                                                          \
            else                                                                                                       \
            {                                                                                                          \
                printf("%d", 0);                                                                                       \
            }                                                                                                          \
            if ((i + 1) % 4 == 0)                                                                                      \
            {                                                                                                          \
                printf(" ");                                                                                           \
            }                                                                                                          \
        }                                                                                                              \
        printf("\r\n");                                                                                                \
    })

#define LOG_U8_BIT_SHOW(X)                                                                                             \
    (void)({                                                                                                           \
        uint8_t __value = (uint8_t)X;                                                                                  \
        printf("\t");                                                                                                  \
        for (int i = 0; i < 8; i++)                                                                                    \
        {                                                                                                              \
            if (__value >> (8 - 1 - i) & 0x01)                                                                         \
            {                                                                                                          \
                printf("%d", 1);                                                                                       \
            }                                                                                                          \
            else                                                                                                       \
            {                                                                                                          \
                printf("%d", 0);                                                                                       \
            }                                                                                                          \
            if ((i + 1) % 4 == 0)                                                                                      \
            {                                                                                                          \
                printf(" ");                                                                                           \
            }                                                                                                          \
        }                                                                                                              \
        printf("\r\n");                                                                                                \
    })

#if (LOG_DBG_TRIGGER)

#define LOG_DBG(fmt, ...)                                                                                              \
    (void)({                                                                                                           \
        printf("\t" fmt, ##__VA_ARGS__);                                                                               \
        printf("\r\n");                                                                                                \
    })

#define LOG_DBG_ERROR(fmt, ...)                                                                                        \
    (void)({                                                                                                           \
        printf("\t-------------------------------------------------------------------------\r\n");                     \
        printf("\tError\t" fmt, ##__VA_ARGS__);                                                                        \
        printf("\r\n");                                                                                                \
        printf("\t-------------------------------------------------------------------------\r\n");                     \
    })

#define LOG_DBG_LINE_LIGHT                                                                                             \
    (void)({ printf("\t-------------------------------------------------------------------------\r\n"); })

#define LOG_DBG_LINE_MIDDLE                                                                                            \
    (void)({ printf("\t=========================================================================\r\n"); })

#define LOG_DBG_LINE_HEAVY                                                                                             \
    (void)({ printf("\t#########################################################################\r\n"); })
#else

#define LOG_DBG(fmt, ...) (void)({})
#define LOG_DBG_ERROR(fmt, ...) (void)({})
#define LOG_DBG_LINE_LIGHT (void)({})
#define LOG_DBG_LINE_MIDDLE (void)({})
#define LOG_DBG_LINE_HEAVY (void)({})
#endif

/**
 *  @}
 */

#endif /* __GLOBAL_LOG_H__ */
