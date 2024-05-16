
/**
 *******************************************************************************
 * @file    generic_fifo.h
 * @brief   Generic FIFO Function Header
 * @author  Disen-Shaw <DisenShaw@gmail.com>
 * @date    2023-09-10
 * @version v1.0.0
 *******************************************************************************
 */

#ifndef __GLOBAL_FIFO_H__
#define __GLOBAL_FIFO_H__

#include <string.h>
#include <stdint.h>

#include "generic_log.h"

/**
 * @brief Generic FIFO
 *
 * @note
 *   user_buffer_flag:
 *      \ref 0 Normal Buffer in Stack memory
 *      \ref 1 Heap memory requested by malloc
 */
struct generic_fifo {
    void *buffer;
    uint32_t in_idx;
    uint32_t out_idx;
    uint32_t size;
    uint8_t user_buffer_flag;
};

/**
 *  @defgroup Generic_FIFO
 *  @brief Generic FIFO's Functions
 *  @{
 */

/**
 * @brief Init the Generic FIFO
 *
 * @param[inout] _fifo address of fifo you wanna init
 * @param[in] _buffer address of fifo's buffer
 * @param[in] _flag user_buffer_flag
 *                  \ref 0 Normal Buffer in Stack memory
 *              \ref 1 Heap memory requested by malloc
 * @param[in] _size max capacity of fifo
 *
 * @retval none
 */
#define generic_fifo_init(_fifo, _buffer, _flag, _size)                        \
    (void)({                                                                   \
        struct generic_fifo *__fifo = _fifo;                                   \
        __fifo->buffer = _buffer;                                              \
        __fifo->size = _size;                                                  \
        __fifo->in_idx = 0;                                                    \
        __fifo->out_idx = 0;                                                   \
        __fifo->user_buffer_flag = _flag;                                      \
    })

/**
 * @brief deinit the fifo
 *
 * @param[inout] _fifo address of fifo you wanna deinit
 *
 * @retval none
 */
#define generic_fifo_deinit(_fifo)                                             \
    (void)({                                                                   \
        struct generic_fifo *__fifo = _fifo;                                   \
        if ((__fifo->buffer != NULL) && (__fifo->user_buffer_flag == 1)) {     \
            free(__fifo->buffer);                                              \
            __fifo->buffer = NULL;                                             \
        }                                                                      \
        __fifo->buffer = NULL;                                                 \
        __fifo->in_idx = 0;                                                    \
        __fifo->out_idx = 0;                                                   \
    })

/**
 * @brief Determine whether the fifo is full
 *
 * @param[in] _fifo fifo's address
 *
 * @retval
 *      \ref 0 not full
 *      \ref 1 fifo is full
 */
#define generic_fifo_full(_fifo)                                               \
    (uint8_t)({                                                                \
        struct generic_fifo *__fifo = _fifo;                                   \
        ((__fifo->in_idx + 1) % __fifo->size) == __fifo->out_idx;              \
    })

/**
 * @brief Determine whether the fifo is empty
 *
 * @param[in] _fifo fifo's address
 *
 * @retval
 *      \ref 0 not empty
 *      \ref 1 fifo is empty
 */
#define generic_fifo_empty(_fifo)                                              \
    (uint8_t)({                                                                \
        struct generic_fifo *__fifo = _fifo;                                   \
        (__fifo->in_idx == __fifo->out_idx);                                   \
    })

/**
 * @brief Get the number of vaild elements
 *
 * @param[in] _fifo fifo's address
 *
 * @retval number of vaild elements
 */
#define generic_fifo_valid_count(_fifo)                                        \
    (int)({                                                                    \
        struct generic_fifo *__fifo = _fifo;                                   \
        (__fifo->in_idx - __fifo->out_idx + __fifo->size) % __fifo->size;      \
    })

/**
 * @brief Query remaining space of fifo
 *
 * @param[in] _fifo fifo's address
 *
 * @retval remaining space
 */
#define generic_fifo_unused_count(_fifo)                                       \
    (int)({                                                                    \
        struct generic_fifo *__fifo = _fifo;                                   \
        __fifo->size -                                                         \
            ((__fifo->in_idx - __fifo->out_idx + __fifo->size) % __fifo->size) \
    })

/**
 * @brief Insert element into fifo
 *
 * @param[inout] _fifo fifo's address
 * @param[in]   _value address of element you wanna insert
 * @param[in]   _type type of eleement you wanna remove
 *
 * @retval
 *      \ref 0 insert failed, fifo is full
 *      \ref 1 insert successfully
 */
#define generic_fifo_insert(_fifo, _value, _type)                              \
    (int)({                                                                    \
        int __ret = !generic_fifo_full(_fifo);                                 \
        struct generic_fifo *__temp = _fifo;                                   \
        if (__ret) {                                                           \
            memcpy((_type *)__temp->buffer + __temp->in_idx, _value,           \
                   sizeof(_type));                                             \
            __temp->in_idx = (__temp->in_idx + 1) % __temp->size;              \
        }                                                                      \
        __ret;                                                                 \
    })

/**
 * @brief Remove element from fifo
 *
 * @param[inout] _fifo fifo's element
 * @param[out] _value address of element to output
 * @param[in] _type type of eleement you wanna remove
 *
 * @retval
 *      \ref 0 remove failed, fifo is empty
 *      \ref 1 remove successfully
 */
#define generic_fifo_remove(_fifo, _value, _type)                              \
    (int)({                                                                    \
        int __ret = !generic_fifo_empty(_fifo);                                \
        struct generic_fifo *__temp = _fifo;                                   \
        if (__ret) {                                                           \
            memcpy(_value, (_type *)__temp->buffer + __temp->out_idx,          \
                   sizeof(_type));                                             \
            __temp->out_idx = (__temp->out_idx + 1) % __temp->size;            \
        }                                                                      \
        __ret;                                                                 \
    })

/**
 * @brief Change value of fifo's element
 *
 * @param[inout] _fifo fifo's address
 * @param[in] _index index of vaild value
 * @param[in] _value value address you wanna change to
 * @param[in] _type type of eleement you wanna change
 *
 * @retval
 *      \ref 0 index over range
 *      \ref 1 change successfully
 */
#define generic_fifo_change(_fifo, _value, _index, _type)                      \
    (int)({                                                                    \
        int __ret = (generic_fifo_valid_count(_fifo) > _index);                \
        struct generic_fifo *__temp = _fifo;                                   \
        if (__ret) {                                                           \
            memcpy(((_type *)__temp->buffer +                                  \
                    ((__temp->out_idx + _index) % __temp->size)),              \
                   _value, sizeof(_type));                                     \
        }                                                                      \
        __ret;                                                                 \
    })

/**
 * @brief Find the specified element in fifo
 *
 * @param[in] _fifo fifo's address
 * @param[in] _value element address you wanna find
 * @param[in] _type type of eleement you wanna change
 *
 * @retval
 *      \ref -1 Element not found
 *      \ref n index of vaild value
 */
#define generic_fifo_select_value(_fifo, _value, _type)                        \
    (int)({                                                                    \
        struct generic_fifo *__temp = _fifo;                                   \
        int __ret = -1;                                                        \
        int __buffer_offset = __temp->out_idx;                                 \
        int __count = 0;                                                       \
        int __valid = generic_fifo_valid_count(_fifo);                         \
        if (!generic_fifo_empty(_fifo)) {                                      \
            while (__count < __valid) {                                        \
                if (memcmp(_value,                                             \
                           ((_type *)__temp->buffer + __buffer_offset),        \
                           sizeof(_type)) == 0) {                              \
                    __ret = __count;                                           \
                    break;                                                     \
                }                                                              \
                __count++;                                                     \
                __buffer_offset = (__temp->out_idx + __count) % __temp->size;  \
            }                                                                  \
        }                                                                      \
        __ret;                                                                 \
    })

/**
 * @brief Query elements at a specified index
 *
 * @param[in] _fifo fifo's address
 * @param[in] _index index of element
 * @param[out] _value address of element to output
 * @param[in] _type type of eleement you wanna change
 *
 * @retval
 *      \ref 0 index over range
 *      \ref 1 Query successfully
 */
#define generic_fifo_select_index(_fifo, _value, _index, _type)                \
    (int)({                                                                    \
        struct generic_fifo *__temp = _fifo;                                   \
        int __ret = 0;                                                         \
        if (generic_fifo_valid_count(_fifo) > _index) {                        \
            memcpy(_value, (_type *)__temp->buffer + __temp->out_idx + _index, \
                   sizeof(_type));                                             \
            __ret = 1;                                                         \
        }                                                                      \
        __ret;                                                                 \
    })

/**
 * @brief Reset fifo
 *
 * @param[in] _fifo fifo you wanna reset
 *
 * @retval none
 */
#define generic_fifo_reset(_fifo)                                              \
    (void)({                                                                   \
        struct generic_fifo *__fifo = _fifo;                                   \
        __fifo->in_idx = 0;                                                    \
        __fifo->out_idx = 0;                                                   \
    })

/**
 * @brief free the fifo's memory
 *
 * @param[in] _fifo fifo's address
 *
 * @retval none
 */
#define generic_fifo_free(_fifo)                                               \
    (void)({                                                                   \
        struct generic_fifo *__fifo = _fifo;                                   \
        if ((__fifo->buffer != NULL) && (__fifo->user_buffer_flag == 1)) {     \
            free(__fifo->buffer);                                              \
            __fifo->buffer = NULL;                                             \
        }                                                                      \
    })

/**
 *  @}
 */

void generic_fifo_show(struct generic_fifo *fifo);

#endif /* __GLOBAL_FIFO_H__ */
