
#ifndef __GENERIC_LOG_H__
#define __GENERIC_LOG_H__

#include <stdint.h>

/**
 * @brief log level defination
 */
#define GENERIC_LOG_LEVEL_NONE 0
#define GENERIC_LOG_LEVEL_ERROR 1
#define GENERIC_LOG_LEVEL_WARNING 2
#define GENERIC_LOG_LEVEL_INFO 3
#define GENERIC_LOG_LEVEL_DEBUG 4

/**
 * @brief config log function
 */
#define GENERIC_LOG_CONFIG_LEVEL GENERIC_LOG_LEVEL_NONE
#define GENERIC_LOG_CONFIG_COLOR 0
#define GENERIC_LOG_CONFIG_TICK 0

/**
 * @brief color defination
 */
#if GENERIC_LOG_CONFIG_COLOR
#define __GENERIC_LOG_COLOR_BLACK "30"
#define __GENERIC_LOG_COLOR_RED "31"
#define __GENERIC_LOG_COLOR_GREEN "32"
#define __GENERIC_LOG_COLOR_YELLOW "33"
#define __GENERIC_LOG_COLOR_BLUE "34"
#define __GENERIC_LOG_COLOR_MAGENTA "35"
#define __GENERIC_LOG_COLOR_CYAN "36"
#define __GENERIC_LOG_COLOR(COLOR) "\033[0;" COLOR "m"
#define __GENERIC_LOG_BOLD(COLOR) "\033[1;" COLOR "m"
#define __GENERIC_LOG_UNDERLINE (COLOR) "\033[4;" COLOR "m"
#define __GENERIC_LOG_BLINK(COLOR) "\033[1;" COLOR "m"
#define __GENERIC_LOG_COLOR_END "\033[0m"
#define __GENERIC_LOG_COLOR_E __GENERIC_LOG_COLOR(__GENERIC_LOG_COLOR_RED)
#define __GENERIC_LOG_COLOR_W __GENERIC_LOG_COLOR(__GENERIC_LOG_COLOR_YELLOW)
#define __GENERIC_LOG_COLOR_I __GENERIC_LOG_COLOR(__GENERIC_LOG_COLOR_CYAN)
#define __GENERIC_LOG_COLOR_D __GENERIC_LOG_COLOR(__GENERIC_LOG_COLOR_GREEN)
#define __GENERIC_LOG_COLOR_V __GENERIC_LOG_COLOR(__GENERIC_LOG_COLOR_BLUE)
#else // GENERIC_LOG_CONFIG_COLOR
#define __GENERIC_LOG_COLOR_E
#define __GENERIC_LOG_COLOR_W
#define __GENERIC_LOG_COLOR_I
#define __GENERIC_LOG_COLOR_D
#define __GENERIC_LOG_COLOR_V
#define __GENERIC_LOG_COLOR_END
#endif //! GENERIC_LOG_CONFIG_COLOR

#if GENERIC_LOG_CONFIG_LEVEL
#if GENERIC_LOG_CONFIG_TICK

#define __GENERIC_LOG_FMT(T, F) __GENERIC_LOG_COLOR_##T "[ " #T " ] (%u): " F __GENERIC_LOG_COLOR_END "\r\n"

#define generic_log_error(fmt, ...) generic_log_printf(GENERIC_LOG_LEVEL_ERROR, __GENERIC_LOG_FMT(E, fmt), generic_log_getTick(), ##__VA_ARGS__)
#define generic_log_warning(fmt, ...) generic_log_printf(GENERIC_LOG_LEVEL_WARNING, __GENERIC_LOG_FMT(W, fmt), generic_log_getTick(), ##__VA_ARGS__)
#define generic_log_info(fmt, ...) generic_log_printf(GENERIC_LOG_LEVEL_INFO, __GENERIC_LOG_FMT(I, fmt), generic_log_getTick(), ##__VA_ARGS__)
#define generic_log_debug(fmt, ...) generic_log_printf(GENERIC_LOG_LEVEL_DEBUG, __GENERIC_LOG_FMT(D, fmt), generic_log_getTick(), ##__VA_ARGS__)
#else // GENERIC_LOG_CONFIG_COLOR

#define __GENERIC_LOG_FMT(T, F) __GENERIC_LOG_COLOR_##T "[ " #T " ]: " F __GENERIC_LOG_COLOR_END "\r\n"

#define generic_log_error(fmt, ...) generic_log_printf(GENERIC_LOG_LEVEL_ERROR, __GENERIC_LOG_FMT(E, fmt), ##__VA_ARGS__)
#define generic_log_warning(fmt, ...) generic_log_printf(GENERIC_LOG_LEVEL_WARNING, __GENERIC_LOG_FMT(W, fmt), ##__VA_ARGS__)
#define generic_log_info(fmt, ...) generic_log_printf(GENERIC_LOG_LEVEL_INFO, __GENERIC_LOG_FMT(I, fmt), ##__VA_ARGS__)
#define generic_log_debug(fmt, ...) generic_log_printf(GENERIC_LOG_LEVEL_DEBUG, __GENERIC_LOG_FMT(D, fmt), ##__VA_ARGS__)

#endif //! GENERIC_LOG_CONFIG_COLOR
#else
// Do nothing
#define generic_log_error(fmt, ...)
#define generic_log_warning(fmt, ...)
#define generic_log_info(fmt, ...)
#define generic_log_debug(fmt, ...)
#endif //! GENERIC_LOG_CONFIG_LEVEL

int generic_log_printf(uint8_t level, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

#if GENERIC_LOG_CONFIG_TICK
uint32_t generic_log_getTick(void);
#endif //! GENERIC_LOG_CONFIG_TICK

#endif //! __GENERIC_LOG_H__
