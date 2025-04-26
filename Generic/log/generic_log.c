
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include "generic_log.h"

static int (*g_log_vprintf)(const char *, va_list) = vprintf;
static uint8_t g_log_level = GENERIC_LOG_CONFIG_LEVEL;

/**
 * @brief Print the log message
 *        default use vprintf as output interface
 *
 * @param level - log level
 * @param fmt - format string
 *
 * @retval ret
 */
int generic_log_printf(uint8_t level, const char *fmt, ...)
{
    int ret = 0;
    va_list list;
    va_start(list, fmt);
    if (fmt && level <= g_log_level)
    {
        ret = g_log_vprintf(fmt, list);
    }
    return ret;
}

/**
 * @brief tick api
 *        you can change to return your tick
 *        such as HAL_GetTick() in stm32 hal driver
 */
uint32_t generic_log_getTick(void)
{
    // return your tick
    return 0;
}
