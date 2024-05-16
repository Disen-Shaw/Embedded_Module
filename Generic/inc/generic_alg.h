
#ifndef __GENERIC_ALG_H__
#define __GENERIC_ALG_H__

#include <stdint.h>

uint16_t generic_alg_crc_check(const void *buffer, uint32_t length);
uint16_t generic_alg_trimmed_mean_filter(uint16_t *array, uint32_t count, uint32_t cut_button, uint32_t cut_top);
void generic_alg_sort(uint16_t *array, uint32_t count);

#endif /* __GENERIC_ALG_H__ */

