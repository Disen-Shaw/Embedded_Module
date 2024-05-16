
/**
 *******************************************************************************
 * @file    generic_fifo.c
 * @brief   Generic FIFO Function
 * @author  Disen-Shaw <DisenShaw@gmail.com>
 * @date    2023-09-10
 * @version v1.0.0
 *******************************************************************************
 */

#include "generic_fifo.h"
#include "generic_log.h"

void generic_fifo_show(struct generic_fifo *fifo) 
{
	LOG("fifo in_idx\t\t%d"		,fifo->in_idx);
	LOG("fifo out_idx\t\t%d"	,fifo->out_idx);
	LOG("fifo size\t\t%d"		,fifo->size);
	LOG("fifo buffer addr\t%p" 	,fifo->buffer);
	LOG("fifo attribute\t%d"	,fifo->user_buffer_flag);
}



