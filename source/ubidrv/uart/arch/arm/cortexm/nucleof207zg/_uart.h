/*
 * Copyright (c) 2022 Sung Ho Park and CSOS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _UART_H_
#define _UART_H_

#ifdef __cplusplus
extern "C"
{
#endif

#define UBIDEV_UART_IO_OPTION__NONE    0x0000
#define UBIDEV_UART_IO_OPTION__TIMED   0x0001
#define UBIDEV_UART_IO_OPTION__BLOCKED 0x0002

#define UBIDRV_UART_FILE_NUM            1
#define UBIDRV_UART_CHECK_INTERVAL_MS   1000
#define UBIDRV_UART_READ_BUFFER_SIZE    (512)
#define UBIDRV_UART_WRITE_BUFFER_SIZE   (1024 * 10)

typedef struct _ubidrv_uart_file_t
{
    unsigned int  init :1;
    unsigned int  in_init :1;
    unsigned int  echo :1;
    unsigned int  autocr :1;

    unsigned int  need_reset :1;
    unsigned int  need_rx_restart :1;
    unsigned int  need_tx_restart :1;

    cbuf_pt read_cbuf;
    cbuf_pt write_cbuf;

    sem_pt read_sem;
    sem_pt write_sem;

    mutex_pt put_lock;
    mutex_pt get_lock;
    mutex_pt reset_lock;

    unsigned int  rx_overflow_count;
    unsigned int  tx_overflow_count;
    unsigned int  reset_count;

    UART_HandleTypeDef * hal_uart;
} ubidrv_uart_file_t;

extern ubidrv_uart_file_t _g_ubidrv_uart_files[UBIDRV_UART_FILE_NUM];

#ifdef __cplusplus
}
#endif

#endif /* _UART_H_ */
