/*
 * Copyright (c) 2022 Sung Ho Park and CSOS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ubinos.h>

#if (UBINOS__UBIDRV__INCLUDE_UART_IO == 1)
#if (UBINOS__BSP__BOARD_MODEL == UBINOS__BSP__BOARD_MODEL__NUCLEOF207ZG)

#if (INCLUDE__UBINOS__UBIK != 1)
    #error "ubik is necessary"
#endif

#include <ubinos/ubidrv/uart.h>
#include <ubinos/ubidrv/uart_io.h>
#include <ubinos/bsp/arch.h>

#include <assert.h>
#include <string.h>

#include "main.h"

#include "_uart.h"

#define UBIDRV_UART_IO_OPTION__TIMED 0x0001

static ubi_st_t ubidrv_uart_io_read_advan(int fd, uint8_t *buffer, uint32_t length, uint32_t *read, uint16_t io_option, uint32_t timeoutms, uint32_t *remain_timeoutms);
static ubi_st_t ubidrv_uart_io_write_advan(int fd, uint8_t *buffer, uint32_t length, uint32_t *written, uint16_t io_option, uint32_t timeoutms, uint32_t *remain_timeoutms);
static ubi_st_t ubidrv_uart_io_read_buf_clear_advan(int fd, uint16_t io_option, uint32_t timeoutms, uint32_t *remain_timeoutms);
static ubi_st_t ubidrv_uart_io_flush_advan(int fd, uint16_t io_option, uint32_t timeoutms, uint32_t *remain_timeoutms);

static ubi_st_t ubidrv_uart_io_read_advan(int fd, uint8_t *buffer, uint32_t length, uint32_t *read, uint16_t io_option, uint32_t timeoutms, uint32_t *remain_timeoutms)
{
    ubi_st_t ubi_err;
    int r;
    uint8_t * buf;
    uint32_t read_tmp;
    uint32_t read_tmp2;
    assert(buffer != NULL);
    (void) r;
    (void) ubi_err;

    ubi_assert(0 < fd && fd <= UBIDRV_UART_FILE_NUM);
    ubidrv_uart_file_t * uart_file = &_g_ubidrv_uart_files[fd - 1];
    ubi_assert(uart_file->init == 1);

    do
    {
        if ((io_option & UBIDRV_UART_IO_OPTION__TIMED) != 0)
        {
            r = mutex_lock_timedms(uart_file->get_lock, timeoutms);
            timeoutms = task_getremainingtimeoutms();
            if (r == UBIK_ERR__TIMEOUT)
            {
                ubi_err = UBI_ST_TIMEOUT;
                break;
            }
            assert(r == 0);
        }
        else
        {
            r = mutex_lock(uart_file->get_lock);
            assert(r == 0);
        }

        read_tmp = 0;
        read_tmp2 = 0;

        for (;;)
        {
            if (uart_file->need_rx_restart)
            {
                buf = cbuf_get_tail_addr(uart_file->read_cbuf);
                uart_file->need_rx_restart = 0;
                if (HAL_UART_Receive_IT(uart_file->hal_uart, buf, 1) != HAL_OK)
                {
                    uart_file->need_rx_restart = 1;
                }
            }

            ubi_err = cbuf_read(uart_file->read_cbuf, &buffer[read_tmp], length - read_tmp, &read_tmp2);
            assert(ubi_err == UBI_ST_OK || ubi_err == UBI_ST_ERR_BUF_EMPTY);
            read_tmp += read_tmp2;

            if (read_tmp >= length)
            {
                ubi_err = UBI_ST_OK;
                break;
            }
            else
            {
                if ((io_option & UBIDRV_UART_IO_OPTION__TIMED) != 0)
                {
                    if (timeoutms == 0)
                    {
                        ubi_err = UBI_ST_TIMEOUT;
                        break;
                    }
                    r = sem_take_timedms(uart_file->read_sem, timeoutms);
                    timeoutms = task_getremainingtimeoutms();
                    if (r == UBIK_ERR__TIMEOUT)
                    {
                        ubi_err = UBI_ST_TIMEOUT;
                        break;
                    }
                    assert(r == 0);
                }
                else
                {
                    r = sem_take(uart_file->read_sem);
                    assert(r == 0);
                }
            }
        }

        if (read)
        {
            *read = read_tmp;
        }

        if ((io_option & UBIDRV_UART_IO_OPTION__TIMED) != 0)
        {
            if (remain_timeoutms)
            {
                *remain_timeoutms = timeoutms;
            }
        }

        r = mutex_unlock(uart_file->get_lock);
        assert(r == 0);
    } while (0);

    return ubi_err;
}

static ubi_st_t ubidrv_uart_io_write_advan(int fd, uint8_t *buffer, uint32_t length, uint32_t *written, uint16_t io_option, uint32_t timeoutms, uint32_t *remain_timeoutms)
{
    ubi_st_t ubi_err;
    int r;
    uint8_t *buf;
    uint32_t written_tmp;
    assert(buffer != NULL);

    ubi_assert(0 < fd && fd <= UBIDRV_UART_FILE_NUM);
    ubidrv_uart_file_t * uart_file = &_g_ubidrv_uart_files[fd - 1];
    ubi_assert(uart_file->init == 1);

    do
    {
        if (length <= 0)
        {
            ubi_err = UBI_ST_OK;
            break;
        }

        if ((io_option & UBIDRV_UART_IO_OPTION__TIMED) != 0)
        {
            r = mutex_lock_timedms(uart_file->put_lock, timeoutms);
            timeoutms = task_getremainingtimeoutms();
            if (r == UBIK_ERR__TIMEOUT)
            {
                ubi_err = UBI_ST_TIMEOUT;
                break;
            }
            assert(r == 0);
        }
        else
        {
            r = mutex_lock(uart_file->put_lock);
            assert(r == 0);
        }

        if (cbuf_get_len(uart_file->write_cbuf) == 0)
        {
            sem_clear(uart_file->write_sem);
            uart_file->need_tx_restart = 1;
        }

        written_tmp = 0;

        ubi_err = cbuf_write(uart_file->write_cbuf, buffer, length, &written_tmp);
        assert(ubi_err == UBI_ST_OK || ubi_err == UBI_ST_ERR_BUF_FULL);
        if (written_tmp == 0)
        {
            uart_file->tx_overflow_count++;
        }
        else
        {
            if (uart_file->need_tx_restart)
            {
                buf = cbuf_get_head_addr(uart_file->write_cbuf);
                uart_file->need_tx_restart = 0;
                for (uint32_t i = 0;; i++)
                {
                    if (HAL_UART_Transmit_IT(uart_file->hal_uart, buf, 1) == HAL_OK)
                    {
                        break;
                    }
                    if (i >= 99)
                    {
                        uart_file->need_tx_restart = 1;
                        ubi_err = UBI_ST_ERR_IO;
                        break;
                    }
                }
            }
        }

        if (written)
        {
            *written = written_tmp;
        }

        if ((io_option & UBIDRV_UART_IO_OPTION__TIMED) != 0)
        {
            if (remain_timeoutms)
            {
                *remain_timeoutms = timeoutms;
            }
        }

        r = mutex_unlock(uart_file->put_lock);
        assert(r == 0);
    } while (0);

    return ubi_err;
}

static ubi_st_t ubidrv_uart_io_read_buf_clear_advan(int fd, uint16_t io_option, uint32_t timeoutms, uint32_t *remain_timeoutms)
{
    ubi_st_t ubi_err;
    int r;
    (void) r;
    (void) ubi_err;

    ubi_assert(0 < fd && fd <= UBIDRV_UART_FILE_NUM);
    ubidrv_uart_file_t * uart_file = &_g_ubidrv_uart_files[fd - 1];
    ubi_assert(uart_file->init == 1);

    do
    {
        if ((io_option & UBIDRV_UART_IO_OPTION__TIMED) != 0)
        {
            r = mutex_lock_timedms(uart_file->get_lock, timeoutms);
            timeoutms = task_getremainingtimeoutms();
            if (r == UBIK_ERR__TIMEOUT)
            {
                ubi_err = UBI_ST_TIMEOUT;
                break;
            }
            assert(r == 0);
        }
        else
        {
            r = mutex_lock(uart_file->get_lock);
            assert(r == 0);
        }

        ubi_err = cbuf_clear(uart_file->read_cbuf);
        assert(ubi_err == UBI_ST_OK);

        if ((io_option & UBIDRV_UART_IO_OPTION__TIMED) != 0)
        {
            if (remain_timeoutms)
            {
                *remain_timeoutms = timeoutms;
            }
        }

        r = mutex_unlock(uart_file->get_lock);
        assert(r == 0);
    } while (0);

    return ubi_err;
}

static ubi_st_t ubidrv_uart_io_flush_advan(int fd, uint16_t io_option, uint32_t timeoutms, uint32_t *remain_timeoutms)
{
    ubi_st_t ubi_err;
    int r;
    (void) r;
    (void) ubi_err;

    ubi_assert(0 < fd && fd <= UBIDRV_UART_FILE_NUM);
    ubidrv_uart_file_t * uart_file = &_g_ubidrv_uart_files[fd - 1];
    ubi_assert(uart_file->init == 1);

    do
    {
        ubi_err = UBI_ST_OK;

        if ((io_option & UBIDRV_UART_IO_OPTION__TIMED) != 0)
        {
            r = mutex_lock_timedms(uart_file->put_lock, timeoutms);
            timeoutms = task_getremainingtimeoutms();
            if (r == UBIK_ERR__TIMEOUT)
            {
                ubi_err = UBI_ST_TIMEOUT;
                break;
            }
            assert(r == 0);
        }
        else
        {
            r = mutex_lock(uart_file->put_lock);
            assert(r == 0);
        }

        for (;;)
        {
            if (cbuf_get_len(uart_file->write_cbuf) == 0)
            {
                break;
            }
            r = sem_take_timedms(uart_file->write_sem, timeoutms);
            timeoutms = task_getremainingtimeoutms();
            if (r == UBIK_ERR__TIMEOUT)
            {
                ubi_err = UBI_ST_TIMEOUT;
                break;
            }
        }

        if ((io_option & UBIDRV_UART_IO_OPTION__TIMED) != 0)
        {
            if (remain_timeoutms)
            {
                *remain_timeoutms = timeoutms;
            }
        }

        r = mutex_unlock(uart_file->put_lock);
        assert(r == 0);
    } while (0);

    return ubi_err;
}

ubi_st_t ubidrv_uart_io_read(int fd, uint8_t *buffer, uint32_t length, uint32_t *read)
{
    return ubidrv_uart_io_read_advan(fd, buffer, length, read, 0, 0, NULL);
}

ubi_st_t ubidrv_uart_io_read_timedms(int fd, uint8_t *buffer, uint32_t length, uint32_t *read, uint32_t timeoutms, uint32_t *remain_timeoutms)
{
    return ubidrv_uart_io_read_advan(fd, buffer, length, read, UBIDRV_UART_IO_OPTION__TIMED, timeoutms, remain_timeoutms);
}

ubi_st_t ubidrv_uart_io_write(int fd, uint8_t *buffer, uint32_t length, uint32_t *written)
{
    return ubidrv_uart_io_write_advan(fd, buffer, length, written, 0, 0, NULL);
}

ubi_st_t ubidrv_uart_io_write_timedms(int fd, uint8_t *buffer, uint32_t length, uint32_t *written, uint32_t timeoutms, uint32_t *remain_timeoutms)
{
    return ubidrv_uart_io_write_advan(fd, buffer, length, written, UBIDRV_UART_IO_OPTION__TIMED, timeoutms, remain_timeoutms);
}

ubi_st_t ubidrv_uart_io_read_buf_clear(int fd)
{
    return ubidrv_uart_io_read_buf_clear_advan(fd, 0, 0, NULL);
}

ubi_st_t ubidrv_uart_io_read_buf_clear_timedms(int fd, uint32_t timeoutms, uint32_t *remain_timeoutms)
{
    return ubidrv_uart_io_read_buf_clear_advan(fd, UBIDRV_UART_IO_OPTION__TIMED, timeoutms, remain_timeoutms);
}

ubi_st_t ubidrv_uart_io_flush(int fd)
{
    return ubidrv_uart_io_flush_advan(fd, 0, 0, NULL);
}

ubi_st_t ubidrv_uart_io_flush_timedms(int fd, uint32_t timeoutms, uint32_t *remain_timeoutms)
{
    return ubidrv_uart_io_flush_advan(fd, UBIDRV_UART_IO_OPTION__TIMED, timeoutms, remain_timeoutms);
}

#endif /* (UBINOS__BSP__BOARD_MODEL == UBINOS__BSP__BOARD_MODEL__NUCLEOF207ZG) */
#endif /* (UBINOS__UBIDRV__INCLUDE_UART_IO == 1) */

