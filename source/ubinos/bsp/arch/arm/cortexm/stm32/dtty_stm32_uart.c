/*
 * Copyright (c) 2021 Sung Ho Park and CSOS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ubinos.h>

#if (INCLUDE__UBINOS__BSP == 1)

#if (UBINOS__BSP__USE_DTTY == 1)

#if (UBINOS__BSP__DTTY_TYPE == UBINOS__BSP__DTTY_TYPE__EXTERNAL)

#if (STM32CUBEF2__DTTY_STM32_UART_ENABLE == 1)

#if (INCLUDE__UBINOS__UBIK != 1)
    #error "ubik is necessary"
#endif

#include <ubinos/bsp.h>
#include <ubinos/bsp_ubik.h>

#include <assert.h>

#include "main.h"

#define SLEEP_TIMEMS	1

extern int _g_bsp_dtty_init;
extern int _g_bsp_dtty_echo;
extern int _g_bsp_dtty_autocr;

#define DTTY_UART_READ_BUFFER_SIZE (512)
#define DTTY_UART_WRITE_BUFFER_SIZE (1024 * 10)

cbuf_def_init(_g_dtty_uart_rbuf, DTTY_UART_READ_BUFFER_SIZE);
cbuf_def_init(_g_dtty_uart_wbuf, DTTY_UART_WRITE_BUFFER_SIZE);

uint32_t _g_dtty_uart_rx_overflow_count = 0;
uint8_t _g_dtty_uart_tx_busy = 0;

sem_pt _g_dtty_uart_rsem = NULL;
sem_pt _g_dtty_uart_wsem = NULL;

void dtty_stm32_uart_rx_callback(void)
{
    int need_signal = 0;
    uint8_t *buf;
    uint32_t len;
    cbuf_pt rbuf = _g_dtty_uart_rbuf;
    sem_pt rsem = _g_dtty_uart_rsem;

    do
    {
        len = 1;

        if (cbuf_is_full(rbuf))
        {
            _g_dtty_uart_rx_overflow_count++;
            break;
        }

        if (cbuf_get_len(rbuf) == 0)
        {
            need_signal = 1;
        }

        cbuf_write(rbuf, NULL, len, NULL);

        if (need_signal && _bsp_kernel_active)
        {
            sem_give(rsem);
        }

        buf = cbuf_get_tail_addr(rbuf);
        HAL_UART_Receive_IT(&DTTY_STM32_UART_HANDLE, buf, len);
    } while (0);
}

void dtty_stm32_uart_tx_callback(void)
{
    uint8_t *buf;
    uint32_t len;
    cbuf_pt wbuf = _g_dtty_uart_wbuf;
    sem_pt wsem = _g_dtty_uart_wsem;

    do
    {
        len = 1;

        cbuf_read(wbuf, NULL, len, NULL);

        if (cbuf_get_len(wbuf) > 0)
        {
            buf = cbuf_get_head_addr(wbuf);
            len = 1;
            HAL_UART_Transmit_IT(&DTTY_STM32_UART_HANDLE, buf, len);
        }
        else
        {
            _g_dtty_uart_tx_busy = 0;
            sem_give(wsem);
        }
    } while (0);
}

int dtty_init(void)
{
    int r;
    HAL_StatusTypeDef stm_err;
    uint8_t * buf;
    uint32_t len;
    (void) r;
    (void) stm_err;

    if (!_g_bsp_dtty_init && !bsp_isintr() && _bsp_kernel_active)
    {
        r = semb_create(&_g_dtty_uart_rsem);
        assert(r == 0);
        r = semb_create(&_g_dtty_uart_wsem);
        assert(r == 0);

        _g_bsp_dtty_echo = 1;
        _g_bsp_dtty_autocr = 1;

        DTTY_STM32_UART_HANDLE.Instance = DTTY_STM32_UART;
        DTTY_STM32_UART_HANDLE.Init.BaudRate = 115200;
        DTTY_STM32_UART_HANDLE.Init.WordLength = UART_WORDLENGTH_8B;
        DTTY_STM32_UART_HANDLE.Init.StopBits = UART_STOPBITS_1;
        DTTY_STM32_UART_HANDLE.Init.Parity = UART_PARITY_NONE;
        DTTY_STM32_UART_HANDLE.Init.HwFlowCtl = UART_HWCONTROL_NONE;
        DTTY_STM32_UART_HANDLE.Init.Mode = UART_MODE_TX_RX;
        DTTY_STM32_UART_HANDLE.Init.OverSampling = UART_OVERSAMPLING_16;
        stm_err = HAL_UART_Init(&DTTY_STM32_UART_HANDLE);
        assert(stm_err == HAL_OK);

        _g_bsp_dtty_init = 1;

        if (!cbuf_is_full(_g_dtty_uart_rbuf))
        {
            buf = cbuf_get_tail_addr(_g_dtty_uart_rbuf);
            len = 1;
            HAL_UART_Receive_IT(&DTTY_STM32_UART_HANDLE, buf, len);
        }
    }

    return 0;
}

int dtty_enable(void)
{
    return 0;
}

int dtty_disable(void)
{
    return 0;
}

int dtty_geterror(void)
{
    return 0;
}

int dtty_getc(char *ch_p)
{
    ubi_err_t ubi_err;

    if (!_g_bsp_dtty_init)
    {
        dtty_init();
    }

    if (_g_bsp_dtty_init && !bsp_isintr())
    {
        for (;;)
        {
            ubi_err = cbuf_read(_g_dtty_uart_rbuf, (uint8_t*) ch_p, 1, NULL);
            if (ubi_err == UBI_ERR_OK)
            {
                break;
            }
            else
            {
                sem_take(_g_dtty_uart_rsem);
            }
        }
    }

    if (0 != _g_bsp_dtty_echo)
    {
        dtty_putc(*ch_p);
    }

    return 0;
}

int dtty_putc(int ch)
{
    uint32_t written;
    uint8_t * buf;
    size_t len;
    uint8_t data[2];
    int ret;

    if (!_g_bsp_dtty_init)
    {
        dtty_init();
    }

    ret = 0;
    do
    {
        if (0 != _g_bsp_dtty_autocr && '\n' == ch)
        {
            data[0] = '\r';
            data[1] = '\n';
            len = 2;
        }
        else
        {
            data[0] = (uint8_t) ch;
            len = 1;
        }

        cbuf_write(_g_dtty_uart_wbuf, data, len, &written);
        if (written == 0)
        {
            break;
        }
        ret = 1;

        if (_g_bsp_dtty_init && !bsp_isintr())
        {
            if (!_g_dtty_uart_tx_busy)
            {
                buf = cbuf_get_head_addr(_g_dtty_uart_wbuf);
                len = 1;
                _g_dtty_uart_tx_busy = 1;
                for (uint32_t i = 0;; i++)
                {
                    if (HAL_UART_Transmit_IT(&DTTY_STM32_UART_HANDLE, buf, len) == HAL_OK)
                    {
                        break;
                    }
                    if (i >= 99)
                    {
                        break;
                    }
                }
            }

        }
    } while (0);

    return ret;
}

int dtty_putn(const char *str, int len)
{
    int i = 0;

    if (!_g_bsp_dtty_init)
    {
        dtty_init();
    }

    if (_g_bsp_dtty_init && !bsp_isintr())
    {
        if (NULL == str)
        {
            return -2;
        }

        if (0 > len)
        {
            return -3;
        }

        for (i = 0; i < len; i++)
        {
            dtty_putc(*str);
            str++;
        }
    }
    return i;
}

int dtty_kbhit(void)
{
    int r = 0;

    if (!_g_bsp_dtty_init)
    {
        dtty_init();
    }

    if (_g_bsp_dtty_init && !bsp_isintr())
    {
        if (cbuf_get_len(_g_dtty_uart_rbuf) != 0)
        {
            r = 1;
        }
        else
        {
            r = 0;
        }
    }

    return r;
}

#endif /* (STM32CUBEF2__DTTY_STM32_UART_ENABLE == 1) */

#endif /* (UBINOS__BSP__DTTY_TYPE == UBINOS__BSP__DTTY_TYPE__EXTERNAL) */

#endif /* (UBINOS__BSP__USE_DTTY == 1) */

#endif /* (INCLUDE__UBINOS__BSP == 1) */

