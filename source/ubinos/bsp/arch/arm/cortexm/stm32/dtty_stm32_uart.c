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

#define DTTY_UART_CHECK_INTERVAL_MS 1000

cbuf_def_init(_g_dtty_uart_rbuf, DTTY_UART_READ_BUFFER_SIZE);
cbuf_def_init(_g_dtty_uart_wbuf, DTTY_UART_WRITE_BUFFER_SIZE);

sem_pt _g_dtty_uart_rsem = NULL;
sem_pt _g_dtty_uart_wsem = NULL;

mutex_pt _g_dtty_uart_putlock = NULL;
mutex_pt _g_dtty_uart_getlock = NULL;
mutex_pt _g_dtty_uart_resetlock = NULL;

uint32_t _g_dtty_uart_rx_overflow_count = 0;
uint32_t _g_dtty_uart_tx_overflow_count = 0;
uint32_t _g_dtty_uart_reset_count = 0;

uint8_t _g_dtty_uart_need_reset = 0;
uint8_t _g_dtty_uart_need_rx_restart = 0;
uint8_t _g_dtty_uart_need_tx_restart = 0;

static void dtty_stm32_uart_reset(void);

static void dtty_stm32_uart_reset(void)
{
    HAL_StatusTypeDef stm_err;

    mutex_lock(_g_dtty_uart_resetlock);

    if (_g_dtty_uart_need_reset)
    {
        DTTY_STM32_UART_HANDLE.Instance = DTTY_STM32_UART;
        DTTY_STM32_UART_HANDLE.Init.BaudRate = 115200;
        DTTY_STM32_UART_HANDLE.Init.WordLength = UART_WORDLENGTH_8B;
        DTTY_STM32_UART_HANDLE.Init.StopBits = UART_STOPBITS_1;
        DTTY_STM32_UART_HANDLE.Init.Parity = UART_PARITY_NONE;
        DTTY_STM32_UART_HANDLE.Init.HwFlowCtl = UART_HWCONTROL_NONE;
        DTTY_STM32_UART_HANDLE.Init.Mode = UART_MODE_TX_RX;
        DTTY_STM32_UART_HANDLE.Init.OverSampling = UART_OVERSAMPLING_16;

        stm_err = HAL_UART_DeInit(&DTTY_STM32_UART_HANDLE);
        assert(stm_err == HAL_OK);

        _g_dtty_uart_need_reset = 0;
        _g_dtty_uart_need_tx_restart = 1;
        _g_dtty_uart_need_rx_restart = 1;

        stm_err = HAL_UART_Init(&DTTY_STM32_UART_HANDLE);
        assert(stm_err == HAL_OK);

        HAL_NVIC_SetPriority(DTTY_STM32_UART_IRQn, NVIC_PRIO_MIDDLE, 0);

        _g_dtty_uart_reset_count++;
    }

    mutex_unlock(_g_dtty_uart_resetlock);
}

void dtty_stm32_uart_rx_callback(void)
{
    uint8_t *buf;
    uint16_t len;
    cbuf_pt rbuf = _g_dtty_uart_rbuf;
    sem_pt rsem = _g_dtty_uart_rsem;
    int need_signal = 0;

    do
    {
        if (DTTY_STM32_UART_HANDLE.ErrorCode != HAL_UART_ERROR_NONE)
        {
            break;
        }

        if (_g_dtty_uart_need_reset)
        {
            break;
        }

        if (_g_dtty_uart_need_rx_restart)
        {
            bsp_abortsystem();
        }

        len = 1;

        if (cbuf_is_full(rbuf))
        {
            _g_dtty_uart_rx_overflow_count++;
        }
        else
        {
            if (cbuf_get_len(rbuf) == 0)
            {
                need_signal = 1;
            }

            cbuf_write(rbuf, NULL, len, NULL);

            if (need_signal && _bsp_kernel_active)
            {
                sem_give(rsem);
            }
        }

        buf = cbuf_get_tail_addr(rbuf);
        _g_dtty_uart_need_rx_restart = 0;
        if (HAL_UART_Receive_IT(&DTTY_STM32_UART_HANDLE, buf, len) != HAL_OK)
        {
            _g_dtty_uart_need_rx_restart = 1;
            break;
        }
    } while (0);
}

void dtty_stm32_uart_tx_callback(void)
{
    uint8_t *buf;
    uint16_t len;
    cbuf_pt wbuf = _g_dtty_uart_wbuf;
    sem_pt wsem = _g_dtty_uart_wsem;

    do
    {
        if (DTTY_STM32_UART_HANDLE.ErrorCode != HAL_UART_ERROR_NONE)
        {
            break;
        }

        if (_g_dtty_uart_need_reset)
        {
            break;
        }

        if (_g_dtty_uart_need_tx_restart)
        {
            bsp_abortsystem();
        }

        len = 1;

        cbuf_read(wbuf, NULL, len, NULL);

        if (cbuf_get_len(wbuf) == 0)
        {
            if (_bsp_kernel_active)
            {
                sem_give(wsem);
            }
            _g_dtty_uart_need_tx_restart = 1;
            break;
        }

        buf = cbuf_get_head_addr(wbuf);
        _g_dtty_uart_need_tx_restart = 0;
        if (HAL_UART_Transmit_IT(&DTTY_STM32_UART_HANDLE, buf, len) != HAL_OK)
        {
            _g_dtty_uart_need_tx_restart = 1;
            break;
        }
    } while (0);
}

void dtty_stm32_uart_err_callback(void)
{
    _g_dtty_uart_need_reset = 1;
}

int dtty_init(void)
{
    int r;
    uint8_t * buf;
    uint16_t len;

    if (!_g_bsp_dtty_init && !bsp_isintr() && _bsp_kernel_active)
    {
        r = semb_create(&_g_dtty_uart_rsem);
        assert(r == 0);
        r = semb_create(&_g_dtty_uart_wsem);
        assert(r == 0);
        r = mutex_create(&_g_dtty_uart_resetlock);
        assert(r == 0);
        r = mutex_create(&_g_dtty_uart_putlock);
        assert(r == 0);
        r = mutex_create(&_g_dtty_uart_getlock);
        assert(r == 0);

        _g_bsp_dtty_echo = 1;
        _g_bsp_dtty_autocr = 1;

        _g_dtty_uart_rx_overflow_count = 0;
        _g_dtty_uart_tx_overflow_count = 0;
        _g_dtty_uart_need_reset = 1;

        dtty_stm32_uart_reset();

        _g_dtty_uart_reset_count = 0;

        _g_bsp_dtty_init = 1;

        do
        {
            if (cbuf_is_full(_g_dtty_uart_rbuf))
            {
                _g_dtty_uart_rx_overflow_count++;
                _g_dtty_uart_need_rx_restart = 1;
                break;
            }

            len = 1;

            buf = cbuf_get_tail_addr(_g_dtty_uart_rbuf);
            _g_dtty_uart_need_rx_restart = 0;
            if (HAL_UART_Receive_IT(&DTTY_STM32_UART_HANDLE, buf, len) != HAL_OK)
            {
                _g_dtty_uart_need_rx_restart = 1;
                break;
            }
        } while(0);
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
    int r;
    ubi_err_t ubi_err;
    uint8_t * buf;
    uint16_t len;

    r = -1;
    if (!bsp_isintr())
    {
        mutex_lock(_g_dtty_uart_getlock);
        do
        {
            if (!_g_bsp_dtty_init)
            {
                dtty_init();
                if (!_g_bsp_dtty_init)
                {
                    break;
                }
            }

            for (;;)
            {
                if (_g_dtty_uart_need_reset)
                {
                    dtty_stm32_uart_reset();
                }

                if (_g_dtty_uart_need_rx_restart)
                {
                    len = 1;

                    buf = cbuf_get_tail_addr(_g_dtty_uart_rbuf);
                    _g_dtty_uart_need_rx_restart = 0;
                    if (HAL_UART_Receive_IT(&DTTY_STM32_UART_HANDLE, buf, len) != HAL_OK)
                    {
                        _g_dtty_uart_need_rx_restart = 1;
                    }
                }

                ubi_err = cbuf_read(_g_dtty_uart_rbuf, (uint8_t*) ch_p, 1, NULL);
                if (ubi_err == UBI_ERR_OK)
                {
                    break;
                }

                sem_take_timedms(_g_dtty_uart_rsem, DTTY_UART_CHECK_INTERVAL_MS);
            }

            if (0 != _g_bsp_dtty_echo)
            {
                dtty_putc(*ch_p);
            }

            r = 0;
        } while (0);
        mutex_unlock(_g_dtty_uart_getlock);
    }

    return r;
}

int dtty_putc(int ch)
{
    int r;
    uint8_t * buf;
    uint16_t len;
    uint32_t written;
    uint8_t data[2];

    r = -1;
    if (!bsp_isintr())
    {
        mutex_lock(_g_dtty_uart_putlock);
        do
        {
            if (!_g_bsp_dtty_init)
            {
                dtty_init();
                if (!_g_bsp_dtty_init)
                {
                    break;
                }
            }

            if (_g_dtty_uart_need_reset)
            {
                dtty_stm32_uart_reset();
            }

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

            if (cbuf_get_len(_g_dtty_uart_wbuf) == 0)
            {
                sem_clear(_g_dtty_uart_wsem);
                _g_dtty_uart_need_tx_restart = 1;
            }

            cbuf_write(_g_dtty_uart_wbuf, data, len, &written);
            if (written == 0)
            {
                _g_dtty_uart_tx_overflow_count++;
            }

            if (_g_dtty_uart_need_tx_restart)
            {
                len = 1;

                buf = cbuf_get_head_addr(_g_dtty_uart_wbuf);
                _g_dtty_uart_need_tx_restart = 0;
                if (HAL_UART_Transmit_IT(&DTTY_STM32_UART_HANDLE, buf, len) != HAL_OK)
                {
                    _g_dtty_uart_need_tx_restart = 1;
                    break;
                }
            }

            r = 0;
        } while (0);
        mutex_unlock(_g_dtty_uart_putlock);
    }

    return r;
}

int dtty_flush(void)
{
    int r;
    uint8_t * buf;
    uint16_t len;

    r = -1;
    if (!bsp_isintr())
    {
        mutex_lock(_g_dtty_uart_putlock);
        for (;;)
        {
            if (_g_dtty_uart_need_reset)
            {
                dtty_stm32_uart_reset();
            }

            if (_g_dtty_uart_need_tx_restart && cbuf_get_len(_g_dtty_uart_wbuf) > 0)
            {
                len = 1;

                buf = cbuf_get_head_addr(_g_dtty_uart_wbuf);
                _g_dtty_uart_need_tx_restart = 0;
                if (HAL_UART_Transmit_IT(&DTTY_STM32_UART_HANDLE, buf, len) != HAL_OK)
                {
                    _g_dtty_uart_need_tx_restart = 1;
                    break;
                }
            }

            if (cbuf_get_len(_g_dtty_uart_wbuf) == 0)
            {
                r = 0;
                break;
            }

            sem_take_timedms(_g_dtty_uart_wsem, DTTY_UART_CHECK_INTERVAL_MS);
        }
        mutex_unlock(_g_dtty_uart_putlock);
    }

    return r;
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

