/*
 * Copyright (c) 2022 Sung Ho Park and CSOS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ubinos.h>

#if (UBINOS__UBIDRV__INCLUDE_UART == 1)
#if (UBINOS__BSP__BOARD_MODEL == UBINOS__BSP__BOARD_MODEL__NUCLEOF207ZG)

#if (INCLUDE__UBINOS__UBIK != 1)
    #error "ubik is necessary"
#endif

#include <ubinos/ubidrv/uart.h>
#include <ubinos/bsp/arch.h>

#include <assert.h>
#include <string.h>

#include "main.h"

#define UBIDEV_UART_IO_OPTION__NONE    0x0000
#define UBIDEV_UART_IO_OPTION__TIMED   0x0001
#define UBIDEV_UART_IO_OPTION__BLOCKED 0x0002

#define UBIDRV_UART_FILE_NUM            1
#define UBIDRV_UART_CHECK_INTERVAL_MS   1000
#define UBIDRV_UART_READ_BUFFER_SIZE    (512)
#define UBIDRV_UART_WRITE_BUFFER_SIZE   (1024 * 10)

static const char * _g_ubidrv_uart_file_names[UBIDRV_UART_FILE_NUM] =
{
    "/dev/tty1",
};

static USART_TypeDef * _g_ubidrv_uart_file_instance[UBIDRV_UART_FILE_NUM] =
{
    UBIDRV_UART_UART1,
};

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

static ubidrv_uart_file_t _g_ubidrv_uart_files[UBIDRV_UART_FILE_NUM];

UART_HandleTypeDef _g_ubidrv_uart_uart1_handle[UBIDRV_UART_FILE_NUM];

static void _ubidrv_uart_reset(int fd);
static ubi_err_t _ubidrv_uart_init(int fd);
static ubi_err_t _ubidrv_uart_getc_advan(int fd, char *ch_p, uint16_t io_option, uint32_t timeoutms, uint32_t *remain_timeoutms);

static void _ubidrv_uart_reset(int fd)
{
    HAL_StatusTypeDef stm_err;
    (void) stm_err;

    ubi_assert(0 < fd && fd <= UBIDRV_UART_FILE_NUM);
    ubidrv_uart_file_t * file = &_g_ubidrv_uart_files[fd - 1];
    ubi_assert(file->init == 1);

    mutex_lock(file->reset_lock);

    if (file->need_reset)
    {
        stm_err = HAL_UART_DeInit(file->hal_uart);
        ubi_assert(stm_err == HAL_OK);

        file->need_reset = 0;
        file->need_tx_restart = 1;
        file->need_rx_restart = 1;

        stm_err = HAL_UART_Init(file->hal_uart);
        ubi_assert(stm_err == HAL_OK);

        HAL_NVIC_SetPriority(UBIDRV_UART_UART1_IRQn, NVIC_PRIO_MIDDLE, 0);

        file->reset_count++;
    }

    mutex_unlock(file->reset_lock);
}

static ubi_err_t _ubidrv_uart_init(int fd)
{
    int r;
    ubi_err_t ubi_err;
    uint8_t * buf;
    uint16_t len;

    ubi_assert(0 < fd && fd <= UBIDRV_UART_FILE_NUM);
    ubidrv_uart_file_t * file = &_g_ubidrv_uart_files[fd - 1];

    do
    {
        if (bsp_isintr() || 0 != _bsp_critcount)
        {
            ubi_err = UBI_ERR_INVALID_STATE;
            break;
        }

        if (!_bsp_kernel_active)
        {
            ubi_err = UBI_ERR_INVALID_STATE;
            break;
        }

        if (file->init || file->in_init)
        {
            ubi_err = UBI_ERR_INVALID_STATE;
            break;
        }

        file->in_init = 1;

        r = cbuf_create(&file->read_cbuf, UBIDRV_UART_READ_BUFFER_SIZE);
        ubi_assert(r == 0);
        r = cbuf_create(&file->write_cbuf, UBIDRV_UART_WRITE_BUFFER_SIZE);
        ubi_assert(r == 0);
        r = semb_create(&file->read_sem);
        ubi_assert(r == 0);
        r = semb_create(&file->write_sem);
        ubi_assert(r == 0);
        r = mutex_create(&file->reset_lock);
        ubi_assert(r == 0);
        r = mutex_create(&file->put_lock);
        ubi_assert(r == 0);
        r = mutex_create(&file->get_lock);
        ubi_assert(r == 0);

        file->echo = 0;
        file->autocr = 0;

        file->rx_overflow_count = 0;
        file->tx_overflow_count = 0;
        file->need_reset = 1;

        file->init = 1;

        _ubidrv_uart_reset(fd);

        file->reset_count = 0;

        cbuf_clear(file->read_cbuf);

        buf = cbuf_get_tail_addr(file->read_cbuf);
        len = 1;
        file->need_rx_restart = 0;
        if (HAL_UART_Receive_IT(file->hal_uart, buf, len) != HAL_OK)
        {
            file->need_rx_restart = 1;
        }

        file->in_init = 0;

        ubi_err = UBI_ERR_OK;
        break;
    } while (1);

    return ubi_err;
}

static ubi_err_t _ubidrv_uart_getc_advan(int fd, char *ch_p, uint16_t io_option, uint32_t timeoutms, uint32_t *remain_timeoutms)
{
    int r;
    ubi_err_t ubi_err;
    uint8_t * buf;
    uint16_t len;
    uint32_t _remain_timeoutms = timeoutms;

    ubi_assert(0 < fd && fd <= UBIDRV_UART_FILE_NUM);
    ubidrv_uart_file_t * file = &_g_ubidrv_uart_files[fd - 1];
    ubi_assert(file->init == 1);

    do
    {
        if (bsp_isintr() || 0 != _bsp_critcount)
        {
            ubi_err = UBI_ERR_INVALID_STATE;
            break;
        }

        if (!_bsp_kernel_active)
        {
            ubi_err = UBI_ERR_INVALID_STATE;
            break;
        }

        if (!file->init)
        {
            ubi_err = UBI_ERR_INIT;
            break;
        }

        switch (io_option)
        {
        case UBIDEV_UART_IO_OPTION__NONE:
            r = mutex_lock_timedms(file->get_lock, 0);
            break;
        case UBIDEV_UART_IO_OPTION__TIMED:
            r = mutex_lock_timedms(file->get_lock, timeoutms);
            if (NULL != remain_timeoutms)
            {
                *remain_timeoutms = task_getremainingtimeoutms();
            }
            break;
        case UBIDEV_UART_IO_OPTION__BLOCKED:
            r = mutex_lock(file->get_lock);
            break;
        }

        if (r != 0)
        {
            ubi_err = UBI_ERR_BUSY;
            break;
        }

        for (;;)
        {
            if (file->need_reset)
            {
                _ubidrv_uart_reset(fd);
            }

            if (file->need_rx_restart)
            {
                len = 1;

                buf = cbuf_get_tail_addr(file->read_cbuf);
                file->need_rx_restart = 0;
                if (HAL_UART_Receive_IT(file->hal_uart, buf, len) != HAL_OK)
                {
                    file->need_rx_restart = 1;
                }
            }

            ubi_err = cbuf_read(file->read_cbuf, (uint8_t*) ch_p, 1, NULL);
            if (ubi_err == UBI_ERR_OK)
            {
                break;
            }
            else
            {
                switch (io_option)
                {
                case UBIDEV_UART_IO_OPTION__NONE:
                    ubi_err = UBI_ERR_BUF_EMPTY;
                    break;
                case UBIDEV_UART_IO_OPTION__TIMED:
                    if (_remain_timeoutms <= 0)
                    {
                        ubi_err = UBI_ERR_TIMEOUT;
                        break;
                    }
                    sem_take_timedms(file->read_sem, min(_remain_timeoutms, UBIDRV_UART_CHECK_INTERVAL_MS));
                    _remain_timeoutms = task_getremainingtimeoutms();
                    if (NULL != remain_timeoutms)
                    {
                        *remain_timeoutms = _remain_timeoutms;
                    }
                    ubi_err = UBI_ERR_OK;
                    break;
                case UBIDEV_UART_IO_OPTION__BLOCKED:
                    sem_take_timedms(file->read_sem, UBIDRV_UART_CHECK_INTERVAL_MS);
                    ubi_err = UBI_ERR_OK;
                    break;
                }
                if (ubi_err != UBI_ERR_OK)
                {
                    break;
                }
            }
        }

        if (0 == r && 0 != file->echo)
        {
            ubidrv_uart_putc(fd, *ch_p);
        }

        mutex_unlock(file->get_lock);

        break;
    } while (1);

    return ubi_err;
}

void ubidrv_uart_rx_callback(int fd)
{
    uint8_t *buf;
    uint16_t len;
    int need_signal = 0;

    ubi_assert(0 < fd && fd <= UBIDRV_UART_FILE_NUM);
    ubidrv_uart_file_t * file = &_g_ubidrv_uart_files[fd - 1];
    ubi_assert(file->init == 1);

    do
    {
        if (file->hal_uart->ErrorCode != HAL_UART_ERROR_NONE)
        {
            break;
        }

        if (file->need_reset)
        {
            break;
        }

        if (file->need_rx_restart)
        {
            bsp_abortsystem();
        }

        len = 1;

        if (cbuf_is_full(file->read_cbuf))
        {
            file->rx_overflow_count++;
        }
        else
        {
            if (cbuf_get_len(file->read_cbuf) == 0)
            {
                need_signal = 1;
            }

            cbuf_write(file->read_cbuf, NULL, len, NULL);

            if (need_signal && _bsp_kernel_active)
            {
                sem_give(file->read_sem);
            }
        }

        buf = cbuf_get_tail_addr(file->read_cbuf);
        file->need_rx_restart = 0;
        if (HAL_UART_Receive_IT(file->hal_uart, buf, len) != HAL_OK)
        {
            file->need_rx_restart = 1;
            break;
        }
    } while (0);
}

void ubidrv_uart_tx_callback(int fd)
{
    uint8_t *buf;
    uint16_t len;

    ubi_assert(0 < fd && fd <= UBIDRV_UART_FILE_NUM);
    ubidrv_uart_file_t * file = &_g_ubidrv_uart_files[fd - 1];
    ubi_assert(file->init == 1);

    do
    {
        if (file->hal_uart->ErrorCode != HAL_UART_ERROR_NONE)
        {
            break;
        }

        if (file->need_reset)
        {
            break;
        }

        if (file->need_tx_restart)
        {
            bsp_abortsystem();
        }

        len = 1;

        cbuf_read(file->write_cbuf, NULL, len, NULL);

        if (cbuf_get_len(file->write_cbuf) == 0)
        {
            if (_bsp_kernel_active)
            {
                sem_give(file->write_sem);
            }
            file->need_tx_restart = 1;
            break;
        }

        buf = cbuf_get_head_addr(file->write_cbuf);
        file->need_tx_restart = 0;
        if (HAL_UART_Transmit_IT(file->hal_uart, buf, len) != HAL_OK)
        {
            file->need_tx_restart = 1;
            break;
        }
    } while (0);
}

void ubidrv_uart_err_callback(int fd)
{
    ubi_assert(0 < fd && fd <= UBIDRV_UART_FILE_NUM);
    ubidrv_uart_file_t * file = &_g_ubidrv_uart_files[fd - 1];
    ubi_assert(file->init == 1);

    file->need_reset = 1;
}


ubi_err_t ubidrv_uart_open(ubidrv_uart_t * uart)
{
    ubi_err_t ubi_err;
    ubidrv_uart_file_t * file = NULL;

    ubi_assert(uart != NULL);

    do
    {
        for (int i = 0; i < UBIDRV_UART_FILE_NUM; i++)
        {
            if (0 == strncmp(_g_ubidrv_uart_file_names[i], uart->file_name, UBIDRV_UART_FILE_NAME_MAX))
            {
                file = &_g_ubidrv_uart_files[i];
                file->hal_uart = &_g_ubidrv_uart_uart1_handle[i];
                uart->fd = i + 1;
                break;
            }
        }
        if (file == NULL)
        {
            ubi_err = UBI_ERR_NOT_FOUND;
            break;
        }

        file->hal_uart->Instance = _g_ubidrv_uart_file_instance[uart->fd - 1];
        file->hal_uart->Init.BaudRate = uart->baud_rate;
        switch (uart->data_bits)
        {
            case UBIDRV_UART_DATA_BITS_9:
                file->hal_uart->Init.WordLength = UART_WORDLENGTH_9B;
                break;
            default:
            case UBIDRV_UART_DATA_BITS_8:
                file->hal_uart->Init.WordLength = UART_WORDLENGTH_8B;
                break;
        }
        switch (uart->stop_bits)
        {
            case UBIDRV_UART_STOP_BITS_1:
                file->hal_uart->Init.StopBits = UART_STOPBITS_1;
                break;
            default:
            case UBIDRV_UART_STOP_BITS_2:
                file->hal_uart->Init.StopBits = UART_STOPBITS_2;
                break;
        }
        switch (uart->parity_type)
        {
            case UBIDRV_UART_PARITY_TYPE_EVEN:
                file->hal_uart->Init.Parity = UART_PARITY_EVEN;
                break;
            case UBIDRV_UART_PARITY_TYPE_ODD:
                file->hal_uart->Init.Parity = UART_PARITY_ODD;
                break;
            default:
            case UBIDRV_UART_PARITY_TYPE_NONE:
                file->hal_uart->Init.Parity = UART_PARITY_NONE;
                break;
        }
        switch (uart->hw_flow_ctl)
        {
            case UBIDRV_UART_HW_FLOW_CTRL_CTS:
                file->hal_uart->Init.HwFlowCtl = UART_HWCONTROL_CTS;
                break;
            case UBIDRV_UART_HW_FLOW_CTRL_RTS:
                file->hal_uart->Init.HwFlowCtl = UART_HWCONTROL_RTS;
                break;
            case UBIDRV_UART_HW_FLOW_CTRL_RTS_CTS:
                file->hal_uart->Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
                break;
            default:
            case UBIDRV_UART_HW_FLOW_CTRL_NONE:
                file->hal_uart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
                break;
        }
        file->hal_uart->Init.Mode = UART_MODE_TX_RX;
        file->hal_uart->Init.OverSampling = UART_OVERSAMPLING_16;

        ubi_err = _ubidrv_uart_init(uart->fd);

        break;
    } while (1);

    return ubi_err;
}

ubi_err_t ubidrv_uart_close(ubidrv_uart_t * uart)
{
    return UBI_ERR_NOT_SUPPORTED;
}

ubi_err_t ubidrv_uart_getc(int fd, char *ch_p)
{
    return _ubidrv_uart_getc_advan(fd, ch_p, UBIDEV_UART_IO_OPTION__BLOCKED, 0, NULL);
}

ubi_err_t ubidrv_uart_getc_unblocked(int fd, char *ch_p)
{
    return _ubidrv_uart_getc_advan(fd, ch_p, UBIDEV_UART_IO_OPTION__NONE, 0, NULL);
}

ubi_err_t ubidrv_uart_getc_timedms(int fd, char *ch_p, uint32_t timeoutms, uint32_t *remain_timeoutms)
{
    return _ubidrv_uart_getc_advan(fd, ch_p, UBIDEV_UART_IO_OPTION__TIMED, timeoutms, remain_timeoutms);
}

ubi_err_t ubidrv_uart_putc(int fd, int ch)
{
    ubi_err_t ubi_err;
    uint8_t * buf;
    uint16_t len;
    uint32_t written;
    uint8_t data[2];

    ubi_assert(0 < fd && fd <= UBIDRV_UART_FILE_NUM);
    ubidrv_uart_file_t * file = &_g_ubidrv_uart_files[fd - 1];
    ubi_assert(file->init == 1);

    do
    {
        if (bsp_isintr() || 0 != _bsp_critcount)
        {
            ubi_err = UBI_ERR_INVALID_STATE;
            break;
        }

        if (!_bsp_kernel_active)
        {
            ubi_err = UBI_ERR_INVALID_STATE;
            break;
        }

        if (!file->init)
        {
            ubi_err = UBI_ERR_INIT;
            break;
        }

        mutex_lock(file->put_lock);

        do
        {
            if (file->need_reset)
            {
                _ubidrv_uart_reset(fd);
            }

            if (0 != file->autocr && '\n' == ch)
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

            if (cbuf_get_len(file->write_cbuf) == 0)
            {
                sem_clear(file->write_sem);
                file->need_tx_restart = 1;
            }

            cbuf_write(file->write_cbuf, data, len, &written);
            if (written == 0)
            {
                file->tx_overflow_count++;
            }

            if (file->need_tx_restart)
            {
                len = 1;
                buf = cbuf_get_head_addr(file->write_cbuf);
                file->need_tx_restart = 0;
                if (HAL_UART_Transmit_IT(file->hal_uart, buf, len) != HAL_OK)
                {
                    file->need_tx_restart = 1;
                    break;
                }
            }

            ubi_err = UBI_ERR_OK;
            break;
        } while (1);

        mutex_unlock(file->put_lock);

        break;
    } while (1);

    return ubi_err;
}

ubi_err_t ubidrv_uart_flush(int fd)
{
    ubi_err_t ubi_err;
    uint8_t * buf;
    uint16_t len;

    ubi_assert(0 < fd && fd <= UBIDRV_UART_FILE_NUM);
    ubidrv_uart_file_t * file = &_g_ubidrv_uart_files[fd - 1];
    ubi_assert(file->init == 1);

    do
    {
        if (bsp_isintr() || 0 != _bsp_critcount)
        {
            ubi_err = UBI_ERR_INVALID_STATE;
            break;
        }

        if (!_bsp_kernel_active)
        {
            ubi_err = UBI_ERR_INVALID_STATE;
            break;
        }

        if (!file->init)
        {
            ubi_err = UBI_ERR_INIT;
            break;
        }

        mutex_lock(file->put_lock);

        for (;;)
        {
            if (file->need_reset)
            {
                _ubidrv_uart_reset(fd);
            }

            if (file->need_tx_restart && cbuf_get_len(file->write_cbuf) > 0)
            {
                len = 1;
                buf = cbuf_get_head_addr(file->write_cbuf);
                file->need_tx_restart = 0;
                if (HAL_UART_Transmit_IT(file->hal_uart, buf, len) != HAL_OK)
                {
                    file->need_tx_restart = 1;
                    ubi_err = UBI_ERR_BUSY;
                    break;
                }
            }

            if (cbuf_get_len(file->write_cbuf) == 0)
            {
                ubi_err = UBI_ERR_OK;
                break;
            }

            sem_take_timedms(file->write_sem, UBIDRV_UART_CHECK_INTERVAL_MS);
        }

        mutex_unlock(file->put_lock);

        break;
    } while (1);

    return ubi_err;
}


int ubidrv_uart_putn(int fd, const char *str, int len)
{
    int r;

    ubi_assert(0 < fd && fd <= UBIDRV_UART_FILE_NUM);
    ubidrv_uart_file_t * file = &_g_ubidrv_uart_files[fd - 1];
    ubi_assert(file->init == 1);

    r = -1;
    do
    {
        if (bsp_isintr() || 0 != _bsp_critcount)
        {
            break;
        }

        if (!_bsp_kernel_active)
        {
            break;
        }

        if (!file->init)
        {
            break;
        }

        if (NULL == str)
        {
            r = -2;
            break;
        }

        if (0 > len)
        {
            r = -3;
            break;
        }

        for (r = 0; r < len; r++)
        {
            ubidrv_uart_putc(fd, *str);
            str++;
        }

        break;
    } while (1);

    return r;
}

int ubidrv_uart_kbhit(int fd)
{
    int r;

    ubi_assert(0 < fd && fd <= UBIDRV_UART_FILE_NUM);
    ubidrv_uart_file_t * file = &_g_ubidrv_uart_files[fd - 1];
    ubi_assert(file->init == 1);

    r = -1;
    do
    {
        if (bsp_isintr() || 0 != _bsp_critcount)
        {
            break;
        }

        if (!_bsp_kernel_active)
        {
            break;
        }

        if (!file->init)
        {
            break;
        }

        if (cbuf_get_len(file->read_cbuf) != 0)
        {
            r = 1;
        }
        else
        {
            r = 0;
        }

        break;
    } while (1);

    return r;
}

int ubidrv_uart_puts(int fd, const char *str, int max)
{
    int i;
    ubi_err_t ubi_err;

    if (NULL == str)
    {
        return -2;
    }

    if (0 > max)
    {
        return -3;
    }

    for (i = 0; i < max; i++)
    {
        if ('\0' == *str)
        {
            break;
        }
        ubi_err = ubidrv_uart_putc(fd, *str);
        if (ubi_err != UBI_ERR_OK)
        {
            break;
        }
        str++;
    }

    return i;
}

int ubidrv_uart_gets(int fd, char *str, int max)
{
    int i;
    ubi_err_t ubi_err;

    ubi_assert(0 < fd && fd <= UBIDRV_UART_FILE_NUM);
    ubidrv_uart_file_t * file = &_g_ubidrv_uart_files[fd - 1];
    ubi_assert(file->init == 1);

    if (NULL == str)
    {
        return -2;
    }

    if (0 > max)
    {
        return -3;
    }

    for (i = 0; i < max; i++)
    {
        ubi_err = ubidrv_uart_getc(fd, &str[i]);
        if (UBI_ERR_OK != ubi_err || '\0' == str[i] || '\n' == str[i] || '\r' == str[i])
        {
            break;
        }
    }
    if (0 != i && max == i)
    {
        i--;
    }
    str[i] = '\0';

    return i;
}

ubi_err_t ubidrv_uart_setecho(int fd, int echo)
{
    ubi_assert(0 < fd && fd <= UBIDRV_UART_FILE_NUM);
    ubidrv_uart_file_t * file = &_g_ubidrv_uart_files[fd - 1];
    ubi_assert(file->init == 1);

    file->echo = echo;

    return UBI_ERR_OK;
}

int ubidrv_uart_getecho(int fd)
{
    ubi_assert(0 < fd && fd <= UBIDRV_UART_FILE_NUM);
    ubidrv_uart_file_t * file = &_g_ubidrv_uart_files[fd - 1];
    ubi_assert(file->init == 1);

    return file->echo;
}

ubi_err_t ubidrv_uart_setautocr(int fd, int autocr)
{
    ubi_assert(0 < fd && fd <= UBIDRV_UART_FILE_NUM);
    ubidrv_uart_file_t * file = &_g_ubidrv_uart_files[fd - 1];
    ubi_assert(file->init == 1);

    file->autocr = autocr;

    return UBI_ERR_OK;
}

int ubidrv_uart_getautocr(int fd)
{
    ubi_assert(0 < fd && fd <= UBIDRV_UART_FILE_NUM);
    ubidrv_uart_file_t * file = &_g_ubidrv_uart_files[fd - 1];
    ubi_assert(file->init == 1);

    return file->autocr;
}

#endif /* (UBINOS__BSP__BOARD_MODEL == UBINOS__BSP__BOARD_MODEL__NUCLEOF207ZG) */
#endif /* (UBINOS__UBIDRV__INCLUDE_UART == 1) */

