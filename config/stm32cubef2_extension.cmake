set(INCLUDE__STM32CUBEF2_EXTENSION TRUE)
set(PROJECT_UBINOS_LIBRARIES ${PROJECT_UBINOS_LIBRARIES} stm32cubef2_extension)

set_cache_default(STM32CUBEF2__DTTY_STM32_UART_ENABLE FALSE BOOL "")

set_cache_default(STM32CUBEF2__DTTY_STM32_UART_READ_BUFFER_SIZE "512" STRING "stm32cubef2 dtty uart read buffer size")
set_cache_default(STM32CUBEF2__DTTY_STM32_UART_WRITE_BUFFER_SIZE "1024 * 10" STRING "stm32cubef2 dtty uart read buffer size")

