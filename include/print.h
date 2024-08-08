#ifndef __SERIAL_PRINT_H__
#define __SERIAL_PRINT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"
#include <stdio.h>
#include <stdarg.h>

void print_serial(
  UART_HandleTypeDef* huart2,
  char* _msg_buf,
  size_t _msg_buf_size,
  const char* data,
  ...
);

#ifdef __cplusplus
}
#endif

#endif // __SERIAL_PRINT_H__