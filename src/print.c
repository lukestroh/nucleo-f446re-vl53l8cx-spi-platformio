#include "print.h"


void print_serial(UART_HandleTypeDef* huart2, char* _msg_buf, size_t _msg_buf_size, const char* data, ...) {
  va_list args;
  va_start(args, data);
  memset(_msg_buf, 0, _msg_buf_size);
  vsnprintf((char*)_msg_buf, _msg_buf_size, data, args);
  HAL_UART_Transmit(huart2, (uint8_t*)_msg_buf, _msg_buf_size, 100);
  va_end(args);
}