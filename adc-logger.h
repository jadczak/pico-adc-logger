#include "hardware/uart.h"
#include "pico/stdlib.h"

absolute_time_t debug_get_time(void);
int64_t debug_time_diff(absolute_time_t start, absolute_time_t end);
void debug_uart_print(uart_inst_t *uart, const char *str);
bool sample_adc_callback(struct repeating_timer *t);