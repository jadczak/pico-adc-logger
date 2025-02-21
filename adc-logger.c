#include "adc-logger.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include <stdio.h>

// #define TIMING

/*
====TIMING INFO====
ADC read:
  2uS
UART write 1382400 "Hex 0FCE Counts 4046 Voltage 3.2597\r\n":
  185uS
UART write 1382400 baud "0fad\n"
  33uS
*/

absolute_time_t debug_get_time() {
#ifdef TIMING
  return get_absolute_time();
#else
  return (absolute_time_t)0;
#endif // TIMING
}

int64_t debug_time_diff([[maybe_unused]] absolute_time_t start,
                        [[maybe_unused]] absolute_time_t end) {
#ifdef TIMING
  return absolute_time_diff_us(start, end);
#else
  return (int64_t)0;
#endif // TIMING
}

void debug_uart_print([[maybe_unused]] uart_inst_t *uart,
                      [[maybe_unused]] const char *str) {
#ifdef TIMING
  uart_puts(uart, str);
#endif // TIMEING
}

struct adc_data {
  char buf[64];
  uint16_t sample;
  int err;
  uart_inst_t *uart;
};

bool sample_adc_callback(struct repeating_timer *t) {
  /* take a look at the instructions the debugger is assocating
  with the FIXME lines... I *think* these instructions didn't
  exist before I turned this into a callback.  Maybe has something
  to do with adding userdata to the callback? */
  struct adc_data *data = t->user_data;
  absolute_time_t debug_start;
  absolute_time_t debug_end;
  int64_t debug_elapsed;
  char debug_buf[64];
  int debug_err;
  debug_start = debug_get_time();
  data->sample = adc_read();
  debug_end = debug_get_time();
  debug_elapsed = debug_time_diff(debug_start, debug_end);
  debug_err = sprintf(debug_buf, "ADC sample time (us) %lld\r\n",
                      debug_elapsed); // FIXME: add r0, sp, #16 ???
  if (debug_err < 0) {
    debug_uart_print(data->uart, "Error writing to debug buffer.\r\n");
  } else {
    debug_uart_print(data->uart, debug_buf);
  }
  float voltage = (float)3.3 * (float)data->sample /
                  (1 << 12); // FIXME: ldrh.w r3, [r4, #64] @ 0x40 ???
  debug_err = sprintf(debug_buf, "Hex %04X Counts %4u Voltage %6.4F\r\n",
                      data->sample, data->sample,
                      voltage); // FIXME: vldr s13 [pc, #180] 0x1000031c ???
  if (debug_err < 0) {
    debug_uart_print(data->uart, "Error writing to debug buffer.\r\n");
  } else {
    debug_uart_print(data->uart, debug_buf);
  }

  int err = sprintf(data->buf, "%04x\n", data->sample);
  if (err < 0) {
    uart_puts(data->uart, "FFFF\n");
  } else {
    debug_start = debug_get_time();
    uart_puts(data->uart, data->buf);
    debug_end = debug_get_time();
    debug_elapsed = debug_time_diff(debug_start, debug_end);
    debug_err = sprintf(debug_buf, "UART Write time (us) %lld\r\n",
                        debug_elapsed); // FIXME: mov r2, #0 ???
    if (debug_err < 0) {
      debug_uart_print(data->uart, "Error writing to debug buffer.\r\n");
    } else {
      debug_uart_print(data->uart, debug_buf);
    }
  }

  return true;
}

int main(void) {
  stdio_init_all();

  // this is kind of stupid.  There is a function select table located at
  // https://www.raspberrypi.com/documentation/pico-sdk/hardware.html#group_hardware_gpio
  // that defines what the pins are allowed to be.  Pin 0 has to be TX if you
  // are using it as a UART, Pin 1 has to be RX.
  uart_inst_t *uart = uart0;
  const uint tx_pin = 0;
  const uint rx_pin = 1;
  gpio_set_function(tx_pin, UART_FUNCSEL_NUM(uart, tx_pin));
  gpio_set_function(rx_pin, UART_FUNCSEL_NUM(uart, rx_pin));
#define UART_115200 115200
#define UART_230400 230400
#define UART_460800 460800
#define UART_921600 921600
#define UART_1382400 1382400
  const uint baudrate = UART_1382400;
  uart_init(uart, baudrate);

  // Same kind of thing there but different. GPIO26 = ADC0, GPIO27 = ADC1...
  // https://www.raspberrypi.com/documentation/pico-sdk/hardware.html#detailed-description
  adc_init();
  const uint gpio_pin = 26;
  const uint adc_input = 0;
  adc_gpio_init(gpio_pin);
  adc_select_input(adc_input);
  struct repeating_timer timer;
  struct adc_data data = {.buf = {}, .sample = 0, .err = 0, .uart = uart};
  add_repeating_timer_ms(-1000, sample_adc_callback, &data, &timer);
  while (1) {
  }
  return 0;
}
