#include "adc-logger.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include <arm_acle.h>
#include <stdio.h>

#define TIMING

/*
------------------TIMING INFO-----------------
ADC Read x1                             2   uS
ADX Read x16 + max/min                  40  uS
Accumulation                            1   uS
SIMD Accumulation                       0   uS (WTF?)
UART TX @1382400 "0fae0fd20fc0\n"       86  uS


Other timing stuff.  Wrapping the debug printing stuff with
a variadic and using vsprintf ended up with things taking
longer (but I guess who cares if I'm just measuring timing stuff)
but also caused timings to jitter, which is more of a concern.
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
  uint16_t min;
  uint16_t max;
};

bool sample_adc_callback(struct repeating_timer *t) {
  /* take a look at the instructions the debugger is assocating
  with the FIXME lines... I *think* these instructions didn't
  exist before I turned this into a callback.  Maybe has something
  to do with adding userdata to the callback? */
  const char *error_string = "Error writing to debug buffer\r\n";
  struct adc_data *data = t->user_data;
  absolute_time_t debug_start;
  absolute_time_t debug_end;
  int64_t debug_elapsed;
  char debug_buf[64];
  int debug_err;
  uint16_t max = 0;
  uint16_t min = 0xFFFF;
  const int n_samples = 16;
  union adc_samples {
    uint16_t u16[n_samples];
    uint16x2_t u16x2[n_samples / 2];
  };
  union adc_samples samples = {};
  union sample_accumulator {
    uint16x2_t u16x2;
    struct {
      uint16_t u16_1;
      uint16_t u16_2;
    };
  };
  union sample_accumulator accumulator = {};
  uint16_t result = {};
  // gather samples and track max / min
  debug_start = debug_get_time();
  for (int i = 0; i < n_samples; ++i) {
    samples.u16[i] = adc_read();
    if (samples.u16[i] > max) {
      max = samples.u16[i];
    }
    if (samples.u16[i] < min) {
      min = samples.u16[i];
    }
  }
  debug_end = debug_get_time();
  debug_elapsed = debug_time_diff(debug_start, debug_end);
  debug_err =
      sprintf(debug_buf, "ADC Sample time x16 (uS) %lld\r\n", debug_elapsed);
  if (debug_err < 0) {
    debug_uart_print(data->uart, error_string);
  } else {
    debug_uart_print(data->uart, debug_buf);
  }

  debug_start = debug_get_time();
#if 1 // SIMD SUM
  for (int i = 0; i < n_samples / 2; ++i) {
    accumulator.u16x2 = __uadd16(accumulator.u16x2, samples.u16x2[i]);
  }
  result = accumulator.u16_1 + accumulator.u16_2;
#else
  for (int i = 0; i < n_samples; ++i) {
    result += samples.u16[i];
  }
#endif // SIMD SUM
  debug_end = debug_get_time();
  debug_elapsed = debug_time_diff(debug_start, debug_end);
  debug_err =
      sprintf(debug_buf, "Sample accumulation (uS) %lld\r\n", debug_elapsed);
  if (debug_err < 0) {
    debug_uart_print(data->uart, error_string);
  } else {
    debug_uart_print(data->uart, debug_buf);
  }

  result = result >> 4;
  data->max = max;
  data->min = min;
  data->sample = result;
  float voltage = (float)3.3 * (float)result /
                  (1 << 12); // FIXME: ldrh.w r3, [r4, #64] @ 0x40 ???
  debug_err =
      sprintf(debug_buf, "Average: Hex %04X Counts %4u Voltage %6.4F\r\n",
              result, result,
              voltage); // FIXME: vldr s13 [pc, #180] 0x1000031c ???
  if (debug_err < 0) {
    debug_uart_print(data->uart, error_string);
  } else {
    debug_uart_print(data->uart, debug_buf);
  }
  debug_err = sprintf(debug_buf, "Min %04X Max %04X Diff %04d\r\n", data->min,
                      data->max, (data->max - data->min));
  if (debug_err < 0) {
    debug_uart_print(data->uart, error_string);
  } else {
    debug_uart_print(data->uart, debug_buf);
  }
  int err = sprintf(data->buf, "%04x%04x%04x\n", min, max, result);
  if (err < 0) {
    uart_puts(data->uart, "FFFFFFFFFFFF\n");
  } else {
    debug_start = debug_get_time();
    uart_puts(data->uart, data->buf);
    debug_end = debug_get_time();
    debug_elapsed = debug_time_diff(debug_start, debug_end);
    debug_err = sprintf(debug_buf, "UART Write time (us) %lld\r\n",
                        debug_elapsed); // FIXME: mov r2, #0 ???
    if (debug_err < 0) {
      debug_uart_print(data->uart, error_string);
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
  struct adc_data data = {
      .buf = {}, .sample = 0, .err = 0, .uart = uart, .min = 0, .max = 0};
  add_repeating_timer_ms(-100, sample_adc_callback, &data, &timer);

  while (1) {
  }
  return 0;
}
