# Pico ADC logger

This is a _very_ simple ADC logger for the Raspberry Pi Pico 2.  I needed a quick an dirty logger and an excuse to buy a new Raspberry Pi Pico so here we are.

Some things worth noting if in case you are foolish enough to try using this thing:
* GPIO26 is the ADC channel being sampled
* The UART is configured for 1382400 8n1
* Uses SIMD, so no original Pico by default
* Uart data formated as AAAABBBBCCCC\n
* AAAA = Min adc counts for 16 consecutive samples in hex encoded in ascii
* BBBB = Max adc counts for 16 consecutive samples in hex encoded in ascii
* CCCC = Average adc counts for 16 consecutive samples in hex encoded in ascii


