# ATMEGA8-NRF24L01-REPEATER

[![Build](https://github.com/a5021/ATMEGA8-NRF24L01-REPEATER/actions/workflows/build.yml/badge.svg)](https://github.com/a5021/ATMEGA8-NRF24L01-REPEATER/actions/workflows/build.yml) [![MCU](https://img.shields.io/badge/MCU-ATmega8-00A9E0)]() [![Radio](https://img.shields.io/badge/Radio-nRF24L01+-00A9E0)]() [![License](https://img.shields.io/badge/License-MIT-yellow)]()

Wireless sensor data repeater for ATmega8 and nRF24L01+. Receives environmental telemetry from a remote multisensor node (BMP180, Si7021, BH1750, ADC), controls a load relay based on light level, and rebroadcasts the datagram across multiple channels with acknowledgment and status reporting.

The sensor node firmware is available at [STM32F030F4P6-WIRELESS-MULTISENSOR](https://github.com/a5021/STM32F030F4P6-WIRELESS-MULTISENSOR).

## Features

- Register-level, bare-metal firmware for ATmega8 (no Arduino, no HAL)
- System clock 1 MHz (internal RC oscillator)
- Receives wireless datagrams from remote nRF24L01+ sensor nodes
- Supports payload sizes: 5, 7, 8, and 10 bytes
- Decodes sensor fields: BMP180 (pressure, temp), Si7021 (humidity, temp), BH1750 (illuminance), ADC (Vbat, Vcc, temp)
- Automatic load control based on BH1750 light threshold
- Multi-channel retransmission: acknowledged delivery on P0, status on ST, burst broadcast on 4 channels
- Visual indicators: red/green LEDs, buzzer with morse-coded greeting
- Watchdog timer for system recovery
- Sleep (IDLE) mode between packets
- Configurable serial debug output (UART 9600 baud)

## Hardware Specification

| Component | Detail |
|-----------|--------|
| MCU | Microchip ATmega8 (AVR, 8 KB Flash, 1 KB SRAM, 512 B EEPROM) |
| Clock | Internal RC oscillator, 1 MHz |
| Radio | Nordic Semiconductor nRF24L01+ (2.4 GHz, 2 Mbps) |
| Sensors (remote node) | BMP180 (pressure/temp), Si7021 (humidity/temp), BH1750 (light), ADC (Vbat/Vcc/temp) |
| Outputs | Red LED (PD6), Green LED (PD5), Buzzer (PB1), Load relay (PC2) |
| Radio channels | RX: 77, TX (P0): 0, Status (ST): 61, Broadcast: 14/47/69/101 |
| Debug | UART via PD0/PD1, 9600 baud, 8N1 (optional, `#define SERIAL_DEBUG_ENABLE`) |
| Programming | ISP (SPI) via ICSP header |

## Pin Assignment

| Signal | Pin | Direction | Notes |
|--------|-----|-----------|-------|
| CE | PD7 | Output | nRF24L01+ chip enable |
| CSN | PB2 | Output | nRF24L01+ SPI chip select |
| SCK | PB5 | Output | SPI clock |
| MOSI | PB3 | Output | SPI master out |
| MISO | PB4 | Input | SPI master in |
| IRQ | PD3 | Input | nRF24L01+ interrupt (falling edge) |
| Buzzer | PB1 | Output | PWM via OC1A |
| Red LED | PD6 | Output | Activity/error indicator |
| Green LED | PD5 | Output | Reception indicator |
| Load relay | PC2 | Output | Load ON/OFF control |
| RXD | PD0 | Input | UART RX (debug) |
| TXD | PD1 | Output | UART TX (debug) |

## Radio Protocol

### Datagram Format

The repeater receives a packed bitfield datagram from the remote sensor node:

```
Byte 0-1       Byte 2-3       Byte 4-5       Byte 6-7       Byte 8-9
+------+-----+--------+------+--------+------+--------+------+----------+
| TX   | PKT | BMP180 | BMP  | SI7021 | SI   | ADC    | ADC  | BH1750   |
| STAT | ID  | TEMP   | PRESS| HUMI   | TEMP | VBAT   | VCC  | ILLUM    |
| 1bit |5bits| 10bit  | 7bit | 7bit   |10bit | 9bit   | 8bit | 16bit    |
+------+-----+--------+------+--------+------+--------+------+----------+
```

Valid payload sizes:
| Size | Fields included |
|------|----------------|
| 5 B | TX status, packet ID, BMP180 temp + pressure, Si7021 humi + temp |
| 7 B | + BH1750 illuminance |
| 8 B | + ADC Vbat + Vcc + temp (no BH1750) |
| 10 B | + ADC + BH1750 (full payload) |

### Channel Map

| Channel | Function | Details |
|---------|----------|---------|
| 77 | RX | Primary receive channel from sensor node |
| 0 | P0 | Acknowledged retransmission back to sensor |
| 61 | ST | Status broadcast (transmit stats) |
| 14/47/69/101 | Broadcast | Burst broadcast on 4 channels, 3 repeats each |

## Firmware Architecture

```
  Reset
    |
  main()
    |
  GPIO init          Ports B/C/D: SPI, LEDs, buzzer, load, UART
    |
  Timer1 init        1 kHz Fast PWM (ICR1), overflow IRQ @ 1 ms
    |
  INT1 init          nRF24L01+ IRQ on falling edge
    |
  SPI init           Master, 4 MHz, MSB first
    |
  UART init          9600 baud (optional, SERIAL_DEBUG_ENABLE)
    |
  nrf_init(RX_CHANNEL)   Configure nRF24L01+ on channel 77
    |
  morseGreet()       Buzzer: ".... .-.." (HELLO in morse)
    |
  WDT enable         2 s timeout
    |
  Main loop
    |
  +-- nrf_start_receiving()
  |     |
  |     +-- Sleep IDLE, wait for IRQ or 3 s timeout
  |     |     (green LED flashes every ~1.5 s while waiting)
  |     |
  |     +-- nrf_get_payload_size()  validate: 5/7/8/10 bytes
  |     |
  |     +-- nrf_get_payload()       read datagram
  |     |
  |     +-- nrf_stop_receiving()
  |     |
  |     +-- Decode sensor fields    parse datagram into variables
  |     |
  |     +-- Light control           BH1750 > 4 => load relay OFF
  |     |                           BH1750 <= 4 => load relay ON
  |     |
  |     +-- Retransmit:
  |     |     channel 0  (P0)       ack delivery + stats
  |     |     channel 61 (ST)       transmit status word
  |     |     broadcast (4 ch)      burst on 14/47/69/101
  |     |
  |     +-- nrf_init(RX_CHANNEL)    back to receive mode
  |
  (repeat)
```

## Getting Started

### Prerequisites

- AVR GCC toolchain (`avr-gcc`, `avr-libc`)
- GNU Make

### Build

```sh
make EXTRAINCDIRS=
```

Output: `main.hex`, `main.eep`, `main.elf`.

### Flash

```sh
make program       # requires avrdude with programmer configured
```

### Debug

Define `SERIAL_DEBUG_ENABLE` at the top of `main.c` or in CFLAGS to enable UART output at 9600 baud.

## Project Structure

```
main.c             Application + nRF24L01+ driver + main loop
nrf24l01.h         nRF24L01+ register definitions and pin macros
Makefile           WinAVR-compatible AVR GCC build system
m.cmd              Build script (Windows cmd)
m1.cmd             Build script variant (Windows cmd)
.gitignore         Standard AVR ignores
```

## Related Projects

- [STM32F030F4P6-WIRELESS-MULTISENSOR](https://github.com/a5021/STM32F030F4P6-WIRELESS-MULTISENSOR) — wireless sensor node that transmits the datagrams received by this repeater
- [STM8S-NRF24L01-RECEIVER](https://github.com/a5021/STM8S-NRF24L01-RECEIVER) — STM8S-based nRF24L01+ receiver
- [NRF52832-BME280-RADIO](https://github.com/a5021/NRF52832-BME280-RADIO) — nRF52832-based BME280 radio sensor
