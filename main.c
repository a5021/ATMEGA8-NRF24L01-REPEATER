/*******************************************************
First release: 17.04.2016

Chip type               : ATmega8
AVR Core Clock frequency: 1,000000 MHz
Data Stack size         : 256
*******************************************************/

#include <avr/io.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include "nrf24l01.h"

#define P0_CHANNEL          0
#define ST_CHANNEL          61
#define TX_CHANNEL          69
#define RX_CHANNEL          77
    
#define CSN_LOW()           PORTB &= ~(1<<PORTB2)
#define CSN_HIGH()          PORTB |= (1<<PORTB2)
#define CE_LOW()            PORTD &= ~(1<<PORTD7)
#define CE_HIGH()           PORTD |= (1<<PORTD7)

#define ONE_SEC_FLAG        (1<<0)
#define THREE_SEC_FLAG      (1<<1)
#define NRF_IRQ             (1<<2)
#define TIMEOUT_EXPIRED     (1<<3)
#define FIVE_SEC_FLAG       (1<<4)
#define TEN_SEC_FLAG        (1<<5)
#define QUART_MIN_FLAG      (1<<6)
#define HALF_MIN_FLAG       (1<<7)

#define RED_LED             (1 << PORTD6)
#define GREEN_LED           (1 << PORTD5)
#define LOAD_ON()           PORTC |= (1 << PORTC2)
#define LOAD_OFF()          PORTC &= ~(1 << PORTC2)
#define SWITCH_ON(PIN_NO)   PORTD |= PIN_NO
#define SWITCH_OFF(PIN_NO)  PORTD &= ~PIN_NO
#define TOGGLE(PIN_NO)      PORTD ^= PIN_NO

#define TIM1_START()        TCCR1B |= (1 << CS10);
#define TIM1_STOP()         TCCR1B &= ~(1 << CS10)  ;
  
#define BEEP(DURATION)	    soundDuration = DURATION
#define FLASH(DURATION)     flashDuration = DURATION

#define fieldmask(from, to)  (((1u<<((to)-(from)+1))-1)<<(from))
#define getfield(var, from, to) (((var)&fieldmask(from,to))>>(from))
#define setfield(var, value, from, to) var = ((var)&~fieldmask(from,to))|(((value)<<(from)) & fieldmask(from,to))

typedef struct __attribute__ ((packed)){
  uint32_t tx_status        :1;  //  1
  uint32_t pkt_id           :5;  //  1
  int32_t  bmp180_temp      :10; //  1-2
  uint32_t bmp180_press     :7;  //  3
  uint32_t si7021_humi      :7;  //  3-4
  int32_t  si7021_temp      :10; //  4-5
  // ------------------------------
  int32_t  adc_temp         :7;  //  6
  uint32_t adc_vbat         :9;  //  6-7
  uint32_t adc_vcc          :8;  //  8
  uint32_t bh1750           :16; //  9-10
  uint32_t ext_code         :8;  //  
  uint32_t aux              :8;  //  
} datagram_t;

typedef struct __attribute__ ((packed)) {
  uint32_t pktCount         :32;
  uint32_t pktLost          :32;
} stat_t;

uint8_t broadcast_channel[] = {14, 47, 69, 101};

volatile unsigned char globalFlag = 0;
static uint8_t recv_buf[12];

#ifdef SERIAL_DEBUG_ENABLE
  static int uart_putchar(char c, FILE *stream);
  uint8_t uart_getchar(void);

  static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
#endif

volatile unsigned int soundDuration = 0,
                      flashDuration = 0;

// External Interrupt 1 service routine
ISR(INT1_vect) {
// Place your code here
  globalFlag |= NRF_IRQ;
}

// Timer1 overflow interrupt service routine
ISR(TIMER1_OVF_vect) {
  static unsigned char oneSecCounter, threeSecCounter;

  if (++oneSecCounter == 1000) {
    oneSecCounter = 0;
    globalFlag |= ONE_SEC_FLAG;
    if (++threeSecCounter == 10) {
      globalFlag |= THREE_SEC_FLAG;
      threeSecCounter = 0;
    }
  }

  if (soundDuration > 0) {
    if ((TCCR1A & (1 << COM1A1)) == (1 << COM1A1)) {
      if (--soundDuration == 0) {
        TCCR1A &= ~(1 << COM1A1);
        PORTB &= ~(1 << PORTB1);       // lower PB1 to close transistor
      }
    } else {
      TCCR1A |= (1 << COM1A1);
    }
  }

  if (flashDuration > 0) {
    if ((PORTD & RED_LED) == RED_LED) {
      if (--flashDuration == 0) {
        PORTD &= ~RED_LED;       // lower PB1 to close transistor
      }
    } else {
      PORTD |= RED_LED;
    }
  }
}

// SPI functions
#ifdef SERIAL_DEBUG_ENABLE
  void printBits(uint8_t byte) {

    for (uint8_t i = 1; i != 0; i <<= 1) {
      printf(" %d", ((byte & i) == 0) ? 0 : 1);
    }
  }
#endif

unsigned char spi(unsigned char data) {
  SPDR=data;
  while ((SPSR & (1<<SPIF)) == 0);
  return SPDR;
}


uint8_t nrf_write_cmd(uint8_t cmd) {
  uint8_t s;
  
  CSN_LOW();
  s = spi(cmd);
  CSN_HIGH();
  return s;
}

uint8_t nrf_write_register(uint8_t regNo, uint8_t regVal) {
  uint8_t s;
  
  CSN_LOW();
  s = spi(W_REGISTER | (REGISTER_MASK & regNo));
  spi(regVal);
  CSN_HIGH();
  return s;
}

uint8_t nrf_read_register(uint8_t regNo) {
  uint8_t s;

  CSN_LOW();
  spi(R_REGISTER | (REGISTER_MASK & regNo));
  s = spi(NOP);
  CSN_HIGH();
  return s;
}

void init_tim1_5hz(void) {
  TCCR1B &= ~((1<<CS12) | (1<<CS11) | (1<<CS10));
  // Timer/Counter 1 initialization
  // Clock source: System Clock
  // Clock value: 125,000 kHz
  // Mode: Fast PWM top=ICR1
  // OC1A output: Disconnected
  // OC1B output: Disconnected
  // Noise Canceler: Off
  // Input Capture on Falling Edge
  // Timer Period: 0,2 s
  // Timer1 Overflow Interrupt: On
  // Input Capture Interrupt: Off
  // Compare A Match Interrupt: Off
  // Compare B Match Interrupt: Off
  TCNT1H=0x00;
  TCNT1L=0x00;
  ICR1H=0x61;
  ICR1L=0xA7;
  TCCR1A=(0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (1<<WGM11) | (0<<WGM10);
  TCCR1B=(0<<ICNC1) | (0<<ICES1) | (1<<WGM13) | (1<<WGM12) | (0<<CS12) | (1<<CS11) | (0<<CS10);
}

void init_tim1_10hz(void) {
  TCCR1B &= ~((1<<CS12) | (1<<CS11) | (1<<CS10));

  // Timer/Counter 1 initialization
  // Clock source: System Clock
  // Clock value: 125,000 kHz
  // Mode: Fast PWM top=ICR1
  // OC1A output: Disconnected
  // OC1B output: Disconnected
  // Noise Canceler: Off
  // Input Capture on Falling Edge
  // Timer Period: 0,1 s
  // Timer1 Overflow Interrupt: On
  // Input Capture Interrupt: Off
  // Compare A Match Interrupt: Off
  // Compare B Match Interrupt: Off
  TCNT1H=0x00;
  TCNT1L=0x00;
  ICR1H=0x30;
  ICR1L=0xD3;
  OCR1AH=0x18;
  OCR1AL=0x6A;
  TCCR1A=(0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (1<<WGM11) | (0<<WGM10);
  TCCR1B=(0<<ICNC1) | (0<<ICES1) | (1<<WGM13) | (1<<WGM12) | (0<<CS12) | (1<<CS11) | (0<<CS10);
}

void init_tim1_1khz(void) {
  TCCR1B &= ~((1<<CS12) | (1<<CS11) | (1<<CS10));
  // Timer/Counter 1 initialization
  // Clock source: System Clock
  // Clock value: 1000,000 kHz
  // Mode: Fast PWM top=ICR1
  // OC1A output: Non-Inverted PWM
  // OC1B output: Disconnected
  // Noise Canceler: Off
  // Input Capture on Falling Edge
  // Timer Period: 1 ms
  // Output Pulse(s):
  // OC1A Period: 1 ms Width: 0,5005 ms
  // Timer1 Overflow Interrupt: On
  // Input Capture Interrupt: Off
  // Compare A Match Interrupt: Off
  // Compare B Match Interrupt: Off
  TCNT1H=0x00;
  TCNT1L=0x00;
  ICR1H=0x03;
  ICR1L=0xE7;
  OCR1AH=0x01;
  OCR1AL=0xF4;
  TCCR1A=(0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (1<<WGM11) | (0<<WGM10);
  TCCR1B=(0<<ICNC1) | (0<<ICES1) | (1<<WGM13) | (1<<WGM12) | (0<<CS12) | (0<<CS11) | (1<<CS10);
}  


uint8_t nrf_start_receiving(void) {
  uint8_t nrf_status;

  CE_LOW();

  nrf_write_register(STATUS,     // clear status bits
    1 * NRF24_STATUS_MAX_RT       |
    1 * NRF24_STATUS_TX_DS        |
    1 * NRF24_STATUS_RX_DR
  );

  nrf_write_cmd(FLUSH_RX);       
  nrf_write_cmd(FLUSH_TX);       

  nrf_status = nrf_write_register(CONFIG,     // power up receiver
    1 * NRF24_CONFIG_PRIM_RX      |  // select RX mode
    1 * NRF24_CONFIG_PWR_UP       |  // turn power on
    1 * NRF24_CONFIG_EN_CRC       |  // enable CRC
    0 * NRF24_CONFIG_CRCO         |  // CRC encoding scheme: '0' - 1 byte, '1' – 2 bytes
    1 * NRF24_CONFIG_MAX_RT       |  // MAX_RT interrupt not reflected on the IRQ pin
    1 * NRF24_CONFIG_TX_DS        |  // TX_DS interrupt not reflected on the IRQ pin
    0 * NRF24_CONFIG_RX_DR           // Reflect RX_DR as active low interrupt on the IRQ pin
  );
  CE_HIGH();  // start receiving
  return nrf_status;
}

uint8_t nrf_stop_receiving(void) {
  CE_LOW();
  uint8_t rx_status = nrf_write_register(CONFIG,     // power down receiver
    0 * NRF24_CONFIG_PRIM_RX      |  // select TX mode
    0 * NRF24_CONFIG_PWR_UP       |  // turn power off
    1 * NRF24_CONFIG_EN_CRC       |  // enable CRC. Forced high if one of the bits in the
                                     // EN_AA is high
    0 * NRF24_CONFIG_CRCO         |  // CRC encoding scheme: '0' - 1 byte, '1' – 2 bytes
    1 * NRF24_CONFIG_MAX_RT       |  // MAX_RT interrupt not reflected on the IRQ pin
    1 * NRF24_CONFIG_TX_DS        |  // TX_DS interrupt not reflected on the IRQ pin
    1 * NRF24_CONFIG_RX_DR           // RX_DR interrupt not reflected on the IRQ pin
  );

  nrf_write_cmd(FLUSH_RX);       

  return rx_status;
}

uint8_t nrf_get_payload_size(void) {
  CSN_LOW();
  spi(R_RX_PL_WID);
  uint8_t psize = spi(NOP);
  CSN_HIGH();
  return psize;
} 

void nrf_get_payload(uint8_t buf[], uint8_t len) {
  CSN_LOW();

  spi(R_RX_PAYLOAD);

  for (int i = 0; i < len; i++) {
   buf[i] = spi(NOP);
  }

  CSN_HIGH();
}

uint8_t nrf_transmit(uint8_t src[], uint8_t len) {
  uint8_t i, tx_status;
  
  nrf_write_register(STATUS,     // clear status bits
    1 * NRF24_STATUS_MAX_RT       |
    1 * NRF24_STATUS_TX_DS        |
    1 * NRF24_STATUS_RX_DR
  );

  // nrf_write_cmd(FLUSH_RX);       
  nrf_write_cmd(FLUSH_TX);       // flush tx buffer

  nrf_write_register(CONFIG,     // Transmitter power-up
    0 * NRF24_CONFIG_PRIM_RX      |  // select TX mode
    1 * NRF24_CONFIG_PWR_UP       |  // turn power on
    1 * NRF24_CONFIG_EN_CRC       |  // enable CRC. Forced high if one of the bits in the
                                     // EN_AA is high
    0 * NRF24_CONFIG_CRCO         |  // CRC encoding scheme: '0' - 1 byte, '1' – 2 bytes
    1 * NRF24_CONFIG_MAX_RT       |  // MAX_RT interrupt not reflected on the IRQ pin
    0 * NRF24_CONFIG_TX_DS        |  // Reflect TX_DS as active low interrupt on the IRQ pin
    1 * NRF24_CONFIG_RX_DR           // RX_DR interrupt not reflected on the IRQ pin
  );

  CSN_LOW();

  spi(W_TX_PAYLOAD_NOACK);

  for (i = 0; i < len; i++) {
    spi(src[i]);
  }

  CSN_HIGH();

  _delay_us(500); // wait until oscillations of the clock generator becomes stable

  // start transmitting
  CE_HIGH();
  _delay_us(10);
  CE_LOW();
  
  // T_PLL + T_OA + T_IRQ = 130uS + 64,5uS + 6uS =
  //  _delay_us(190);
  #define TX_TIMEOUT  200

  int tx_timeout = TX_TIMEOUT;

  while (((globalFlag & NRF_IRQ) != NRF_IRQ) && (--tx_timeout != 0))  {
    // set_sleep_mode(SLEEP_MODE_IDLE);
    // sleep_mode();
    // wdt_reset();
    // TOGGLE(GREEN_LED);
  }
  globalFlag &= ~NRF_IRQ;
  
  tx_status = nrf_write_register(CONFIG,     // Transmitter power-down
    0 * NRF24_CONFIG_PRIM_RX      |  // select TX mode
    0 * NRF24_CONFIG_PWR_UP       |  // turn power off
    1 * NRF24_CONFIG_EN_CRC       |  // enable CRC. Forced high if one of the bits in the
                                     // EN_AA is high
    0 * NRF24_CONFIG_CRCO         |  // CRC encoding scheme: '0' - 1 byte, '1' – 2 bytes
    1 * NRF24_CONFIG_MAX_RT       |  // MAX_RT interrupt not reflected on the IRQ pin
    1 * NRF24_CONFIG_TX_DS        |  // TX_DS interrupt not reflected on the IRQ pin
    1 * NRF24_CONFIG_RX_DR           // RX_DR interrupt not reflected on the IRQ pin
  );

  return tx_status;
}

uint16_t nrf_transmit_special(uint8_t src[], uint8_t len) {
  uint8_t i, tx_status;

  nrf_write_register(SETUP_AW,   //    RX/TX Address field width
                                 //            '00' - Illegal
                                 //            '01' - 3 bytes
                                 //            '10' - 4 bytes
                                 //            '11' – 5 bytes
                                 // LSByte is used if address width is below 5 bytes  
     0 * NRF24_SETUP_AW_0        |
     1 * NRF24_SETUP_AW_1
  );                     
  
  nrf_write_register(STATUS,     // clear status bits
    1 * NRF24_STATUS_MAX_RT       |
    1 * NRF24_STATUS_TX_DS        |
    1 * NRF24_STATUS_RX_DR
  );

  // nrf_write_cmd(FLUSH_RX);       
  nrf_write_cmd(FLUSH_TX);       // flush tx buffer

  nrf_write_register(CONFIG,     // Transmitter power-up
    0 * NRF24_CONFIG_PRIM_RX      |  // select TX mode
    1 * NRF24_CONFIG_PWR_UP       |  // turn power on
    1 * NRF24_CONFIG_EN_CRC       |  // enable CRC. Forced high if one of the bits in the
                                     // EN_AA is high
    1 * NRF24_CONFIG_CRCO         |  // CRC encoding scheme: '0' - 1 byte, '1' – 2 bytes
    0 * NRF24_CONFIG_MAX_RT       |  // MAX_RT interrupt not reflected on the IRQ pin
    0 * NRF24_CONFIG_TX_DS        |  // Reflect TX_DS as active low interrupt on the IRQ pin
    1 * NRF24_CONFIG_RX_DR           // RX_DR interrupt not reflected on the IRQ pin
  );

  CSN_LOW();

  spi(W_TX_PAYLOAD);

  for (i = 0; i < len; i++) {
    spi(src[i]);
  }

  CSN_HIGH();

  _delay_us(500); // wait until oscillations of the clock generator becomes stable

  // start transmitting
  CE_HIGH();
    // _delay_us(10);
  
  // T_PLL + T_OA + T_IRQ = 130uS + 64,5uS + 6uS =
  //  _delay_us(190);

  while ((globalFlag & NRF_IRQ) != NRF_IRQ);

  CE_LOW();
  globalFlag &= ~NRF_IRQ;
  
  tx_status = nrf_write_register(CONFIG,     // Transmitter power-down
    0 * NRF24_CONFIG_PRIM_RX      |  // select TX mode
    0 * NRF24_CONFIG_PWR_UP       |  // turn power off
    1 * NRF24_CONFIG_EN_CRC       |  // enable CRC. Forced high if one of the bits in the
                                     // EN_AA is high
    0 * NRF24_CONFIG_CRCO         |  // CRC encoding scheme: '0' - 1 byte, '1' – 2 bytes
    1 * NRF24_CONFIG_MAX_RT       |  // MAX_RT interrupt not reflected on the IRQ pin
    1 * NRF24_CONFIG_TX_DS        |  // TX_DS interrupt not reflected on the IRQ pin
    1 * NRF24_CONFIG_RX_DR           // RX_DR interrupt not reflected on the IRQ pin
  );

  uint8_t observe = nrf_read_register(OBSERVE_TX);

  nrf_write_cmd(FLUSH_TX);       // flush tx buffer

  nrf_write_register(SETUP_AW,   //    RX/TX Address field width
                                 //            '00' - Illegal
                                 //            '01' - 3 bytes
                                 //            '10' - 4 bytes
                                 //            '11' – 5 bytes
                                 // LSByte is used if address width is below 5 bytes  
     1 * NRF24_SETUP_AW_0        |
     0 * NRF24_SETUP_AW_1
  );                     

  return (uint16_t) (observe << 8) | tx_status;
}


uint8_t nrf_broadcast(uint8_t src[], uint8_t len) {
  uint8_t i, j, tx_status;
  
  nrf_write_cmd(FLUSH_TX);       // flush tx buffer

  nrf_write_register(CONFIG,     // Transmitter power-up
    0 * NRF24_CONFIG_PRIM_RX      |  // select TX mode
    1 * NRF24_CONFIG_PWR_UP       |  // turn power on
    1 * NRF24_CONFIG_EN_CRC       |  // enable CRC. Forced high if one of the bits in the
                                     // EN_AA is high
    1 * NRF24_CONFIG_CRCO         |  // CRC encoding scheme: '0' - 1 byte, '1' – 2 bytes
    1 * NRF24_CONFIG_MAX_RT       |  // MAX_RT interrupt not reflected on the IRQ pin
    1 * NRF24_CONFIG_TX_DS        |  // TX_DS interrupt not reflected on the IRQ pin
    1 * NRF24_CONFIG_RX_DR           // RX_DR interrupt not reflected on the IRQ pin
  );

  _delay_us(500); // ??

  /* ================ WRITE PAYLOAD ==================== */

  CSN_LOW();
  spi(W_TX_PAYLOAD_NOACK);
  for (i = 0; i < len; i++) {
    spi(src[i]);
  }
  CSN_HIGH();

  nrf_write_cmd(REUSE_TX_PL);        /* Send everytime the same payload */

  for (i = 0; i < sizeof(broadcast_channel) ; i++) {  // make burst
    nrf_write_register(RF_CH, broadcast_channel[i]); // set radio channel
    for (j = 0; j < 3; j++) {
      // start transmitting
      CE_HIGH();
      _delay_us(10);
      CE_LOW();
      _delay_ms((rand() % 10) + 1);  /* Make random 1 .. 10ms delay */
    }
  }

  tx_status = nrf_write_register(CONFIG,     // Transmitter power-down
    0 * NRF24_CONFIG_PRIM_RX      |  // select TX mode
    0 * NRF24_CONFIG_PWR_UP       |  // turn power off
    1 * NRF24_CONFIG_EN_CRC       |  // enable CRC. Forced high if one of the bits in the
                                     // EN_AA is high
    0 * NRF24_CONFIG_CRCO         |  // CRC encoding scheme: '0' - 1 byte, '1' – 2 bytes
    1 * NRF24_CONFIG_MAX_RT       |  // MAX_RT interrupt not reflected on the IRQ pin
    1 * NRF24_CONFIG_TX_DS        |  // TX_DS interrupt not reflected on the IRQ pin
    1 * NRF24_CONFIG_RX_DR           // RX_DR interrupt not reflected on the IRQ pin
  );

  nrf_write_cmd(FLUSH_TX);       // flush tx buffer (reset REUSE_TX_PL)

  return tx_status;
}

uint8_t nrf_init(uint8_t channel) {

  nrf_write_register(SETUP_AW,   //    RX/TX Address field width
                                 //            '00' - Illegal
                                 //            '01' - 3 bytes
                                 //            '10' - 4 bytes
                                 //            '11' – 5 bytes
                                 // LSByte is used if address width is below 5 bytes  
     1 * NRF24_SETUP_AW_0        |
     0 * NRF24_SETUP_AW_1
  );

  nrf_write_register(SETUP_RETR, 
    //
    //     Auto Retransmit Count
    // ‘0000’ –Re-Transmit disabled
    // ‘0001’ – Up to 1 Re-Transmit on fail of AA
    // ……
    // ‘1111’ – Up to 15 Re-Transmit on fail of AA

    1 * NRF24_SETUP_RETR_ARC_0 |      // 1111 bin -- 15 retries
    1 * NRF24_SETUP_RETR_ARC_1 |
    1 * NRF24_SETUP_RETR_ARC_2 |
    1 * NRF24_SETUP_RETR_ARC_3 |

    // 
    //  Auto Retransmit Delay
    //  ‘0000’ – Wait 250µS
    //  ‘0001’ – Wait 500µS
    //  ‘0010’ – Wait 750µS
    //  ……..
    //  ‘1111’ – Wait 4000µS
    //  (Delay defined from end of transmission to start of
    //  next transmission)

    0 * NRF24_SETUP_RETR_ARD_0 |
    0 * NRF24_SETUP_RETR_ARD_1 |
    0 * NRF24_SETUP_RETR_ARD_2 |
    0 * NRF24_SETUP_RETR_ARD_3
  );  
  
  // nrf_write_register(EN_AA, 0);  // disable 'Auto Acknowledgment'

  nrf_write_register(RF_CH, channel); // set radio channel
  
  nrf_write_register(RF_SETUP,   // set radio channel
      //  Set RF output power in TX mode
      //  '00' – -18dBm
      //  '01' – -12dBm
      //  '10' – -6dBm
      //  '11' – 0dBm
    1 * NRF24_RF_SETUP_RF_PWR_0  |
    1 * NRF24_RF_SETUP_RF_PWR_1  |
      //  Select between the high speed data rates. This bit
      //  is don’t care if RF_DR_LOW is set.
      //  Encoding:
      //  [RF_DR_LOW, RF_DR_HIGH]:
      //  ‘00’ – 1Mbps
      //  ‘01’ – 2Mbps
      //  ‘10’ – 250kbps
      //  ‘11’ – Reserved
    1 * NRF24_RF_SETUP_RF_DR_HIGH |
    0 * NRF24_RF_SETUP_PLL_LOCK   |
    0 * NRF24_RF_SETUP_RF_DR_LOW
      // bit 6 -- reserved
    // 0 * NRF24_RF_SETUP_CONT_WAVE
  );                     
  
  nrf_write_register(FEATURE,    // set payload parameters
    1 * NRF24_FEATURE_EN_DYN_ACK  |  // enable W_TX_PAYLOAD_NOACK command
    0 * NRF24_FEATURE_EN_ACK_PAY  |                  
    1 * NRF24_FEATURE_EN_DPL         // enable dynamic payload
  );
  
  return nrf_write_register(DYNPD,   // dynamic payload requisites
    1 * NRF24_DYNPD_DPL_P0        |  // set dynamic payload flag
    0 * NRF24_DYNPD_DPL_P1        |
    0 * NRF24_DYNPD_DPL_P2        |
    0 * NRF24_DYNPD_DPL_P3        |
    0 * NRF24_DYNPD_DPL_P4        |
    0 * NRF24_DYNPD_DPL_P5        
  );
}

datagram_t	*dg = (datagram_t*) recv_buf;

void clearDatagram(void) {
  uint32_t *dPtr = (uint32_t*)recv_buf;
  for (uint8_t i = 0; i < 3; i++) {
    *dPtr++ = 0;
  }
}

#ifdef SERIAL_DEBUG_ENABLE
void printDatagram(uint8_t cnt) {
 
  for (uint8_t i = 0; i < cnt; i++) {
    printf(" %02X", recv_buf[i]);
  }
}
#endif

void morseDot(void) {
  BEEP(60);
  while (soundDuration != 0);
}

void morseDish(void) {
  BEEP(180);
  while (soundDuration != 0);
}

void morseGreet(void) {
  for (int i = 0; i < 4; i++) {
    morseDot();
    _delay_ms(60);
  }

  _delay_ms(180);

  for (int i = 0; i < 2; i++) {
    morseDot();
    _delay_ms(60);
  }
  _delay_ms(180);
}

uint8_t si7021_humi, tx_status, pkt_id, prev_pkt_id, ext_code, aux;
uint16_t bmp180_press, remote_vbat, remote_vcc, bh1750;
int8_t remote_temp;
int16_t bmp180_temp, si7021_temp;

int main(void)  {
  uint8_t lightSwitch = 0, lightSwitchPrev = 0;

// Declare your local variables here

    // Reset Source checking
    if (MCUCSR & (1<<PORF)) {
       // Power-on Reset
       MCUCSR=0;
       // Place your code here

    } else if (MCUCSR & (1<<EXTRF)) {
         // External Reset
         MCUCSR=0;
         // Place your code here

    } else if (MCUCSR & (1<<BORF)) {
         // Brown-Out Reset
         MCUCSR=0;
         // Place your code here
    } else {
         // Watchdog Reset
         MCUCSR=0;
         // Place your code here

    }

// Input/Output Ports initialization
// Port B initialization
// Function: Bit7=In Bit6=In Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=Out Bit0=In 
DDRB=(0<<DDB7) | (0<<DDB6) | (1<<DDB5) | (0<<DDB4) | (1<<DDB3) | (1<<DDB2) | (1<<DDB1) | (0<<DDB0);
// State: Bit7=T Bit6=T Bit5=1 Bit4=1 Bit3=1 Bit2=1 Bit1=0 Bit0=T 
PORTB=(0<<PORTB7) | (0<<PORTB6) | (1<<PORTB5) | (0<<PORTB4) | (1<<PORTB3) | (1<<PORTB2) | (0<<PORTB1) | (1<<PORTB0);

// Port C initialization
// Function: Bit6=In Bit5=In Bit4=In Bit3=In Bit2=Out Bit1=In Bit0=In 
DDRC=(0<<DDC6) | (0<<DDC5) | (0<<DDC4) | (0<<DDC3) | (1<<DDC2) | (0<<DDC1) | (0<<DDC0);
// State: Bit6=T Bit5=T Bit4=T Bit3=T Bit2=1 Bit1=T Bit0=T 
PORTC=(1<<PORTC6) | (1<<PORTC5) | (1<<PORTC4) | (1<<PORTC3) | (1<<PORTC2) | (1<<PORTC1) | (1<<PORTC0);

#ifdef SERIAL_DEBUG_ENABLE

// Port D initialization
// Function: Bit7=Out Bit6=Out Bit5=Out Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRD=(1<<DDD7) | (1<<DDD6) | (1<<DDD5) | (0<<DDD4) | (0<<DDD3) | (0<<DDD2) | (0<<DDD1) | (0<<DDD0);
// State: Bit7=0 Bit6=0 Bit5=1 Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTD=(1<<PORTD7) | (0<<PORTD6) | (1<<PORTD5) | (1<<PORTD4) | (0<<PORTD3) | (1<<PORTD2) | (0<<PORTD1) | (0<<PORTD0);

#else

// Port D initialization
// Function: Bit7=Out Bit6=Out Bit5=Out Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRD=(1<<DDD7) | (1<<DDD6) | (1<<DDD5) | (0<<DDD4) | (0<<DDD3) | (0<<DDD2) | (0<<DDD1) | (0<<DDD0);
// State: Bit7=0 Bit6=0 Bit5=1 Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTD=(1<<PORTD7) | (0<<PORTD6) | (1<<PORTD5) | (1<<PORTD4) | (0<<PORTD3) | (1<<PORTD2) | (0<<PORTD1) | (0<<PORTD0);

#endif

// Timer/Counter 0 initialization
// Clock source: System Clock
// Clock value: Timer 0 Stopped
TCCR0=(0<<CS02) | (0<<CS01) | (0<<CS00);
TCNT0=0x00;

// Timer/Counter 1 initialization
// Clock source: System Clock
// Clock value: 1000,000 kHz
// Mode: Fast PWM top=ICR1
// OC1A output: Non-Inverted PWM
// OC1B output: Disconnected
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer Period: 1 ms
// Output Pulse(s):
// OC1A Period: 1 ms Width: 0,5005 ms
// Timer1 Overflow Interrupt: On
// Input Capture Interrupt: Off
// Compare A Match Interrupt: Off
// Compare B Match Interrupt: Off
TCCR1A=(0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (1<<WGM11) | (0<<WGM10);
TCCR1B=(0<<ICNC1) | (0<<ICES1) | (1<<WGM13) | (1<<WGM12) | (0<<CS12) | (0<<CS11) | (1<<CS10);
TCNT1H=0x00;
TCNT1L=0x00;
ICR1H=0x03;
ICR1L=0xE7;
OCR1AH=0x01;
OCR1AL=0xF4;
OCR1BH=0x00;
OCR1BL=0x00;


// Timer/Counter 2 initialization
// Clock source: System Clock
// Clock value: Timer2 Stopped
// Mode: Normal top=0xFF
// OC2 output: Disconnected
//ASSR=0<<AS2;
//TCCR2=(0<<PWM2) | (0<<COM21) | (0<<COM20) | (0<<CTC2) | (0<<CS22) | (0<<CS21) | (0<<CS20);
//TCNT2=0x00;
//OCR2=0x00;

// Timer(s)/Counter(s) Interrupt(s) initialization
TIMSK=(0<<OCIE2) | (0<<TOIE2) | (0<<TICIE1) | (0<<OCIE1A) | (0<<OCIE1B) | (1<<TOIE1) | (0<<TOIE0);

// External Interrupt(s) initialization
// INT0: Off
// INT1: On
// INT1 Mode: Falling Edge
GICR|=(1<<INT1) | (0<<INT0);
MCUCR=(1<<ISC11) | (0<<ISC10) | (0<<ISC01) | (0<<ISC00);
GIFR=(1<<INTF1) | (0<<INTF0);

#ifdef SERIAL_DEBUG_ENABLE
// // USART initialization
// // Communication Parameters: 8 Data, 1 Stop, No Parity
// // USART Receiver: On
// // USART Transmitter: On
// // USART Mode: Asynchronous
// // USART Baud Rate: 38400
// UCSRA=(0<<RXC) | (0<<TXC) | (0<<UDRE) | (0<<FE) | (0<<DOR) | (0<<PE) | (0<<U2X) | (0<<MPCM);
// UCSRB=(0<<RXCIE) | (0<<TXCIE) | (0<<UDRIE) | (1<<RXEN) | (1<<TXEN) | (0<<UCSZ2) | (0<<RXB8) | (0<<TXB8);
// UCSRC=(1<<URSEL) | (0<<UMSEL) | (0<<UPM1) | (0<<UPM0) | (0<<USBS) | (1<<UCSZ1) | (1<<UCSZ0) | (0<<UCPOL);
// UBRRH=0x00;
// UBRRL=0x19;

// USART initialization
// Communication Parameters: 8 Data, 1 Stop, No Parity
// USART Receiver: On
// USART Transmitter: On
// USART Mode: Asynchronous
// USART Baud Rate: 9600 (Double Speed Mode)
UCSRA=(0<<RXC) | (0<<TXC) | (0<<UDRE) | (0<<FE) | (0<<DOR) | (0<<PE) | (1<<U2X) | (0<<MPCM);
UCSRB=(0<<RXCIE) | (0<<TXCIE) | (0<<UDRIE) | (1<<RXEN) | (1<<TXEN) | (0<<UCSZ2) | (0<<RXB8) | (0<<TXB8);
UCSRC=(1<<URSEL) | (0<<UMSEL) | (0<<UPM1) | (0<<UPM0) | (0<<USBS) | (1<<UCSZ1) | (1<<UCSZ0) | (0<<UCPOL);
UBRRH=0x00;
UBRRL=0x0C;

#endif

// Analog Comparator initialization
// Analog Comparator: Off
// The Analog Comparator's positive input is
// connected to the AIN0 pin
// The Analog Comparator's negative input is
// connected to the AIN1 pin
ACSR=(1<<ACD) | (0<<ACBG) | (0<<ACO) | (0<<ACI) | (0<<ACIE) | (0<<ACIC) | (0<<ACIS1) | (0<<ACIS0);
SFIOR=(0<<ACME);

// ADC initialization
// ADC disabled
ADCSRA=(0<<ADEN) | (0<<ADSC) | (0<<ADFR) | (0<<ADIF) | (0<<ADIE) | (0<<ADPS2) | (0<<ADPS1) | (0<<ADPS0);

// SPI initialization
// SPI Type: Master
// SPI Clock Rate: 4000,000 kHz
// SPI Clock Phase: Cycle Start
// SPI Clock Polarity: Low
// SPI Data Order: MSB First
SPCR=(0<<SPIE) | (1<<SPE) | (0<<DORD) | (1<<MSTR) | (0<<CPOL) | (0<<CPHA) | (0<<SPR1) | (0<<SPR0);
SPSR=(1<<SPI2X);

// TWI initialization
// TWI disabled
TWCR=(0<<TWEA) | (0<<TWSTA) | (0<<TWSTO) | (0<<TWEN) | (0<<TWIE);

// Global enable interrupts
sei();

#ifdef SERIAL_DEBUG_ENABLE
  stdout = &mystdout; //Required for printf init
#endif

//  CSN_HIGH();
//  delay_ms(100);

nrf_init(RX_CHANNEL);
#ifdef SERIAL_DEBUG_ENABLE
  printf("STATUS = %02X, RF_CH = %u, POWER = %02X. \r\n", nrf_read_register(STATUS), nrf_read_register(RF_CH), nrf_read_register(RF_SETUP));
#endif
/*
  datagram.bmp180_press = 50;
  datagram.adc_vbat = 202;
  datagram.tx_status = 1;
  datagram.adc_temp = 27;
  datagram.adc_vcc = 202-160;
  datagram.si7021_humi = 81;
  datagram.bmp180_temp = 281;  
  datagram.si7021_temp = 282;
  datagram.pkt_id = 31;
  datagram.bh1750 = 12;

  uint8_t mCnt = 0;
*/
#ifdef SERIAL_DEBUG_ENABLE
  printf("\n");
#endif

  morseGreet();

  //wdt_enable(WDTO_2S);
  //wdt_enable(WDTO_15MS);
  wdt_enable(WDTO_2S);

  while (1) {

    nrf_start_receiving();

    while (flashDuration || soundDuration);

    init_tim1_5hz();

    #define RECV_TIMEOUT  3000

    int recv_timeout = RECV_TIMEOUT;
    int fCount = 0;

    while (((globalFlag & NRF_IRQ) != NRF_IRQ) && (--recv_timeout != 0))  {
      set_sleep_mode(SLEEP_MODE_IDLE);
      sleep_mode();
      wdt_reset();
      if (++fCount == 75) {
        fCount = 0;
        SWITCH_ON(GREEN_LED);
      } else if (fCount == 1) {
        SWITCH_OFF(GREEN_LED);
      }
    }
    globalFlag &= ~NRF_IRQ;

    init_tim1_1khz();

    if (recv_timeout == 0) {
      nrf_stop_receiving();
      BEEP(100);
      continue;
    }

    uint8_t bytesReceived = nrf_get_payload_size();

    if ((bytesReceived != 5) && (bytesReceived != 7) && (bytesReceived != 8) && (bytesReceived != 10)) {
      nrf_stop_receiving();
      continue;
    }

    nrf_get_payload(recv_buf, bytesReceived);
    nrf_stop_receiving();

    // if (recv_buf[0] ==0) {
    //   continue;
    // }

    FLASH(30);

    tx_status = dg->tx_status;
    pkt_id = dg->pkt_id;
    bmp180_press = dg->bmp180_press + 700;
    bmp180_temp = dg->bmp180_temp;
    si7021_humi = dg->si7021_humi;
    si7021_temp = dg->si7021_temp;

    if (bytesReceived == 5) {
      bh1750 = 0;
    } else if (bytesReceived == 7) {
      bh1750 = *(uint16_t*) &recv_buf[5];
    } else {
      remote_vbat = dg->adc_vbat;
      remote_vcc = dg->adc_vcc + 160;
      remote_temp = dg->adc_temp;
      if (bytesReceived == 8) {
        bh1750 = 0;
      } else if (bytesReceived == 10) {
        bh1750 = dg->bh1750;
      }
    }
    
    if (bytesReceived > 10) {
      ext_code = dg->ext_code;
    }
    if (bytesReceived > 11) {
      aux = dg->aux;
    }

#ifdef SERIAL_DEBUG_ENABLE
    printf("#%u received %u bytes:", pkt_id, bytesReceived);
    printDatagram(bytesReceived);
    printf("\r\n");

    int siTempA = si7021_temp / 10;
    int siTempB = si7021_temp % 10;
    int bmpTempA = bmp180_temp / 10;
    int bmpTempB = bmp180_temp % 10;

    printf("T(a)= %dC, T(b)= %d.%dC, T(s)= %d.%dC, ", remote_temp, bmpTempA, bmpTempB, siTempA, siTempB);
    printf("L= %u Lx, P= %u mmHg, H= %u%%, ", bh1750, bmp180_press, si7021_humi);
    printf("Vbat= %u.%u, Vcc= %u.%u \r\n\r\n", remote_vbat / 100, remote_vbat % 100, remote_vcc / 100, remote_vcc % 100);
   // Status = %02X. \r\n", nrf_transmit((uint8_t *)&datagram, 10));
#endif

    lightSwitch <<= 1;
    if (bh1750 > 4) {
      lightSwitch |= 1;
    }

    uint8_t m = lightSwitch & 3;
    uint8_t m0 = lightSwitchPrev & 3;

    if ((m == 0) || (m == 3)) {
      if ((m == 0) && (m0 != 0)) {
        LOAD_ON();
      }
      if ((m == 3) && (m0 != 3)) {
        LOAD_OFF();
      }
    } else {
      BEEP(3);
    }

    lightSwitchPrev = lightSwitch;

    wdt_reset();
    // nrf_init(TX_CHANNEL);

    nrf_write_register(RF_CH, P0_CHANNEL);          // set radio channel
    uint32_t ts = (uint32_t) ((uint32_t) nrf_transmit_special(recv_buf, bytesReceived) << 16) | (prev_pkt_id << 4) | pkt_id;  // do acknowledged delivery
    prev_pkt_id = pkt_id;

    // if ((ts >> 12) != 0) {
    //   // ---
    // }

    nrf_write_register(RF_CH, ST_CHANNEL);          // set radio channel
    nrf_transmit((uint8_t*)&ts, sizeof(ts));

    // nrf_transmit(recv_buf, bytesReceived);
    // _delay_ms(2);
    // nrf_transmit(recv_buf, bytesReceived);
    // _delay_ms(3);
    // nrf_transmit(recv_buf, bytesReceived);

    nrf_broadcast(recv_buf, bytesReceived);

    nrf_init(RX_CHANNEL);
  } 
}

#ifdef SERIAL_DEBUG_ENABLE
static int uart_putchar(char c, FILE *stream) {
    if (c == '\n') uart_putchar('\r', stream);
  
    loop_until_bit_is_set(UCSRA, UDRE);
    UDR = c;
    
    return 0;
}

uint8_t uart_getchar(void) {
    while( !(UCSRA & (1<<RXC)) );
    return(UDR);
}
#endif
