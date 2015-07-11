/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "hal.h"

#define RADIO_REG_DIOMAPPING2   (0x26)
#define RADIO_CLK_DIV1          (0x00)
#define RADIO_CLK_DIV2          (0x01)
#define RADIO_CLK_DIV4          (0x02)
#define RADIO_CLK_DIV8          (0x03)
#define RADIO_CLK_DIV16         (0x04)
#define RADIO_CLK_DIV32         (0x05)
#define RADIO_CLK_RC            (0x06)
#define RADIO_CLK_OFF           (0x07)

/*

The part that I have has the following pins exposed:
  PTA0
  PTA1
  PTA2
  PTA3
  PTA4
  PTA18
  PTA19
  PTA20
  PTB0
  PTB1
  PTB2
  PTB17
  PTC1
  PTC2
  PTC3
  PTC4
  PTC5 -- Bottom pad, also internal connection to transciever SCK -- SPI clock
  PTC6 -- Bottom pad, also internal connection to transciever MOSI -- SPI MISO
  PTC7 -- Bottom pad, also internal connection to transciever MISO -- SPI MOSI
  PTD0 -- Bottom pad, also internal connection to transciever NSS -- SPI CS
  PTD4
  PTD5
  PTD6
  PTD7
  PTE0
  PTE1
  PTE2 -- Internal connection to transciever GPIO DIO0 -- SPI1_SCK
  PTE3 -- Internal connection to transciever GPIO DIO1 -- SPI1_MOSI
  PTE16
  PTE17
  PTE18
  PTE19
  PTE30
*/

#if HAL_USE_PAL || defined(__DOXYGEN__)
/**
 * @brief   PAL setup.
 * @details Digital I/O ports static configuration as defined in @p board.h.
 *          This variable is used by the HAL when initializing the PAL driver.
 */
const PALConfig pal_default_config =
{
  .ports = {
    {
      .port = IOPORT1,  // PORTA
      .pads = {
#if ORCHARD_BOARD_REV == ORCHARD_REV_EVT1
        /* PTA0*/ PAL_MODE_ALTERNATIVE_7,   /* PTA1*/ PAL_MODE_ALTERNATIVE_3,   /* PTA2*/ PAL_MODE_ALTERNATIVE_3,
#else
        /* ECO10: allow fast Tx filling of radio packets by wiring D1 to PTD1 */
        /* PTA0*/ PAL_MODE_ALTERNATIVE_7,   /* PTA1*/ PAL_MODE_ALTERNATIVE_3,   /* PTA2*/ PAL_MODE_INPUT,
#endif
        /* PTA3*/ PAL_MODE_ALTERNATIVE_7,   /* PTA4*/ PAL_MODE_OUTPUT_PUSHPULL,   /* PTA5*/ PAL_MODE_UNCONNECTED,
        /* PTA6*/ PAL_MODE_UNCONNECTED,     /* PTA7*/ PAL_MODE_UNCONNECTED,     /* PTA8*/ PAL_MODE_UNCONNECTED,
        /* PTA9*/ PAL_MODE_UNCONNECTED,     /*PTA10*/ PAL_MODE_UNCONNECTED,     /*PTA11*/ PAL_MODE_UNCONNECTED,
        /*PTA12*/ PAL_MODE_UNCONNECTED,     /*PTA13*/ PAL_MODE_UNCONNECTED,     /*PTA14*/ PAL_MODE_UNCONNECTED,
        /*PTA15*/ PAL_MODE_UNCONNECTED,     /*PTA16*/ PAL_MODE_UNCONNECTED,     /*PTA17*/ PAL_MODE_UNCONNECTED,
        /*PTA18*/ PAL_MODE_INPUT_ANALOG,    /*PTA19*/ PAL_MODE_OUTPUT_PUSHPULL, /*PTA20*/ PAL_MODE_ALTERNATIVE_7,
        /*PTA21*/ PAL_MODE_UNCONNECTED,     /*PTA22*/ PAL_MODE_UNCONNECTED,     /*PTA23*/ PAL_MODE_UNCONNECTED,
        /*PTA24*/ PAL_MODE_UNCONNECTED,     /*PTA25*/ PAL_MODE_UNCONNECTED,     /*PTA26*/ PAL_MODE_UNCONNECTED,
        /*PTA27*/ PAL_MODE_UNCONNECTED,     /*PTA28*/ PAL_MODE_UNCONNECTED,     /*PTA29*/ PAL_MODE_UNCONNECTED,
        /*PTA30*/ PAL_MODE_UNCONNECTED,     /*PTA31*/ PAL_MODE_UNCONNECTED,
      },
    },
    {
      .port = IOPORT2,  // PORTB
      .pads = {
#if ORCHARD_BOARD_REV == ORCHARD_REV_EVT1
        /* PTB0*/ PAL_MODE_INPUT_PULLUP,    /* PTB1*/ PAL_MODE_ALTERNATIVE_3,   /* PTB2*/ PAL_MODE_ALTERNATIVE_3,
#else
  /* ECO9: Swap B0/D4 DC/INT to allow GPIOX to fire interrupts */
        /* PTB0*/ PAL_MODE_OUTPUT_PUSHPULL, /* PTB1*/ PAL_MODE_INPUT_ANALOG,   /* PTB2*/ PAL_MODE_INPUT_ANALOG,
#endif
        /* PTB3*/ PAL_MODE_UNCONNECTED,     /* PTB4*/ PAL_MODE_UNCONNECTED,     /* PTB5*/ PAL_MODE_UNCONNECTED,
        /* PTB6*/ PAL_MODE_UNCONNECTED,     /* PTB7*/ PAL_MODE_UNCONNECTED,     /* PTB8*/ PAL_MODE_UNCONNECTED,
        /* PTB9*/ PAL_MODE_UNCONNECTED,     /*PTB10*/ PAL_MODE_UNCONNECTED,     /*PTB11*/ PAL_MODE_UNCONNECTED,
        /*PTB12*/ PAL_MODE_UNCONNECTED,     /*PTB13*/ PAL_MODE_UNCONNECTED,     /*PTB14*/ PAL_MODE_UNCONNECTED,
        /*PTB15*/ PAL_MODE_UNCONNECTED,     /*PTB16*/ PAL_MODE_UNCONNECTED,     /*PTB17*/ PAL_MODE_ALTERNATIVE_3,
        /*PTB18*/ PAL_MODE_UNCONNECTED,     /*PTB19*/ PAL_MODE_UNCONNECTED,     /*PTB20*/ PAL_MODE_UNCONNECTED,
        /*PTB21*/ PAL_MODE_UNCONNECTED,     /*PTB22*/ PAL_MODE_UNCONNECTED,     /*PTB23*/ PAL_MODE_UNCONNECTED,
        /*PTB24*/ PAL_MODE_UNCONNECTED,     /*PTB25*/ PAL_MODE_UNCONNECTED,     /*PTB26*/ PAL_MODE_UNCONNECTED,
        /*PTB27*/ PAL_MODE_UNCONNECTED,     /*PTB28*/ PAL_MODE_UNCONNECTED,     /*PTB29*/ PAL_MODE_UNCONNECTED,
        /*PTB30*/ PAL_MODE_UNCONNECTED,     /*PTB31*/ PAL_MODE_UNCONNECTED,
      },
    },
    {
      .port = IOPORT3,  // PORTC
      .pads = {
        /* PTC0*/ PAL_MODE_INPUT_ANALOG,    /* PTC1*/ PAL_MODE_ALTERNATIVE_2,   /* PTC2*/ PAL_MODE_ALTERNATIVE_2,
        /* PTC3*/ PAL_MODE_INPUT,           /* PTC4*/ PAL_MODE_INPUT,           /* PTC5*/ PAL_MODE_ALTERNATIVE_2,
        /* PTC6*/ PAL_MODE_ALTERNATIVE_2,   /* PTC7*/ PAL_MODE_ALTERNATIVE_2,   /* PTC8*/ PAL_MODE_UNCONNECTED,
        /* PTC9*/ PAL_MODE_UNCONNECTED,     /*PTC10*/ PAL_MODE_UNCONNECTED,     /*PTC11*/ PAL_MODE_UNCONNECTED,
        /*PTC12*/ PAL_MODE_UNCONNECTED,     /*PTC13*/ PAL_MODE_UNCONNECTED,     /*PTC14*/ PAL_MODE_UNCONNECTED,
        /*PTC15*/ PAL_MODE_UNCONNECTED,     /*PTC16*/ PAL_MODE_UNCONNECTED,     /*PTC17*/ PAL_MODE_UNCONNECTED,
        /*PTC18*/ PAL_MODE_UNCONNECTED,     /*PTC19*/ PAL_MODE_UNCONNECTED,     /*PTC20*/ PAL_MODE_UNCONNECTED,
        /*PTC21*/ PAL_MODE_UNCONNECTED,     /*PTC22*/ PAL_MODE_UNCONNECTED,     /*PTC23*/ PAL_MODE_UNCONNECTED,
        /*PTC24*/ PAL_MODE_UNCONNECTED,     /*PTC25*/ PAL_MODE_UNCONNECTED,     /*PTC26*/ PAL_MODE_UNCONNECTED,
        /*PTC27*/ PAL_MODE_UNCONNECTED,     /*PTC28*/ PAL_MODE_UNCONNECTED,     /*PTC29*/ PAL_MODE_UNCONNECTED,
        /*PTC30*/ PAL_MODE_UNCONNECTED,     /*PTC31*/ PAL_MODE_UNCONNECTED,
      },
    },
    {
      .port = IOPORT4,  // PORTD
      .pads = {
	/* PTD0*/ PAL_MODE_OUTPUT_PUSHPULL, /* PTD1*/ PAL_MODE_UNCONNECTED,     /* PTD2*/ PAL_MODE_UNCONNECTED,
#if ORCHARD_BOARD_REV == ORCHARD_REV_EVT1
        /* PTD3*/ PAL_MODE_UNCONNECTED,     /* PTD4*/ PAL_MODE_OUTPUT_PUSHPULL, /* PTD5*/ PAL_MODE_ALTERNATIVE_2,
#else
  /* ECO9: Swap B0/D4 DC/INT to allow GPIOX to fire interrupts */
        /* PTD3*/ PAL_MODE_UNCONNECTED,     /* PTD4*/ PAL_MODE_INPUT_PULLUP,    /* PTD5*/ PAL_MODE_ALTERNATIVE_2,
#endif
        /* PTD6*/ PAL_MODE_ALTERNATIVE_3,   /* PTD7*/ PAL_MODE_ALTERNATIVE_3,   /* PTD8*/ PAL_MODE_UNCONNECTED,
        /* PTD9*/ PAL_MODE_UNCONNECTED,     /*PTD10*/ PAL_MODE_UNCONNECTED,     /*PTD11*/ PAL_MODE_UNCONNECTED,
        /*PTD12*/ PAL_MODE_UNCONNECTED,     /*PTD13*/ PAL_MODE_UNCONNECTED,     /*PTD14*/ PAL_MODE_UNCONNECTED,
        /*PTD15*/ PAL_MODE_UNCONNECTED,     /*PTD16*/ PAL_MODE_UNCONNECTED,     /*PTD17*/ PAL_MODE_UNCONNECTED,
        /*PTD18*/ PAL_MODE_UNCONNECTED,     /*PTD19*/ PAL_MODE_UNCONNECTED,     /*PTD20*/ PAL_MODE_UNCONNECTED,
        /*PTD21*/ PAL_MODE_UNCONNECTED,     /*PTD22*/ PAL_MODE_UNCONNECTED,     /*PTD23*/ PAL_MODE_UNCONNECTED,
        /*PTD24*/ PAL_MODE_UNCONNECTED,     /*PTD25*/ PAL_MODE_UNCONNECTED,     /*PTD26*/ PAL_MODE_UNCONNECTED,
        /*PTD27*/ PAL_MODE_UNCONNECTED,     /*PTD28*/ PAL_MODE_UNCONNECTED,     /*PTD29*/ PAL_MODE_UNCONNECTED,
        /*PTD30*/ PAL_MODE_UNCONNECTED,     /*PTD31*/ PAL_MODE_UNCONNECTED,
      },
    },
    {
      .port = IOPORT5,  // PORTE
      .pads = {
        /* PTE0*/ PAL_MODE_ALTERNATIVE_2,   /* PTE1*/ PAL_MODE_ALTERNATIVE_2,   /* PTE2*/ PAL_MODE_ALTERNATIVE_2,
        /* PTE3*/ PAL_MODE_ALTERNATIVE_2,   /* PTE4*/ PAL_MODE_UNCONNECTED,     /* PTE5*/ PAL_MODE_UNCONNECTED,
        /* PTE6*/ PAL_MODE_UNCONNECTED,     /* PTE7*/ PAL_MODE_UNCONNECTED,     /* PTE8*/ PAL_MODE_UNCONNECTED,
        /* PTE9*/ PAL_MODE_UNCONNECTED,     /*PTE10*/ PAL_MODE_UNCONNECTED,     /*PTE11*/ PAL_MODE_UNCONNECTED,
        /*PTE12*/ PAL_MODE_UNCONNECTED,     /*PTE13*/ PAL_MODE_UNCONNECTED,     /*PTE14*/ PAL_MODE_UNCONNECTED,
        /*PTE15*/ PAL_MODE_UNCONNECTED,     /*PTE16*/ PAL_MODE_INPUT_ANALOG,    /*PTE17*/ PAL_MODE_OUTPUT_PUSHPULL,
#if ORCHARD_BOARD_REV == ORCHARD_REV_EVT1
        /*PTE18*/ PAL_MODE_OUTPUT_PUSHPULL, /*PTE19*/ PAL_MODE_INPUT_ANALOG,    /*PTE20*/ PAL_MODE_UNCONNECTED,
#else
        /* ECO7: E19 RESET/DAC swap  */
        /*PTE18*/ PAL_MODE_OUTPUT_PUSHPULL, /*PTE19*/ PAL_MODE_OUTPUT_PUSHPULL, /*PTE20*/ PAL_MODE_UNCONNECTED,
#endif
        /*PTE21*/ PAL_MODE_UNCONNECTED,     /*PTE22*/ PAL_MODE_UNCONNECTED,     /*PTE23*/ PAL_MODE_UNCONNECTED,
        /*PTE24*/ PAL_MODE_UNCONNECTED,     /*PTE25*/ PAL_MODE_UNCONNECTED,     /*PTE26*/ PAL_MODE_UNCONNECTED,
        /*PTE27*/ PAL_MODE_UNCONNECTED,     /*PTE28*/ PAL_MODE_UNCONNECTED,     /*PTE29*/ PAL_MODE_UNCONNECTED,
#if ORCHARD_BOARD_REV == ORCHARD_REV_EVT1
        /*PTE30*/ PAL_MODE_OUTPUT_PUSHPULL, /*PTE31*/ PAL_MODE_UNCONNECTED,
#else
        /* ECO7: E30 DAC/RESET swap */
        /*PTE30*/ PAL_MODE_INPUT_ANALOG,    /*PTE31*/ PAL_MODE_UNCONNECTED,
#endif
      },
    },
  },
};
#endif

static void radio_reset(void)
{
#if ORCHARD_BOARD_REV == ORCHARD_REV_EVT1
  GPIOE->PSOR = (1 << 30);
#else
  GPIOE->PSOR = (1 << 19);
#endif
}

static void radio_enable(void)
{
#if ORCHARD_BOARD_REV == ORCHARD_REV_EVT1
  GPIOE->PCOR = (1 << 30);
#else
  GPIOE->PCOR = (1 << 19);
#endif
}

static void assert_cs(void)
{
  GPIOD->PCOR = (1 << 0);
}

static void deassert_cs(void)
{
  GPIOD->PSOR = (1 << 0);
}

static void spi_read_status(void)
{
  (void)SPI0->S;
}

/**
 * @brief   Send a byte and discard the response
 *
 * @notapi
 */
void spi_xmit_byte_sync(uint8_t byte)
{
  /* Send the byte */
  SPI0->DL = byte;

  /* Wait for the byte to be transmitted */
  while (!(SPI0->S & SPIx_S_SPTEF))
    asm("");

  /* Discard the response */
  (void)SPI0->DL;
}

/**
 * @brief   Send a dummy byte and return the response
 *
 * @return              value read from said register
 *
 * @notapi
 */
uint8_t spi_recv_byte_sync(void)
{
  /* Send the byte */
  SPI0->DL = 0;

  /* Wait for the byte to be transmitted */
  while (!(SPI0->S & SPIx_S_SPRF))
    asm("");

  /* Discard the response */
  return SPI0->DL;
}


/**
 * @brief   Read a value from a specified radio register
 *
 * @param[in] addr      radio register to read from
 * @return              value read from said register
 *
 * @notapi
 */
uint8_t radio_read_register(uint8_t addr) {
  uint8_t val;

  spi_read_status();
  assert_cs();

  spi_xmit_byte_sync(addr);
  spi_recv_byte_sync();
  val = spi_recv_byte_sync();

  deassert_cs();

  return val;
}

/**
 * @brief   Write a value to a specified radio register
 *
 * @param[in] addr      radio register to write to
 * @param[in] val       value to write to said register
 *
 * @notapi
 */
void radio_write_register(uint8_t addr, uint8_t val) {

  spi_read_status();
  assert_cs();
  /* Send the address to write */
  spi_xmit_byte_sync(addr | 0x80);

  /* Send the actual value */
  spi_xmit_byte_sync(val);

  (void)spi_recv_byte_sync();
  deassert_cs();
}

static void early_usleep(int usec) {
  int j, k;

  for (j = 0; j < usec; j++)
    for (k = 0; k < 30; k++)
        asm("");
}

static void early_msleep(int msec) {
  int i, j, k;

  for (i = 0; i < msec; i++)
    for (j = 0; j < 1000; j++)
      for (k = 0; k < 30; k++)
        asm("");
}

/**
 * @brief   Power cycle the radio
 * @details Put the radio into reset, then wait 100 us.  Bring it out of
 *          reset, then wait 5 ms.  Ish.
 */
int radio_power_cycle(void)
{
  /* @AC RESET sequence from SX1233 datasheet:
   * RESET high Z
   * RESET = 1, then wait 100us
   * RESET = 0 (high Z), then wait 5ms before using the Radio. 
   */

  radio_reset();

  /* Cheesy usleep(100).*/
  early_usleep(100);

  radio_enable();

  /* Cheesy msleep(5).*/
  early_msleep(5);

  return 1;
}

/**
 * @brief   Set up mux ports, enable SPI, and set up GPIO.
 * @details The MCU communicates to the radio via SPI and a few GPIO lines.
 *          Set up the pinmux for these GPIO lines, and set up SPI.
 */
void early_init_radio(void)
{

  /* Enable Reset GPIO and SPI PORT clocks by unblocking
   * PORTC, PORTD, and PORTE.*/
  SIM->SCGC5 |= (SIM_SCGC5_PORTC | SIM_SCGC5_PORTD | SIM_SCGC5_PORTE);

  /* Map Reset to a GPIO, which is looped from PTE30 back into
     the RESET_B_XCVR port.*/
#if ORCHARD_BOARD_REV == ORCHARD_REV_EVT1
  PORTE->PCR[30] &= PORTx_PCRn_MUX_MASK;
  PORTE->PCR[30] |= PORTx_PCRn_MUX(1);
  GPIOE->PDDR |= (1 << 30);
#else
  PORTE->PCR[19] &= PORTx_PCRn_MUX_MASK;
  PORTE->PCR[19] |= PORTx_PCRn_MUX(1);
  GPIOE->PDDR |= (1 << 19);
#endif

  /* Enable SPI clock.*/
  SIM->SCGC4 |= SIM_SCGC4_SPI0;

  /* Mux PTD0 as a GPIO, since it's used for Chip Select.*/
  PORTD->PCR[0] &= PORTx_PCRn_MUX_MASK;
  PORTD->PCR[0] |= PORTx_PCRn_MUX(1);
  GPIOD->PDDR |= (1 << 0);

  /* Mux PTC5 as SCK */
  PORTC->PCR[5] &= PORTx_PCRn_MUX_MASK;
  PORTC->PCR[5] |= PORTx_PCRn_MUX(2);

  /* Mux PTC6 as MISO */
  PORTC->PCR[6] &= PORTx_PCRn_MUX_MASK;
  PORTC->PCR[6] |= PORTx_PCRn_MUX(2);

  /* Mux PTC7 as MOSI */
  PORTC->PCR[7] &= PORTx_PCRn_MUX_MASK;
  PORTC->PCR[7] |= PORTx_PCRn_MUX(2);

  /* Keep the radio in reset.*/
  radio_reset();

  /* Initialize the SPI peripheral default values.*/
  SPI0->C1 = 0;
  SPI0->C2 = 0;
  SPI0->BR = 0;

  /* Enable SPI system, and run as a Master.*/
  SPI0->C1 |= (SPIx_C1_SPE | SPIx_C1_MSTR);

  spi_read_status();
  deassert_cs();
}



/**
 * @brief   Configure the radio to output a given clock frequency
 *
 * @param[in] osc_div   a factor of division, of the form 2^n
 *
 * @notapi
 */
static void radio_configure_clko(uint8_t osc_div)
{
  radio_write_register(RADIO_REG_DIOMAPPING2,
      (radio_read_register(RADIO_REG_DIOMAPPING2) & ~7) | osc_div);
}

#define SCL_PIN (1 << 1)
#define SDA_PIN (1 << 2)

static void set_scl(void) {
  FGPIOC->PSOR = SCL_PIN;
}

static void clr_scl(void) {
  FGPIOC->PCOR = SCL_PIN;
}

static void set_sda(void) {
  FGPIOC->PSOR = SDA_PIN;
}

static void clr_sda(void) {
  FGPIOC->PCOR = SDA_PIN;
}

/**
 * @brief   Get I2C into a known-good state
 * @details If the MCU is reset when I2C is in the middle of a transaction,
 *          the I2C slaves may hold the line low waiting for the clock line
 *          to transition.
 *          It is possible to "unblock" the bus by jamming at least 9 high
 *          pulses onto the line while toggling SCL.  This will cause the
 *          slave device to NAK when it's finished, thus freeing the bus.
 *
 * @notapi
 */
static void orchard_i2c_deblock(void) {
  int i;

  /* Allow access to PORTC */
  SIM->SCGC5 |= SIM_SCGC5_PORTC;

  /* Deblock the I2C bus, in case we were reset during a transaction.
   * Set the pins as GPIOs and toggle the clock line 9 times.
   */
  PORTC->PCR[1] = PORTx_PCRn_MUX(1) | PORTx_PCRn_PE | PORTx_PCRn_PS;
  PORTC->PCR[2] = PORTx_PCRn_MUX(1) | PORTx_PCRn_PE | PORTx_PCRn_PS;

  /* Set SCL and SDA as output GPIOs */
  FGPIOC->PDDR |= SCL_PIN | SDA_PIN;

  FGPIOC->PSOR = SCL_PIN | SDA_PIN;

  /* Send 9 clocks with the SDA line high */
  for (i = 0; i < 9; i++) {
    clr_scl();
    early_usleep(15);
    set_scl();
    early_usleep(15);
  }

  /* Send the STOP bit */
  clr_sda();
  early_usleep(15);
  set_scl();
  set_sda();
  early_usleep(15);
}

/**
 * @brief   Early initialization code.
 * @details This initialization must be performed just after stack setup
 *          and before any other initialization.
 */
void __early_init(void) {

  orchard_i2c_deblock();
  early_init_radio();

  radio_power_cycle();

  /* 32Mhz/4 = 8 MHz CLKOUT.*/
  radio_configure_clko(RADIO_CLK_DIV1);

  kl1x_clock_init();
}

/**
 * @brief   Board-specific initialization code.
 * @todo    Add your board-specific code, if any.
 */
void boardInit(void) {
}
