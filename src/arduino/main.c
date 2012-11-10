#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>


/* SPI module */

static inline void spi_setup_master(void)
{
  /* doc8161.pdf, ch.18 */

  /* ss is used by avr spi to determine master */
  /* set output mode even if pb2 not used by us */
  DDRB |= (1 << 2);

  /* spi output pins: sck pb5, mosi pb3 */
  DDRB |= (1 << 5) | (1 << 3);

  /* spi input pins: miso pb4 */
  DDRB &= ~(1 << 4);
  /* disable pullup (already by default) */
  PORTB &= ~(1 << 4);

  /* enable spi, msb first, master, freq / 128 (125khz), sck low idle */
  SPCR = (1 << SPE) | (1 << MSTR) | (3 << SPR0) | (1 << CPOL);

  /* clear double speed */
  SPSR &= ~(1 << SPI2X);
}

static inline void spi_set_sck_freq(uint8_t x)
{
  /* x one of SPI_SCK_FREQ_FOSCX */
  /* where spi sck = fosc / X */
  /* see atmega328 specs, table 18.5 */
#define SPI_SCK_FREQ_FOSC2 ((1 << 2) | 0)
#define SPI_SCK_FREQ_FOSC4 ((0 << 2) | 0)
#define SPI_SCK_FREQ_FOSC8 ((1 << 2) | 1)
#define SPI_SCK_FREQ_FOSC16 ((0 << 2) | 1)
#define SPI_SCK_FREQ_FOSC32 ((1 << 2) | 2)
#define SPI_SCK_FREQ_FOSC64 ((0 << 2) | 2)
#define SPI_SCK_FREQ_FOSC128 ((0 << 2) | 3)

  SPCR &= ~(3 << SPR0);
  SPCR |= (x & 3) << SPR0;

  SPSR &= ~(1 << SPI2X);
  SPSR |= (((x >> 2) & 1) << SPI2X);
}

static inline void spi_write_uint8(uint8_t x)
{
  /* write the byte and wait for transmission */

#if 1 /* FIXME: needed for sd_read_block to work */
  __asm__ __volatile__ ("nop\n\t");
  __asm__ __volatile__ ("nop\n\t");
#endif

  SPDR = x;

#if 0
  if (SPSR & (1 << WCOL))
  {
#if 1
    PRINT_FAIL();
#else
    /* access SPDR */
    volatile uint8_t fubar = SPDR;
    __asm__ __volatile__ ("" :"=m"(fubar));
    goto redo;
#endif
  }
#endif

  while ((SPSR & (1 << SPIF)) == 0) ;
}

static void spi_write_uint16(uint16_t x)
{
  spi_write_uint8((x >> 8) & 0xff);
  spi_write_uint8((x >> 0) & 0xff);
}


/* reference: SLAS399A.pdf */

static inline void dac7554_sync_low(void)
{
#define DAC7554_SYNC_MASK (1 << 2)
  PORTB &= ~DAC7554_SYNC_MASK;
}

static inline void dac7554_sync_high(void)
{
  PORTB |= DAC7554_SYNC_MASK;
}

static void dac7554_setup(void)
{
  /* sync pin controled with PB0, 8 */
  DDRB |= DAC7554_SYNC_MASK;
  dac7554_sync_high();
}

static inline void dac7554_write(uint16_t data, uint16_t chan)
{
  /* assume spi initialized */

  /* refer to SLAS399A, p.14 table.1 for commands */

  /* assume: 0 <= data < 4096 */
  /* assume: 0 <= chan <= 3 */

  /* 2 to update both input and DAC registers */  
  const uint16_t cmd = (2 << 14) | (chan << 12) | data;

  /* pulse sync */
  dac7554_sync_high();
  __asm__ __volatile__ ("nop");
  __asm__ __volatile__ ("nop");
  dac7554_sync_low();

  /* wait for SCLK falling edge setup time, 4ns */
  __asm__ __volatile__ ("nop");
  __asm__ __volatile__ ("nop");

  spi_write_uint16(cmd);

  /* wait for SLCK to rise (0 ns) and set high */
  __asm__ __volatile__ ("nop");
  __asm__ __volatile__ ("nop");
  dac7554_sync_high();
}


/* main */

int main(void)
{
  /* assume: sclk <= 50mhz */
  spi_setup_master();
  spi_set_sck_freq(SPI_SCK_FREQ_FOSC2);

  dac7554_setup();

  while (1)
  {
    dac7554_write(4096 / 2, 0);
    dac7554_write(4096 / 8, 0);
  }

  return 0;
}
