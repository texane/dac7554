#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


/* SPI module */

void spi_init_master(void)
{
  /* arduino SPI is routed as following
     MOSI, PB3, 11
     SLCK, PB5, 13
   */

  /* set MOSI and SCK output, all others input */
  DDRB |= (1 << 3) | (1 << 5);

  /* enable SPI, Master, set clock rate fck / 2 = 8mhz */
  SPCR = (1 << SPE) | (1 << MSTR) | (4 << SPR0);
}

static inline void spi_write_uint8(uint8_t x)
{
  /* start transmission and wait for completion */
  SPDR = x;
  while ((SPSR & (1 << SPIF)) == 0) ;
}

static inline void spi_write_uint16(uint16_t x)
{
  spi_write_uint8((x >> 8) & 0xff);
  spi_write_uint8((x >> 0) & 0xff);
}


/* reference: SLAS399A.pdf */

static void dac7554_delay(void)
{
  volatile unsigned int i;
  for (i = 0; i < 1000; ++i) __asm__ __volatile__("nop");
}

static void dac7554_init(void)
{
  /* assume: sclk <= 50mhz */
  spi_init_master();

  /* sync pin controled with PB0, 8 */
  DDRB |= (1 << 0);
  PORTB &= ~(1 << 0);
}

static inline void dac7554_write(uint16_t data, uint16_t chan)
{
  /* refer to SLAS399A, p.14 table.1 for commands */

  /* assume: 0 <= data < 4096 */
  /* assume: 0 <= chan <= 3 */

  /* 2 to update both input and DAC registers */  
  const uint16_t cmd = (2 << 14) | (chan << 12) | data;

  /* set SYNC low */
  PORTB &= ~(1 << 0);

  /* wait for SCLK falling edge setup time, 4ns */
  __asm__ __volatile__ ("nop");

  spi_write_uint16(cmd);

  /* wait for SLCK to rise (0 ns) and set high */
  __asm__ __volatile__ ("nop");
  PORTB |= 1 << 0;
}


/* main */

int main(void)
{
  dac7554_init();
  dac7554_write(4096 / 2, 0);
  dac7554_write(4096 / 4, 1);
  dac7554_write(4096 / 8, 2);
  while (1) ;
  return 0;
}
