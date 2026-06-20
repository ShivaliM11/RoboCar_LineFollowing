#include <stdio.h>
#include <unistd.h>
#include "/home/sh/cmpen473/hw6/include/import_registers.h"
#include "/home/sh/cmpen473/hw6/include/gpio.h"
#include "/home/sh/cmpen473/hw6/include/io_peripherals.h"

#define IR_LEFT_PIN    24
#define IR_RIGHT_PIN   25
#define IR_CENTER_PIN  18

int main(void)
{
  struct io_peripherals *io = import_registers();
  if (!io) { printf("failed to map registers\n"); return 1; }

  io->gpio->GPFSEL2.field.FSEL4 = 0;  // GPIO 24 = left   IR = input
  io->gpio->GPFSEL2.field.FSEL5 = 0;  // GPIO 25 = right  IR = input
  io->gpio->GPFSEL1.field.FSEL8 = 0;  // GPIO 18 = center IR = input

  printf("=== IR Sensor Tester ===\n");
  printf("1=black  0=white\n");
  printf("Ctrl+C to stop\n\n");

  int prev_c = -1, prev_l = -1, prev_r = -1;

  while (1)
  {
    int c = (GPIO_READ(io->gpio, IR_CENTER_PIN) != 0) ? 1 : 0;
    int l = (GPIO_READ(io->gpio, IR_LEFT_PIN)   != 0) ? 1 : 0;
    int r = (GPIO_READ(io->gpio, IR_RIGHT_PIN)  != 0) ? 1 : 0;

    // only print when something changes so output is readable
    if (c != prev_c || l != prev_l || r != prev_r)
    {
      printf("CENTER=%d  LEFT=%d  RIGHT=%d", c, l, r);

      // also print what it means
      if (c && !l && !r)       printf("  -> straight (center on line)");
      else if (!c && l && !r)  printf("  -> drifted right (left on line)");
      else if (!c && !l && r)  printf("  -> drifted left  (right on line)");
      else if (c && l && !r)   printf("  -> sharp left curve");
      else if (c && !l && r)   printf("  -> sharp right curve");
      else if (!c && !l && !r) printf("  -> all white (off line)");
      else if (c && l && r)    printf("  -> all black");
      printf("\n");
      fflush(stdout);

      prev_c = c; prev_l = l; prev_r = r;
    }
    usleep(50000);  // 50ms
  }
  return 0;
}
