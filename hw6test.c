#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include <termios.h>
#include <fcntl.h>
#include <pthread.h>
#include "../include/import_registers.h"
#include "../include/cm.h"
#include "../include/gpio.h"
#include "../include/uart.h"
#include "../include/spi.h"
#include "../include/bsc.h"
#include "../include/pwm.h"
#include "../include/enable_pwm_clock.h"
#include "../include/io_peripherals.h"
#include "../include/wait_period.h"
#include "../include/FIFO.h"
#include "../include/MPU6050.h"
#include "../include/MPU9250.h"
#include "../include/wait_key.h"
#include "keypress.h"



void main(void) {
	struct io_peripherals *io;

	io = import_registers();
	if (io != NULL) {
		io->gpio->GPFSEL2.field.FSEL4 = GPFSEL_OUTPUT; // GPIO24
	}
	
	int left_sensor_val = (io->gpio->GPLEV0 >> 24) & 1;
	printf("%d\n", left_sensor_val);
	
	
}
