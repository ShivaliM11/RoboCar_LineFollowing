#include <stdint.h>
#include "./include/pwm.h"
#include "./include/io_peripherals.h"
void pwm_setup(int pwm_range, struct io_peripherals *io)
{
    io->pwm->RNG1 = pwm_range;
    io->pwm->RNG2 = pwm_range;
    io->pwm->DAT1 = 1;
    io->pwm->DAT2 = 1;
    io->pwm->CTL.field.MODE1 = 0;
    io->pwm->CTL.field.MODE2 = 0;
    io->pwm->CTL.field.RPTL1 = 1;
    io->pwm->CTL.field.RPTL2 = 1;
    io->pwm->CTL.field.SBIT1 = 0;
    io->pwm->CTL.field.SBIT2 = 0;
    io->pwm->CTL.field.POLA1 = 0;
    io->pwm->CTL.field.POLA2 = 0;
    io->pwm->CTL.field.USEF1 = 0;
    io->pwm->CTL.field.USEF2 = 0;
    io->pwm->CTL.field.MSEN1 = 1;
    io->pwm->CTL.field.MSEN2 = 1;
    io->pwm->CTL.field.CLRF1 = 1;
    io->pwm->CTL.field.PWEN1 = 1;
    io->pwm->CTL.field.PWEN2 = 1;
}
