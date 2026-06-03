// Homework 4 - RoboCar Motor Control : Shivali Munshi
//files used form hw0 = 
// hw0c6fifo3cont.c - thread/FIFO structure
// hw0c1blink97forward.c - GPIO direction logic
// hw0c5pwm1dim.c  - hardware PWM setup
// 4 threads: KeyRead -> Control -> LeftMotor + RightMotor
// 3 FIFOs:   key_fifo, left_fifo, right_fifo

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
#include "pwmsetup.h"

#define FIFO_LENGTH  1024
#define PWM_RANGE    100
#define SPEED_INIT    50
#define SPEED_STEP     5
#define SPEED_MAX    100
#define SPEED_MIN      0
#define RAMP_STEPS    10

// FIFO message: command + argument (speed value)
struct thread_command { uint8_t command; uint8_t argument; };
FIFO_TYPE(struct thread_command, FIFO_LENGTH, fifo_t);

// KeyRead param
struct key_param {
  struct fifo_t * key_fifo;
  bool          * quit_flag;
};

// Control param
struct control_param {
  struct fifo_t * key_fifo;
  struct fifo_t * left_fifo;
  struct fifo_t * right_fifo;
  bool          * quit_flag;
};

// Motor thread param — handles both direction & speed for one motor
struct motor_param {
  const char                    * name;
  volatile struct gpio_register * gpio;
  struct io_peripherals         * io;
  int                             pin1;     // AI1 or BI1
  int                             pin2;     // AI2 or BI2
  int                             pwm_ch;   // 1=DAT1(left) 2=DAT2(right)
  struct fifo_t                 * fifo;
  bool                          * quit_flag;
};

// helper: insert same command into both motor FIFOs
static void send_both(struct fifo_t *lf, struct fifo_t *rf, uint8_t cmd, uint8_t arg)
{
  struct thread_command c = {cmd, arg};
  if (!(FIFO_FULL(lf))) FIFO_INSERT(lf, c);
  if (!(FIFO_FULL(rf))) FIFO_INSERT(rf, c);
}

// THREAD 1: KeyRead — reads key, puts in key_fifo
void *KeyRead(void *arg)
{
  struct key_param *p = (struct key_param *)arg;
  struct thread_command cmd = {0, 0};
  int key;
  struct timespec ts;
  wait_period_initialize(&ts);
  wait_period(&ts, 10u);

  while (!*(p->quit_flag))
  {
    key = get_pressed_key();
    if (key != -1 && key != 10)  // ignore -1 and Enter
    {
      printf("HW4> %c\n", (char)key);
      fflush(stdout);
      cmd.command = (uint8_t)key;
      cmd.argument = 0;
      if (!(FIFO_FULL(p->key_fifo))) FIFO_INSERT(p->key_fifo, cmd);
      if (key == 113) *p->quit_flag = true;  // 'q'
    }
    wait_period(&ts, 10u);
  }
  printf("KeyRead done\n");
  return NULL;
}

// THREAD 2: Control — reads key_fifo, sends commands to motor FIFOs
// 'w'=forward 'b'=backward 's'=stop 'v'=set speed 'q'=quit
void *Control(void *arg)
{
  struct control_param *p = (struct control_param *)arg;
  struct thread_command cmd = {0, 0};
  struct timespec ts;
  int cur_speed = SPEED_INIT;
  int cur_dir   = 0;   // 0=stop 1=fwd -1=bwd
  int step, ramp;

  wait_period_initialize(&ts);
  wait_period(&ts, 10u);

  while (!*(p->quit_flag))
  {
    if (!(FIFO_EMPTY(p->key_fifo)))
    {
      FIFO_REMOVE(p->key_fifo, &cmd);

      switch (cmd.command)
      {
        case 115:  // 's' = stop // ASCII VAlues 
          send_both(p->left_fifo, p->right_fifo, 'v', 0);   // speed 0
          send_both(p->left_fifo, p->right_fifo, 's', 0);   // stop direction
          cur_speed = 0; cur_dir = 0;
          printf(" STOP\n");
        break;

        case 119:  // 'w' = forward
          if (cur_dir == -1)  // ramp down from backward
          {
            for (step = RAMP_STEPS; step >= 0; step--)
            {
              ramp = cur_speed * step / RAMP_STEPS;
              send_both(p->left_fifo, p->right_fifo, 'v', (uint8_t)ramp);
              wait_period(&ts, 10u);
            }
            send_both(p->left_fifo, p->right_fifo, 's', 0);
            wait_period(&ts, 10u);
          }
          send_both(p->left_fifo, p->right_fifo, 'f', 0);   // forward direction
          wait_period(&ts, 10u);
          if (cur_dir == -1)  // ramp up
          {
            for (step = 0; step <= RAMP_STEPS; step++)
            {
              ramp = cur_speed * step / RAMP_STEPS;
              send_both(p->left_fifo, p->right_fifo, 'v', (uint8_t)ramp);
              wait_period(&ts, 10u);
            }
          }
          else
          {
            if (cur_speed == 0) cur_speed = SPEED_INIT;
            send_both(p->left_fifo, p->right_fifo, 'v', (uint8_t)cur_speed);
          }
          cur_dir = 1;
          printf(" FORWARD %d%%\n", cur_speed);
        break;

        case 120:  // 'x' = backward
          if (cur_dir == 1)  // ramp down from forward
          {
            for (step = RAMP_STEPS; step >= 0; step--)
            {
              ramp = cur_speed * step / RAMP_STEPS;
              send_both(p->left_fifo, p->right_fifo, 'v', (uint8_t)ramp);
              wait_period(&ts, 10u);
            }
            send_both(p->left_fifo, p->right_fifo, 's', 0);
            wait_period(&ts, 10u);
          }
          send_both(p->left_fifo, p->right_fifo, 'b', 0);   // backward direction
          wait_period(&ts, 10u);
          if (cur_dir == 1)  // ramp up
          {
            for (step = 0; step <= RAMP_STEPS; step++)
            {
              ramp = cur_speed * step / RAMP_STEPS;
              send_both(p->left_fifo, p->right_fifo, 'v', (uint8_t)ramp);
              wait_period(&ts, 10u);
            }
          }
          else
          {
            if (cur_speed == 0) cur_speed = SPEED_INIT;
            send_both(p->left_fifo, p->right_fifo, 'v', (uint8_t)cur_speed);
          }
          cur_dir = -1;
          printf(" BACKWARD %d%%\n", cur_speed);
        break;

        case 105:  // 'i' = faster
          cur_speed += SPEED_STEP;
          if (cur_speed > SPEED_MAX) cur_speed = SPEED_MAX;
          if (cur_dir != 0) send_both(p->left_fifo, p->right_fifo, 'v', (uint8_t)cur_speed);
          printf(" FASTER %d%%\n", cur_speed);
        break;

        case 106:  // 'j' = slower
          cur_speed -= SPEED_STEP;
          if (cur_speed < SPEED_MIN) cur_speed = SPEED_MIN;
          send_both(p->left_fifo, p->right_fifo, 'v', (uint8_t)cur_speed);
          if (cur_speed == 0) cur_dir = 0;
          printf(" SLOWER %d%%\n", cur_speed);
        break;

        case 113:  // 'q' = quit
          send_both(p->left_fifo, p->right_fifo, 'v', 0);
          send_both(p->left_fifo, p->right_fifo, 'q', 0);
          printf(" QUIT\n");
        break;
      }
    }
    wait_period(&ts, 10u);
  }
  printf("Control done\n");
  return NULL;
}

// THREAD 3 & 4: MotorThread — one thread per motor (left and right)
// handles BOTH direction (GPIO pins) AND speed (DAT1 or DAT2)
// f=forward, b=backward , s=stop, v=set speed , q=quit
void *MotorThread(void *arg)
{
  struct motor_param *p = (struct motor_param *)arg;
  struct thread_command cmd = {0, 0};
  struct timespec ts;
  wait_period_initialize(&ts);
  wait_period(&ts, 10u);

  while (!*(p->quit_flag))
  {
    if (!(FIFO_EMPTY(p->fifo)))
    {
      FIFO_REMOVE(p->fifo, &cmd);

      switch (cmd.command)
      {
        case 'f':  // forward: pin1=CLR pin2=SET (from hw0c1blink97forward.c)
          GPIO_CLR(p->gpio, p->pin1);
          GPIO_SET(p->gpio, p->pin2);
        break;

        case 'b':  // backward: pin1=SET pin2=CLR (from hw0c1blink98backward.c)
          GPIO_SET(p->gpio, p->pin1);
          GPIO_CLR(p->gpio, p->pin2);
        break;

        case 's':  // stop: both LOW (from hw0c1blink96stop.c)
          GPIO_CLR(p->gpio, p->pin1);
          GPIO_CLR(p->gpio, p->pin2);
        break;

        case 'v':  // set speed: write to DAT1 or DAT2
          if (p->pwm_ch == 1) p->io->pwm->DAT1 = (uint32_t)cmd.argument;
          else                p->io->pwm->DAT2 = (uint32_t)cmd.argument;
        break;

        case 'q':  // quit: stop everything
          GPIO_CLR(p->gpio, p->pin1);
          GPIO_CLR(p->gpio, p->pin2);
          if (p->pwm_ch == 1) p->io->pwm->DAT1 = 0;
          else                p->io->pwm->DAT2 = 0;
        break;
      }
    }
    wait_period(&ts, 10u);
  }
  printf("%s done\n", p->name);
  return NULL;
}

// main()
int main(void)
{
  struct io_peripherals *io;
  pthread_t tk, tc, tl, tr;

  struct fifo_t key_fifo   = {{}, 0, 0, PTHREAD_MUTEX_INITIALIZER};
  struct fifo_t left_fifo  = {{}, 0, 0, PTHREAD_MUTEX_INITIALIZER};
  struct fifo_t right_fifo = {{}, 0, 0, PTHREAD_MUTEX_INITIALIZER};
  bool quit_flag = false;

  struct key_param     kp = {&key_fifo, &quit_flag};
  struct control_param cp = {&key_fifo, &left_fifo, &right_fifo, &quit_flag};
  struct motor_param   lp = {"Lmotor", NULL, NULL, 5,  6,  1, &left_fifo,  &quit_flag};
  struct motor_param   rp = {"Rmotor", NULL, NULL, 22, 23, 2, &right_fifo, &quit_flag};

  io = import_registers();
  if (io != NULL)
  {
    printf("mem at 0x%8.8X\n", (unsigned int)io);

    enable_pwm_clock(io->cm, io->pwm);

    // GPIO12/13 = hardware PWM (from hw0c5pwm1dim.c)
    io->gpio->GPFSEL1.field.FSEL2 = GPFSEL_ALTERNATE_FUNCTION0;
    io->gpio->GPFSEL1.field.FSEL3 = GPFSEL_ALTERNATE_FUNCTION0;

    // GPIO05/06/22/23 = OUTPUT for direction
    io->gpio->GPFSEL0.field.FSEL5 = GPFSEL_OUTPUT;
    io->gpio->GPFSEL0.field.FSEL6 = GPFSEL_OUTPUT;
    io->gpio->GPFSEL2.field.FSEL2 = GPFSEL_OUTPUT;
    io->gpio->GPFSEL2.field.FSEL3 = GPFSEL_OUTPUT;

    // all pins start LOW
    GPIO_CLR(io->gpio, 5);  GPIO_CLR(io->gpio, 6);
    GPIO_CLR(io->gpio, 22); GPIO_CLR(io->gpio, 23);

    // PWM setup (from hw0c5pwm1dim.c)
    io->pwm->RNG1 = PWM_RANGE; io->pwm->RNG2 = PWM_RANGE;
    io->pwm->DAT1 = 0;         io->pwm->DAT2 = 0;
    io->pwm->CTL.field.MODE1 = 0; io->pwm->CTL.field.MODE2 = 0;
    io->pwm->CTL.field.RPTL1 = 1; io->pwm->CTL.field.RPTL2 = 1;
    io->pwm->CTL.field.SBIT1 = 0; io->pwm->CTL.field.SBIT2 = 0;
    io->pwm->CTL.field.POLA1 = 0; io->pwm->CTL.field.POLA2 = 0;
    io->pwm->CTL.field.USEF1 = 0; io->pwm->CTL.field.USEF2 = 0;
    io->pwm->CTL.field.MSEN1 = 1; io->pwm->CTL.field.MSEN2 = 1;
    io->pwm->CTL.field.CLRF1 = 1;
    io->pwm->CTL.field.PWEN1 = 1; io->pwm->CTL.field.PWEN2 = 1;

    lp.gpio = io->gpio; lp.io = io;
    rp.gpio = io->gpio; rp.io = io;

    printf("\n HW4 RoboCar = Tests Forwards/Backward & Speed \n");
    printf("  w=forward  x=backward  s=stop\n");
    printf("  i=faster   j=slower    q=quit\n\n");
    printf("  Lift wheels before testing\n\n");

    pthread_create(&tl, NULL, MotorThread, (void *)&lp);
    pthread_create(&tr, NULL, MotorThread, (void *)&rp);
    pthread_create(&tk, NULL, KeyRead,     (void *)&kp);
    pthread_create(&tc, NULL, Control,     (void *)&cp);

    pthread_join(tl, NULL);
    pthread_join(tr, NULL);
    pthread_join(tk, NULL);
    pthread_join(tc, NULL);

    // cleanup
    io->pwm->DAT1 = 0; io->pwm->DAT2 = 0;
    GPIO_CLR(io->gpio, 5);  GPIO_CLR(io->gpio, 6);
    GPIO_CLR(io->gpio, 22); GPIO_CLR(io->gpio, 23);
    io->gpio->GPFSEL1.field.FSEL2 = GPFSEL_INPUT;
    io->gpio->GPFSEL1.field.FSEL3 = GPFSEL_INPUT;
    io->gpio->GPFSEL0.field.FSEL5 = GPFSEL_INPUT;
    io->gpio->GPFSEL0.field.FSEL6 = GPFSEL_INPUT;
    io->gpio->GPFSEL2.field.FSEL2 = GPFSEL_INPUT;
    io->gpio->GPFSEL2.field.FSEL3 = GPFSEL_INPUT;
  }
  printf("testing motors main done\n");
  return 0;
}
