// hw5munshi.c
// Homework 5 - RoboCar Motor Control with Turning
// By Shivali Munshi
// CMPEN 473, Penn State University
//
// ADDED from hw4munshi.c:
//   - 'a' = turn left  by dT degrees
//   - 'd' = turn right by dT degrees
//   - 'o' = increase dT by 5 (max 90)
//   - 'k' = decrease dT by 5 (min 5)
//   - dT starts at 5 degrees
//   - turning = slow one motor, keep other at full speed
//   - car does NOT stop during or after turning
//   - prompt changed to HW5>
//
// HOW TURNING WORKS:
//   To turn LEFT:  left motor slows down, right motor stays at cur_speed
//   To turn RIGHT: right motor slows down, left motor stays at cur_speed
//   Duration of turn = dT/90 * TURN_TICKS ticks (each tick = 10ms)
//   After turn duration, both motors go back to cur_speed
//   Car never stops — smooth continuous turn
//
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

#define FIFO_LENGTH   1024
#define PWM_RANGE     100
#define SPEED_INIT     50
#define SPEED_STEP      5
#define SPEED_MAX     100
#define SPEED_MIN       0
#define RAMP_STEPS     10
#define DT_INIT         5   // starting turn angle in degrees
#define DT_MIN          5   // minimum turn angle
#define DT_MAX         90   // maximum turn angle
// TURN_TICKS: how many 10ms ticks = 90 degree turn
// 90 degree turn takes 300ms = 30 ticks
// so dT degrees takes (dT/90 * 30) ticks
#define TURN_TICKS_90  30

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

// Motor thread param
struct motor_param {
  const char                    * name;
  volatile struct gpio_register * gpio;
  struct io_peripherals         * io;
  int                             pin1;
  int                             pin2;
  int                             pwm_ch;
  struct fifo_t                 * fifo;
  bool                          * quit_flag;
};

// helper: send same command to both motor FIFOs
static void send_both(struct fifo_t *lf, struct fifo_t *rf, uint8_t cmd, uint8_t arg)
{
  struct thread_command c = {cmd, arg};
  if (!(FIFO_FULL(lf))) FIFO_INSERT(lf, c);
  if (!(FIFO_FULL(rf))) FIFO_INSERT(rf, c);
}

// helper: send different commands to left and right FIFOs
static void send_split(struct fifo_t *lf, struct fifo_t *rf,
                       uint8_t lcmd, uint8_t larg,
                       uint8_t rcmd, uint8_t rarg)
{
  struct thread_command lc = {lcmd, larg};
  struct thread_command rc = {rcmd, rarg};
  if (!(FIFO_FULL(lf))) FIFO_INSERT(lf, lc);
  if (!(FIFO_FULL(rf))) FIFO_INSERT(rf, rc);
}

// THREAD 1: KeyRead — unchanged from hw4
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
    if (key != -1 && key != 10)
    {
      printf("HW5> %c\n", (char)key);  // CHANGED: HW5> prompt
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

// THREAD 2: Control
// ADDED: a, d, o, k cases for turning
void *Control(void *arg)
{
  struct control_param *p = (struct control_param *)arg;
  struct thread_command cmd = {0, 0};
  struct timespec ts;
  int cur_speed = SPEED_INIT;
  int cur_dir   = 0;    // 0=stop 1=fwd -1=bwd
  int step, ramp;

  // ADDED: turning variables
  int dT          = DT_INIT;   // current turning angle in degrees
  int turn_ticks  = 0;         // countdown timer for turn duration
  int turn_dir    = 0;         // 0=none 1=left -1=right
  int slow_speed  = 0;         // speed of the slower motor during turn

  wait_period_initialize(&ts);
  wait_period(&ts, 10u);

  while (!*(p->quit_flag))
  {
    // ADDED: turn countdown — runs every 10ms tick
    // when turn_ticks > 0 we are mid-turn
    // when it hits 0, restore both motors to cur_speed
    if (turn_ticks > 0)
    {
      turn_ticks--;
      if (turn_ticks == 0 && cur_dir != 0)
      {
        // turn done — restore both motors to same speed
        send_both(p->left_fifo, p->right_fifo, 'v', (uint8_t)cur_speed);
        printf(" STRAIGHT restored %d%%\n", cur_speed);
      }
    }

    if (!(FIFO_EMPTY(p->key_fifo)))
    {
      FIFO_REMOVE(p->key_fifo, &cmd);

      switch (cmd.command)
      {
        case 115:  // 's' = stop
          turn_ticks = 0;  // cancel any ongoing turn
          send_both(p->left_fifo, p->right_fifo, 'v', 0);
          send_both(p->left_fifo, p->right_fifo, 's', 0);
          cur_speed = 0; cur_dir = 0;
          printf(" STOP\n");
        break;

        case 119:  // 'w' = forward
          turn_ticks = 0;
          if (cur_dir == -1)
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
          send_both(p->left_fifo, p->right_fifo, 'f', 0);
          wait_period(&ts, 10u);
          if (cur_dir == -1)
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
          turn_ticks = 0;
          if (cur_dir == 1)
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
          send_both(p->left_fifo, p->right_fifo, 'b', 0);
          wait_period(&ts, 10u);
          if (cur_dir == 1)
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

        // ADDED: 'a' = turn left
        // slow down LEFT motor, keep RIGHT motor at cur_speed
        // duration = dT/90 * TURN_TICKS_90 ticks
        case 97:  // 'a'
          if (cur_dir != 0 && cur_speed > 0)
          {
            // slow motor = cur_speed reduced proportional to dT
            slow_speed = cur_speed - (cur_speed * dT / 90);
            if (slow_speed < 0) slow_speed = 0;
            // left slows, right stays full
            send_split(p->left_fifo,  p->right_fifo,
                       'v', (uint8_t)slow_speed,
                       'v', (uint8_t)cur_speed);
            // set turn countdown
            turn_ticks = dT * TURN_TICKS_90 / 90;
            if (turn_ticks < 1) turn_ticks = 1;
            turn_dir = 1;
            printf(" LEFT turn %d deg slow=%d%%\n", dT, slow_speed);
          }
        break;

        // ADDED: 'd' = turn right
        // slow down RIGHT motor, keep LEFT motor at cur_speed
        case 100:  // 'd'
          if (cur_dir != 0 && cur_speed > 0)
          {
            slow_speed = cur_speed - (cur_speed * dT / 90);
            if (slow_speed < 0) slow_speed = 0;
            // right slows, left stays full
            send_split(p->left_fifo,  p->right_fifo,
                       'v', (uint8_t)cur_speed,
                       'v', (uint8_t)slow_speed);
            turn_ticks = dT * TURN_TICKS_90 / 90;
            if (turn_ticks < 1) turn_ticks = 1;
            turn_dir = -1;
            printf(" RIGHT turn %d deg slow=%d%%\n", dT, slow_speed);
          }
        break;

        // ADDED: 'o' = increase dT by 5 (max 90)
        case 111:  // 'o'
          dT += 5;
          if (dT > DT_MAX) dT = DT_MAX;
          printf(" dT = %d deg\n", dT);
        break;

        // ADDED: 'k' = decrease dT by 5 (min 5)
        case 107:  // 'k'
          dT -= 5;
          if (dT < DT_MIN) dT = DT_MIN;
          printf(" dT = %d deg\n", dT);
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

// THREAD 3 & 4: MotorThread — unchanged from hw4
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
        case 'f':
          GPIO_CLR(p->gpio, p->pin1);
          GPIO_SET(p->gpio, p->pin2);
        break;

        case 'b':
          GPIO_SET(p->gpio, p->pin1);
          GPIO_CLR(p->gpio, p->pin2);
        break;

        case 's':
          GPIO_CLR(p->gpio, p->pin1);
          GPIO_CLR(p->gpio, p->pin2);
        break;

        case 'v':
          if (p->pwm_ch == 1) p->io->pwm->DAT1 = (uint32_t)cmd.argument;
          else                p->io->pwm->DAT2 = (uint32_t)cmd.argument;
        break;

        case 'q':
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

// main() — unchanged from hw4 except prompt
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

    io->gpio->GPFSEL1.field.FSEL2 = GPFSEL_ALTERNATE_FUNCTION0;
    io->gpio->GPFSEL1.field.FSEL3 = GPFSEL_ALTERNATE_FUNCTION0;
    io->gpio->GPFSEL0.field.FSEL5 = GPFSEL_OUTPUT;
    io->gpio->GPFSEL0.field.FSEL6 = GPFSEL_OUTPUT;
    io->gpio->GPFSEL2.field.FSEL2 = GPFSEL_OUTPUT;
    io->gpio->GPFSEL2.field.FSEL3 = GPFSEL_OUTPUT;

    GPIO_CLR(io->gpio, 5);  GPIO_CLR(io->gpio, 6);
    GPIO_CLR(io->gpio, 22); GPIO_CLR(io->gpio, 23);

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

    printf("\n=== HW5 RoboCar ===\n");
    printf("  w=forward  x=backward  s=stop\n");
    printf("  i=faster   j=slower    q=quit\n");
    printf("  a=left     d=right\n");
    printf("  o=dT+5     k=dT-5  (dT starts at 5 deg)\n\n");
    printf("  Lift wheels before testing\n\n");

    pthread_create(&tl, NULL, MotorThread, (void *)&lp);
    pthread_create(&tr, NULL, MotorThread, (void *)&rp);
    pthread_create(&tk, NULL, KeyRead,     (void *)&kp);
    pthread_create(&tc, NULL, Control,     (void *)&cp);

    pthread_join(tl, NULL);
    pthread_join(tr, NULL);
    pthread_join(tk, NULL);
    pthread_join(tc, NULL);

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
  printf("main done\n");
  return 0;
}