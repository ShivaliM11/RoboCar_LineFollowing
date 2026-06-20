/* hw6 - Line Following
m1 works , m2 goes straight w/o detecting shit
*hw6== IRRead, LineTrace threads, m1/m2 mode switch
* IR sensors: GPIO 24 = l, GPIO 25 = r, GPIO 18 = center
*   1 = white, 0 = black
* sensor byte = (center << 2) | (left << 1) | right
*   center on line + l&r off = straight
*   l on = drifted r, slow r motor
*   r on = drifted l, slow l motor
*   c+l on = sharp left curve
*   c+r on = sharp right curve
*   all off = lost line, hold last correction
* Tune: LT_FULL (straight speed) and LT_SLOW (correction speed)
***************************************************/

#include <stdio.h>      
#include <stdlib.h>     
#include <stdint.h>     
#include <unistd.h>     
#include <string.h>    
#include <stdbool.h>   
#include <termios.h>  
#include <fcntl.h>     
#include <pthread.h>  
#include "include/import_registers.h"
#include "include/cm.h"
#include "include/gpio.h"
#include "include/uart.h"
#include "include/spi.h"
#include "include/bsc.h"
#include "include/pwm.h"
#include "include/enable_pwm_clock.h"
#include "include/io_peripherals.h"
#include "include/wait_period.h"
#include "include/FIFO.h"
#include "include/MPU6050.h"
#include "include/MPU9250.h"
#include "include/wait_key.h"
#include "keypress.h"
#include "pwmsetup.h"

#define FIFO_LENGTH   1024  
#define PWM_RANGE      100  
#define SPEED_INIT      50  
#define SPEED_STEP       5  
#define SPEED_MAX      100
#define SPEED_MIN        0
#define RAMP_STEPS      10  
#define TURN_TICKS_90   30  
#define DT_INIT          5  
#define DT_MIN           5
#define DT_MAX          90
#define IR_CENTER_PIN   18  // center sensor - on line = straight
#define IR_LEFT_PIN     24  
#define IR_RIGHT_PIN    25  
#define LT_FULL         60 
#define LT_SLOW         20  
#define SQUARE_TICKS   100 

struct thread_command { uint8_t command; uint8_t argument; }; // every message passed between threads= 1Bcmd char & 1B value
FIFO_TYPE(struct thread_command, FIFO_LENGTH, fifo_t);

volatile int  g_mode    = 1;      // 1=manual 2=line trace
volatile bool g_quit    = false;  // set true to shut down all threads
volatile bool g_tracing = false;  // true only after w pressed in m2
volatile bool g_motor_stop = false;

// each thread gets one of these passed in as its argument--> holds pointers to whatever FIFOs and hardware that thread needs
//--> pthread only lets you pass one void* to a thread so everything is bundled into a struct
struct key_param       { struct fifo_t *key_fifo; };
struct control_param   { struct fifo_t *key_fifo, *left_fifo, *right_fifo; };
struct irread_param    { volatile struct gpio_register *gpio; struct fifo_t *ir_fifo; };
struct linetrace_param { struct fifo_t *ir_fifo, *left_fifo, *right_fifo; };
struct motor_param     { const char *name; volatile struct gpio_register *gpio;
                         struct io_peripherals *io; int pin1, pin2, pwm_ch;
                         struct fifo_t *fifo; };

// helper to send the same command to both motor FIFOs at once --basicaly checks full before inserting so it never overflows
static void send_both(struct fifo_t *lf, struct fifo_t *rf, uint8_t cmd, uint8_t arg)
{
  struct thread_command c = {cmd, arg};
  if (!(FIFO_FULL(lf))) FIFO_INSERT(lf, c);
  if (!(FIFO_FULL(rf))) FIFO_INSERT(rf, c);
}

// same but sends a different command to each moto- used for pivots where one motor goes fwd and  other bwd
static void send_split(struct fifo_t *lf, struct fifo_t *rf,
                       uint8_t lc, uint8_t la, uint8_t rc, uint8_t ra)
{
  struct thread_command l = {lc, la}, r = {rc, ra};
  if (!(FIFO_FULL(lf))) FIFO_INSERT(lf, l);
  if (!(FIFO_FULL(rf))) FIFO_INSERT(rf, r);
}


void *KeyRead(void * arg)
{
  struct  key_param * param = (struct key_param *)arg;
  struct  thread_command cmd = {0, 0};
  int     key;
  int     m_pending = 0;  // tracks if last key was 'm', waiting for 1 or 2
  struct  timespec  timer_state;

  // wait_period_initialize setsstarting reference point fortimer
  // every call to waitperiod then sleeps until exactly 10ms has passed
  // since the last wakeup - keeps thread running at exact 100Hz
  wait_period_initialize( &timer_state );
  wait_period( &timer_state, 10u );

  while (!g_quit)
  {
    key = get_pressed_key();
    if (key != -1 && key != 10)
    {
      switch (key)
      {
        case 109:   // mm
          m_pending = 1;
          printf("HW6m%d> m", g_mode); fflush(stdout);
        break;

        case 49:    // m1
          if (m_pending)
          {
            m_pending = 0;
            printf("1\n"); fflush(stdout);
            cmd.command = 'M'; cmd.argument = 1;
            if (!(FIFO_FULL(param->key_fifo))) FIFO_INSERT(param->key_fifo, cmd);
          }
        break;

        case 50:    // m2
          if (m_pending)
          {
            m_pending = 0;
            printf("2\n"); fflush(stdout);
            cmd.command = 'M'; cmd.argument = 2;
            if (!(FIFO_FULL(param->key_fifo))) FIFO_INSERT(param->key_fifo, cmd);
          }
        break;

        case 113:   // q
          m_pending = 0;
          printf("HW6m%d> q\n", g_mode); fflush(stdout);
          cmd.command = 113; cmd.argument = 0;
          if (!(FIFO_FULL(param->key_fifo))) FIFO_INSERT(param->key_fifo, cmd);
          g_quit = true;  // set directly so all threads start shutting down
        break;

        default:
          m_pending = 0;
          printf("HW6m%d> %c\n", g_mode, (char)key); fflush(stdout);
          cmd.command = (uint8_t)key; cmd.argument = 0;
          if (!(FIFO_FULL(param->key_fifo))) FIFO_INSERT(param->key_fifo, cmd);
        break;
      }
    }
    wait_period( &timer_state, 10u );
  }

  printf("KeyRead function done\n");
  return NULL;
}


void *Control(void * arg)
{
  struct  control_param * param = (struct control_param *)arg;
  struct  thread_command cmd = {0, 0};
  struct  timespec  timer_state;

  int  cur_speed  = SPEED_INIT;
  int  last_speed = SPEED_INIT;
  int  cur_dir    = 0;        // 0=stopped 1=fwd-1=bwd
  int  step, ramp;
  int  dT         = DT_INIT;  // current turn angle in degrees
  int  turn_ticks = 0;        // countdown timer for pivot turns

  wait_period_initialize( &timer_state );
  wait_period( &timer_state, 10u );

  while (!g_quit)
  {
    // timed pivot turns- pressing a or d sets turn_ticks to some count. each 10ms tick decrements it, when it hits zero straight motion is restored
    if (turn_ticks > 0 && g_mode == 1)
    {
      turn_ticks--;
      if (turn_ticks == 0 && cur_dir != 0)
      {
        if (cur_dir == 1) send_split(param->left_fifo, param->right_fifo, 'f',0,'f',0);
        else              send_split(param->left_fifo, param->right_fifo, 'b',0,'b',0);
        send_both(param->left_fifo, param->right_fifo, 'v', (uint8_t)cur_speed);
        printf(" STRAIGHT restored %d%%\n", cur_speed); fflush(stdout);
      }
    }

    if (!(FIFO_EMPTY(param->key_fifo)))
    {
      FIFO_REMOVE(param->key_fifo, &cmd);

      /// combine mode & key into one value so one switch handles everything
      // universal s stop -- works in both m1 and m2
      if (cmd.command == 115)  // 's' = stop, always works
      {
        turn_ticks = 0;
        g_tracing = false;  // stop LineTrace immediately
        if (cur_speed > 0) last_speed = cur_speed;
        send_both(param->left_fifo, param->right_fifo, 'v', 0);
        send_both(param->left_fifo, param->right_fifo, 's', 0);
        cur_speed = 0; cur_dir = 0;
        printf(" STOP\n"); fflush(stdout);
        goto next_tick;
      }

      switch ((g_mode << 8) | cmd.command)
      {
        case (1 << 8) | 'M':   // mode switch (from either mode)
        case (2 << 8) | 'M':
          turn_ticks = 0; last_speed = cur_speed;
          g_tracing  = false;  // stop LineTrace before switching
          send_both(param->left_fifo, param->right_fifo, 'v', 0);
          send_both(param->left_fifo, param->right_fifo, 's', 0);
          cur_speed = 0; cur_dir = 0;
          g_mode = (int)cmd.argument;
          if (g_mode == 1) printf(" MODE -> m1 (manual)\n");
          else             printf(" MODE -> m2 (line trace) - press w to start\n");
          fflush(stdout);
        break;

        case (2 << 8) | 115:   // m2 s pause
          // g_tracing=false first so LineTrace stops writing to motor FIFOs--b4 the stop commands arrive, otherwise LineTrace overwrites them
          g_tracing = false;
          send_both(param->left_fifo, param->right_fifo, 'v', 0);
          send_both(param->left_fifo, param->right_fifo, 's', 0);
          cur_dir = 0;
          printf(" m2 PAUSED\n"); fflush(stdout);
        break;

        case (2 << 8) | 119:   // m2 w start tracing
          // direction before speed, g_tracing set last so LineTrace--doesnt take over until the initial commands are already queued
          send_both(param->left_fifo, param->right_fifo, 'f', 0);
          send_both(param->left_fifo, param->right_fifo, 'v', LT_FULL);
          cur_dir = 1; g_tracing = true;
          printf(" m2 TRACING started\n"); fflush(stdout);
        break;

        case (2 << 8) | 113:   // m2 q
          g_tracing = false;
          send_both(param->left_fifo, param->right_fifo, 'v', 0);
          send_both(param->left_fifo, param->right_fifo, 'q', 0);
          g_quit = true;
          printf(" QUIT\n"); fflush(stdout);
        break;

        case (1 << 8) | 115:   // m1 s stop
          turn_ticks = 0;
          if (cur_speed > 0) last_speed = cur_speed;
          send_both(param->left_fifo, param->right_fifo, 'v', 0);
          send_both(param->left_fifo, param->right_fifo, 's', 0);
          cur_speed = 0; cur_dir = 0;
          printf(" STOP\n"); fflush(stdout);
        break;

        case (1 << 8) | 119:   // m1 w forward
          turn_ticks = 0;
          // if currently going backward, ramp speed down to 0 first -->then brake, then set forward, then ramp back up
          // toprevent slamming motors into reverse
          if (cur_dir == -1)
          {
            for (step = RAMP_STEPS; step >= 0; step--)
            { ramp = cur_speed * step / RAMP_STEPS;
              send_both(param->left_fifo, param->right_fifo, 'v', (uint8_t)ramp);
              wait_period(&timer_state, 10u); }
            send_both(param->left_fifo, param->right_fifo, 's', 0);
            wait_period(&timer_state, 10u);
          }
          send_both(param->left_fifo, param->right_fifo, 'f', 0);
          wait_period(&timer_state, 10u);
          if (cur_dir == -1)
          { for (step = 0; step <= RAMP_STEPS; step++)
            { ramp = cur_speed * step / RAMP_STEPS;
              send_both(param->left_fifo, param->right_fifo, 'v', (uint8_t)ramp);
              wait_period(&timer_state, 10u); } }
          else
          { if (cur_speed == 0) cur_speed = (last_speed > 0) ? last_speed : SPEED_INIT;
            send_both(param->left_fifo, param->right_fifo, 'v', (uint8_t)cur_speed); }
          cur_dir = 1;
          printf(" FORWARD %d%%\n", cur_speed); fflush(stdout);
        break;

        case (1 << 8) | 120:   // m1 x backward - mirror of forward case
          turn_ticks = 0;
          if (cur_dir == 1)
          { for (step = RAMP_STEPS; step >= 0; step--)
            { ramp = cur_speed * step / RAMP_STEPS;
              send_both(param->left_fifo, param->right_fifo, 'v', (uint8_t)ramp);
              wait_period(&timer_state, 10u); }
            send_both(param->left_fifo, param->right_fifo, 's', 0);
            wait_period(&timer_state, 10u); }
          send_both(param->left_fifo, param->right_fifo, 'b', 0);
          wait_period(&timer_state, 10u);
          if (cur_dir == 1)
          { for (step = 0; step <= RAMP_STEPS; step++)
            { ramp = cur_speed * step / RAMP_STEPS;
              send_both(param->left_fifo, param->right_fifo, 'v', (uint8_t)ramp);
              wait_period(&timer_state, 10u); } }
          else
          { if (cur_speed == 0) cur_speed = (last_speed > 0) ? last_speed : SPEED_INIT;
            send_both(param->left_fifo, param->right_fifo, 'v', (uint8_t)cur_speed); }
          cur_dir = -1;
          printf(" BACKWARD %d%%\n", cur_speed); fflush(stdout);
        break;

        case (1 << 8) | 105:   // m1 i faster
          cur_speed += SPEED_STEP; if (cur_speed > SPEED_MAX) cur_speed = SPEED_MAX;
          if (cur_dir != 0) send_both(param->left_fifo, param->right_fifo, 'v', (uint8_t)cur_speed);
          printf(" FASTER %d%%\n", cur_speed); fflush(stdout);
        break;

        case (1 << 8) | 106:   // m1 j slower
          cur_speed -= SPEED_STEP; if (cur_speed < SPEED_MIN) cur_speed = SPEED_MIN;
          send_both(param->left_fifo, param->right_fifo, 'v', (uint8_t)cur_speed);
          if (cur_speed == 0) cur_dir = 0;
          printf(" SLOWER %d%%\n", cur_speed); fflush(stdout);
        break;

        case (1 << 8) | 97:    // m1 a pivot l
          // l motor bwd, r motor fwd = pivot l
          // turn_ticks formula scales TURN_TICKS_90 to requested angle dT
          // +89 is ceiling division so small angles dont round down to 0
          if (cur_dir != 0 && cur_speed > 0)
          { send_split(param->left_fifo, param->right_fifo, 'b',0,'f',0);
            send_both(param->left_fifo, param->right_fifo, 'v', (uint8_t)cur_speed);
            turn_ticks = (dT * TURN_TICKS_90 + 89) / 90;
            if (turn_ticks < 3) turn_ticks = 3;
            printf(" LEFT pivot %d deg  ticks=%d\n", dT, turn_ticks); fflush(stdout); }
          else { printf(" (not moving - press w or x first)\n"); fflush(stdout); }
        break;

        case (1 << 8) | 100:   // m1 d pivot right - mirror of left
          if (cur_dir != 0 && cur_speed > 0)
          { send_split(param->left_fifo, param->right_fifo, 'f',0,'b',0);
            send_both(param->left_fifo, param->right_fifo, 'v', (uint8_t)cur_speed);
            turn_ticks = (dT * TURN_TICKS_90 + 89) / 90;
            if (turn_ticks < 3) turn_ticks = 3;
            printf(" RIGHT pivot %d deg  ticks=%d\n", dT, turn_ticks); fflush(stdout); }
          else { printf(" (not moving - press w or x first)\n"); fflush(stdout); }
        break;

        case (1 << 8) | 111:   // m1 o dT +5
          dT += 5; if (dT > DT_MAX) dT = DT_MAX;
          printf(" dT = %d deg\n", dT); fflush(stdout);
        break;

        case (1 << 8) | 107:   // m1 k dT -5
          dT -= 5; if (dT < DT_MIN) dT = DT_MIN;
          printf(" dT = %d deg\n", dT); fflush(stdout);
        break;

        case (1 << 8) | 122:   // m1 z - drive 1m x 1m square, return to start
        {
          // 4 sides: go straight SQUARE_TICKS*10ms then pivot right 90 deg
          // tune SQUARE_TICKS for ~1m travel and TURN_TICKS_90 for exact 90 deg
          int side, s;
          int z_abort = 0;  // set to 1 if s pressed during square
          struct thread_command z_cmd = {0,0};

          send_both(param->left_fifo, param->right_fifo, 'f', 0);
          send_both(param->left_fifo, param->right_fifo, 'v', (uint8_t)cur_speed);
          cur_dir = 1;

          for (side = 0; side < 4 && !z_abort; side++)
          {
            // drive straight - check for s each tick
            for (s = 0; s < SQUARE_TICKS && !z_abort; s++)
            {
              wait_period(&timer_state, 10u);
              if (!FIFO_EMPTY(param->key_fifo))
              { FIFO_REMOVE(param->key_fifo, &z_cmd);
                if (z_cmd.command == 115 || z_cmd.command == 113) z_abort = 1; }
            }
            if (z_abort) break;

            // stop before turn
            send_both(param->left_fifo, param->right_fifo, 'v', 0);
            send_both(param->left_fifo, param->right_fifo, 's', 0);
            for (s = 0; s < 5; s++) { wait_period(&timer_state, 10u); }

            if (side < 3)  // only turn on first 3 corners, not after last side
            {
              // pivot right 90 deg
              send_split(param->left_fifo, param->right_fifo, 'f',0,'b',0);
              send_both(param->left_fifo, param->right_fifo, 'v', (uint8_t)cur_speed);
              for (s = 0; s < TURN_TICKS_90 && !z_abort; s++)
              {
                wait_period(&timer_state, 10u);
                if (!FIFO_EMPTY(param->key_fifo))
                { FIFO_REMOVE(param->key_fifo, &z_cmd);
                  if (z_cmd.command == 115 || z_cmd.command == 113) z_abort = 1; }
              }
              if (z_abort) break;

              // stop after turn, then start next side
              send_both(param->left_fifo, param->right_fifo, 'v', 0);
              send_both(param->left_fifo, param->right_fifo, 's', 0);
              for (s = 0; s < 5; s++) { wait_period(&timer_state, 10u); }

              send_split(param->left_fifo, param->right_fifo, 'f',0,'f',0);
              send_both(param->left_fifo, param->right_fifo, 'v', (uint8_t)cur_speed);
            }
          }
          // final stop always
          send_both(param->left_fifo, param->right_fifo, 'v', 0);
          send_both(param->left_fifo, param->right_fifo, 's', 0);
          cur_speed = 0; cur_dir = 0;
          if (z_abort) printf(" SQUARE aborted\n");
          else          printf(" SQUARE done\n");
          fflush(stdout);
        }
        break;

        case (1 << 8) | 113:   // m1 q
          turn_ticks = 0;
          send_both(param->left_fifo, param->right_fifo, 'v', 0);
          send_both(param->left_fifo, param->right_fifo, 'q', 0);
          g_quit = true;
          printf(" QUIT\n"); fflush(stdout);
        break;
      }
    }

    next_tick:
    wait_period( &timer_state, 10u );
  }

  printf("Control function done\n");
  return NULL;
}

void *IRRead(void * arg)
{
  struct  irread_param * param = (struct irread_param *)arg;
  struct  thread_command cmd = {0, 0};
  struct  timespec  timer_state;
  int     center_val, left_val, right_val, print_count = 0;

  wait_period_initialize( &timer_state );
  wait_period( &timer_state, 10u );

  while (!g_quit)
  {
    center_val = (GPIO_READ(param->gpio, IR_CENTER_PIN) != 0) ? 1 : 0; // center sensor
    left_val   = (GPIO_READ(param->gpio, IR_LEFT_PIN)   != 0) ? 1 : 0; // read raw GPIO pin, !=0 normalizes it to exactly 0 or 1
    right_val  = (GPIO_READ(param->gpio, IR_RIGHT_PIN)  != 0) ? 1 : 0;

    print_count++;
    if (print_count >= 50)
    { printf("  [IR C=%d L=%d R=%d]  (1=black 0=white)\n", center_val, left_val, right_val);
      fflush(stdout); print_count = 0; }

    // pack all 3 sensor values into 1 byte: center=bit2 left=bit1 right=bit0
    // both white=0b000=0, center only=0b100=4, left=0b010=2, right=0b001=1 etc
    cmd.command  = 'I';
    cmd.argument = (uint8_t)((center_val << 2) | (left_val << 1) | right_val);

    // drop oldest reading if full -->no data buildup when stopped--> otherisw LineTrace would burn through old readings before acting on current data & fuck up
    if (FIFO_FULL(param->ir_fifo))
    { struct thread_command stale; FIFO_REMOVE(param->ir_fifo, &stale); }
    FIFO_INSERT(param->ir_fifo, cmd);

    wait_period( &timer_state, 10u );
  }

  printf("IRRead function done\n");
  return NULL;
}

void *LineTrace(void * arg)
{
  struct  linetrace_param * param = (struct linetrace_param *)arg;
  struct  thread_command cmd = {0, 0};
  struct  thread_command cl, cr;
  struct  timespec  timer_state;
  int     last_dir = 0;   // 0=straight, 1=left corr, -1=right corr
  int     c, l, r;

  wait_period_initialize( &timer_state );
  wait_period( &timer_state, 10u );

  while (!g_quit)
  {
    if (!(FIFO_EMPTY(param->ir_fifo)))
    {
      FIFO_REMOVE(param->ir_fifo, &cmd);

      //switch on command - only I(IR reading) does anything here --g_mode& g_tracing see if motors actually drive
      switch (cmd.command)
      {
        case 'I':
        {// g_tracing cleared by control any time s pressed --> check it here so LineTrace stops within 1 10ms tick
          if (g_mode == 2 && g_tracing)
          {
            struct thread_command fwd = {'f', 0};

            // unpack 3 sensor bits
            c = (cmd.argument >> 2) & 1;  // center
            l = (cmd.argument >> 1) & 1;  // left
            r = (cmd.argument)      & 1;  // right

            // send fwd dir every tick while tracing -- make sure right dir before speed command arrives
            if (!(FIFO_FULL(param->left_fifo)))  FIFO_INSERT(param->left_fifo,  fwd);
            if (!(FIFO_FULL(param->right_fifo))) FIFO_INSERT(param->right_fifo, fwd);

            // 2-sensor logic (center + left only, right sensor unreliable):
            // center on line = straight
            // center off + left off = drifted right = correct right (slow left)
            // center off + left on  = drifted left  = correct left  (slow right)
            // center on  + left on  = sharp left curve = slow right more
            if (c && !l)
            { cl.argument = LT_FULL; cr.argument = LT_FULL; last_dir = 0; }
            else if (!c && !l)
            { cl.argument = LT_SLOW; cr.argument = LT_FULL; last_dir = -1; }
            else if (!c && l)
            { cl.argument = LT_FULL; cr.argument = LT_SLOW; last_dir = 1; }
            else  // c && l - sharp left
            { cl.argument = LT_FULL; cr.argument = LT_SLOW/2; last_dir = 1; }

            cl.command = 'v'; cr.command = 'v';
            if (g_tracing)// check g_tracing one final time before posting to motors
            { if (!(FIFO_FULL(param->left_fifo)))  FIFO_INSERT(param->left_fifo,  cl); // handles race where Control clears it between check above and here
              if (!(FIFO_FULL(param->right_fifo))) FIFO_INSERT(param->right_fifo, cr); }
          }
        }
        break;
      }
    }
    wait_period( &timer_state, 10u );
  }

  printf("LineTrace function done\n");
  return NULL;
}


void *MotorThread(void * arg)
{
  struct  motor_param * param = (struct motor_param *)arg;
  struct  thread_command cmd = {0, 0};
  struct  timespec  timer_state;

  wait_period_initialize( &timer_state );
  wait_period( &timer_state, 10u );

  while (!g_quit)
  {
    if (!(FIFO_EMPTY(param->fifo)))
    {
      FIFO_REMOVE(param->fifo, &cmd);
      // motor driver reads pin1& pin2 as2 bit code
      // 01=forward 10=backward 00=stop, 11 would short circuit-- dont  sent
      switch (cmd.command)
      {
        case 'f': GPIO_CLR(param->gpio, param->pin1); GPIO_SET(param->gpio, param->pin2); break;
        case 'b': GPIO_SET(param->gpio, param->pin1); GPIO_CLR(param->gpio, param->pin2); break;
        case 's': GPIO_CLR(param->gpio, param->pin1); GPIO_CLR(param->gpio, param->pin2); break;
        case 'v':
          if (param->pwm_ch == 1) param->io->pwm->DAT1 = (uint32_t)cmd.argument;
          else                    param->io->pwm->DAT2 = (uint32_t)cmd.argument;
        break;
        case 'q':
          GPIO_CLR(param->gpio, param->pin1); GPIO_CLR(param->gpio, param->pin2);
          if (param->pwm_ch == 1) param->io->pwm->DAT1 = 0;
          else                    param->io->pwm->DAT2 = 0;
        break;
      }
    }
    wait_period( &timer_state, 10u );
  }

  printf("%s function done\n", param->name);
  return NULL;
}

int main( void )
{
  struct io_peripherals *io;
  pthread_t tk, tc, tir, tlt, tl, tr;
  struct fifo_t key_fifo   = {{}, 0, 0, PTHREAD_MUTEX_INITIALIZER};// initialize each FIFO->empty buffer, h=0, t=0 with mutex for thread safe access
  struct fifo_t ir_fifo    = {{}, 0, 0, PTHREAD_MUTEX_INITIALIZER};
  struct fifo_t left_fifo  = {{}, 0, 0, PTHREAD_MUTEX_INITIALIZER};
  struct fifo_t right_fifo = {{}, 0, 0, PTHREAD_MUTEX_INITIALIZER};

  struct key_param       kp  = {&key_fifo};
  struct control_param   cp  = {&key_fifo, &left_fifo, &right_fifo};
  struct irread_param    irp = {NULL, &ir_fifo};
  struct linetrace_param ltp = {&ir_fifo, &left_fifo, &right_fifo};
  struct motor_param     lp  = {"Lmotor", NULL, NULL, 5,  6,  1, &left_fifo}; //l motor-GPIO 5, 6 for dir, PWM channel 1
  struct motor_param     rp  = {"Rmotor", NULL, NULL, 22, 23, 2, &right_fifo};//r motor: GPIO 22,23 for dir, PWM channel 2

  io = import_registers();
  if (io != NULL)
  {
    printf( "mem at 0x%8.8X\n", (unsigned int)io );

    enable_pwm_clock(io->cm, io->pwm);

    //allt fn connects GPIO 12/13 to hardware PWM peripheral
    io->gpio->GPFSEL1.field.FSEL2 = GPFSEL_ALTERNATE_FUNCTION0;   //GPIO 12 - l  motor PWM
    io->gpio->GPFSEL1.field.FSEL3 = GPFSEL_ALTERNATE_FUNCTION0;   // 13 - r motor PWM
    io->gpio->GPFSEL0.field.FSEL5 = GPFSEL_OUTPUT;                // 05 - l motor dir
    io->gpio->GPFSEL0.field.FSEL6 = GPFSEL_OUTPUT;                // 06 - l motor dir
    io->gpio->GPFSEL2.field.FSEL2 = GPFSEL_OUTPUT;                // 22 - r motor dir
    io->gpio->GPFSEL2.field.FSEL3 = GPFSEL_OUTPUT;                // 23 - r motor dir
    io->gpio->GPFSEL2.field.FSEL4 = GPFSEL_INPUT;                 // 24 - l IR sensor
    io->gpio->GPFSEL2.field.FSEL5 = GPFSEL_INPUT;                 // 25 - r IR sensor
    io->gpio->GPFSEL1.field.FSEL8 = GPFSEL_INPUT;                 // 18 - center IR sensor

    GPIO_CLR(io->gpio, 5);  GPIO_CLR(io->gpio, 6);
    GPIO_CLR(io->gpio, 22); GPIO_CLR(io->gpio, 23);

    // RNG=100 so DAT goes 0-100, MSEN=M/S mode meaning output = high
    // for DAT/RNG fraction of the period, PWEN enables the channel
    io->pwm->RNG1 = PWM_RANGE;        io->pwm->RNG2 = PWM_RANGE;
    io->pwm->DAT1 = 0;                io->pwm->DAT2 = 0;
    io->pwm->CTL.field.MODE1 = 0;     io->pwm->CTL.field.MODE2 = 0;
    io->pwm->CTL.field.RPTL1 = 1;     io->pwm->CTL.field.RPTL2 = 1;
    io->pwm->CTL.field.SBIT1 = 0;     io->pwm->CTL.field.SBIT2 = 0;
    io->pwm->CTL.field.POLA1 = 0;     io->pwm->CTL.field.POLA2 = 0;
    io->pwm->CTL.field.USEF1 = 0;     io->pwm->CTL.field.USEF2 = 0;
    io->pwm->CTL.field.MSEN1 = 1;     io->pwm->CTL.field.MSEN2 = 1;
    io->pwm->CTL.field.CLRF1 = 1;
    io->pwm->CTL.field.PWEN1 = 1;     io->pwm->CTL.field.PWEN2 = 1;
    irp.gpio = io->gpio;
    lp.gpio  = io->gpio;  lp.io = io;
    rp.gpio  = io->gpio;  rp.io = io;

    printf("\nhw6\n");
    printf("  m1: w=fwd x=bwd s=stop i=faster j=slower a=left d=right o/k=dT z=square q=quit\n");
    printf("  m2: w=start s=pause q=quit  ; switch: m1 or m2\n");
    printf("  watch [IR C= L= R=] -- 1=black 0=white\n\n");

    // start all threads, (void*) cast required since pthread takes a generic pointer
    // inside each thread function it gets cast back to the right struct type
    pthread_create(&tl,  NULL, MotorThread, (void *)&lp);
    pthread_create(&tr,  NULL, MotorThread, (void *)&rp);
    pthread_create(&tir, NULL, IRRead,      (void *)&irp);
    pthread_create(&tlt, NULL, LineTrace,   (void *)&ltp);
    pthread_create(&tk,  NULL, KeyRead,     (void *)&kp);
    pthread_create(&tc,  NULL, Control,     (void *)&cp);

    // main waits here until each thread exits--> all threads check g_quit&return when its true
    pthread_join(tl,  NULL);  pthread_join(tr,  NULL);
    pthread_join(tir, NULL);  pthread_join(tlt, NULL);
    pthread_join(tk,  NULL);  pthread_join(tc,  NULL);
    io->pwm->DAT1 = 0;  io->pwm->DAT2 = 0; // reset all pins back to input (high impedance) so nothing is driven after exit
    GPIO_CLR(io->gpio, 5);  GPIO_CLR(io->gpio, 6);
    GPIO_CLR(io->gpio, 22); GPIO_CLR(io->gpio, 23);
    io->gpio->GPFSEL1.field.FSEL2 = GPFSEL_INPUT;
    io->gpio->GPFSEL1.field.FSEL3 = GPFSEL_INPUT;
    io->gpio->GPFSEL1.field.FSEL8 = GPFSEL_INPUT;  // 18 center IR
    io->gpio->GPFSEL0.field.FSEL5 = GPFSEL_INPUT;
    io->gpio->GPFSEL0.field.FSEL6 = GPFSEL_INPUT;
    io->gpio->GPFSEL2.field.FSEL2 = GPFSEL_INPUT;
    io->gpio->GPFSEL2.field.FSEL3 = GPFSEL_INPUT;
    io->gpio->GPFSEL2.field.FSEL4 = GPFSEL_INPUT;
    io->gpio->GPFSEL2.field.FSEL5 = GPFSEL_INPUT;
  }
  else
  { ; /* warning message already issued */ }

  printf( "main function done\n" );
  return 0;
}