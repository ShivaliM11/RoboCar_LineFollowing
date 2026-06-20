/* hw6munshi.c

*
* EXACT SAME thread/FIFO structure 
* Only GPIO pins and motor direction logic changed to match our hardware
*
* OUR GPIO PINS:
*   GPIO 05, 06  = left  motor direction (AI1, AI2)
*   GPIO 22, 23  = right motor direction (BI1, BI2)
*   GPIO 12      = left  motor PWM (DAT2)
*   GPIO 13      = right motor PWM (DAT1)
*   GPIO 24      = left  IR sensor
*   GPIO 25      = right IR sensor
*   GPIO 18      = center IR sensor
*
* MOTOR DIRECTION (from hw0c1blink97forward.c):
*   forward:  pin1=SET pin2=CLR (GPIO05=1 GPIO06=0 for left)
*   backward: pin1=CLR pin2=SET (GPIO05=0 GPIO06=1 for left)
*   stop:     pin1=CLR pin2=CLR
*
* THREAD STRUCTURE 
*   t1  KeyRead
*   t2  LineTraceRight  (reads GPIO 25 right IR)
*   t3  LineTraceLeft   (reads GPIO 24 left  IR)
*   t4  LineTraceControl
*   t5  StateControl
*   t6  MotorController
*   t7  RightSpeed      (DAT1)
*   t8  LeftSpeed       (DAT2)
*   t9  RightTurn       (GPIO 22,23)
*   t10 LeftTurn        (GPIO 05,06)
***************************************************/
#include <fcntl.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <termios.h>
#include <unistd.h>
#include "include/FIFO.h"
#include "include/MPU6050.h"
#include "include/MPU9250.h"
#include "include/bsc.h"
#include "include/cm.h"
#include "include/enable_pwm_clock.h"
#include "include/gpio.h"
#include "include/import_registers.h"
#include "include/io_peripherals.h"
#include "include/pwm.h"
#include "include/spi.h"
#include "include/uart.h"
#include "include/wait_key.h"
#include "include/wait_period.h"
#include "keypress.h"
#include "pwmsetup.h"

#define PWM_RANGE 100
#define FIFO_LENGTH 1024
#define THREE_QUARTERS (FIFO_LENGTH * 3 / 4)

struct thread_command {
  uint8_t command;
  uint8_t argument;
};
FIFO_TYPE(struct thread_command, FIFO_LENGTH, fifo_t);

struct key_thread_param {
  const char*    name;
  struct fifo_t* key_fifo;
  bool*          quit_flag;
};

struct lineTrace_thread_param {
  const char*                    name;
  volatile struct gpio_register* gpio;
  int                            pin_number;
  int                            cmd_int;
  struct fifo_t*                 fifo;
  struct fifo_t*                 key_fifo;
  bool*                          quit_flag;
};

struct lineTrace_control_thread_param {
  const char*    name;
  struct fifo_t* lineTrace_right_fifo;
  struct fifo_t* lineTrace_left_fifo;
  struct fifo_t* lineTrace_control_fifo;
  bool*          quit_flag;
};

struct state_control_thread_param {
  const char*    name;
  struct fifo_t* key_fifo;
  struct fifo_t* lineTrace_control_fifo;
  struct fifo_t* state_control_fifo;
  bool*          quit_flag;
};

struct motor_controller_thread_param {
  const char*    name;
  struct fifo_t* state_control_fifo;
  struct fifo_t* right_turn_fifo;
  struct fifo_t* left_turn_fifo;
  struct fifo_t* right_speed_fifo;
  struct fifo_t* left_speed_fifo;
  bool*          quit_flag;
};

struct turn_thread_param {
  const char*                    name;
  volatile struct gpio_register* gpio;
  int                            pin_number_1;
  int                            pin_number_2;
  struct fifo_t*                 fifo;
  bool*                          quit_flag;
};

struct right_speed_thread_param {
  const char*              name;
  volatile struct pwm_register* pwm;
  struct fifo_t*           fifo;
  bool*                    quit_flag;
};

struct left_speed_thread_param {
  const char*              name;
  volatile struct pwm_register* pwm;
  struct fifo_t*           fifo;
  bool*                    quit_flag;
};

// ── THREAD 1: KeyRead 
void* KeyRead(void* arg) {
  struct key_thread_param* param = (struct key_thread_param*)arg;
  struct thread_command cmd = { 0, 0 };
  int keyhit = 0;
  struct timespec timer_state;

  wait_period_initialize(&timer_state);
  wait_period(&timer_state, 10u);

  while (!*(param->quit_flag)) {
    keyhit = get_pressed_key();
    if (keyhit != -1) {
      switch (keyhit) {
        case 113: {  // 'q'
          cmd.command = 113; cmd.argument = 0;
          if (!(FIFO_FULL(param->key_fifo))) FIFO_INSERT(param->key_fifo, cmd);
          *param->quit_flag = true;
          goto endOfKeyReadFunction;
        } break;
        default: {
          cmd.command = keyhit; cmd.argument = 0;
          if (!(FIFO_FULL(param->key_fifo))) FIFO_INSERT(param->key_fifo, cmd);
        }
      }
    }
    wait_period(&timer_state, 10u);
  endOfKeyReadFunction:;
  }
  printf("%s function done\n", param->name);
  return NULL;
}

// ── THREADS 2,3: LineTrace - sends sensor state (1=black,0=white) every cycle ─
void* LineTrace(void* arg) {
  struct lineTrace_thread_param* param = (struct lineTrace_thread_param*)arg;
  struct thread_command cmd = { 0, 0 };
  struct timespec timer_state;

  wait_period_initialize(&timer_state);
  wait_period(&timer_state, 10u);

  while (!*(param->quit_flag)) {
    // send raw sensor state every cycle: command = which sensor ('d' or 'a' tag),
    // argument = 1 if black, 0 if white
    cmd.command  = (int)param->cmd_int;
    cmd.argument = (GPIO_READ(param->gpio, (int)param->pin_number) != 0) ? 1 : 0;
    if (!(FIFO_FULL(param->fifo))) FIFO_INSERT(param->fifo, cmd);

    if (!(FIFO_EMPTY(param->key_fifo))) {
      FIFO_REMOVE(param->key_fifo, &cmd);
      if ((int)cmd.command == 113) {
        cmd.command = 113; cmd.argument = 0;
        if (!(FIFO_FULL(param->fifo))) FIFO_INSERT(param->fifo, cmd);
        goto endLineTrace;
      }
    }
    wait_period(&timer_state, 10u);
  endLineTrace:;
  }
  printf("%s function done\n", param->name);
  return NULL;
}

// ── THREAD 4: LineTraceControl - combines left+right sensor state into one code ─
// argument sent to lineTrace_control_fifo: bit1=left bit0=right (1=black,0=white)
void* LineTraceControl(void* arg) {
  struct lineTrace_control_thread_param* param = (struct lineTrace_control_thread_param*)arg;
  struct thread_command cmd = { 0, 0 };
  struct thread_command out = { 0, 0 };
  struct timespec timer_state;
  int left_val = 0, right_val = 0;

  wait_period_initialize(&timer_state);
  wait_period(&timer_state, 10u);

  while (!*(param->quit_flag)) {
    if (!(FIFO_EMPTY(param->lineTrace_right_fifo))) {
      FIFO_REMOVE(param->lineTrace_right_fifo, &cmd);
      if (cmd.command == 113) goto endLineTraceControl;
      right_val = cmd.argument;
    }
    if (!(FIFO_EMPTY(param->lineTrace_left_fifo))) {
      FIFO_REMOVE(param->lineTrace_left_fifo, &cmd);
      if (cmd.command == 113) goto endLineTraceControl;
      left_val = cmd.argument;
    }
    // combine: send code 'L' with argument = (left<<1)|right
    out.command  = 'L';
    out.argument = (uint8_t)((left_val << 1) | right_val);
    if (!(FIFO_FULL(param->lineTrace_control_fifo)))
      FIFO_INSERT(param->lineTrace_control_fifo, out);
    wait_period(&timer_state, 10u);
  endLineTraceControl:;
  }
  printf("%s function done\n", param->name);
  return NULL;
}

// ── THREAD 5: StateControl 
void* StateControl(void* arg) {
  struct state_control_thread_param* param = (struct state_control_thread_param*)arg;
  struct thread_command cmd = { 0, 0 };
  int Motor1LineTrace2 = 1;
  bool StateSwitchLoop = false;
  struct timespec timer_state;
  int stop_flag = 0;
  int mode_flag = 0;

  wait_period_initialize(&timer_state);
  wait_period(&timer_state, 10u);

  while (!*(param->quit_flag)) {
    // Mode 1 key entry
    if (Motor1LineTrace2 == 1) {
      if (!(FIFO_EMPTY(param->key_fifo))) {
        FIFO_REMOVE(param->key_fifo, &cmd);
        switch ((int)cmd.command) {
          case 113: {  // 'q'
            cmd.command = 113; cmd.argument = 0;
            if (!(FIFO_FULL(param->state_control_fifo))) FIFO_INSERT(param->state_control_fifo, cmd);
            goto endStateControl;
          } break;
          case 115: {  // 's' stop
            stop_flag = 1;
            cmd.command = 115; cmd.argument = 0;
            if (!(FIFO_FULL(param->state_control_fifo))) FIFO_INSERT(param->state_control_fifo, cmd);
            printf("\nHW6m1> "); fflush(stdout);
          } break;
          case 119: {  // 'w' forward
            stop_flag = 0;
            cmd.command = 119; cmd.argument = 0;
            if (!(FIFO_FULL(param->state_control_fifo))) FIFO_INSERT(param->state_control_fifo, cmd);
            printf("\nHW6m1> "); fflush(stdout);
          } break;
          case 120: {  // 'x' backward
            stop_flag = 0;
            cmd.command = 120; cmd.argument = 0;
            if (!(FIFO_FULL(param->state_control_fifo))) FIFO_INSERT(param->state_control_fifo, cmd);
            printf("\nHW6m1> "); fflush(stdout);
          } break;
          case 97: {  // 'a' turn left
            cmd.command = 97; cmd.argument = 0;
            if (!(FIFO_FULL(param->state_control_fifo))) FIFO_INSERT(param->state_control_fifo, cmd);
            printf("\nHW6m1> "); fflush(stdout);
          } break;
          case 100: {  // 'd' turn right
            cmd.command = 100; cmd.argument = 0;
            if (!(FIFO_FULL(param->state_control_fifo))) FIFO_INSERT(param->state_control_fifo, cmd);
            printf("\nHW6m1> "); fflush(stdout);
          } break;
          case 105: {  // 'i' faster
            cmd.command = 105; cmd.argument = 0;
            if (!(FIFO_FULL(param->state_control_fifo))) FIFO_INSERT(param->state_control_fifo, cmd);
            printf("\nHW6m1> "); fflush(stdout);
          } break;
          case 106: {  // 'j' slower
            cmd.command = 106; cmd.argument = 0;
            if (!(FIFO_FULL(param->state_control_fifo))) FIFO_INSERT(param->state_control_fifo, cmd);
            printf("\nHW6m1> "); fflush(stdout);
          } break;
          case 109: {  // 'm' mode select
            if (stop_flag == 1) mode_flag = 1;
            printf("\nHW6m1> "); fflush(stdout);
          } break;
          default: {
            printf("\nHW6m1> "); fflush(stdout);
          } break;
        }
      }
      if (!(FIFO_EMPTY(param->lineTrace_control_fifo))) {
        FIFO_REMOVE(param->lineTrace_control_fifo, &cmd);
      }
    }

    // Mode 2 key entry
    if (Motor1LineTrace2 == 2) {
      if (!(FIFO_EMPTY(param->key_fifo))) {
        FIFO_REMOVE(param->key_fifo, &cmd);
        switch ((int)cmd.command) {
          case 113: {  // 'q'
            cmd.command = 113; cmd.argument = 0;
            if (!(FIFO_FULL(param->state_control_fifo))) FIFO_INSERT(param->state_control_fifo, cmd);
            goto endStateControl;
          } break;
          case 119: {  // 'w' start line trace
            stop_flag = 0;
            // flush any stale turn commands queued while car was stopped
            while (!(FIFO_EMPTY(param->lineTrace_control_fifo))) {
              struct thread_command flush_cmd;
              FIFO_REMOVE(param->lineTrace_control_fifo, &flush_cmd);
            }
            cmd.command = 121; cmd.argument = 0;  // 'y' = line trace forward
            if (!(FIFO_FULL(param->state_control_fifo))) FIFO_INSERT(param->state_control_fifo, cmd);
            printf("\nHW6m2> "); fflush(stdout);
          } break;
          case 115: {  // 's' stop
            stop_flag = 1;
            // flush stale turn commands so 's' isn't stuck behind a backlog
            while (!(FIFO_EMPTY(param->state_control_fifo))) {
              struct thread_command flush_cmd;
              FIFO_REMOVE(param->state_control_fifo, &flush_cmd);
            }
            cmd.command = 115; cmd.argument = 0;
            if (!(FIFO_FULL(param->state_control_fifo))) FIFO_INSERT(param->state_control_fifo, cmd);
            printf("\nHW6m2> "); fflush(stdout);
          } break;
          case 109: {  // 'm' mode select
            if (stop_flag == 1) mode_flag = 1;
            printf("\nHW6m2> "); fflush(stdout);
          } break;
          default: {
            printf("\nHW6m2> "); fflush(stdout);
          } break;
        }
      }
      if (!(FIFO_EMPTY(param->lineTrace_control_fifo))) {
        FIFO_REMOVE(param->lineTrace_control_fifo, &cmd);
        if (stop_flag == 0) {  // only act on sensor data while actively tracing
          if (cmd.command == 'L') {
            // pass combined left+right sensor state straight to MotorController
            if (!(FIFO_FULL(param->state_control_fifo))) FIFO_INSERT(param->state_control_fifo, cmd);
          }
        }
        // else: discard the command, car is stopped
      }
    }

    // Mode select
    if (stop_flag == 1 && mode_flag == 1) {
      printf(" Mode Select (1=manual 2=line trace): "); fflush(stdout);
      while (!StateSwitchLoop) {
        if (!(FIFO_EMPTY(param->key_fifo))) {
          FIFO_REMOVE(param->key_fifo, &cmd);
          switch ((int)cmd.command) {
            case 49: {  // '1' manual mode
              Motor1LineTrace2 = 1;
              printf("\n=== m1 manual ===\n");
              printf("  w=fwd x=bwd s=stop i=faster j=slower\n");
              printf("  a=left d=right q=quit m=mode select\n\nHW6m1> ");
              fflush(stdout);
              StateSwitchLoop = true; mode_flag = 0;
              cmd.command = 116; cmd.argument = 0;  // 't' reset speed
              if (!(FIFO_FULL(param->state_control_fifo))) FIFO_INSERT(param->state_control_fifo, cmd);
              goto endStateSwitchLoop;
            } break;
            case 50: {  // '2' line trace mode
              Motor1LineTrace2 = 2;
              // flush stale turn commands accumulated while in m1
              while (!(FIFO_EMPTY(param->lineTrace_control_fifo))) {
                struct thread_command flush_cmd;
                FIFO_REMOVE(param->lineTrace_control_fifo, &flush_cmd);
              }
              printf("\n=== m2 line trace ===\n");
              printf("  w=start s=pause q=quit m=mode select\n\nHW6m2> ");
              fflush(stdout);
              StateSwitchLoop = true; mode_flag = 0;
              goto endStateSwitchLoop;
            } break;
            case 113: {  // 'q'
              cmd.command = 113; cmd.argument = 0;
              if (!(FIFO_FULL(param->state_control_fifo))) FIFO_INSERT(param->state_control_fifo, cmd);
              goto endStateControl;
            } break;
            default: break;
          }
        }
        if (!(FIFO_EMPTY(param->lineTrace_control_fifo)))
          FIFO_REMOVE(param->lineTrace_control_fifo, &cmd);
        wait_period(&timer_state, 10u);
      endStateSwitchLoop:;
      }
    }
    StateSwitchLoop = false;
    wait_period(&timer_state, 10u);
  endStateControl:;
  }
  printf("%s function done\n", param->name);
  return NULL;
}

// ── THREAD 6: MotorController (same logic as friend, our GPIO directions) ────
void* MotorController(void* arg) {
  struct motor_controller_thread_param* param = (struct motor_controller_thread_param*)arg;
  struct thread_command cmd1 = { 0, 0 };
  struct thread_command cmd2 = { 0, 0 };
  struct timespec timer_state;
  int leftMotorSpeedValue  = 0;
  int rightMotorSpeedValue = 0;
  int tempLeftMotorSpeedValue  = 0;
  int tempRightMotorSpeedValue = 0;
  int leftMotorDirection  = 0;
  int rightMotorDirection = 0;
  int lineTraceSpeed = 3;
  int last_dir = 0;  // 0=forward(straight), 1=left, 2=right -- persists through gaps in detection

  wait_period_initialize(&timer_state);
  wait_period(&timer_state, 10u);

  while (!*(param->quit_flag)) {
    if (!(FIFO_EMPTY(param->state_control_fifo))) {
      FIFO_REMOVE(param->state_control_fifo, &cmd1);
      switch ((int)cmd1.command) {
        case 115: {  // 's' stop
          rightMotorDirection = 115; leftMotorDirection = 115;
          cmd2.command = 115; cmd2.argument = 0;
          if (!(FIFO_FULL(param->right_turn_fifo)) && !(FIFO_FULL(param->left_turn_fifo))) {
            FIFO_INSERT(param->right_turn_fifo, cmd2);
            FIFO_INSERT(param->left_turn_fifo,  cmd2);
          }
          // zero out speeds too so no leftover correction speed lingers
          cmd2.command = 115; cmd2.argument = 0;
          if (!(FIFO_FULL(param->right_speed_fifo))) FIFO_INSERT(param->right_speed_fifo, cmd2);
          if (!(FIFO_FULL(param->left_speed_fifo)))  FIFO_INSERT(param->left_speed_fifo,  cmd2);
        } break;

        case 119: {  // 'w' forward -- our forward: pin1=SET pin2=CLR
          leftMotorDirection  = 102;  // 'f'
          rightMotorDirection = 102;
          cmd2.command = 102; cmd2.argument = 0;
          if (!(FIFO_FULL(param->right_turn_fifo)) && !(FIFO_FULL(param->left_turn_fifo))) {
            FIFO_INSERT(param->right_turn_fifo, cmd2);
            FIFO_INSERT(param->left_turn_fifo,  cmd2);
          }
        } break;

        case 120: {  // 'x' backward -- our backward: pin1=CLR pin2=SET
          leftMotorDirection  = 98;  // 'b'
          rightMotorDirection = 98;
          cmd2.command = 98; cmd2.argument = 0;
          if (!(FIFO_FULL(param->right_turn_fifo)) && !(FIFO_FULL(param->left_turn_fifo))) {
            FIFO_INSERT(param->right_turn_fifo, cmd2);
            FIFO_INSERT(param->left_turn_fifo,  cmd2);
          }
        } break;

        case 105: {  // 'i' faster
          rightMotorSpeedValue += 5; leftMotorSpeedValue += 5;
          if (rightMotorSpeedValue > 100) rightMotorSpeedValue = 100;
          if (leftMotorSpeedValue  > 100) leftMotorSpeedValue  = 100;
          cmd2.command = 115; cmd2.argument = rightMotorSpeedValue;
          if (!(FIFO_FULL(param->right_speed_fifo))) FIFO_INSERT(param->right_speed_fifo, cmd2);
          cmd2.argument = leftMotorSpeedValue;
          if (!(FIFO_FULL(param->left_speed_fifo)))  FIFO_INSERT(param->left_speed_fifo,  cmd2);
        } break;

        case 106: {  // 'j' slower
          rightMotorSpeedValue -= 5; leftMotorSpeedValue -= 5;
          if (rightMotorSpeedValue < 0) rightMotorSpeedValue = 0;
          if (leftMotorSpeedValue  < 0) leftMotorSpeedValue  = 0;
          cmd2.command = 115; cmd2.argument = rightMotorSpeedValue;
          if (!(FIFO_FULL(param->right_speed_fifo))) FIFO_INSERT(param->right_speed_fifo, cmd2);
          cmd2.argument = leftMotorSpeedValue;
          if (!(FIFO_FULL(param->left_speed_fifo)))  FIFO_INSERT(param->left_speed_fifo,  cmd2);
        } break;

        case 97: {  // 'a' pivot left: left motor backward, right motor forward
          cmd2.command = 115; cmd2.argument = 0;
          if (!(FIFO_FULL(param->right_turn_fifo)) && !(FIFO_FULL(param->left_turn_fifo))) {
            FIFO_INSERT(param->right_turn_fifo, cmd2);
            FIFO_INSERT(param->left_turn_fifo,  cmd2);
          }
          // brief pause before turn, checking for s/q so it can interrupt
          {
            int s, abort_turn = 0;
            for (s = 0; s < 20 && !abort_turn; s++) {
              wait_period(&timer_state, 10u);
              if (!FIFO_EMPTY(param->state_control_fifo)) {
                struct thread_command peek;
                FIFO_REMOVE(param->state_control_fifo, &peek);
                if (peek.command == 115 || peek.command == 113) {
                  // re-insert so outer loop handles the stop/quit properly
                  if (!(FIFO_FULL(param->state_control_fifo))) FIFO_INSERT(param->state_control_fifo, peek);
                  abort_turn = 1;
                }
              }
            }
            if (abort_turn) break;
          }
          cmd2.command = 115; cmd2.argument = 100;
          if (!(FIFO_FULL(param->right_speed_fifo)) && !(FIFO_FULL(param->left_speed_fifo))) {
            FIFO_INSERT(param->right_speed_fifo, cmd2);
            FIFO_INSERT(param->left_speed_fifo,  cmd2);
          }
          // left backward, right forward = pivot left
          cmd2.command = 98; cmd2.argument = 0;   // 'b' left motor backward
          if (!(FIFO_FULL(param->left_turn_fifo))) FIFO_INSERT(param->left_turn_fifo, cmd2);
          cmd2.command = 102; cmd2.argument = 0;  // 'f' right motor forward
          if (!(FIFO_FULL(param->right_turn_fifo))) FIFO_INSERT(param->right_turn_fifo, cmd2);
          // delay
          cmd2.command = 0; cmd2.argument = 20;
          if (!(FIFO_FULL(param->right_speed_fifo)) && !(FIFO_FULL(param->left_speed_fifo)) &&
              !(FIFO_FULL(param->right_turn_fifo))  && !(FIFO_FULL(param->left_turn_fifo))) {
            FIFO_INSERT(param->right_speed_fifo, cmd2);
            FIFO_INSERT(param->left_speed_fifo,  cmd2);
            FIFO_INSERT(param->right_turn_fifo,  cmd2);
            FIFO_INSERT(param->left_turn_fifo,   cmd2);
          }
          // stop then restore forward at line trace speed
          cmd2.command = 115; cmd2.argument = 0;
          if (!(FIFO_FULL(param->right_turn_fifo)) && !(FIFO_FULL(param->left_turn_fifo))) {
            FIFO_INSERT(param->right_turn_fifo, cmd2);
            FIFO_INSERT(param->left_turn_fifo,  cmd2);
          }
          cmd2.command = 115; cmd2.argument = lineTraceSpeed;
          if (!(FIFO_FULL(param->left_speed_fifo)))  FIFO_INSERT(param->left_speed_fifo,  cmd2);
          if (!(FIFO_FULL(param->right_speed_fifo))) FIFO_INSERT(param->right_speed_fifo, cmd2);
          cmd2.command = 102; cmd2.argument = 0;  // 'f' forward both
          if (!(FIFO_FULL(param->left_turn_fifo)))  FIFO_INSERT(param->left_turn_fifo,  cmd2);
          if (!(FIFO_FULL(param->right_turn_fifo))) FIFO_INSERT(param->right_turn_fifo, cmd2);
          // flush any backlog of stale a/d commands that piled up during this turn
          while (!FIFO_EMPTY(param->state_control_fifo)) {
            struct thread_command flush_cmd;
            FIFO_REMOVE(param->state_control_fifo, &flush_cmd);
            if (flush_cmd.command == 115 || flush_cmd.command == 113) {
              if (!(FIFO_FULL(param->state_control_fifo))) FIFO_INSERT(param->state_control_fifo, flush_cmd);
              break;
            }
          }
        } break;

        case 100: {  // 'd' pivot right: left forward, right backward
          cmd2.command = 115; cmd2.argument = 0;
          if (!(FIFO_FULL(param->right_turn_fifo)) && !(FIFO_FULL(param->left_turn_fifo))) {
            FIFO_INSERT(param->right_turn_fifo, cmd2);
            FIFO_INSERT(param->left_turn_fifo,  cmd2);
          }
          // brief pause before turn, checking for s/q so it can interrupt
          {
            int s, abort_turn = 0;
            for (s = 0; s < 20 && !abort_turn; s++) {
              wait_period(&timer_state, 10u);
              if (!FIFO_EMPTY(param->state_control_fifo)) {
                struct thread_command peek;
                FIFO_REMOVE(param->state_control_fifo, &peek);
                if (peek.command == 115 || peek.command == 113) {
                  if (!(FIFO_FULL(param->state_control_fifo))) FIFO_INSERT(param->state_control_fifo, peek);
                  abort_turn = 1;
                }
              }
            }
            if (abort_turn) break;
          }
          cmd2.command = 115; cmd2.argument = 100;
          if (!(FIFO_FULL(param->right_speed_fifo)) && !(FIFO_FULL(param->left_speed_fifo))) {
            FIFO_INSERT(param->right_speed_fifo, cmd2);
            FIFO_INSERT(param->left_speed_fifo,  cmd2);
          }
          // right backward, left forward = pivot right
          cmd2.command = 98; cmd2.argument = 0;   // 'b' right motor backward
          if (!(FIFO_FULL(param->right_turn_fifo))) FIFO_INSERT(param->right_turn_fifo, cmd2);
          cmd2.command = 102; cmd2.argument = 0;  // 'f' left motor forward
          if (!(FIFO_FULL(param->left_turn_fifo))) FIFO_INSERT(param->left_turn_fifo, cmd2);
          // delay
          cmd2.command = 0; cmd2.argument = 20;
          if (!(FIFO_FULL(param->right_speed_fifo)) && !(FIFO_FULL(param->left_speed_fifo)) &&
              !(FIFO_FULL(param->right_turn_fifo))  && !(FIFO_FULL(param->left_turn_fifo))) {
            FIFO_INSERT(param->right_speed_fifo, cmd2);
            FIFO_INSERT(param->left_speed_fifo,  cmd2);
            FIFO_INSERT(param->right_turn_fifo,  cmd2);
            FIFO_INSERT(param->left_turn_fifo,   cmd2);
          }
          // stop then restore forward at line trace speed
          cmd2.command = 115; cmd2.argument = 0;
          if (!(FIFO_FULL(param->right_turn_fifo)) && !(FIFO_FULL(param->left_turn_fifo))) {
            FIFO_INSERT(param->right_turn_fifo, cmd2);
            FIFO_INSERT(param->left_turn_fifo,  cmd2);
          }
          cmd2.command = 115; cmd2.argument = lineTraceSpeed;
          if (!(FIFO_FULL(param->left_speed_fifo)))  FIFO_INSERT(param->left_speed_fifo,  cmd2);
          if (!(FIFO_FULL(param->right_speed_fifo))) FIFO_INSERT(param->right_speed_fifo, cmd2);
          cmd2.command = 102; cmd2.argument = 0;  // 'f' forward both
          if (!(FIFO_FULL(param->left_turn_fifo)))  FIFO_INSERT(param->left_turn_fifo,  cmd2);
          if (!(FIFO_FULL(param->right_turn_fifo))) FIFO_INSERT(param->right_turn_fifo, cmd2);
          // flush any backlog of stale a/d commands that piled up during this turn
          while (!FIFO_EMPTY(param->state_control_fifo)) {
            struct thread_command flush_cmd;
            FIFO_REMOVE(param->state_control_fifo, &flush_cmd);
            if (flush_cmd.command == 115 || flush_cmd.command == 113) {
              if (!(FIFO_FULL(param->state_control_fifo))) FIFO_INSERT(param->state_control_fifo, flush_cmd);
              break;
            }
          }
        } break;

        case 121: {  // 'y' line trace forward at lineTraceSpeed
          cmd2.command = 102; cmd2.argument = 0;  // 'f' forward
          if (!(FIFO_FULL(param->right_turn_fifo)) && !(FIFO_FULL(param->left_turn_fifo))) {
            FIFO_INSERT(param->right_turn_fifo, cmd2);
            FIFO_INSERT(param->left_turn_fifo,  cmd2);
          }
          cmd2.command = 115; cmd2.argument = lineTraceSpeed;
          if (!(FIFO_FULL(param->right_speed_fifo))) FIFO_INSERT(param->right_speed_fifo, cmd2);
          if (!(FIFO_FULL(param->left_speed_fifo)))  FIFO_INSERT(param->left_speed_fifo,  cmd2);
          last_dir = 0;  // reset to straight when tracing starts
        } break;

        case 'L': {  // combined sensor state from LineTraceControl: bit1=left bit0=right
          int left_black  = (cmd1.argument >> 1) & 1;
          int right_black = (cmd1.argument)      & 1;
          int new_left_speed, new_right_speed;

          if (left_black && right_black) {
            // both black -> straight
            new_left_speed  = lineTraceSpeed;
            new_right_speed = lineTraceSpeed;
            last_dir = 0;
          } else if (left_black && !right_black) {
            // left sees black, right white -> drifted right -> turn left (slow left motor)
            new_left_speed  = lineTraceSpeed / 2;
            new_right_speed = lineTraceSpeed;
            last_dir = 1;
          } else if (!left_black && right_black) {
            // right sees black, left white -> drifted left -> turn right (slow right motor)
            new_left_speed  = lineTraceSpeed;
            new_right_speed = lineTraceSpeed / 2;
            last_dir = 2;
          } else {
            // both white - last direction (sharp correction) instead of going straight /--> in v3 it was going straight don't add that
            if (last_dir == 1) {        // keep turning left
              new_left_speed  = lineTraceSpeed / 4;
              new_right_speed = lineTraceSpeed;
            } else if (last_dir == 2) { // keep turning right
              new_left_speed  = lineTraceSpeed;
              new_right_speed = lineTraceSpeed / 4;
            } else {                    // was already straight -> stay straight
              new_left_speed  = lineTraceSpeed;
              new_right_speed = lineTraceSpeed;
            }
          }

          cmd2.command = 115; cmd2.argument = (uint8_t)new_left_speed;
          if (!(FIFO_FULL(param->left_speed_fifo)))  FIFO_INSERT(param->left_speed_fifo,  cmd2);
          cmd2.argument = (uint8_t)new_right_speed;
          if (!(FIFO_FULL(param->right_speed_fifo))) FIFO_INSERT(param->right_speed_fifo, cmd2);
        } break;

        case 116: {  // 't' reset speed from line trace
          cmd2.command = 115; cmd2.argument = rightMotorSpeedValue;
          if (!(FIFO_FULL(param->right_speed_fifo))) FIFO_INSERT(param->right_speed_fifo, cmd2);
          cmd2.argument = leftMotorSpeedValue;
          if (!(FIFO_FULL(param->left_speed_fifo)))  FIFO_INSERT(param->left_speed_fifo,  cmd2);
        } break;

        case 113: {  // 'q' quit
          cmd2.command = 113; cmd2.argument = 0;
          if (!(FIFO_FULL(param->right_speed_fifo)) && !(FIFO_FULL(param->left_speed_fifo)) &&
              !(FIFO_FULL(param->right_turn_fifo))  && !(FIFO_FULL(param->left_turn_fifo))) {
            FIFO_INSERT(param->right_speed_fifo, cmd2);
            FIFO_INSERT(param->left_speed_fifo,  cmd2);
            FIFO_INSERT(param->right_turn_fifo,  cmd2);
            FIFO_INSERT(param->left_turn_fifo,   cmd2);
          }
          goto endOfControllerFunction;
        } break;
      }
    }
    wait_period(&timer_state, 10u);
  endOfControllerFunction:;
  }
  printf("%s function done\n", param->name);
  return NULL;
}

// THREADS 7,8: TurnThread 
// OUR forward:  pin1=SET pin2=CLR  (GPIO05=1,GPIO06=0 for left)
// OUR backward: pin1=CLR pin2=SET  (GPIO05=0,GPIO06=1 for left)
// OUR stop:     pin1=CLR pin2=CLR
void* TurnThread(void* arg) {
  struct turn_thread_param* param = (struct turn_thread_param*)arg;
  struct thread_command cmd = { 0, 0 };
  struct timespec timer_state;
  bool busy = false;

  wait_period_initialize(&timer_state);
  wait_period(&timer_state, 10u);

  while (!*(param->quit_flag)) {
    if (!busy) {
      if (!(FIFO_EMPTY(param->fifo))) {
        FIFO_REMOVE(param->fifo, &cmd);
        switch (cmd.command) {
          case 115: {  // 's' stop
            GPIO_CLR(io->gpio, (int)param->pin_number_1);
            GPIO_CLR(io->gpio, (int)param->pin_number_2);
          } break;
          case 102: {  // 'f' forward: pin1=CLR pin2=SET (your working hw5 logic)
            GPIO_CLR(io->gpio, (int)param->pin_number_1);
            GPIO_SET(io->gpio, (int)param->pin_number_2);
          } break;
          case 98: {   // 'b' backward: pin1=SET pin2=CLR
            GPIO_SET(io->gpio, (int)param->pin_number_1);
            GPIO_CLR(io->gpio, (int)param->pin_number_2);
          } break;
          case 0: {    // delay
            if (cmd.argument != 0) busy = true;
          } break;
          case 113: goto endOfTurnThread; break;
          default: break;
        }
      }
    } else {
      if (cmd.argument != 0) cmd.argument--;
      else busy = false;
    }
    wait_period(&timer_state, 10u);
  endOfTurnThread:;
  }
  printf("%s function done\n", param->name);
  return NULL;
}

// ── THREAD 9: RightSpeedThread (DAT1 = right motor)
void* RightSpeedThread(void* arg) {
  struct right_speed_thread_param* param = (struct right_speed_thread_param*)arg;
  struct thread_command cmd = { 0, 0 };
  struct timespec timer_state;
  bool busy = false;

  wait_period_initialize(&timer_state);
  wait_period(&timer_state, 10u);

  while (!*(param->quit_flag)) {
    if (!busy) {
      if (!(FIFO_EMPTY(param->fifo))) {
        FIFO_REMOVE(param->fifo, &cmd);
        switch (cmd.command) {
          case 0:   if (cmd.argument != 0) busy = true; break;
          case 115: param->pwm->DAT1 = (int)cmd.argument; break;
          case 113: goto endOfRightSpeedThread; break;
          default: break;
        }
      }
    } else {
      if (cmd.argument != 0) cmd.argument--;
      else busy = false;
    }
    wait_period(&timer_state, 10u);
  endOfRightSpeedThread:;
  }
  printf("%s function done\n", param->name);
  return NULL;
}

// THREAD 10: LeftSpeedThread (DAT2 = left motor) 
void* LeftSpeedThread(void* arg) {
  struct left_speed_thread_param* param = (struct left_speed_thread_param*)arg;
  struct thread_command cmd = { 0, 0 };
  struct timespec timer_state;
  bool busy = false;

  wait_period_initialize(&timer_state);
  wait_period(&timer_state, 10u);

  while (!*(param->quit_flag)) {
    if (!busy) {
      if (!(FIFO_EMPTY(param->fifo))) {
        FIFO_REMOVE(param->fifo, &cmd);
        switch (cmd.command) {
          case 0:   if (cmd.argument != 0) busy = true; break;
          case 115: param->pwm->DAT2 = (int)cmd.argument; break;
          case 113: goto endOfLeftSpeedThread; break;
          default: break;
        }
      }
    } else {
      if (cmd.argument != 0) cmd.argument--;
      else busy = false;
    }
    wait_period(&timer_state, 10u);
  endOfLeftSpeedThread:;
  }
  printf("%s function done\n", param->name);
  return NULL;
}

//main()
int main(void) {
  struct io_peripherals* io;
  bool done = false;
  bool quit_flag = false;

  pthread_t t1Key, t2LineTraceRight, t3LineTraceLeft, t4LineTraceControl;
  pthread_t t5StateControl, t6MotorController;
  pthread_t t7RightSpeed, t8LeftSpeed, t9RightTurn, t10LeftTurn;

  struct fifo_t key_fifo                = { {}, 0, 0, PTHREAD_MUTEX_INITIALIZER };
  struct fifo_t lineTrace_right_fifo    = { {}, 0, 0, PTHREAD_MUTEX_INITIALIZER };
  struct fifo_t lineTrace_left_fifo     = { {}, 0, 0, PTHREAD_MUTEX_INITIALIZER };
  struct fifo_t lineTrace_control_fifo  = { {}, 0, 0, PTHREAD_MUTEX_INITIALIZER };
  struct fifo_t state_control_fifo      = { {}, 0, 0, PTHREAD_MUTEX_INITIALIZER };
  struct fifo_t right_turn_fifo         = { {}, 0, 0, PTHREAD_MUTEX_INITIALIZER };
  struct fifo_t left_turn_fifo          = { {}, 0, 0, PTHREAD_MUTEX_INITIALIZER };
  struct fifo_t right_speed_fifo        = { {}, 0, 0, PTHREAD_MUTEX_INITIALIZER };
  struct fifo_t left_speed_fifo         = { {}, 0, 0, PTHREAD_MUTEX_INITIALIZER };

  struct key_thread_param key_param = { "key", &key_fifo, &quit_flag };

  // LineTraceRight reads GPIO 25 (right IR), sends 'd' (turn right) when black
  // LineTraceLeft  reads GPIO 24 (left  IR), sends 'a' (turn left)  when black
  struct lineTrace_thread_param lineTrace_right_param = {
      "LineTraceRight", NULL, 25, 100,  // pin=25 cmd='d'=100
      &lineTrace_right_fifo, &key_fifo, &quit_flag };
  struct lineTrace_thread_param lineTrace_left_param = {
      "LineTraceLeft",  NULL, 24, 97,   // pin=24 cmd='a'=97
      &lineTrace_left_fifo,  &key_fifo, &quit_flag };

  struct lineTrace_control_thread_param lineTrace_control_param = {
      "LineTraceControl", &lineTrace_right_fifo, &lineTrace_left_fifo,
      &lineTrace_control_fifo, &quit_flag };

  struct state_control_thread_param state_control_param = {
      "StateControl", &key_fifo, &lineTrace_control_fifo,
      &state_control_fifo, &quit_flag };

  struct motor_controller_thread_param motor_controller_param = {
      "MotorController", &state_control_fifo, &right_turn_fifo, &left_turn_fifo,
      &right_speed_fifo, &left_speed_fifo, &quit_flag };

  // RightTurn: GPIO 22=pin1, GPIO 23=pin2
  // LeftTurn:  GPIO 05=pin1, GPIO 06=pin2
  struct turn_thread_param right_turn_param = { "RightTurn", NULL, 22, 23, &right_turn_fifo, &quit_flag };
  struct turn_thread_param left_turn_param  = { "LeftTurn",  NULL,  5,  6, &left_turn_fifo,  &quit_flag };

  struct right_speed_thread_param right_speed_param = { "RightSpeed", NULL, &right_speed_fifo, &quit_flag };
  struct left_speed_thread_param  left_speed_param  = { "LeftSpeed",  NULL, &left_speed_fifo,  &quit_flag };

  io = import_registers();
  if (io != NULL) {
    printf("mem at 0x%8.8X\n", (unsigned int)io);
    enable_pwm_clock(io->cm, io->pwm);

    io->gpio->GPFSEL1.field.FSEL2 = GPFSEL_ALTERNATE_FUNCTION0;  // GPIO 12 left  PWM
    io->gpio->GPFSEL1.field.FSEL3 = GPFSEL_ALTERNATE_FUNCTION0;  // GPIO 13 right PWM
    io->gpio->GPFSEL0.field.FSEL5 = GPFSEL_OUTPUT;               // GPIO 05 left  dir
    io->gpio->GPFSEL0.field.FSEL6 = GPFSEL_OUTPUT;               // GPIO 06 left  dir
    io->gpio->GPFSEL2.field.FSEL2 = GPFSEL_OUTPUT;               // GPIO 22 right dir
    io->gpio->GPFSEL2.field.FSEL3 = GPFSEL_OUTPUT;               // GPIO 23 right dir
    io->gpio->GPFSEL2.field.FSEL4 = GPFSEL_INPUT;                // GPIO 24 left  IR
    io->gpio->GPFSEL2.field.FSEL5 = GPFSEL_INPUT;                // GPIO 25 right IR
    io->gpio->GPFSEL1.field.FSEL8 = GPFSEL_INPUT;                // GPIO 18 center IR

    GPIO_CLR(io->gpio, 5);  GPIO_CLR(io->gpio, 6);
    GPIO_CLR(io->gpio, 22); GPIO_CLR(io->gpio, 23);

    io->pwm->RNG1 = PWM_RANGE; io->pwm->RNG2 = PWM_RANGE;
    io->pwm->DAT1 = 1;         io->pwm->DAT2 = 1;
    io->pwm->CTL.field.MODE1 = 0;  io->pwm->CTL.field.MODE2 = 0;
    io->pwm->CTL.field.RPTL1 = 1;  io->pwm->CTL.field.RPTL2 = 1;
    io->pwm->CTL.field.SBIT1 = 0;  io->pwm->CTL.field.SBIT2 = 0;
    io->pwm->CTL.field.POLA1 = 0;  io->pwm->CTL.field.POLA2 = 0;
    io->pwm->CTL.field.USEF1 = 0;  io->pwm->CTL.field.USEF2 = 0;
    io->pwm->CTL.field.MSEN1 = 1;  io->pwm->CTL.field.MSEN2 = 1;
    io->pwm->CTL.field.CLRF1 = 1;
    io->pwm->CTL.field.PWEN1 = 1;  io->pwm->CTL.field.PWEN2 = 1;
    io->pwm->DAT1 = 0; io->pwm->DAT2 = 0;

    lineTrace_right_param.gpio = io->gpio;
    lineTrace_left_param.gpio  = io->gpio;
    right_turn_param.gpio      = io->gpio;
    left_turn_param.gpio       = io->gpio;
    right_speed_param.pwm      = io->pwm;
    left_speed_param.pwm       = io->pwm;

    printf("\n=== HW6 RoboCar ===\n");
    printf("  m1: w=fwd x=bwd s=stop i=faster j=slower a=left d=right q=quit\n");
    printf("  switch mode: s first, then m, then 1 or 2\n\n");
    printf("HW6m1> "); fflush(stdout);

    pthread_create(&t1Key,            NULL, KeyRead,          (void*)&key_param);
    pthread_create(&t2LineTraceRight, NULL, LineTrace,        (void*)&lineTrace_right_param);
    pthread_create(&t3LineTraceLeft,  NULL, LineTrace,        (void*)&lineTrace_left_param);
    pthread_create(&t4LineTraceControl,NULL,LineTraceControl, (void*)&lineTrace_control_param);
    pthread_create(&t5StateControl,   NULL, StateControl,     (void*)&state_control_param);
    pthread_create(&t6MotorController,NULL, MotorController,  (void*)&motor_controller_param);
    pthread_create(&t7RightSpeed,     NULL, RightSpeedThread, (void*)&right_speed_param);
    pthread_create(&t8LeftSpeed,      NULL, LeftSpeedThread,  (void*)&left_speed_param);
    pthread_create(&t9RightTurn,      NULL, TurnThread,       (void*)&right_turn_param);
    pthread_create(&t10LeftTurn,      NULL, TurnThread,       (void*)&left_turn_param);

    pthread_join(t1Key,            NULL);
    pthread_join(t2LineTraceRight, NULL);
    pthread_join(t3LineTraceLeft,  NULL);
    pthread_join(t4LineTraceControl,NULL);
    pthread_join(t5StateControl,   NULL);
    pthread_join(t6MotorController,NULL);
    pthread_join(t7RightSpeed,     NULL);
    pthread_join(t8LeftSpeed,      NULL);
    pthread_join(t9RightTurn,      NULL);
    pthread_join(t10LeftTurn,      NULL);

    io->pwm->DAT1 = 0; io->pwm->DAT2 = 0;
    GPIO_CLR(io->gpio, 5);  GPIO_CLR(io->gpio, 6);
    GPIO_CLR(io->gpio, 22); GPIO_CLR(io->gpio, 23);
    io->gpio->GPFSEL1.field.FSEL2 = GPFSEL_INPUT;
    io->gpio->GPFSEL1.field.FSEL3 = GPFSEL_INPUT;
    io->gpio->GPFSEL1.field.FSEL8 = GPFSEL_INPUT;
    io->gpio->GPFSEL0.field.FSEL5 = GPFSEL_INPUT;
    io->gpio->GPFSEL0.field.FSEL6 = GPFSEL_INPUT;
    io->gpio->GPFSEL2.field.FSEL2 = GPFSEL_INPUT;
    io->gpio->GPFSEL2.field.FSEL3 = GPFSEL_INPUT;
    io->gpio->GPFSEL2.field.FSEL4 = GPFSEL_INPUT;
    io->gpio->GPFSEL2.field.FSEL5 = GPFSEL_INPUT;
  }
  printf("main function done\n");
  return 0;
}