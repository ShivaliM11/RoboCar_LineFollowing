// lidar_LR.c -- TWO lidar sensors L (/dev/ttyACM0) + R (/dev/ttyACM1)
// Displays both grids. COMBINED detection: each frame we take the higher of
// L and R (double > single > empty) and run ONE shared state machine on it.
// That combined state drives: LEDs, the live counters, and the STOP pin.
//   LEDs: RED=empty(GPIO10)  GREEN=single(GPIO15)  BLUE=double(GPIO14)
//   STOP pin = GPIO26 high when combined state is DOUBLE

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include <time.h>
#include "wait_key.h"
#include "../include/import_registers.h"
#include "../include/gpio.h"
#include "../include/io_peripherals.h"

#define LIDAR_WIDTH   8
#define LIDAR_HEIGHT  8

#define LED_RED_GPIO    10   // empty
#define LED_BLUE_GPIO   14   // double
#define LED_GREEN_GPIO  15   // single
#define STOP_GPIO       26   // high = stop the car

#define GAP_MIN              3
#define DOUBLE_VOTES_NEEDED  3
#define STABLE_FRAMES        3
#define SINGLE_CONFIRM_FRAMES 50

#define PLANT_THRESHOLD 127

#define L_TOP   2
#define R_TOP   14
#define COUNT_ROW   26
#define STATUS_ROW  27

struct lidar_frame_row_t { unsigned int pixel[LIDAR_WIDTH]; };
struct lidar_frame_t     { struct lidar_frame_row_t row[LIDAR_HEIGHT]; };

// per-sensor: just its frame buffer, latest raw reading, and grid position
struct sensor_t
{
  const char *          name;
  const char *          port;
  FILE *                serial;
  int                   top_row;
  struct lidar_frame_t  frame;
  int                   latest_raw;   // last full-frame classification (0/1/2)
  bool                  frame_ready;  // got a fresh full frame this pass
};

// ---- combined counters ----
static int counts[3] = {0,0,0};   // [empty, single, double]

static int classify_row( struct lidar_frame_row_t * row, unsigned int threshold )
{
  bool seen_first = false;
  int  gap_run    = 0;
  bool gap_ok     = false;
  bool any        = false;
  for (int c = 0; c < LIDAR_WIDTH; c++)
  {
    bool detected = (row->pixel[c] <= threshold);
    if (detected)
    {
      any = true;
      if (seen_first && gap_ok) return 2;
      seen_first = true;
      gap_run    = 0;
    }
    else if (seen_first)
    {
      gap_run++;
      if (gap_run >= GAP_MIN) gap_ok = true;
    }
  }
  if (!any) return 0;
  return 1;
}

static int classify_frame( struct lidar_frame_t * frame, unsigned int threshold )
{
  int votes[3] = {0,0,0};
  for (int r = 0; r < LIDAR_HEIGHT - 1; r++)
    votes[ classify_row( &frame->row[r], threshold ) ]++;
  if (votes[2] >= DOUBLE_VOTES_NEEDED) return 2;
  if (votes[1] + votes[2] > 0)         return 1;
  return 0;
}

static void sensor_getline( FILE * file, char * buffer, size_t len )
{
  char * line = NULL;
  size_t n = 0;
  ssize_t got;
  buffer[0] = '\0';
  if (file == NULL) return;
  got = getline( &line, &n, file );
  if ((got > 0) && line && ((size_t)got < len))
    strcpy( buffer, line );
  free( line );
}

// read one serial line for a sensor, print its grid row; when a full frame
// arrives, store its raw classification and flag frame_ready
static void sensor_step( struct sensor_t * s, char * serial_line )
{
  s->frame_ready = false;
  if (!((serial_line[0] == 'y') &&
        (serial_line[1] >= '0') && (serial_line[1] <= '7') &&
        (strlen(serial_line) > 3)))
    return;

  int r = serial_line[1] - '0';
  sscanf( &serial_line[3], "%u,%u,%u,%u,%u,%u,%u,%u",
      &s->frame.row[r].pixel[0], &s->frame.row[r].pixel[1],
      &s->frame.row[r].pixel[2], &s->frame.row[r].pixel[3],
      &s->frame.row[r].pixel[4], &s->frame.row[r].pixel[5],
      &s->frame.row[r].pixel[6], &s->frame.row[r].pixel[7] );

  printf( "\x1B[%d;1H%s %d:", s->top_row + 1 + r, s->name, r );
  for (int c = 0; c < LIDAR_WIDTH; c++)
    printf( " %c", (s->frame.row[r].pixel[c] <= PLANT_THRESHOLD) ? '1' : '0' );
  printf( "\x1B[K" );
  fflush( stdout );

  if (r == (LIDAR_HEIGHT - 1))
  {
    s->latest_raw = classify_frame( &s->frame, PLANT_THRESHOLD );
    s->frame_ready = true;
  }
}

static void set_leds( struct io_peripherals * io, int state )
{
  GPIO_CLR( io->gpio, LED_RED_GPIO );
  GPIO_CLR( io->gpio, LED_BLUE_GPIO );
  GPIO_CLR( io->gpio, LED_GREEN_GPIO );
  if      (state == 0) GPIO_SET( io->gpio, LED_RED_GPIO );
  else if (state == 1) GPIO_SET( io->gpio, LED_GREEN_GPIO );
  else if (state == 2) GPIO_SET( io->gpio, LED_BLUE_GPIO );
}

static void redraw_counts( void )
{
  printf( "\x1B[%d;1HEmpty: %-4d  Single: %-4d  Double: %-4d\x1B[K",
      COUNT_ROW, counts[0], counts[1], counts[2] );
  fflush( stdout );
}

int main( void )
{
  struct io_peripherals * io;
  int pressed_key;
  char serial_line[1024];

  io = import_registers();
  if (io == NULL) { printf("could not map I/O registers (run with sudo)\n"); return -1; }

  // LEDs 10/14/15 output
  (*((volatile uint32_t *)&io->gpio->GPFSEL1)) &= ~( (7 << 0) | (7 << 12) | (7 << 15) );
  (*((volatile uint32_t *)&io->gpio->GPFSEL1)) |= ( (1 << 0) | (1 << 12) | (1 << 15) );
  GPIO_CLR( io->gpio, LED_RED_GPIO );
  GPIO_CLR( io->gpio, LED_BLUE_GPIO );
  GPIO_CLR( io->gpio, LED_GREEN_GPIO );

  // STOP pin 26 output, low
  (*((volatile uint32_t *)&io->gpio->GPFSEL2)) &= ~(7 << 18);
  (*((volatile uint32_t *)&io->gpio->GPFSEL2)) |=  (1 << 18);
  GPIO_CLR( io->gpio, STOP_GPIO );

  struct sensor_t L = {0}, R = {0};
  L.name = "L"; L.port = "/dev/ttyACM0"; L.top_row = L_TOP; L.latest_raw = 0;
  R.name = "R"; R.port = "/dev/ttyACM1"; R.top_row = R_TOP; R.latest_raw = 0;

  L.serial = fopen( L.port, "r" );
  R.serial = fopen( R.port, "r" );
  if (L.serial == NULL) printf("Sensor L: cannot open %s\n", L.port);
  if (R.serial == NULL) printf("Sensor R: cannot open %s\n", R.port);

  // ---- ONE shared state machine on the COMBINED reading ----
  int  confirmed_state = -1;
  int  candidate_state = -1;
  int  candidate_count = 0;
  bool holding_single  = false;
  int  hold_frames     = 0;

  printf( "\x1B[2J\x1B[H\x1B[?25l" );
  printf( "\x1B[%d;1H=== Sensor L (%s) ===\x1B[K", L_TOP, L.port );
  printf( "\x1B[%d;1H=== Sensor R (%s) ===\x1B[K", R_TOP, R.port );
  redraw_counts();
  fflush( stdout );

  while ( !wait_key( 1, &pressed_key ) )
  {
    sensor_getline( L.serial, serial_line, sizeof(serial_line) );
    sensor_step( &L, serial_line );

    sensor_getline( R.serial, serial_line, sizeof(serial_line) );
    sensor_step( &R, serial_line );

    // only advance the combined state machine when at least one sensor
    // produced a fresh full frame
    if (L.frame_ready || R.frame_ready)
    {
      // COMBINED reading = higher of the two latest (double>single>empty)
      int combined = (L.latest_raw > R.latest_raw) ? L.latest_raw : R.latest_raw;

      if (combined == candidate_state) candidate_count++;
      else { candidate_state = combined; candidate_count = 1; }

      if ((candidate_count >= STABLE_FRAMES) &&
          (candidate_state != confirmed_state))
      {
        int new_state = candidate_state;

        if (new_state == 1)          // single -> hold, don't count yet
        {
          holding_single = true;
          hold_frames    = 0;
          confirmed_state = 1;
          set_leds( io, -1 );        // off while holding
        }
        else if (new_state == 2)     // double -> count, LED blue, STOP
        {
          holding_single = false;
          hold_frames    = 0;
          confirmed_state = 2;
          counts[2]++;
          redraw_counts();
          set_leds( io, 2 );
          GPIO_SET( io->gpio, STOP_GPIO );
        }
        else                         // empty
        {
          if (holding_single)        // held single ended as empty -> count single
          {
            holding_single = false;
            hold_frames    = 0;
            counts[1]++;
            redraw_counts();
          }
          confirmed_state = 0;
          counts[0]++;
          redraw_counts();
          set_leds( io, 0 );
          GPIO_CLR( io->gpio, STOP_GPIO );   // left double -> release stop
        }
      }

      if (holding_single)
      {
        hold_frames++;
        if (hold_frames >= SINGLE_CONFIRM_FRAMES)   // stayed single -> count it
        {
          holding_single = false;
          hold_frames    = 0;
          counts[1]++;
          redraw_counts();
          set_leds( io, 1 );
        }
      }
    }

    printf( "\x1B[%d;1HCombined: %s   STOP:%s\x1B[K",
        STATUS_ROW,
        confirmed_state==2?"DOUBLE":confirmed_state==1?"single":"empty ",
        confirmed_state==2 ? "YES" : "no " );
    fflush( stdout );
  }

  // final tally
  printf( "\x1B[?25h\x1B[%d;1H\n", STATUS_ROW + 2 );
  printf( "==== FINAL COUNT ====\n" );
  printf( "Single: %d   Double: %d   Empty: %d\n", counts[1], counts[2], counts[0] );

  if (L.serial) fclose( L.serial );
  if (R.serial) fclose( R.serial );
  GPIO_CLR( io->gpio, LED_RED_GPIO );
  GPIO_CLR( io->gpio, LED_BLUE_GPIO );
  GPIO_CLR( io->gpio, LED_GREEN_GPIO );
  GPIO_CLR( io->gpio, STOP_GPIO );
  return 0;
}
