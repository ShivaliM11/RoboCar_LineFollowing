/*
 * lidar1.c -- lidar plant detection, LED classification, change tracking
 *
 * Detection: 8x8 grid, fixed 5-inch (127 mm) threshold.
 * Classification per frame (row voting), LAST ROW EXCLUDED.
 * DOUBLE = >=DOUBLE_VOTES_NEEDED rows show: 1s ... >=GAP_MIN zeros ... 1s
 * SINGLE = detection present, not double
 * EMPTY  = nothing detected
 *
 * GLITCH SUPPRESSION: a SINGLE that is bracketed by EMPTY on both
 * sides (EMPTY -> SINGLE -> EMPTY) is treated as a placement artifact
 * from the calibration frame. It is voided: not counted in the plant
 * total, not counted as a green, and dropped from the history.
 *
 * LEDs (active HIGH): EMPTY->RED GPIO10, SINGLE->BLUE GPIO14,
 * DOUBLE->GREEN GPIO15
 * Window optional: runs headless over SSH. Requires sudo.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include <time.h>
#include <math.h>
#include <cairo/cairo.h>
#include <gtk/gtk.h>
#include <gdk/gdk.h>
#include "pixel_format_RGB.h"
#include "draw_bitmap_multiwindow.h"
#include "wait_key.h"
#include "../include/import_registers.h"
#include "../include/gpio.h"
#include "../include/io_peripherals.h"

#define ARRAYSIZE(X) (sizeof(X) / sizeof((X)[0]))

#define LIDAR_WIDTH  8
#define LIDAR_HEIGHT 8
#define PIXEL_WIDTH  50
#define PIXEL_HEIGHT 50

#define FILTER_DEPTH 4

/* terminal layout */
#define RAW_GRID_TOP   2
#define BIN_GRID_TOP  12
#define STATUS_ROW    (BIN_GRID_TOP + LIDAR_HEIGHT + 1)   /* 21 */
#define COUNT_ROW     (STATUS_ROW)                        /* 21 */
#define HIST_HDR_ROW  (COUNT_ROW + 2)                     /* 24 */
#define HIST_TOP_ROW  (HIST_HDR_ROW + 1)                  /* 25 */
#define HIST_SHOW     12    /* how many history lines stay visible */

/* LED pins: one LED per pin, active high */
#define LED_RED_GPIO    10   /* EMPTY  */
#define LED_BLUE_GPIO   14   /* SINGLE */
#define LED_GREEN_GPIO  15   /* DOUBLE */

/* classification tuning */
#define GAP_MIN          3
#define DOUBLE_VOTES_NEEDED  3

/* change tracking: frames a new state must hold before it registers */
#define STABLE_FRAMES        3

struct lidar_frame_row_t
{
    unsigned int pixel[LIDAR_WIDTH];
};
struct lidar_frame_t
{
    struct lidar_frame_row_t row[LIDAR_HEIGHT];
};

static const char * state_text[3] =
{
  "EMPTY  -> RED",
  "SINGLE -> GREEN",
  "DOUBLE -> BLUE"
};

void my_getline(
    FILE * file,
    char * buffer,
    size_t buffer_length )
{
  char * line = NULL;
  size_t len = 0;
  ssize_t read;

  buffer[0] = '\0';

  if (file == NULL)
  {
    return;
  }

  read = getline( &line, &len, file );
  if ((read > 0) && (line != NULL) && ((size_t)read < buffer_length))
  {
    strcpy( buffer, line );
  }
  free( line );

  return;
}

void draw_block(
    struct pixel_format_RGB * bitmap,
    size_t                    bitmap_width,
    size_t                    block_width,
    size_t                    block_height,
    size_t                    x_offset,
    size_t                    y_offset,
    struct pixel_format_RGB   color )
{
  for (size_t y = y_offset; y < y_offset + block_height; y++)
  {
    for (size_t x = x_offset; x < x_offset + block_width; x++)
    {
      bitmap[(y * bitmap_width) + x] = color;
    }
  }

  return;
}

void calculate_color(
    struct pixel_format_RGB * color,
    size_t                    threshold,
    size_t                    value )
{
  unsigned char gray;

  if (value <= threshold)
  {
    gray = 0;
  }
  else
  {
    gray = 255;
  }

  color->R = gray;
  color->G = gray;
  color->B = gray;

  return;
}

void filter_lidar_data(
    struct lidar_frame_t * lidar_frame_history,
    size_t                  history_depth,
    struct lidar_frame_t * newest_frame,
    struct lidar_frame_t * filtered_frame )
{
  for (size_t i = 1; i < history_depth; i++)
  {
    lidar_frame_history[i-1] = lidar_frame_history[i];
  }

  lidar_frame_history[history_depth-1] = *newest_frame;

  for (size_t row = 0; row < ARRAYSIZE(filtered_frame->row); row++)
  {
    for (size_t column = 0; column < ARRAYSIZE(filtered_frame->row[row].pixel); column++)
    {
      filtered_frame->row[row].pixel[column] = 0;
      for (size_t i = 0; i < history_depth; i++)
      {
        filtered_frame->row[row].pixel[column] += lidar_frame_history[i].row[row].pixel[column];
      }
      filtered_frame->row[row].pixel[column] = filtered_frame->row[row].pixel[column] / history_depth;
    }
  }
}

/* classify one row: 0=empty, 1=single, 2=double */
static int classify_row(
    struct lidar_frame_row_t * row,
    unsigned int               threshold )
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
      if (seen_first && gap_ok)
      {
        return 2;
      }
      seen_first = true;
      gap_run    = 0;
    }
    else if (seen_first)
    {
      gap_run++;
      if (gap_run >= GAP_MIN)
      {
        gap_ok = true;
      }
    }
  }

  if (!any) { return 0; }
  return 1;
}

/* classify the whole frame by voting across rows 0..LIDAR_HEIGHT-2
 * (the LAST row is EXCLUDED -- it is always 1 due to sensor mounting) */
int classify_frame(
    struct lidar_frame_t * frame,
    unsigned int           threshold )
{
  int votes[3] = { 0, 0, 0 };

  for (int r = 0; r < LIDAR_HEIGHT - 1; r++)
  {
    votes[ classify_row( &frame->row[r], threshold ) ]++;
  }

  if (votes[2] >= DOUBLE_VOTES_NEEDED) { return 2; }
  if (votes[1] + votes[2] > 0)         { return 1; }
  return 0;
}

/* light exactly one LED */
void set_leds(
    struct io_peripherals * io,
    int                     classification )
{
  /* independent active-HIGH LEDs: SET = on, CLR = off
   * EMPTY -> RED (GPIO10), SINGLE -> GREEN (GPIO15), DOUBLE -> BLUE (GPIO14) */
  GPIO_CLR( io->gpio, LED_RED_GPIO );
  GPIO_CLR( io->gpio, LED_BLUE_GPIO );
  GPIO_CLR( io->gpio, LED_GREEN_GPIO );

  if (classification == 0)
  {
    GPIO_SET( io->gpio, LED_RED_GPIO );
  }
  else if (classification == 1)
  {
    GPIO_SET( io->gpio, LED_GREEN_GPIO );
  }
  else
  {
    GPIO_SET( io->gpio, LED_BLUE_GPIO );
  }
}

/* ------- change tracking state ------- */
static int  counts[3]  = { 0, 0, 0 };            /* empty, single, double */
static char history[HIST_SHOW][80];              /* rolling display list  */
static int  history_used  = 0;                   /* lines currently held  */
static int  total_changes = 0;                   /* numbering, 1..N       */

/* redraw counters + history list */
static void redraw_status( void )
{
  printf( "\x1B[%d;1HEmpty detected: %-4d Single detected: %-4d Double detected: %-4d\x1B[K",
      COUNT_ROW, counts[0], counts[1], counts[2] );

  printf( "\x1B[%d;1H--- detection history (first to last) ---\x1B[K", HIST_HDR_ROW );
  for (int i = 0; i < HIST_SHOW; i++)
  {
    printf( "\x1B[%d;1H%s\x1B[K",
        HIST_TOP_ROW + i,
        (i < history_used) ? history[i] : "" );
  }
  fflush( stdout );
}

/* register a confirmed state change: counters, history, screen, log file */
static void register_change(
    int  new_state,
    FILE * log_file )
{
  time_t     now = time( NULL );
  struct tm * tm = localtime( &now );

  counts[new_state]++;
  total_changes++;

  /* roll the display history up if full */
  if (history_used == HIST_SHOW)
  {
    for (int i = 1; i < HIST_SHOW; i++)
    {
      strcpy( history[i-1], history[i] );
    }
    history_used--;
  }
  snprintf( history[history_used], sizeof(history[0]),
      "%3d. [%02d:%02d:%02d] %s",
      total_changes, tm->tm_hour, tm->tm_min, tm->tm_sec,
      state_text[new_state] );
  history_used++;

  redraw_status();

  if (log_file != NULL)
  {
    fprintf( log_file, "%3d. [%02d:%02d:%02d] %s\n",
        total_changes, tm->tm_hour, tm->tm_min, tm->tm_sec,
        state_text[new_state] );
    fflush( log_file );
  }
}

/* void the most recently registered change (a sandwiched SINGLE):
 * remove it from the single count / plant total and from the history */
static void remove_last_change(
    FILE * log_file )
{
  if (total_changes > 0) { total_changes--; }
  if (counts[1]     > 0) { counts[1]--;     }   /* it was a SINGLE = a green */

  if (history_used > 0)
  {
    history_used--;
    history[history_used][0] = '\0';
  }

  redraw_status();

  if (log_file != NULL)
  {
    fprintf( log_file, "  -- voided transition SINGLE (frame placement) --\n" );
    fflush( log_file );
  }
}

int main( int argc, char ** argv )
{
  int                                       result;
  struct draw_bitmap_multiwindow_handle_t * bitmap_handle;
  FILE * serial_handle;
  FILE * log_file;
  struct io_peripherals * io;
  int                                       pressed_key;
  char                                      serial_line[1024];
  struct lidar_frame_t                      current_frame;
  struct lidar_frame_t                      frame_history[FILTER_DEPTH];
  struct lidar_frame_t                      filtered_frame;
  struct pixel_format_RGB                    bitmap[LIDAR_WIDTH * PIXEL_WIDTH * LIDAR_HEIGHT * PIXEL_HEIGHT];
  struct pixel_format_RGB                    color;

  /* debounce state */
  int confirmed_state      = -1;   /* -1 = nothing confirmed yet        */
  int prev_confirmed_state = -1;   /* the state before confirmed_state  */
  int candidate_state      = -1;   /* state currently being tested      */
  int candidate_count      = 0;    /* consecutive frames it held        */

  memset( &current_frame,  0, sizeof(current_frame) );
  memset( frame_history,   0, sizeof(frame_history) );
  memset( &filtered_frame, 0, sizeof(filtered_frame) );

  io = import_registers();
  if (io == NULL)
  {
    printf( "could not map I/O registers (run with sudo)\n" );
    return -1;
  }

  /* Clear function select bits for GPIO 10 (bits 0-2), 14 (bits 12-14), 15 (bits 15-17) in GPFSEL1 */
  /* and replace them with 001 (Output Mode) using direct register math to avoid padding issues */
  (*((volatile uint32_t *)&io->gpio->GPFSEL1)) &= ~( (7 << 0) | (7 << 12) | (7 << 15) );
  (*((volatile uint32_t *)&io->gpio->GPFSEL1)) |= ( (1 << 0) | (1 << 12) | (1 << 15) );

  GPIO_CLR( io->gpio, (1 << LED_RED_GPIO) );
  GPIO_CLR( io->gpio, (1 << LED_BLUE_GPIO) );
  GPIO_CLR( io->gpio, (1 << LED_GREEN_GPIO) );

  log_file = fopen( "detections.log", "a" );

  result = draw_bitmap_start( argc, argv );
  if (result == 0)
  {
    bitmap_handle = draw_bitmap_create_window( LIDAR_WIDTH * PIXEL_WIDTH, LIDAR_HEIGHT * PIXEL_HEIGHT );
    /* window is OPTIONAL: NULL handle = headless (SSH), everything else runs */
    {
      serial_handle = fopen( "/dev/ttyACM0", "r" );
      if (serial_handle != NULL)
      {
        unsigned int plant_threshold = 127;

        printf( "\x1B[2J\x1B[H\x1B[?25l" );
        printf( "\x1B[%d;1H--- distance (mm) ---\x1B[K", RAW_GRID_TOP - 1 );
        printf( "\x1B[%d;1H--- detection (1 = within 5 in) ---\x1B[K", BIN_GRID_TOP - 1 );
        printf( "\x1B[%d;1HEmpty detected: 0    Single detected: 0    Double detected: 0\x1B[K", COUNT_ROW );
        printf( "\x1B[%d;1H--- detection history (first to last) ---\x1B[K", HIST_HDR_ROW );
        fflush( stdout );

        while ( ((bitmap_handle == NULL) || !draw_bitmap_window_closed( bitmap_handle )) &&
                !wait_key( 1, &pressed_key))
        {
          my_getline( serial_handle, serial_line, sizeof(serial_line) );
          if ((serial_line[0] == 'y') &&
              (serial_line[1] >= '0') && (serial_line[1] <= '7') &&
              (strlen(serial_line) > 3))
          {
            int r = serial_line[1] - '0';

            sscanf( &serial_line[3], "%u,%u,%u,%u,%u,%u,%u,%u",
                &current_frame.row[r].pixel[0],
                &current_frame.row[r].pixel[1],
                &current_frame.row[r].pixel[2],
                &current_frame.row[r].pixel[3],
                &current_frame.row[r].pixel[4],
                &current_frame.row[r].pixel[5],
                &current_frame.row[r].pixel[6],
                &current_frame.row[r].pixel[7] );

            printf( "\x1B[%d;1H%d: %4u %4u %4u %4u %4u %4u %4u %4u\x1B[K",
                RAW_GRID_TOP + r, r,
                current_frame.row[r].pixel[0],
                current_frame.row[r].pixel[1],
                current_frame.row[r].pixel[2],
                current_frame.row[r].pixel[3],
                current_frame.row[r].pixel[4],
                current_frame.row[r].pixel[5],
                current_frame.row[r].pixel[6],
                current_frame.row[r].pixel[7] );

            printf( "\x1B[%d;1H%d:", BIN_GRID_TOP + r, r );
            for (int c = 0; c < LIDAR_WIDTH; c++)
            {
              printf( " %c", (current_frame.row[r].pixel[c] <= plant_threshold) ? '1' : '0' );
            }
            printf( "\x1B[K" );
            fflush( stdout );

            if (r == (LIDAR_HEIGHT - 1))
            {
              int frame_state = classify_frame( &current_frame, plant_threshold );

              /* ---- debounce: state must hold STABLE_FRAMES frames ---- */
              if (frame_state == candidate_state)
              {
                candidate_count++;
              }
              else
              {
                candidate_state = frame_state;
                candidate_count = 1;
              }

              if ((candidate_count >= STABLE_FRAMES) &&
                  (candidate_state != confirmed_state))
              {
                /* glitch suppression: EMPTY -> SINGLE -> EMPTY.
                 * When we settle back to EMPTY and the SINGLE we are
                 * leaving itself came from EMPTY, that SINGLE was just
                 * the frame being placed/removed. Void it. */
                if ((candidate_state      == 0) &&
                    (confirmed_state      == 1) &&
                    (prev_confirmed_state == 0))
                {
                  remove_last_change( log_file );
                  confirmed_state = 0;          /* back to EMPTY (already were) */
                  set_leds( io, 0 );            /* RED */
                  /* prev_confirmed_state stays 0; no new change registered */
                }
                else
                {
                  prev_confirmed_state = confirmed_state;
                  confirmed_state      = candidate_state;
                  set_leds( io, confirmed_state );
                  register_change( confirmed_state, log_file );
                }
              }

              /* black/white window (when a display exists) */
              filter_lidar_data( frame_history, FILTER_DEPTH, &current_frame, &filtered_frame );

              for (size_t row = 0; row < ARRAYSIZE(filtered_frame.row); row++)
              {
                for (size_t column = 0; column < ARRAYSIZE(filtered_frame.row[row].pixel); column++)
                {
                  calculate_color(
                      &color,
                      plant_threshold,
                      filtered_frame.row[row].pixel[column] );
                  draw_block(
                      bitmap,
                      LIDAR_WIDTH * PIXEL_WIDTH,
                      PIXEL_WIDTH,
                      PIXEL_HEIGHT,
                      column  * PIXEL_WIDTH,
                      row     * PIXEL_HEIGHT,
                      color );
                }
              }

              if (bitmap_handle != NULL)
              {
                draw_bitmap_display( bitmap_handle, bitmap );
              }

              usleep( 10000 ); /* 0.5s read throttle */
            }
          }
          else
          {
            ;
          }
        }

        printf( "\x1B[?25h\x1B[%d;1H", HIST_TOP_ROW + HIST_SHOW + 1 );
        printf( "==== STAND COUNT REPORT ====\n" );
        printf( "S: %d, D: %d, E: %d\n", counts[1], counts[2], counts[0] );
        if (log_file != NULL)
        {
          fprintf( log_file, "==== STAND COUNT REPORT ====\n" );
          fprintf( log_file, "S: %d, D: %d, E: %d\n", counts[1], counts[2], counts[0] );
          fflush( log_file );
        }

        fclose( serial_handle );
      }
      else
      {
        perror( "unable to open serial port /dev/ttyACM0" );
      }

      if (bitmap_handle != NULL)
      {
        draw_bitmap_close_window( bitmap_handle );
      }
    }

    draw_bitmap_stop();
  }
  else
  {
    printf( "could not start thread\n" );
  }

  if (log_file != NULL)
  {
    fclose( log_file );
  }

  GPIO_CLR( io->gpio, (1 << LED_RED_GPIO) );
  GPIO_CLR( io->gpio, (1 << LED_BLUE_GPIO) );
  GPIO_CLR( io->gpio, (1 << LED_GREEN_GPIO) );
  
  /* Reset direction to input on exit */
  (*((volatile uint32_t *)&io->gpio->GPFSEL1)) &= ~( (7 << 0) | (7 << 12) | (7 << 15) );

  return 0;
}
