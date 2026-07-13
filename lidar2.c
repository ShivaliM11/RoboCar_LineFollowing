// lidar2.c -- lidar plant detection, stops the car on a double
// 8x8 grid, 3 inch threshold, last row ignored (always 1 from mounting)
// single is held ~0.5s before commiting it, so double isn't seen as single first
// on a confirmed double we raise the STOP pin so the motor code halts

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

// prints on terminal
#define RAW_GRID_TOP   2
#define BIN_GRID_TOP  12
#define STATUS_ROW    (BIN_GRID_TOP + LIDAR_HEIGHT + 1)
#define COUNT_ROW     (STATUS_ROW)
#define HIST_HDR_ROW  (COUNT_ROW + 2)
#define HIST_TOP_ROW  (HIST_HDR_ROW + 1)
#define HIST_SHOW     12

// LED pins
#define LED_RED_GPIO    10   // empty
#define LED_BLUE_GPIO   14   // double
#define LED_GREEN_GPIO  15   // single
#define STOP_GPIO       26   // high = tell the motor to stop

// tuning
#define GAP_MIN          3
#define DOUBLE_VOTES_NEEDED  3
#define STABLE_FRAMES        3
#define SINGLE_CONFIRM_FRAMES  50   // 0.5s hold before a single counts

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

// check one row: 0=empty, 1=single, 2=double
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

// see across the rows, skip the last one
int classify_frame(
    struct lidar_frame_t * frame,
    unsigned int           threshold )
{
  int votes[3] = { 0, 0, 0 };

  for (int r = 0; r < LIDAR_HEIGHT - 1; r++)   // last row is always 1, ignore it
  {
    votes[ classify_row( &frame->row[r], threshold ) ]++;
  }

  if (votes[2] >= DOUBLE_VOTES_NEEDED) { return 2; }
  if (votes[1] + votes[2] > 0)         { return 1; }
  return 0;
}

// turn on one LED (-1 means all off while we're still holding a single)
void set_leds(
    struct io_peripherals * io,
    int                     classification )
{
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
  else if (classification == 2)
  {
    GPIO_SET( io->gpio, LED_BLUE_GPIO );
  }
}

// counters + history
static int  counts[3]  = { 0, 0, 0 };
static char history[HIST_SHOW][80];
static int  history_used  = 0;
static int  total_changes = 0;

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

// log a real state change
static void register_change(
    int  new_state,
    FILE * log_file )
{
  time_t     now = time( NULL );
  struct tm * tm = localtime( &now );

  counts[new_state]++;
  total_changes++;

  if (history_used == HIST_SHOW)   // list is full, scroll it up
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

// commit a single, but skip it if it came right after a double
// (that single is just the double sliding out of the frame)
static void commit_single( int prev_state, FILE * log_file )
{
  if (prev_state == 2)   // was a double a moment ago -> trailing stalk, drop it
  {
    if (log_file != NULL)
    {
      fprintf( log_file, "  -- voided trailing SINGLE (double leaving frame) --\n" );
      fflush( log_file );
    }
    return;
  }
  register_change( 1, log_file );
}

// undo the last single (frame-placement glitch)
static void remove_last_change(
    FILE * log_file )
{
  if (total_changes > 0) { total_changes--; }
  if (counts[1]     > 0) { counts[1]--;     }

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

// runs once when a double is confirmed -> stop the car
static void on_double_confirmed( struct io_peripherals * io, FILE * log_file )
{
  GPIO_SET( io->gpio, STOP_GPIO );   // raise stop pin, motor code halts
  if (log_file != NULL)
  {
    fprintf( log_file, "  >> DOUBLE confirmed: STOP pin high\n" );
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

  // debounce
  int confirmed_state      = -1;
  int prev_confirmed_state = -1;
  int candidate_state      = -1;
  int candidate_count      = 0;

  // single hold window
  bool holding_single      = false;
  int  hold_frames         = 0;
  int  hold_prev_state     = -1;   // what state came before this held single

  memset( &current_frame,  0, sizeof(current_frame) );
  memset( frame_history,   0, sizeof(frame_history) );
  memset( &filtered_frame, 0, sizeof(filtered_frame) );

  io = import_registers();
  if (io == NULL)
  {
    printf( "could not map I/O registers (run with sudo)\n" );
    return -1;
  }

  // set LED pins 10,14,15 to output
  (*((volatile uint32_t *)&io->gpio->GPFSEL1)) &= ~( (7 << 0) | (7 << 12) | (7 << 15) );
  (*((volatile uint32_t *)&io->gpio->GPFSEL1)) |= ( (1 << 0) | (1 << 12) | (1 << 15) );

  GPIO_CLR( io->gpio, (1 << LED_RED_GPIO) );
  GPIO_CLR( io->gpio, (1 << LED_BLUE_GPIO) );
  GPIO_CLR( io->gpio, (1 << LED_GREEN_GPIO) );

  // set stop pin 26 to output and start it low
  (*((volatile uint32_t *)&io->gpio->GPFSEL2)) &= ~(7 << 18);
  (*((volatile uint32_t *)&io->gpio->GPFSEL2)) |=  (1 << 18);
  GPIO_CLR( io->gpio, STOP_GPIO );

  log_file = fopen( "detections.log", "a" );

  result = draw_bitmap_start( argc, argv );
  if (result == 0)
  {
    bitmap_handle = draw_bitmap_create_window( LIDAR_WIDTH * PIXEL_WIDTH, LIDAR_HEIGHT * PIXEL_HEIGHT );
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

            if (r == (LIDAR_HEIGHT - 1))   // gets whole frame now
            {
              int frame_state = classify_frame( &current_frame, plant_threshold );

              // wait few frames so a flicker doesn't count
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
                int new_state = candidate_state;

                if (new_state == 1)   // saw a single -> hold it, don't count yet
                {
                  holding_single = true;
                  hold_frames    = 0;
                  hold_prev_state = confirmed_state;   // remember what came before
                  set_leds( io, -1 );   // LEDs off while holding
                  prev_confirmed_state = confirmed_state;
                  confirmed_state = 1;
                }
                else if (new_state == 2)   // double wins -> count it, stop car
                {
                  holding_single = false;
                  hold_frames    = 0;
                  prev_confirmed_state = confirmed_state;
                  confirmed_state = 2;
                  set_leds( io, 2 );
                  register_change( 2, log_file );
                  on_double_confirmed( io, log_file );
                }
                else   // empty
                {
                  if (holding_single)   // single scrolled away -> it was real, count it
                  {
                    holding_single = false;
                    hold_frames    = 0;
                    set_leds( io, 1 );
                    commit_single( hold_prev_state, log_file );
                  }

                  // empty -> single -> empty means it was just the frame, drop it
                  if ((confirmed_state == 1) && (prev_confirmed_state == 0))
                  {
                    remove_last_change( log_file );
                  }

                  prev_confirmed_state = confirmed_state;
                  confirmed_state = 0;
                  set_leds( io, 0 );
                  register_change( 0, log_file );
                }
              }

              // count down the single hold
              if (holding_single)
              {
                hold_frames++;
                if (hold_frames >= SINGLE_CONFIRM_FRAMES)   // time up, still single
                {
                  holding_single = false;
                  hold_frames    = 0;
                  set_leds( io, 1 );
                  commit_single( hold_prev_state, log_file );
                }
              }

              // grayscale window if a display is plugged in
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
  GPIO_CLR( io->gpio, STOP_GPIO );   // let the car move again on exit

  (*((volatile uint32_t *)&io->gpio->GPFSEL1)) &= ~( (7 << 0) | (7 << 12) | (7 << 15) );

  return 0;
}
