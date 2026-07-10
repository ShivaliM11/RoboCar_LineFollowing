/* latest working lidar */
/*
 * lidar1.c -- hard black/white lidar display with startup calibration
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include <cairo/cairo.h>
#include <gtk/gtk.h>
#include <gdk/gdk.h>
#include "pixel_format_RGB.h"
#include "draw_bitmap_multiwindow.h"
#include "wait_key.h"

#define ARRAYSIZE(X) (sizeof(X) / sizeof((X)[0]))

#define LIDAR_WIDTH  8
#define LIDAR_HEIGHT 8
#define PIXEL_WIDTH  50
#define PIXEL_HEIGHT 50

#define FILTER_DEPTH 4

struct lidar_frame_row_t
{
    unsigned int pixel[LIDAR_WIDTH];
};
struct lidar_frame_t
{
    struct lidar_frame_row_t row[LIDAR_HEIGHT];
};

/* read bytes from the file until newline or buffer_length is reached */
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
    return;   /* nothing to read from; leave buffer empty */
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

/* Hard threshold color: distance at or nearer than threshold => BLACK (plant),
 * anything farther => WHITE (background). */
void calculate_color(
    struct pixel_format_RGB * color,
    size_t                    threshold,
    size_t                    value )
{
  unsigned char gray;

  if (value <= threshold)
  {
    gray = 0;     /* close -> black (plant detected) */
  }
  else
  {
    gray = 255;   /* far   -> white (background)     */
  }

  color->R = gray;
  color->G = gray;
  color->B = gray;

  return;
}

void filter_lidar_data(
    struct lidar_frame_t *  lidar_frame_history,
    size_t                  history_depth,
    struct lidar_frame_t *  newest_frame,
    struct lidar_frame_t *  filtered_frame )
{
  /* age the history */
  for (size_t i = 1; i < history_depth; i++)
  {
    lidar_frame_history[i-1] = lidar_frame_history[i];
  }

  /* add new history entry */
  lidar_frame_history[history_depth-1] = *newest_frame;

  /* simple averaging filter */
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

/* One-time startup calibration with EMPTY slot. */
#define BASELINE_FRAMES   8
#define BASELINE_MARGIN   150   /* mm */

unsigned int calibrate_baseline( FILE * serial_handle )
{
  unsigned long sum      = 0;
  unsigned int  count    = 0;
  int           done     = 0;
  int           attempts = 0;

  if (serial_handle == NULL)
  {
    return 127;   /* no serial; fixed 5-inch fallback */
  }

  printf( "Calibrating -- aim sensor at EMPTY slot, press Enter...\n" );
  getchar();

  while (done < BASELINE_FRAMES)
  {
    char *  ln   = NULL;
    size_t  len  = 0;
    ssize_t read;

    read = getline( &ln, &len, serial_handle );
    if ((read <= 0) || (ln == NULL))
    {
      free( ln );
      attempts++;
      if (attempts > 1000) { break; }
      continue;
    }

    if ((ln[0] == 'y') && (ln[1] >= '0') && (ln[1] <= '7') && (read > 3))
    {
      unsigned int v[8] = {4000,4000,4000,4000,4000,4000,4000,4000};
      sscanf( &ln[3], "%u,%u,%u,%u,%u,%u,%u,%u",
              &v[0],&v[1],&v[2],&v[3],&v[4],&v[5],&v[6],&v[7] );
      for (int c = 0; c < 8; c++)
      {
        if (v[c] < 4000) { sum += v[c]; count++; }
      }
      if (ln[1] == '7') { done++; }
    }
    free( ln );

    attempts++;
    if (attempts > 1000) { break; }
  }

  unsigned int baseline  = count ? (unsigned int)(sum / count) : 1000;
  unsigned int threshold = (baseline > BASELINE_MARGIN)
                            ? (baseline - BASELINE_MARGIN)
                            : baseline / 2;
  printf( "Baseline: %u mm  threshold set to: %u mm\n", baseline, threshold );
  return threshold;
}

int main( int argc, char ** argv )
{
  int                                       result;
  struct draw_bitmap_multiwindow_handle_t * bitmap_handle;
  FILE *                                    serial_handle;
  int                                       pressed_key;
  char                                      serial_line[1024];
  struct lidar_frame_t                      current_frame;
  struct lidar_frame_t                      frame_history[FILTER_DEPTH];
  struct lidar_frame_t                      filtered_frame;
  struct pixel_format_RGB                   bitmap[LIDAR_WIDTH * PIXEL_WIDTH * LIDAR_HEIGHT * PIXEL_HEIGHT];
  struct pixel_format_RGB                   color;

  memset( &current_frame,  0, sizeof(current_frame) );
  memset( frame_history,   0, sizeof(frame_history) );
  memset( &filtered_frame, 0, sizeof(filtered_frame) );

  result = draw_bitmap_start( argc, argv );
  if (result == 0)
  {
    bitmap_handle = draw_bitmap_create_window( LIDAR_WIDTH * PIXEL_WIDTH, LIDAR_HEIGHT * PIXEL_HEIGHT );
    if (bitmap_handle != NULL)
    {
      serial_handle = fopen( "/dev/ttyACM0", "r" );
      if (serial_handle != NULL)
      {
        /* calibrate once at startup with empty slot */
        unsigned int plant_threshold = 127;   /* fixed 5-inch threshold, no calibration */

        while ( !draw_bitmap_window_closed( bitmap_handle ) &&
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

            /* raw distance grid, terminal rows 1-8 */
            printf( "%c[%d;0H%d: %4u, %4u, %4u, %4u, %4u, %4u, %4u, %4u\n",
                0x1B, r + 1, r,
                current_frame.row[r].pixel[0],
                current_frame.row[r].pixel[1],
                current_frame.row[r].pixel[2],
                current_frame.row[r].pixel[3],
                current_frame.row[r].pixel[4],
                current_frame.row[r].pixel[5],
                current_frame.row[r].pixel[6],
                current_frame.row[r].pixel[7] );

            /* 0/1 detection grid, terminal rows 10-17 */
            printf( "%c[%d;0H%d:", 0x1B, r + LIDAR_HEIGHT + 2, r );
            for (int c = 0; c < LIDAR_WIDTH; c++)
            {
              printf( " %c", (current_frame.row[r].pixel[c] <= plant_threshold) ? '1' : '0' );
            }
            printf( "\n" );

            if (r == (LIDAR_HEIGHT - 1))
            {
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

              draw_bitmap_display( bitmap_handle, bitmap );
            }
          }
          else
          {
            ; /* throw out the partial line */
          }
        }

        fclose( serial_handle );
      }
      else
      {
        perror( "unable to open serial port /dev/ttyACM0" );
      }

      draw_bitmap_close_window( bitmap_handle );
    }
    else
    {
      printf( "could not create window\n" );
    }

    draw_bitmap_stop();
  }
  else
  {
    printf( "could not start thread\n" );
  }

  return 0;
}
