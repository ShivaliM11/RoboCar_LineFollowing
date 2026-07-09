/*
 * main.cpp
 *
 *  Created on: Apr 11, 2026
 *      Author: steveb
 */

#include <stdio.h>
#include <stdlib.h>
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

  read = getline( &line, &len, file );
  if (read < buffer_length)
  {
    strcpy( buffer, line );
  }
  else
  {
    buffer[0] = '\0';
  }
  free( line );

  return;
}

void draw_block(
    struct pixel_format_RGB * bitmap,       // the bitmap to be drawn to
    size_t                    bitmap_width, // the width of the bitmap (height not needed)
    size_t                    block_width,  // the width of the block to draw in pixels
    size_t                    block_height, // the height of the block to draw in pixels
    size_t                    x_offset,     // the X offset where the block is to be drawn in pixels
    size_t                    y_offset,     // the Y offset where the block is to be drawn in pixels
    struct pixel_format_RGB   color )       // the color to draw the block
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
    struct pixel_format_RGB * color,      // the result of the color calculation
    size_t                    min_value,  // the minimum value of the value's range
    size_t                    max_value,  // the maximum value of the value's range
    size_t                    value )     // the value whose color is to be computed
{
  size_t range = max_value - min_value;
  size_t clamped_value;
  unsigned char gray;

  // clamp value into [min_value, max_value] in case sensor noise sends
  // something slightly out of range
  if (value < min_value)
  {
    clamped_value = min_value;
  }
  else if (value > max_value)
  {
    clamped_value = max_value;
  }
  else
  {
    clamped_value = value;
  }

  // black = close (small value, plant detected), white = far (background)
  // this is a direct linear map, no sine-wave color scheme needed
  gray = (unsigned char)(255 * (clamped_value - min_value) / range);

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
  // age the history
  for (size_t i = 1; i < history_depth; i++)
  {
    lidar_frame_history[i-1] = lidar_frame_history[i];
  }

  // add new history entry
  lidar_frame_history[history_depth-1] = *newest_frame;

#if 1
  // filter the frame, just a simple averaging filter
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
#else
  // filter the frame, this is a deglitch algorithm, not a "filter"
  for (size_t row = 0; row < ARRAYSIZE(filtered_frame->row); row++)
  {
    for (size_t column = 0; column < ARRAYSIZE(filtered_frame->row[row].pixel); column++)
    {
      unsigned int historical_average;

      historical_average = 0;
      for (size_t i = 0; i < (history_depth - 1); i++)
      {
        historical_average += lidar_frame_history[i].row[row].pixel[column];
      }
      historical_average = historical_average / (history_depth - 1);

      if ((newest_frame->row[row].pixel[column] > (0.75 * historical_average)) ||
          (newest_frame->row[row].pixel[column] < (0.25 * historical_average)))
      {
        filtered_frame->row[row].pixel[column] = historical_average;
      }
      else
      {
        filtered_frame->row[row].pixel[column] = newest_frame->row[row].pixel[column];
      }
    }
  }
#endif
}

/* One-time startup calibration.
 * Call with EMPTY slot (no plant). Reads BASELINE_FRAMES frames, averages
 * ROI pixel distances, returns threshold = average - MARGIN.
 * Anything closer than this threshold = plant (black).
 * Anything farther = background (white). */
#define BASELINE_FRAMES   8
#define BASELINE_MARGIN   150   /* mm -- lower = stricter, higher = more sensitive */

static unsigned int calibrate_baseline( FILE * serial_handle )
{
  char         line[1024];
  unsigned long sum   = 0;
  unsigned int  count = 0;
  int           done  = 0;

  printf( "Calibrating -- aim sensor at EMPTY slot, press Enter...\n" );
  getchar();

  while (done < BASELINE_FRAMES)
  {
    char * ln = NULL;
    size_t len = 0;
    getline( &ln, &len, serial_handle );
    if (ln && ln[0] == 'y')
    {
      unsigned int v[8];
      sscanf( &ln[3], "%u,%u,%u,%u,%u,%u,%u,%u",
              &v[0],&v[1],&v[2],&v[3],&v[4],&v[5],&v[6],&v[7] );
      for (int c = 0; c < 8; c++)
        if (v[c] < 4000) { sum += v[c]; count++; }
      if (ln[1] == '7') done++;
    }
    free(ln);
  }

  unsigned int baseline = count ? (unsigned int)(sum / count) : 1000;
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
#if 1
  char                                      serial_line[1024];
  struct lidar_frame_t                      current_frame;
  struct lidar_frame_t                      frame_history[FILTER_DEPTH];
  struct lidar_frame_t                      filtered_frame;
#endif
  struct pixel_format_RGB                   bitmap[LIDAR_WIDTH * PIXEL_WIDTH * LIDAR_HEIGHT * PIXEL_HEIGHT];
  struct pixel_format_RGB                   color;

  result = draw_bitmap_start( argc, argv );
  if (result == 0)
  {
    bitmap_handle = draw_bitmap_create_window( LIDAR_WIDTH * PIXEL_WIDTH, LIDAR_HEIGHT * PIXEL_HEIGHT );
    if (bitmap_handle != NULL)
    {
      serial_handle = fopen( "/dev/ttyACM0", "r" ); // this device continuously outputs serial data
      if (serial_handle >= 0)
      {
        /* calibrate once at startup with empty slot -- sets plant_threshold */
        unsigned int plant_threshold = calibrate_baseline( serial_handle );

#if 1
        // draw the bitmap until the window is closed or a key is pressed
        while ( !draw_bitmap_window_closed( bitmap_handle ) &&
                !wait_key( 1, &pressed_key))
        {
          /* What the data looks like:
           * y0:1229,1474,4000,4000,4000,4000,4000,4000,
           *
           * y1:1504,4000,4000,4000,4000,4000,4000,4000,
           *
           * y2:4000,4000,4000,4000,4000,4000,4000,4000,
           */
          my_getline( serial_handle, serial_line, sizeof(serial_line) );
          if (serial_line[0] == 'y')
          {
            // got a full line of text, parse the numbers
            sscanf( &serial_line[3], "%u,%u,%u,%u,%u,%u,%u,%u",
                &current_frame.row[serial_line[1] - '0'].pixel[0],
                &current_frame.row[serial_line[1] - '0'].pixel[1],
                &current_frame.row[serial_line[1] - '0'].pixel[2],
                &current_frame.row[serial_line[1] - '0'].pixel[3],
                &current_frame.row[serial_line[1] - '0'].pixel[4],
                &current_frame.row[serial_line[1] - '0'].pixel[5],
                &current_frame.row[serial_line[1] - '0'].pixel[6],
                &current_frame.row[serial_line[1] - '0'].pixel[7] );

            printf( "%c[%c;0H%c: %4.0u, %4.0u, %4.0u, %4.0u, %4.0u, %4.0u, %4.0u, %4.0u\n",
                0x1B,
                serial_line[1],
                serial_line[1],
                current_frame.row[serial_line[1] - '0'].pixel[0],
                current_frame.row[serial_line[1] - '0'].pixel[1],
                current_frame.row[serial_line[1] - '0'].pixel[2],
                current_frame.row[serial_line[1] - '0'].pixel[3],
                current_frame.row[serial_line[1] - '0'].pixel[4],
                current_frame.row[serial_line[1] - '0'].pixel[5],
                current_frame.row[serial_line[1] - '0'].pixel[6],
                current_frame.row[serial_line[1] - '0'].pixel[7] );

            if (serial_line[1] == '7')
            {
              // filter the data
              filter_lidar_data( frame_history, FILTER_DEPTH, &current_frame, &filtered_frame );

              // populate the bitmap
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

              // draw the bitmap
              draw_bitmap_display( bitmap_handle, bitmap );
            }
            else
            {
              ; // not yet at the end of the current scan
            }
          }
          else
          {
            ; // throw out the partial line
          }
        }
#else
        for (size_t y = 0; y < LIDAR_HEIGHT; y++)
        {
          for (size_t x = 0; x < LIDAR_WIDTH; x++)
          {
            calculate_color(
                &color,
                plant_threshold,
                4000 / 64 * (y * LIDAR_WIDTH + x) );
            draw_block(
                bitmap,
                LIDAR_WIDTH * PIXEL_WIDTH,
                PIXEL_WIDTH,
                PIXEL_HEIGHT,
                x * PIXEL_WIDTH,
                y * PIXEL_HEIGHT,
                color );
          }
        }
        draw_bitmap_display( bitmap_handle, bitmap );
        while ( !draw_bitmap_window_closed( bitmap_handle ) &&
                !wait_key( 1, &pressed_key))
        {
        }
#endif

        fclose( serial_handle );
      }
      else
      {
        printf( "unable to open serial port\n" );
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
