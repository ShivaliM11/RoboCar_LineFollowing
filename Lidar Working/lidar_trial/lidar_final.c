/*
 * lidar_final.c
 *
 * Classification / debounce / state-machine logic:
 *   taken EXACTLY from lidar3.c (col_active, classify, cand/confirmed/
 *   holding/hold state machine, THRESH/GAP_COLS/STABLE/SINGLE_HOLD values,
 *   and the LED color mapping RED=empty, GREEN=single, BLUE=double).
 *
 * Display:
 *   style from lidar1.c -- raw distance grid, binary detection grid,
 *   a status line, a scrolling timestamped history, a log file
 *   (detections.log), and an optional live grayscale preview window --
 *   all constantly refreshed in place via ANSI cursor addressing, with
 *   ANSI colors matching whichever LED is lit.
 *
 *   The status line tracks the ACTUAL LED state (not just the confirmed
 *   classification): whenever the LEDs are off -- i.e. during the brief
 *   window after a single line is first seen but before it's been
 *   confirmed as single vs. double -- the display reads "Soil" instead
 *   of "single" or "empty".
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include <time.h>
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

/* ---- grid + classification tuning: verbatim from lidar3.c ---- */
#define W 8
#define H 8
#define THRESH 127
#define GAP_COLS 2          /* >=2 empty columns separates two lines   */
#define STABLE 2            /* frames to confirm a state               */
#define SINGLE_HOLD 30      /* frames to wait before confirming single */

/* LEDs (active HIGH) -- exact mapping from lidar3.c */
#define LED_RED   10        /* empty  */
#define LED_BLUE  14        /* double */
#define LED_GREEN 15        /* single */

/* ---- preview window sizing (from lidar1.c) ---- */
#define PIXEL_WIDTH  50
#define PIXEL_HEIGHT 50
#define FILTER_DEPTH 4

/* ---- terminal layout (from lidar1.c) ---- */
#define RAW_GRID_TOP   2
#define BIN_GRID_TOP  12
#define STATUS_ROW    (BIN_GRID_TOP + H + 1)
#define HIST_HDR_ROW  (STATUS_ROW + 2)
#define HIST_TOP_ROW  (HIST_HDR_ROW + 1)
#define HIST_SHOW     12     /* history lines kept visible */

/* ANSI colors matched to each LED */
#define COL_RED    "\x1B[31m"
#define COL_GREEN  "\x1B[32m"
#define COL_BLUE   "\x1B[34m"
#define COL_YELLOW "\x1B[33m"
#define COL_RESET  "\x1B[0m"

struct row_t   { unsigned int p[W]; };
struct frame_t { struct row_t row[H]; };

static const char *state_name[3]  = { "EMPTY", "single", "DOUBLE" };
static const char *state_color[3] = { COL_RED, COL_GREEN, COL_BLUE };

/* tracks what the LEDs are ACTUALLY set to: -1=off, 0=red, 1=green, 2=blue */
static int led_state = -1;

/* ============================================================
 * Classification -- EXACTLY as in lidar3.c
 * ============================================================ */

/* a column is active if any non-last row is within threshold */
static bool col_active(struct frame_t *f, int c){
  for(int r=0;r<H-1;r++)          /* ignore last row (always 1) */
    if(f->row[r].p[c]<=THRESH) return true;
  return false;
}

/* count vertical lines (column-groups) separated by >=GAP_COLS empty cols
 * works at ANY column position. 0=empty, 1=single, 2=double(2+ lines) */
static int classify(struct frame_t *f){
  int groups=0, gap=GAP_COLS; bool in=false;
  for(int c=0;c<W;c++){
    if(col_active(f,c)){
      if(!in){ groups++; in=true; }
      gap=0;
    } else {
      gap++;
      if(gap>=GAP_COLS) in=false;
    }
  }
  if(groups==0) return 0;
  if(groups==1) return 1;
  return 2;
}

static void rdline(FILE*f,char*b,size_t n){
  char*l=NULL;size_t z=0;ssize_t g;b[0]=0;
  if(!f)return; g=getline(&l,&z,f); if(g>0&&l&&(size_t)g<n)strcpy(b,l); free(l);
}

/* LED driver -- exact mapping/logic from lidar3.c (s=-1 clears all).
 * Also mirrors what it set into led_state, purely for display purposes. */
static void leds(struct io_peripherals*io,int s){
  GPIO_CLR(io->gpio,LED_RED);GPIO_CLR(io->gpio,LED_BLUE);GPIO_CLR(io->gpio,LED_GREEN);
  if(s==0)GPIO_SET(io->gpio,LED_RED);
  else if(s==1)GPIO_SET(io->gpio,LED_GREEN);
  else if(s==2)GPIO_SET(io->gpio,LED_BLUE);
  led_state = s;
}

/* ============================================================
 * Display + history + logging -- style from lidar1.c
 * ============================================================ */

static int  counts[3]      = {0,0,0};   /* [0]=empty [1]=single [2]=double */
static char history[HIST_SHOW][96];
static int  history_used   = 0;
static int  total_changes  = 0;

/* redraw the status line + the scrolling history block.
 * The status line reflects the ACTUAL led_state: LED off -> "Soil". */
static void redraw_status(void){
  const char *name, *color;

  switch(led_state){
    case 0:  name = "empty";  color = COL_RED;   break;
    case 1:  name = "single"; color = COL_GREEN; break;
    case 2:  name = "DOUBLE"; color = COL_BLUE;  break;
    default: name = "Soil";   color = COL_YELLOW; break;   /* LED off */
  }

  printf("\x1B[%d;1HState:%s%s%s  E:%d S:%d D:%d\x1B[K",
      STATUS_ROW, color, name, COL_RESET, counts[0], counts[1], counts[2]);

  printf("\x1B[%d;1H--- detection history (first to last) ---\x1B[K", HIST_HDR_ROW);
  for(int i=0;i<HIST_SHOW;i++){
    printf("\x1B[%d;1H%s\x1B[K", HIST_TOP_ROW+i, (i<history_used)?history[i]:"");
  }
  fflush(stdout);
}

/* record a confirmed state change: bump its counter, push it into the
 * scrolling history, and append it to the log file. Display refresh is
 * handled separately (once per frame) by redraw_status(). */
static void register_change(int state, FILE *log_file){
  time_t     now = time(NULL);
  struct tm *tm  = localtime(&now);

  counts[state]++;
  total_changes++;

  if(history_used==HIST_SHOW){
    for(int i=1;i<HIST_SHOW;i++) strcpy(history[i-1], history[i]);
    history_used--;
  }
  snprintf(history[history_used], sizeof(history[0]),
      "%3d. [%02d:%02d:%02d] %s%s%s",
      total_changes, tm->tm_hour, tm->tm_min, tm->tm_sec,
      state_color[state], state_name[state], COL_RESET);
  history_used++;

  if(log_file){
    fprintf(log_file, "%3d. [%02d:%02d:%02d] %s\n",
        total_changes, tm->tm_hour, tm->tm_min, tm->tm_sec, state_name[state]);
    fflush(log_file);
  }
}

/* ---- optional live grayscale preview window (from lidar1.c) ---- */

static void draw_block(struct pixel_format_RGB *bitmap, size_t bitmap_width,
    size_t block_width, size_t block_height,
    size_t x_offset, size_t y_offset, struct pixel_format_RGB color){
  for(size_t y=y_offset;y<y_offset+block_height;y++)
    for(size_t x=x_offset;x<x_offset+block_width;x++)
      bitmap[(y*bitmap_width)+x]=color;
}

static void calculate_color(struct pixel_format_RGB *color, size_t threshold, size_t value){
  unsigned char gray = (value<=threshold)?0:255;
  color->R=gray; color->G=gray; color->B=gray;
}

static void filter_lidar_data(struct frame_t *hist_arr, size_t depth,
    struct frame_t *newest, struct frame_t *filtered){
  for(size_t i=1;i<depth;i++) hist_arr[i-1]=hist_arr[i];
  hist_arr[depth-1]=*newest;

  for(int row=0; row<H; row++){
    for(int col=0; col<W; col++){
      unsigned int sum=0;
      for(size_t i=0;i<depth;i++) sum += hist_arr[i].row[row].p[col];
      filtered->row[row].p[col] = sum/depth;
    }
  }
}

int main(void){
  struct io_peripherals *io = import_registers();
  if(!io){ printf("run with sudo\n"); return -1; }

  (*((volatile uint32_t*)&io->gpio->GPFSEL1)) &= ~((7<<0)|(7<<12)|(7<<15));
  (*((volatile uint32_t*)&io->gpio->GPFSEL1)) |=  ((1<<0)|(1<<12)|(1<<15));
  leds(io,-1);

  FILE *s = fopen("/dev/ttyACM0","r");
  if(!s){ printf("cannot open /dev/ttyACM0\n"); return -1; }

  FILE *log_file = fopen("detections.log","a");

  /* optional preview window -- headless (SSH) still works if this fails */
  int result = draw_bitmap_start(0, NULL);
  struct draw_bitmap_multiwindow_handle_t *bitmap_handle = NULL;
  static struct pixel_format_RGB bitmap[W*PIXEL_WIDTH*H*PIXEL_HEIGHT];
  struct frame_t frame_history[FILTER_DEPTH];
  struct frame_t filtered_frame;
  memset(frame_history,0,sizeof(frame_history));
  memset(&filtered_frame,0,sizeof(filtered_frame));

  if(result==0){
    bitmap_handle = draw_bitmap_create_window(W*PIXEL_WIDTH, H*PIXEL_HEIGHT);
  }

  char line[1024];
  struct frame_t fr; memset(&fr,0,sizeof(fr));
  int confirmed=-1, cand=-1, cand_n=0, hold=0; bool holding=false;
  int key;

  printf("\x1B[2J\x1B[H\x1B[?25l");
  printf("\x1B[%d;1H--- distance (mm) ---\x1B[K", RAW_GRID_TOP-1);
  printf("\x1B[%d;1H--- detection (1 = within 5 in) ---\x1B[K", BIN_GRID_TOP-1);
  redraw_status();

  while( ((bitmap_handle==NULL) || !draw_bitmap_window_closed(bitmap_handle)) &&
         !wait_key(1,&key) ){
    if(key=='q') break;

    rdline(s,line,sizeof(line));
    if(!(line[0]=='y'&&line[1]>='0'&&line[1]<='7'&&strlen(line)>3)) continue;
    int r=line[1]-'0';
    sscanf(&line[3],"%u,%u,%u,%u,%u,%u,%u,%u",
      &fr.row[r].p[0],&fr.row[r].p[1],&fr.row[r].p[2],&fr.row[r].p[3],
      &fr.row[r].p[4],&fr.row[r].p[5],&fr.row[r].p[6],&fr.row[r].p[7]);

    /* raw distance row (constantly refreshed in place) */
    printf("\x1B[%d;1H%d:", RAW_GRID_TOP+r, r);
    for(int c=0;c<W;c++) printf(" %4u", fr.row[r].p[c]);
    printf("\x1B[K");

    /* binary detection row, colored green where a hit is seen */
    printf("\x1B[%d;1H%d:", BIN_GRID_TOP+r, r);
    for(int c=0;c<W;c++){
      bool on = fr.row[r].p[c] <= THRESH;
      if(on) printf(" %s1%s", COL_GREEN, COL_RESET);
      else   printf(" 0");
    }
    printf("\x1B[K");
    fflush(stdout);

    if(r!=H-1) continue;

    /* ---- state machine: EXACTLY the logic from lidar3.c ---- */
    int st=classify(&fr);
    if(st==cand) cand_n++; else { cand=st; cand_n=1; }

    if(cand_n>=STABLE && cand!=confirmed){
      if(cand==2){                        /* two lines -> DOUBLE now */
        holding=false; confirmed=2; leds(io,2);
        register_change(2, log_file);
      } else if(cand==1){                 /* one line -> hold, watch for 2nd */
        holding=true; hold=0; confirmed=1; leds(io,-1);
      } else {                            /* no lines -> empty */
        if(holding){
          holding=false;
          register_change(1, log_file);   /* finalize the pending single */
        }
        confirmed=0; leds(io,0);
        register_change(0, log_file);
      }
    }

    /* while holding a single: if a 2nd line shows up -> double; else after
     * the window, confirm green */
    if(holding){
      int now=classify(&fr);
      if(now==2){                          /* 2nd line appeared */
        holding=false; confirmed=2; leds(io,2);
        register_change(2, log_file);
      } else {
        hold++;
        if(hold>=SINGLE_HOLD){              /* stayed single -> confirmed */
          holding=false; leds(io,1);
          register_change(1, log_file);
        }
      }
    }

    /* status line + history: always reflects the REAL led_state, so LED
     * off during the holding window reads "Soil" instead of single/empty */
    redraw_status();

    /* ---- optional live grayscale preview window ---- */
    filter_lidar_data(frame_history, FILTER_DEPTH, &fr, &filtered_frame);
    if(bitmap_handle != NULL){
      struct pixel_format_RGB color;
      for(int row=0; row<H; row++){
        for(int col=0; col<W; col++){
          calculate_color(&color, THRESH, filtered_frame.row[row].p[col]);
          draw_block(bitmap, W*PIXEL_WIDTH, PIXEL_WIDTH, PIXEL_HEIGHT,
              col*PIXEL_WIDTH, row*PIXEL_HEIGHT, color);
        }
      }
      draw_bitmap_display(bitmap_handle, bitmap);
    }
  }

  printf("\x1B[?25h\x1B[%d;1H", HIST_TOP_ROW + HIST_SHOW + 1);
  printf("==== FINAL ==== S:%d D:%d E:%d\n", counts[1], counts[2], counts[0]);
  if(log_file){
    fprintf(log_file, "==== FINAL ==== S:%d D:%d E:%d\n", counts[1], counts[2], counts[0]);
    fflush(log_file);
    fclose(log_file);
  }

  fclose(s);
  if(bitmap_handle != NULL) draw_bitmap_close_window(bitmap_handle);
  draw_bitmap_stop();

  leds(io,-1);
  (*((volatile uint32_t*)&io->gpio->GPFSEL1)) &= ~((7<<0)|(7<<12)|(7<<15));

  return 0;
}
