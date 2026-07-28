#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <linux/videodev2.h>
#include <time.h>
#include <gtk/gtk.h>
#include "pixel_format_RGB.h"
#include "video_interface.h"
#include "wait_key.h"
#include "scale_image_data.h"

#define SCALE_REDUCTION_PER_AXIS  1

/* ---- Region sweep tuning (same idea as original single/double detector) ---- */
#define MIN_REGION_PLANT_PIXELS   20
#define MAX_EMPTY_RUN             10
#define MIN_REGION_WIDTH          20
#define MAX_REGIONS               10

/* ---- Exact RGB ranges (as measured/specified) ---- */
#define YELLOW_R_MIN   180
#define YELLOW_R_MAX   255
#define YELLOW_G_MIN   140
#define YELLOW_G_MAX   255
#define YELLOW_B_MIN   0
#define YELLOW_B_MAX   100

#define BROWN_R_MIN    60
#define BROWN_R_MAX    165
#define BROWN_G_MIN    30
#define BROWN_G_MAX    110
#define BROWN_B_MIN    0
#define BROWN_B_MAX    60

#define GREEN_MIN_LEVEL     40
#define GREEN_MARGIN_R      20
#define GREEN_MARGIN_B      20

#define COLOR_NONE    0
#define COLOR_GREEN   1
#define COLOR_YELLOW  2

#define STATE_NONE    0
#define STATE_GREEN   1
#define STATE_YELLOW  2

static GtkWidget *window;
static GtkWidget *image_widget;
static GtkWidget *status_label;
static GtkWidget *total_counts_label;
static GdkPixbuf *pixbuf = NULL;

static unsigned long total_green_detections  = 0;
static unsigned long total_yellow_detections = 0;

static void on_window_destroy(GtkWidget *widget, gpointer data)
{
    gtk_main_quit();
}

static inline bool in_range(int v, int lo, int hi)
{
    return v >= lo && v <= hi;
}

static inline bool is_yellow(int R, int G, int B)
{
    return in_range(R, YELLOW_R_MIN, YELLOW_R_MAX) &&
           in_range(G, YELLOW_G_MIN, YELLOW_G_MAX) &&
           in_range(B, YELLOW_B_MIN, YELLOW_B_MAX);
}

static inline bool is_brown(int R, int G, int B)
{
    return in_range(R, BROWN_R_MIN, BROWN_R_MAX) &&
           in_range(G, BROWN_G_MIN, BROWN_G_MAX) &&
           in_range(B, BROWN_B_MIN, BROWN_B_MAX);
}

static inline bool is_green(int R, int G, int B)
{
    return G >= GREEN_MIN_LEVEL && (G - R) >= GREEN_MARGIN_R && (G - B) >= GREEN_MARGIN_B;
}

static int classify_pixel(unsigned char R, unsigned char G, unsigned char B)
{
    if (is_brown(R, G, B))
    {
        return COLOR_NONE;
    }

    if (is_yellow(R, G, B))
    {
        return COLOR_YELLOW;
    }

    if (is_green(R, G, B))
    {
        return COLOR_GREEN;
    }

    return COLOR_NONE;
}

int main( int argc, char * argv[] )
{
  struct video_interface_handle_t * handle;
  static struct image_t               image;
  unsigned char * scaled_data;
  struct pixel_format_RGB * scaled_RGB_data;
  unsigned int                       scaled_height;
  unsigned int                       scaled_width;

  int stable_state = STATE_NONE;
  int candidate_state = STATE_NONE;
  int consecutive_frames = 0;

  gtk_init(&argc, &argv);

  handle = video_interface_open( "/dev/video0" );

  if (video_interface_set_mode_auto( handle ))
  {
    scaled_width  = handle->configured_width/SCALE_REDUCTION_PER_AXIS;
    scaled_height = handle->configured_height/SCALE_REDUCTION_PER_AXIS;
    scaled_data     = (unsigned char *)malloc( sizeof(image)/(SCALE_REDUCTION_PER_AXIS*SCALE_REDUCTION_PER_AXIS) );
    scaled_RGB_data = (struct pixel_format_RGB *)scaled_data;

    window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(window), "Plant Green / Yellow Detection");
    gtk_window_set_default_size(GTK_WINDOW(window), scaled_width, scaled_height + 60);
    g_signal_connect(window, "destroy", G_CALLBACK(on_window_destroy), NULL);

    GtkWidget *vbox = gtk_vbox_new(FALSE, 5);
    gtk_container_add(GTK_CONTAINER(window), vbox);

    GtkWidget *top_bar = gtk_hbox_new(FALSE, 5);
    total_counts_label = gtk_label_new(NULL);
    gtk_box_pack_end(GTK_BOX(top_bar), total_counts_label, FALSE, FALSE, 10);
    gtk_box_pack_start(GTK_BOX(vbox), top_bar, FALSE, FALSE, 2);

    image_widget = gtk_image_new();
    gtk_box_pack_start(GTK_BOX(vbox), image_widget, TRUE, TRUE, 0);

    status_label = gtk_label_new(NULL);
    gtk_box_pack_start(GTK_BOX(vbox), status_label, FALSE, FALSE, 8);

    gtk_widget_show_all(window);

    pixbuf = gdk_pixbuf_new_from_data(
        (guchar *)scaled_RGB_data, GDK_COLORSPACE_RGB, FALSE, 8,
        scaled_width, scaled_height, scaled_width * 3, NULL, NULL
    );

    while (true)
    {
      while (gtk_events_pending()) {
          gtk_main_iteration();
      }

      if (wait_key(1, NULL)) {
          break;
      }

      if (video_interface_get_image( handle, &image ))
      {
        scale_image_data(
            (struct pixel_format_RGB *)&image, handle->configured_height,
            handle->configured_width, scaled_RGB_data, SCALE_REDUCTION_PER_AXIS, SCALE_REDUCTION_PER_AXIS );

        int region_starts[MAX_REGIONS] = {0};
        int region_ends[MAX_REGIONS]   = {0};
        int region_types[MAX_REGIONS]  = {0};
        int region_count = 0;

        uint8_t *target_map = (uint8_t *)calloc(scaled_width * scaled_height, sizeof(uint8_t));
        size_t vertical_cutoff = (size_t)(scaled_height * 2 / 3);

        int current_start = -1;
        int empty_run = 0;

        for (size_t x = 0; x < scaled_width; x++)
        {
            unsigned int plant_pixels = 0;

            for (size_t y = 0; y < vertical_cutoff; y++)
            {
                size_t index = (y * scaled_width) + x;
                unsigned char R = scaled_RGB_data[index].R;
                unsigned char G = scaled_RGB_data[index].G;
                unsigned char B = scaled_RGB_data[index].B;

                if (classify_pixel(R, G, B) != COLOR_NONE)
                {
                    plant_pixels++;
                }
            }

            if (plant_pixels >= MIN_REGION_PLANT_PIXELS) {
                if (current_start == -1) {
                    current_start = (int)x;
                }
                empty_run = 0;
            }
            else if (current_start != -1) {
                empty_run++;
                if (empty_run >= MAX_EMPTY_RUN || x == (scaled_width - 1)) {
                    int end_x = (int)x - empty_run;
                    int width = end_x - current_start;

                    if (width >= MIN_REGION_WIDTH && region_count < MAX_REGIONS) {
                        region_starts[region_count] = current_start;
                        region_ends[region_count]   = end_x;

                        unsigned int green_votes  = 0;
                        unsigned int yellow_votes = 0;

                        for (int sx = current_start; sx <= end_x; sx++) {
                            for (size_t sy = 0; sy < vertical_cutoff; sy++) {
                                size_t s_idx = (sy * scaled_width) + (size_t)sx;
                                unsigned char R = scaled_RGB_data[s_idx].R;
                                unsigned char G = scaled_RGB_data[s_idx].G;
                                unsigned char B = scaled_RGB_data[s_idx].B;

                                int c = classify_pixel(R, G, B);
                                if (c == COLOR_GREEN)  green_votes++;
                                else if (c == COLOR_YELLOW) yellow_votes++;
                            }
                        }

                        region_types[region_count] = (yellow_votes > green_votes) ? COLOR_YELLOW : COLOR_GREEN;

                        for (int sx = current_start; sx <= end_x; sx++) {
                            for (size_t sy = 0; sy < scaled_height; sy++) {
                                size_t s_idx = (sy * scaled_width) + (size_t)sx;
                                unsigned char R = scaled_RGB_data[s_idx].R;
                                unsigned char G = scaled_RGB_data[s_idx].G;
                                unsigned char B = scaled_RGB_data[s_idx].B;

                                int c = classify_pixel(R, G, B);
                                if (c != COLOR_NONE) {
                                    target_map[s_idx] = (uint8_t)c;
                                }
                            }
                        }

                        region_count++;
                    }
                    current_start = -1;
                    empty_run = 0;
                }
            }
        }

        int green_regions_this_frame  = 0;
        int yellow_regions_this_frame = 0;
        for (int i = 0; i < region_count; i++) {
            if (region_types[i] == COLOR_YELLOW) yellow_regions_this_frame++;
            else                                  green_regions_this_frame++;
        }

        int current_state;
        if (yellow_regions_this_frame > 0)
            current_state = STATE_YELLOW;
        else if (green_regions_this_frame > 0)
            current_state = STATE_GREEN;
        else
            current_state = STATE_NONE;

        if (current_state == candidate_state)
        {
            consecutive_frames++;

            if (consecutive_frames >= 3)
            {
                if (stable_state != candidate_state)
                {
                    if (candidate_state == STATE_GREEN)
                        total_green_detections  += (unsigned long)green_regions_this_frame;
                    else if (candidate_state == STATE_YELLOW)
                        total_yellow_detections += (unsigned long)yellow_regions_this_frame;
                }
                stable_state = candidate_state;
            }
        }
        else
        {
            candidate_state = current_state;
            consecutive_frames = 1;
        }

        char markup_buffer[256];
        switch (stable_state)
        {
            case STATE_YELLOW:
                snprintf(markup_buffer, sizeof(markup_buffer),
                        "<span size='20000' weight='bold' foreground='#CCA300'>YELLOW PLANT DETECTED (%d region%s)</span>",
                        yellow_regions_this_frame, yellow_regions_this_frame == 1 ? "" : "s");
                break;
            case STATE_GREEN:
                snprintf(markup_buffer, sizeof(markup_buffer),
                        "<span size='20000' weight='bold' foreground='#00AA00'>GREEN PLANT DETECTED (%d region%s)</span>",
                        green_regions_this_frame, green_regions_this_frame == 1 ? "" : "s");
                break;
            default:
                snprintf(markup_buffer, sizeof(markup_buffer),
                        "<span size='20000' weight='bold' foreground='#888888'>NO PLANT DETECTED</span>");
                break;
        }
        gtk_label_set_markup(GTK_LABEL(status_label), markup_buffer);

        char totals_buffer[256];
        snprintf(totals_buffer, sizeof(totals_buffer),
                 "<span size='12000' weight='bold' foreground='#222222'>Total Green: %lu | Total Yellow: %lu</span>",
                 total_green_detections, total_yellow_detections);
        gtk_label_set_markup(GTK_LABEL(total_counts_label), totals_buffer);

        printf("Green regions: %d | Yellow regions: %d | TOTAL Green: %lu | TOTAL Yellow: %lu\n",
               green_regions_this_frame, yellow_regions_this_frame,
               total_green_detections, total_yellow_detections);
        fflush(stdout);

        for (size_t idx = 0; idx < (size_t)scaled_width * scaled_height; idx++)
        {
            if (target_map[idx] == COLOR_NONE)
            {
                uint8_t gray = (uint8_t)(0.299f * scaled_RGB_data[idx].R +
                                         0.587f * scaled_RGB_data[idx].G +
                                         0.114f * scaled_RGB_data[idx].B);
                scaled_RGB_data[idx].R = gray;
                scaled_RGB_data[idx].G = gray;
                scaled_RGB_data[idx].B = gray;
            }
        }

        free(target_map);
        gtk_image_set_from_pixbuf(GTK_IMAGE(image_widget), pixbuf);
      }
    }
  }

  if (pixbuf) g_object_unref(pixbuf);
  video_interface_close( handle );
  free( scaled_data );
  return 0;
}
