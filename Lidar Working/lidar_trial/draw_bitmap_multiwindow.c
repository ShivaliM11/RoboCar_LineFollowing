#include <stdlib.h>
#include <stdbool.h>
#include <cairo/cairo.h>
#include <gtk/gtk.h>
#include <pthread.h>
#include <string.h>
#include "pixel_format_RGB.h"
#include "draw_bitmap_multiwindow.h"

#define ARRAYSIZE(A) (sizeof(A)/sizeof(A[0]))

#define REPORT_TIME_DIFFERENCE  0
#define REPORT_PIXEL_VALUES     0

/*
 * GTK   API reference: https://docs.gtk.org/gtk4/index.html
 * GDK   API reference: https://docs.gtk.org/gdk4/index.html
 * Cairo API reference: https://www.cairographics.org/manual/
 */

#if REPORT_TIME_DIFFERENCE
static long time_difference_ms( struct timespec *start_time, struct timespec *end_time )
{
  return (end_time->tv_sec - start_time->tv_sec)*1000 + (end_time->tv_nsec - start_time->tv_nsec)/(1000*1000);
}
#endif

static int                                        gtk_argc;
static char **                                    gtk_argv;
static GtkApplication *                           application;
static gint                                       timeout_handle;
static pthread_t                                  thread_id;
static struct draw_bitmap_multiwindow_handle_t *  handle_list_head = NULL;
static struct draw_bitmap_multiwindow_handle_t *  handle_list_tail = NULL;

/* insert a new handle into the list of displayed windows */
static void insert_handle( struct draw_bitmap_multiwindow_handle_t * handle )
{
  /* insert the new handle at the beginning of the list */

  handle->next                  = handle_list_head;
  handle->previous              = NULL;
  if (handle_list_head != NULL)
  {
    handle_list_head->previous  = handle;
  }
  else
  {
    handle_list_tail            = handle;
  }
  handle_list_head              = handle;

  return;
}

/* remove a handle from the list of displayed windows */
static void remove_handle( struct draw_bitmap_multiwindow_handle_t * handle )
{
  /* remove the handle from the list */

  if (handle_list_head != handle)
  {
    handle->previous->next      = handle->next;
  }
  else
  {
    handle_list_head            = handle->next;
  }
  if (handle_list_tail != handle)
  {
    handle->next->previous      = handle->previous;
  }
  else
  {
    handle_list_tail            = handle->previous;
  }
  handle->next                  = NULL;
  handle->previous              = NULL;

  return;
}

/* Redraw the screen from the backing pixmap */
static void draw_event( GtkDrawingArea *drawing_area, cairo_t *cairo_context, int width, int height, gpointer data )
{
  struct draw_bitmap_multiwindow_handle_t * handle = (struct draw_bitmap_multiwindow_handle_t *)data;
  cairo_surface_t *                         image_surface;
  unsigned int                              most_recently_updated;

  /* determine what is going on and what data is being used when updating the window */
  pthread_mutex_lock( &(handle->data_interface.lock) );
  handle->data_interface.GUI_reading                = true;
  most_recently_updated                             = handle->data_interface.most_recently_updated;
  pthread_mutex_unlock( &(handle->data_interface.lock) );

  /* update the window */
#if REPORT_PIXEL_VALUES
  printf( "R[%d]: [%3.0d, %3.0d, %3.0d] [%3.0d, %3.0d, %3.0d]\n",
      most_recently_updated,
      data_interface.bitmap[most_recently_updated][0].R,
      data_interface.bitmap[most_recently_updated][0].G,
      data_interface.bitmap[most_recently_updated][0].B,
      data_interface.bitmap[most_recently_updated][1].R,
      data_interface.bitmap[most_recently_updated][1].G,
      data_interface.bitmap[most_recently_updated][1].B );
#endif
  image_surface = cairo_image_surface_create_for_data(
      (unsigned char *)handle->data_interface.bitmap[most_recently_updated],
      handle->cairo_format,
      handle->data_interface.width,
      handle->data_interface.height,
      handle->cairo_stride );
  cairo_set_source_surface( cairo_context, image_surface, 0, 0 );
  cairo_paint( cairo_context );
  cairo_show_page( cairo_context );
  cairo_surface_destroy( image_surface );

#if REPORT_PIXEL_VALUES
  printf( "R[%d]: done\n", most_recently_updated );
#endif

  /* release the bitmap for future updates */
  pthread_mutex_lock( &(handle->data_interface.lock) );
  handle->data_interface.GUI_reading = false;
  pthread_mutex_unlock( &(handle->data_interface.lock) );

  return;
}

/* timeout event handler */
static gint timeout_event( gpointer )
{
  struct draw_bitmap_multiwindow_handle_t * handle;
  bool                                      bitmap_updated;
#if REPORT_TIME_DIFFERENCE
  struct timespec                           start_time;
  struct timespec                           stop_time;

  clock_gettime( CLOCK_REALTIME, &start_time );
#endif

  for (handle = handle_list_head; handle != NULL; handle = handle->next)
  {
    /* determine what is going on and what data is being used when updating the window */
    pthread_mutex_lock( &(handle->data_interface.lock) );
    bitmap_updated                                    = handle->data_interface.updated_since_last_display;
    handle->data_interface.updated_since_last_display = false;
    pthread_mutex_unlock( &(handle->data_interface.lock) );

    if (bitmap_updated)
    {
      gtk_widget_queue_draw( handle->drawing_area );
    }
    else
    {
      ; /* there is no sense displaying data that has not been updated */
    }

#if REPORT_TIME_DIFFERENCE
    clock_gettime( CLOCK_REALTIME, &stop_time );
    printf( "%6.6ld\n", time_difference_ms( &start_time, &stop_time ) );
#endif
  }

  return TRUE;
}

/* the function that shuts down the event loop when the window is destroyed */
static void destroy( GtkWidget *widget, gpointer data )
{
  struct draw_bitmap_multiwindow_handle_t * handle = (struct draw_bitmap_multiwindow_handle_t *)data;

  /* unlink the window so that it will not receive bitmap updates */
  remove_handle( handle );

  /* indicate that the window is no longer being refreshed */
  handle->data_interface.window_closed = true;

  return;
}

/* start the timer once the application is active */
static void activate( GtkApplication *app, gpointer user_data )
{
  timeout_handle = g_timeout_add( 30, timeout_event, NULL );

  return;
}

/* the GUI thread, creates the window and waits for events */
static void *GUI_thread( void *thread_parameter )
{
  application = gtk_application_new( NULL, G_APPLICATION_DEFAULT_FLAGS );

  /* increase the use count of the application to keep it running */
  g_application_hold( G_APPLICATION( application ) );

  /* Signals and events */
  g_signal_connect( G_APPLICATION( application ), "activate", G_CALLBACK (activate), NULL );

  /* start the global GUI event loop */
  g_application_run( G_APPLICATION( application ), gtk_argc, gtk_argv );

  /* clean up */
  g_source_remove( timeout_handle );
  g_object_unref( application );

  return NULL;
}

/* use an idle handler to register a new window for display, setting up GUI events and linking it into the list of windows to poll for bitmap updates */
static gboolean idle_add_window( gpointer data )
{
  struct draw_bitmap_multiwindow_handle_t * handle = (struct draw_bitmap_multiwindow_handle_t *)data;
  GtkWidget *                               box;

  /* create the window and lay it out */
  handle->window = gtk_application_window_new( application );
  gtk_window_set_title( GTK_WINDOW( handle->window ), "Draw Bitmap" );

  /* create a box to position the drawing area in the center of the window */
  box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);
  gtk_widget_set_halign( box, GTK_ALIGN_CENTER );
  gtk_widget_set_valign( box, GTK_ALIGN_CENTER );
  gtk_window_set_child( GTK_WINDOW( handle->window ), box );

  /* Create the drawing area */
  handle->drawing_area = gtk_drawing_area_new();
  gtk_drawing_area_set_content_width( GTK_DRAWING_AREA( handle->drawing_area ), handle->data_interface.width );
  gtk_drawing_area_set_content_height( GTK_DRAWING_AREA( handle->drawing_area ), handle->data_interface.height );
  gtk_widget_set_size_request( GTK_WIDGET( handle->drawing_area ), handle->data_interface.width, handle->data_interface.height );
  gtk_box_append( GTK_BOX( box ), handle->drawing_area );
  gtk_widget_show( handle->drawing_area );

  /* Signals and events */
  g_signal_connect( handle->window,       "destroy",              G_CALLBACK( destroy ),              handle );
  gtk_drawing_area_set_draw_func( GTK_DRAWING_AREA( handle->drawing_area ), draw_event, handle, NULL );

  /* display the window and handle events */
  gtk_window_present( GTK_WINDOW( handle->window ) );

  /* link the newly created window in with the others to monitor for bitmap updates */
  insert_handle( handle );

  return G_SOURCE_REMOVE;
}

/* use an idle handler to destroy a window */
static gboolean idle_remove_window( gpointer data )
{
  struct draw_bitmap_multiwindow_handle_t * handle = (struct draw_bitmap_multiwindow_handle_t *)data;

  /* unregister callbacks for the window being destroyed, the destroy event will handle the unlinking */
  gtk_window_destroy( GTK_WINDOW( handle->window ) );

  /* notify the handle owner that it is now safe to clean up */
  pthread_mutex_lock( &(handle->window_operation_lock) );
  handle->window_operation_complete     = true;
  pthread_cond_signal( &(handle->window_operation_cond) );
  pthread_mutex_unlock( &(handle->window_operation_lock) );

  return G_SOURCE_REMOVE;
}

/* use an idle handler to remove all windows and shut down the GUI thread */
static gboolean idle_GUI_shutdown( gpointer )
{
  /* dereference and close all open windows */
  while (handle_list_head != NULL)
  {
    idle_remove_window( handle_list_head );
  }

  /* shut down the GUI event loop */
  g_application_release( G_APPLICATION( application ) );

  return G_SOURCE_REMOVE;
}

/* see header for function descriptions */
int draw_bitmap_start(
    int           argc,
    char **       argv )
{
  int return_value;

  /* put argc and argv somewhere where the GUI thread can access and start the thread */
  gtk_argc = argc;
  gtk_argv = argv;

  return_value = pthread_create( &thread_id, NULL, GUI_thread, NULL );

  return return_value;
}

struct draw_bitmap_multiwindow_handle_t * draw_bitmap_create_window(
    unsigned int  width,
    unsigned int  height )
{
  struct draw_bitmap_multiwindow_handle_t * handle;
  unsigned int                              index;
  static const pthread_mutex_t              pthread_mutex_static_initializer = PTHREAD_MUTEX_INITIALIZER;
  static const pthread_cond_t               pthread_cond_static_initializer = PTHREAD_COND_INITIALIZER;

  handle = malloc( sizeof(*handle) );
  if (handle != NULL)
  {
    /* initialize the handle */
    handle->data_interface.lock                       = pthread_mutex_static_initializer;
    handle->data_interface.most_recently_updated      = 0;
    handle->data_interface.GUI_reading                = false;
    handle->data_interface.updated_since_last_display = false;
    handle->data_interface.window_closed              = false;
    handle->data_interface.width                      = width;
    handle->data_interface.height                     = height;
    handle->cairo_format                              = CAIRO_FORMAT_RGB24;
    handle->cairo_stride                              = cairo_format_stride_for_width( handle->cairo_format, handle->data_interface.width );
    for (index = 0; index < ARRAYSIZE(handle->data_interface.bitmap); index++)
    {
      handle->data_interface.bitmap[index]            = (struct draw_bitmap_multiwindow_cairo_RGB24_t *)malloc( handle->cairo_stride * handle->data_interface.height );
      memset( handle->data_interface.bitmap[index], 0, handle->cairo_stride * handle->data_interface.height );
    }
    handle->drawing_area                              = NULL;
    handle->window                                    = NULL;
    handle->next                                      = NULL;
    handle->previous                                  = NULL;
    handle->window_operation_lock                     = pthread_mutex_static_initializer;
    handle->window_operation_cond                     = pthread_cond_static_initializer;
    handle->window_operation_complete                 = true;

    /* hand the handle off to the GUI thread to create the widget, register callbacks for window events, and link into the list of handles to monitor for bitmap updates */
    g_idle_add( idle_add_window, handle );
  }
  else
  {
    // could not allocate memory
  }

  return handle;
}

void draw_bitmap_display(
    struct draw_bitmap_multiwindow_handle_t * handle,
    struct pixel_format_RGB *                 bitmap )
{
  unsigned int  index;
  unsigned int  bitmap_to_update;

  /* the writer always updates the oldest data, the reader only ever reads the newest data */
  pthread_mutex_lock( &(handle->data_interface.lock) );
  bitmap_to_update = (handle->data_interface.most_recently_updated + 1) % ARRAYSIZE(handle->data_interface.bitmap);
  pthread_mutex_unlock( &(handle->data_interface.lock) );

  for (index = 0; index < handle->data_interface.width*handle->data_interface.height; index++)
  {
    handle->data_interface.bitmap[bitmap_to_update][index].R      = bitmap[index].R;
    handle->data_interface.bitmap[bitmap_to_update][index].G      = bitmap[index].G;
    handle->data_interface.bitmap[bitmap_to_update][index].B      = bitmap[index].B;
    handle->data_interface.bitmap[bitmap_to_update][index].unused = 0;
  }

  /* if there is no reader present, let the reader know that there is updated data */
  pthread_mutex_lock( &(handle->data_interface.lock) );
  if (handle->data_interface.GUI_reading)
  {
    ; /* there is a reader using the newest data, do not try to touch it */
  }
  else
  {
    handle->data_interface.most_recently_updated      = bitmap_to_update;
    handle->data_interface.updated_since_last_display = true;
  }
  pthread_mutex_unlock( &(handle->data_interface.lock) );

  return;
}

void draw_bitmap_close_window(
    struct draw_bitmap_multiwindow_handle_t * handle )
{
  unsigned int  index;

  if (!handle->data_interface.window_closed)
  {
    /* indicate that a window operation is needed and kick off a remove_window operation */
    handle->window_operation_complete = false;
    g_idle_add( idle_remove_window, handle );

    /* wait for the window operation to complete */
    pthread_mutex_lock( &(handle->window_operation_lock) );
    while (!handle->window_operation_complete)
    {
      pthread_cond_wait( &(handle->window_operation_cond), &(handle->window_operation_lock) );
    }
    pthread_mutex_unlock( &(handle->window_operation_lock) );
  }
  else
  {
    ; /* nothing to wait for, the window was closed through the GUI and has already been dereferenced */
  }

  /* clean up the window now that it is no longer referenced by the GUI system */
  for (index = 0; index < ARRAYSIZE(handle->data_interface.bitmap); index++)
  {
    free( handle->data_interface.bitmap[index] );
  }
  free( handle );

  return;
}

bool draw_bitmap_window_closed(
    struct draw_bitmap_multiwindow_handle_t * handle )
{
  return handle->data_interface.window_closed;
}

void draw_bitmap_stop( void )
{
  /* initiate GUI shutdown */
  g_idle_add( idle_GUI_shutdown, NULL );

  /* wait for the GUI thread to shut down */
  pthread_join( thread_id, NULL );

  return;
}
