# software that needs to be installed (sudo apt install <package>):
# libswscale-dev
# libgtk2.0-dev

TARGET = camera_project_v4l2_gui

SOURCES = camera_project_v4l2_gui.c \
          video_interface.c \
          draw_bitmap.c \
          wait_key.c \
          scale_image_data.c \
          import_registers.c

OBJECTS = $(patsubst %.cpp,%.o,$(patsubst %.c,%.o,$(SOURCES)))

CFLAGS = `pkg-config --cflags --libs gtk+-2.0` \
         -Wall \
         -Werror \
         -Wno-deprecated-declarations \
         -g

all: $(TARGET)

clean:
	rm -f $(OBJECTS) $(TARGET)

$(TARGET): $(OBJECTS)
	gcc $^ $(CFLAGS) -o $@ -lm -lpthread -lswscale

%.o:%.c
	gcc $(CFLAGS) -c $< -o $@# software that needs to be installed (sudo apt install <package>):
# libswscale-dev
# libgtk2.0-dev

TARGET = camera_project_v4l2_gui

SOURCES = camera_project_v4l2_gui.c \
          video_interface.c \
          draw_bitmap.c \
          wait_key.c \
          scale_image_data.c \
	  import_registers.c

OBJECTS = $(patsubst %.cpp,%.o,$(patsubst %.c,%.o,$(SOURCES)))

CFLAGS = `pkg-config --cflags --libs gtk+-2.0` \
         -Wall \
         -Werror \
         -Wno-deprecated-declarations \
         -g

all: $(TARGET)

clean:
	rm -f $(OBJECTS) $(TARGET)

$(TARGET): $(OBJECTS)
	gcc $^ $(CFLAGS) -o $@ -lm -lpthread -lswscale

%.o:%.c
	gcc $(CFLAGS) -c $< -o $@
