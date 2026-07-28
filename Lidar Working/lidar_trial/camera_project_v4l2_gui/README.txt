20240327 KChoi

10. USB Camera to Raspberry Pi 4 computer:

10.1 camera_project_v4l2.zip : Sample program for USB camera on Raspberry Pi 4, to get an image.
 It uses a library to scale images, so do the following library installation only once
 on your Raspberry Pi 4 before using this sample program:
 "sudo apt-get install libswscale-dev"
 To run, type “sudo ./video_interface_test” after ‘make’ command.
 The program generates camera information and produces a still picture image file.
 Use an image viewing tool to display the image file.
"video_interface_test.c" is the main application for this project. This file uses the
 functions in "video_interface.c".
"video_interface.c" and "video_interface.h" provide several functions:
video_interface_open to open the camera device.
video_interface_close to shut down the camera device.
video_interface_set_mode_auto to set the camera image capture with the library's best guess
 at the image format to use to get images.
video_interface_get_image to get an image from the camera.
video_interface_print_modes to print the image formats provided by the camera.
video_interface_set_mode_manual to set the camera image capture to a specific mode
(video_interface_print_modes shows the mode for each size... look for "video interface mode:"
 in the output).
 
10.2 camera_project_v4l2_gui.zip : Sample program for USB camera on Raspberry Pi 4, to get
 an image and to display an image. This program captures 24 FPS (Frames
 Per Second) 640X480 still picture images and displays 160X120 color images,
 continuously in real-time. This program uses two library packages to scale
 images and to display images: libswscale-dev and gtk2.0
 You already have the libswscale-dev package from the above 10.1 and
 the gtk2.0 package is included in the Raspberry Pi Operating System.
 (You can check it by typing: apt list –installed | grep ‘gtk’ on your terminal.)
 (To install, type: sudo apt-get install gtk2.0
 This is GNU Image Manipulation Program (GIMP) Toolkit.)
 With this sample program, you will be able to see what your RoboCar is seeing
 when you place the camera on your RoboCar.

********
