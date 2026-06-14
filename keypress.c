#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
int get_pressed_key(void)
{
  struct termios  original_attributes;
  struct termios  modified_attributes;
  long   oldf, newf;
  int    ch;
  tcgetattr( STDIN_FILENO, &original_attributes );
  modified_attributes = original_attributes;
  modified_attributes.c_lflag &= ~(ICANON | ECHO);
  modified_attributes.c_cc[VMIN] = 1;
  modified_attributes.c_cc[VTIME] = 0;
  tcsetattr( STDIN_FILENO, TCSANOW, &modified_attributes );
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  newf = oldf | O_NONBLOCK;
  fcntl(STDIN_FILENO, F_SETFL, newf);
  ch = getchar();
  fcntl(STDIN_FILENO, F_SETFL, oldf);
  tcsetattr( STDIN_FILENO, TCSANOW, &original_attributes );
  if (ch != -1) {
    if (ch > 47) {
      if (ch < 123) { }
      else {ch = -1;}
    }
    else {ch = -1;}
  }
  return ch;
}
