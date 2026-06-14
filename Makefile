TARGET=hw6munshi
SOURCES=$(TARGET).c keypress.c pwmsetup.c
INCLUDES=-I./include
CFLAGS=-Wall -g $(INCLUDES)
LIBS=-lpthread
DEPS=include/import_registers.c include/wait_period.c include/wait_key.c include/enable_pwm_clock.c

$(TARGET): $(SOURCES)
	gcc $(CFLAGS) -o $(TARGET) $(SOURCES) $(DEPS) $(LIBS)

clean:
	rm -f $(TARGET)
