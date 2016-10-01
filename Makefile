# advanced Makefile 
# compile with "make <filename without .c>
# Note that if you installed your OpenGL libraries in a different place
# the paths that follow -L in LDFLAGS and -I in CFLAGS will have to change
# accordingly

ifeq ($(shell uname -s), Darwin)
LDFLAGS=-L/opt/ports/lib -framework OpenGL -lglfw -lGLEW -lm
else
LDFLAGS=-lX11 -lGL -lGLU -lglfw -lGLEW -lm
endif
CC = gcc
CFLAGS=-g -Wall -I/opt/ports/include
SOURCES=hw3.c boids.c update.c utility.c
OBJECTS=$(SOURCES:.c=.o)
DEPS= hw3.h boids.h utility.h
EXECUTABLE=hw3

%.o: %.c $(DEPS)
		$(CC) -c -o $@ $< $(CFLAGS)

$(EXECUTABLE): $(OBJECTS)
		$(CC) -o $@ $^ $(LDFLAGS) $(CFLAGS)

clean:
		rm *.o $(EXECUTABLE)
