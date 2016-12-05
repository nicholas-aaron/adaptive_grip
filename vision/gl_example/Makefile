# Retrieved from https://openkinect.org/wiki/C%2B%2B_GL_Example
CC = g++
LD = g++
LDFLAGS =
CFLAGS=-g -Wall `pkg-config --cflags libfreenect` -lopengl32 -lglut32
# LIBS = `pkg-config --libs libfreenect` -lGL -lGLU -lglut 
# Replace with clang "frameworks"
LIBS = `pkg-config --libs libfreenect` -framework OpenGL -framework Cocoa -framework GLUT
OBJECTS = main.o

# My CPP 
PROG = mycppview

# OpenGL
OPENGL_INCLUDE=/System/Library/Frameworks/OpenGL.framework/Headers

# FreeNect / LibFreeNect
FREENECT_INCLUDE=/usr/local/Cellar/libfreenect/0.5.5/include/libfreenect

# LibUsb /
LIBUSB_INCLUDE=/usr/local/Cellar/libusb/1.0.20/include/libusb-1.0

all:$(PROG)

# Link the shit together
$(PROG): $(OBJECTS)
	$(LD) $(LDFLAGS) $(LIBS) $(OBJECTS) -o $(PROG)

# Compile shit
%.o: %.cpp
	$(CC) $(CFLAGS) \
		-I$(FREENECT_INCLUDE) -I$(OPENGL_INCLUDE) -I$(LIBUSB_INCLUDE)\
		$(LIBS) -c $<

clean:
	rm -rf *.o $(PROG)
