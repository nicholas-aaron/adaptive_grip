DEFINES=
ORIGDIR=/home/robot/Documents/Group3/original
#INCLUDE=-I$(ORIGDIR)/headers -I/$(LIBDIR)
INCLUDE=-I$(ORIGDIR)/headers -I/$(LIBDIR)
OBJDIR=objects
SRC=$(ORIGDIR)/src
LIBDIR=/home/robot/Documents/Group3/libraries
LD_LIB=-L$(LIBDIR)/readline/shlib
READLINE=-L$(LIBDIR)/readline/shlib -lreadline -lhistory
READLINE_OBJS=$(LIBDIR)/readline

COMMON_OBJECTS= \
	$(OBJDIR)/Robot.o \
	$(OBJDIR)/CRScomm.o \
	$(OBJDIR)/RobotController.o \
	$(OBJDIR)/RobotLimits.o \
	$(OBJDIR)/RobotPosition.o

all : $(COMMON_OBJECTS)

robot_home : robot_home.cpp all
	@g++ -g $(DEFINES) $(INCLUDE) robot_home.cpp -c -o robot_home.o
	@g++ -g $(DEFINES) robot_home.o $(COMMON_OBJECTS) -o robot_home

# TODO : This is a hack, I don't know which objects are actually being used.
# Need termcap library; that part is not a hack.
readline : $(SRC)/robot_shell.cpp $(COMMON_OBJECTS)
	@g++ -g $(DEFINES) $(INCLUDE) $(SRC)/robot_shell.cpp -c -o robot_shell.o
	g++ -g -Wall $(DEFINES)  robot_shell.o $(COMMON_OBJECTS) \
	$(LIBDIR)/readline/*.o \
	-ltermcap \
	-o robot_shell

$(OBJDIR)/%.o : $(SRC)/%.cpp $(wildcard headers/%.h)
	@g++ -g $(DEFINES) $(INCLUDE) -c $< -o $@

clean:
	@rm -rf exec/*
	@rm -rf objects/*
