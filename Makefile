DEFINES=
ORIGDIR=/home/robot/Documents/Group3
INCLUDE=-I$(ORIGDIR)/headers_orig
OBJDIR=objects
SRC=$(ORIGDIR)/src_orig

COMMON_OBJECTS= \
	$(OBJDIR)/Robot.o \
	$(OBJDIR)/CRScomm.o \
	$(OBJDIR)/RobotController.o \
	$(OBJDIR)/RobotLimits.o \
	$(OBJDIR)/RobotPosition.o

all : $(COMMON_OBJECTS)

robot_home : robot_home.cpp all
	g++ -g $(DEFINES) $(INCLUDE) robot_home.cpp -c -o robot_home.o
	g++ -g $(DEFINES) robot_home.o $(COMMON_OBJECTS) -o robot_home

$(OBJDIR)/%.o : $(SRC)/%.cpp $(wildcard headers/%.h)
	g++ -g $(DEFINES) $(INCLUDE) -c $< -o $@

clean:
	rm -rf exec/*
	rm -rf objects/*
