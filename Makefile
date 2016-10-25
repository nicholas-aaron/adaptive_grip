# ===================================================
# 	Name		: Makefile
# 	Author	: AN
#	Purpose	: Makefile used to build:
#						- Object files from 'original' code
#						- New Executables
# ===================================================


# Command-Line Defines (none right now)
DEFINES=

# Directory housing the 'original' source code files.
ORIGDIR=/home/robot/Documents/Group3/original
ORIGINAL_SRC=$(ORIGDIR)/src

INCLUDE=-I$(ORIGDIR)/headers -I/$(LIBDIR)

# Location where object files will be placed
OBJDIR=objects

# =============================================
# New Source Library
# =============================================
SRC=src

# =============================================
# Programs
# =============================================
PROGRAMS=programs

# =============================================
# Libraries (readline)
# =============================================
LIBDIR=/home/robot/Documents/Group3/libraries

# =============================================
# Objects
# =============================================
COMMON_OBJECTS= \
	$(OBJDIR)/Robot.o \
	$(OBJDIR)/CRScomm.o \
	$(OBJDIR)/RobotController.o \
	$(OBJDIR)/RobotLimits.o \
	$(OBJDIR)/RobotPosition.o

# ==================================================
# Object File Recipe
# ==================================================
$(OBJDIR)/%.o : $(ORIGINAL_SRC)/%.cpp $(wildcard headers/%.h)
	g++ -g $(DEFINES) $(INCLUDE) -c $< -o $@

# ==================================================
# Default: Make all of the executables
# ==================================================
all : basic_shell interface_update

# ==================================================
# Recipe for basic_shell executable
# ==================================================
basic_shell : $(PROGRAMS)/basic_shell.cpp $(COMMON_OBJECTS)
	g++ -g $(DEFINES) $(INCLUDE) $(PROGRAMS)/basic_shell.cpp -c -o $(OBJDIR)/basic_shell.o
	g++ -g -w $(DEFINES)  $(OBJDIR)/basic_shell.o $(COMMON_OBJECTS) \
	-std=c++11 \
	$(LIBDIR)/readline/*.o \
	-ltermcap \
	-o exec/basic_shell

# ==================================================
# Recipe for interface_update executable
# ==================================================
interface_update : $(PROGRAMS)/interface_update.cpp $(COMMON_OBJECTS)
	g++ -g $(DEFINES) $(INCLUDE) $(PROGRAMS)/interface_update.cpp -c -o $(OBJDIR)/interface_update.o
	g++ -g -w $(DEFINES)  $(OBJDIR)/interface_update.o $(COMMON_OBJECTS) \
	-std=c++11 \
	$(LIBDIR)/readline/*.o \
	-ltermcap \
	-o exec/interface_update

# =================================================
# Make and launch basic_shell program
# =================================================
shell_target: basic_shell
	exec/basic_shell

# =================================================
# Make and launch interface_update program
# =================================================
interface_target: interface_update
	exec/interface_update
	
# =================================================
# Clean everything
# =================================================
clean:
	rm -rf exec/*
	rm -rf objects/*
