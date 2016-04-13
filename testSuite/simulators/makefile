# ---------VARIABLES-----------

# Compiler
CC = g++

# Location of Arduino sources.
ARDU_SRC = ../../mainProgram

# Compiler flags.
CFLAGS = -O0 -g3 -Wall -I$(ARDU_SRC) -std=c++1y -D SIMULATOR

# Build command.
BUILD_CMD = $(CC) -o $@ $(CFLAGS) $^

# Executables. List all of them below.
EXECUTABLES = atlanticSimulator

# ---------DEPENDENCIES--------

# Atlantic simulator:
ATLANTIC_SIM_DEPS = atlanticSimulator.cpp GpsSimulator.cpp \
                    $(ARDU_SRC)/Gps.cpp $(ARDU_SRC)/Coord.cpp $(ARDU_SRC)/utility.cpp

# ---------TARGETS-------------
# Do not play wth object files, since these programs will
# take less than a second to build!  
           
# make all
all : $(EXECUTABLES)

# Build the Atlantic simulator.
atlanticSimulator : $(ATLANTIC_SIM_DEPS)
	$(BUILD_CMD)
	
# Clean
clean:
	rm -rf $(EXECUTABLES)
