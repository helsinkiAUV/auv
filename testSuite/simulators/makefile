# ---------VARIABLES-----------

# Compiler
CC = g++

# Location of Arduino sources.
ARDU_SRC = ../../arduinoMain

# Location of Raspberry pi sources.
RASPI_SRC = ../../raspiMain

# Compiler flags.
CFLAGS = -O0 -g3 -Wall -I$(ARDU_SRC) -I$(RASPI_SRC) -std=c++1y -D SIMULATOR

# Build command.
BUILD_CMD = $(CC) -o $@ $(CFLAGS) $^

# Executables. List all of them below.
EXECUTABLES = atlanticSimulator GpsAverageTest

# ---------DEPENDENCIES--------

# General dependencies: Arduino and raspi.
GENERAL_DEPS = $(ARDU_SRC)/*.cpp $(RASPI_SRC)/*.cpp

# Atlantic simulator:
ATLANTIC_SIM_DEPS = atlanticSimulator.cpp GpsSimulator.cpp $(GENERAL_DEPS)

# GPS averaging test
GPS_AVERAGING_DEPS = GpsAverageTest.cpp GpsSimulator.cpp $(GENERAL_DEPS)

# ---------TARGETS-------------
# Do not play wth object files, since these programs will
# take less than a second to build!  
           
# make all
all : $(EXECUTABLES)

# Build the Atlantic simulator.
atlanticSimulator : $(ATLANTIC_SIM_DEPS)
	$(BUILD_CMD)
	
# Build the GPS averaging test.
GpsAverageTest : $(GPS_AVERAGING_DEPS)
	$(BUILD_CMD)
	
# Clean
clean:
	rm -rf $(EXECUTABLES)
