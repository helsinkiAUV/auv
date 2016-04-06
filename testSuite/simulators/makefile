# Compiler
CC = g++
# Location of Arduino sources.
ARDU_SRC = ../../mainProgram

# Compiler flags.
CFLAGS = -O0 -g3 -Wall -I$(ARDU_SRC) -D SIMULATOR

# Different dependencies:
ATLANTIC_SIM_DEPS = atlanticSimulator.cpp GpsSimulator.cpp $(ARDU_SRC)/Gps.cpp

# Build the Atlantic simulator.
atlanticSimulator : $(ATLANTIC_SIM_DEPS)
	$(CC) -o $@ $(CFLAGS) $^


