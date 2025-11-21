CXX = g++
CXXFLAGS = -std=c++17 -Wall -O2 -g
INCLUDES = -I./Phase1
LIBS = 

# --- Directory Definitions ---
PHASE1_DIR = Phase1

# Core logic (Graph and Algorithm are still needed)
CORE_SRC = $(PHASE1_DIR)/Graph.cpp \
           $(PHASE1_DIR)/Algorithm.cpp
CORE_OBJ = $(CORE_SRC:.cpp=.o)

# Sample Driver Definitions
SAMPLE_DRIVER_SRC = sampledriver.cpp
SAMPLE_DRIVER_OBJ = sampledriver.o
SAMPLE_TARGET = sampledriver

# --- Targets ---
.PHONY: all clean sampledriver

# Default target: Only builds sampledriver
all: sampledriver

$(SAMPLE_TARGET): $(CORE_OBJ) $(SAMPLE_DRIVER_OBJ)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $(SAMPLE_DRIVER_OBJ) $(CORE_OBJ) $(LIBS)
	@echo "Sample Driver built successfully!"
	@echo "Usage: ./sampledriver <graph.json> <queries.json> <output.json>"

# --- Compilation Rules ---

# Compile object files in Phase1 directory (Graph.cpp, Algorithm.cpp)
$(PHASE1_DIR)/%.o: $(PHASE1_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# Compile sampledriver.cpp (located in root)
$(SAMPLE_DRIVER_OBJ): $(SAMPLE_DRIVER_SRC)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# --- Utilities ---

clean:
	rm -f $(CORE_OBJ) $(SAMPLE_DRIVER_OBJ)
	rm -f $(SAMPLE_TARGET)
	rm -f output.json
	@echo "Clean complete!"