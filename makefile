# Standalone Makefile for escDriver_to_MQTT
# Minimal application - no external dependencies

CXX = g++
CXXFLAGS = -g -std=c++17 -Wall -fPIC
LDFLAGS = -lpthread

SOURCES = Driver.cpp \
          main.cpp

OBJECTS = $(SOURCES:.cpp=.o)
DEPS = $(SOURCES:.cpp=.d)

BINFILENAME = escDriver_to_MQTT

# Build target
all: $(BINFILENAME)

$(BINFILENAME): $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJECTS) $(LDFLAGS)

# Compilation rules
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Dependency generation
%.d: %.cpp
	$(CXX) $(CXXFLAGS) -MM $< > $@

# Include dependency files
-include $(DEPS)

# Clean build artifacts
clean:
	rm -f $(OBJECTS) $(SOURCES:.cpp=.d) $(BINFILENAME)

.PHONY: all clean
