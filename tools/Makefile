# Host build of the ride file inspector.
#
#   make -C tools
#   tools/build/ridedump <ride.bin>

CXX      ?= c++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Wno-unused-parameter

# Header dependency tracking; see the note in tests/Makefile.
DEPFLAGS := -MMD -MP
BUILD    := build

SRC_DIR := ../ApexRide/src

SOURCES := \
	$(SRC_DIR)/core/Crc32.cpp \
	$(SRC_DIR)/format/TelemetryFormat.cpp \
	$(SRC_DIR)/format/SummaryBuilder.cpp \
	ridedump.cpp

OBJECTS := $(patsubst %.cpp,$(BUILD)/%.o,$(notdir $(SOURCES)))
VPATH   := $(sort $(dir $(SOURCES)))

TARGET := $(BUILD)/ridedump

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o $@ $^

$(BUILD)/%.o: %.cpp | $(BUILD)
	$(CXX) $(CXXFLAGS) $(DEPFLAGS) -c -o $@ $<

$(BUILD):
	@mkdir -p $(BUILD)

clean:
	rm -rf $(BUILD)

-include $(OBJECTS:.o=.d)
