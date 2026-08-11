# Host builds of the developer tools.
#
#   make -C tools
#   tools/build/ridedump   <ride.bin>      decode and chart a ride file
#   tools/build/syncserver --dir <path>    serve the sync API over HTTP
#   tools/build/syncclient --host <h>      run the auto-sync loop against it

CXX      ?= c++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Wno-unused-parameter

# Header dependency tracking; see the note in tests/Makefile.
DEPFLAGS := -MMD -MP
BUILD    := build

SRC_DIR := ../ApexRide/src

# Shared by both tools.
COMMON := \
	$(SRC_DIR)/core/Crc32.cpp \
	$(SRC_DIR)/format/TelemetryFormat.cpp \
	$(SRC_DIR)/format/SummaryBuilder.cpp

# syncserver additionally needs the storage and sync layers.
SERVER_ONLY := \
	$(SRC_DIR)/core/Log.cpp \
	$(SRC_DIR)/core/Clock.cpp \
	$(SRC_DIR)/storage/RideStorage.cpp \
	$(SRC_DIR)/sync/SyncService.cpp \
	$(SRC_DIR)/sync/SyncProtocol.cpp \
	../hostfs/HostRideStore.cpp

CLIENT_ONLY := ../refclient/AutoSyncClient.cpp

SOURCES := $(COMMON) $(SERVER_ONLY) $(CLIENT_ONLY) ridedump.cpp syncserver.cpp syncclient.cpp

OBJECTS       := $(patsubst %.cpp,$(BUILD)/%.o,$(notdir $(SOURCES)))
DUMP_OBJECTS  := $(patsubst %.cpp,$(BUILD)/%.o,$(notdir $(COMMON))) $(BUILD)/ridedump.o
SERVER_OBJECTS := $(patsubst %.cpp,$(BUILD)/%.o,$(notdir $(COMMON) $(SERVER_ONLY))) \
	$(BUILD)/syncserver.o

VPATH := $(sort $(dir $(SOURCES)))

CLIENT_OBJECTS := $(BUILD)/Crc32.o $(BUILD)/AutoSyncClient.o $(BUILD)/syncclient.o

TARGETS := $(BUILD)/ridedump $(BUILD)/syncserver $(BUILD)/syncclient

.PHONY: all clean

all: $(TARGETS)

$(BUILD)/ridedump: $(DUMP_OBJECTS)
	$(CXX) $(CXXFLAGS) -o $@ $^

$(BUILD)/syncserver: $(SERVER_OBJECTS)
	$(CXX) $(CXXFLAGS) -o $@ $^

$(BUILD)/syncclient: $(CLIENT_OBJECTS)
	$(CXX) $(CXXFLAGS) -o $@ $^

$(BUILD)/%.o: %.cpp | $(BUILD)
	$(CXX) $(CXXFLAGS) $(DEPFLAGS) -c -o $@ $<

$(BUILD):
	@mkdir -p $(BUILD)

clean:
	rm -rf $(BUILD)

-include $(OBJECTS:.o=.d)
