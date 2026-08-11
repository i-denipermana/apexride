# Host build of the platform-independent firmware core.
#
# Everything under ApexRide/src except the storage backend compiles here
# unchanged; only LittleFsRideStore.cpp is device-only and is replaced by
# HostRideStore.cpp.

CXX      ?= c++
CXXFLAGS ?= -std=c++17 -O1 -g -Wall -Wextra -Wno-unused-parameter
BUILD    := build

SRC_DIR := ../ApexRide/src

SOURCES := \
	$(SRC_DIR)/core/Log.cpp \
	$(SRC_DIR)/core/Crc32.cpp \
	$(SRC_DIR)/core/Clock.cpp \
	$(SRC_DIR)/format/TelemetryFormat.cpp \
	$(SRC_DIR)/format/SummaryBuilder.cpp \
	$(SRC_DIR)/fusion/Orientation.cpp \
	$(SRC_DIR)/sensors/ImuManager.cpp \
	$(SRC_DIR)/sensors/GpsManager.cpp \
	$(SRC_DIR)/sensors/MockImuSensor.cpp \
	$(SRC_DIR)/sensors/MockGnssSensor.cpp \
	$(SRC_DIR)/sim/RideSimulator.cpp \
	$(SRC_DIR)/storage/RideStorage.cpp \
	$(SRC_DIR)/ride/RideManager.cpp \
	$(SRC_DIR)/ride/TelemetryRecorder.cpp \
	$(SRC_DIR)/ride/TelemetrySystem.cpp \
	HostRideStore.cpp \
	test_main.cpp

OBJECTS := $(patsubst %.cpp,$(BUILD)/%.o,$(notdir $(SOURCES)))
VPATH   := $(sort $(dir $(SOURCES)))

TARGET := $(BUILD)/apexride_tests

.PHONY: all test asan clean

all: $(TARGET)

test: $(TARGET)
	@$(TARGET) $(BUILD)/testdata

# Address/UB sanitizers. Rebuilds from scratch because the flags must reach
# both the compile and the link step.
asan:
	@$(MAKE) clean
	@$(MAKE) test CXXFLAGS="-std=c++17 -O1 -g -Wall -Wextra -Wno-unused-parameter \
		-fsanitize=address,undefined -fno-omit-frame-pointer"

$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o $@ $^

$(BUILD)/%.o: %.cpp | $(BUILD)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(BUILD):
	@mkdir -p $(BUILD)

clean:
	rm -rf $(BUILD)
