BIN_DIR:=build
BIN:=$(BIN_DIR)/motion_runner

CXX:=g++
CXXFLAGS:=-std=c++17 -O2 -Wall -Wextra -Werror -I./src
LDFLAGS:=

SRC_MD:=src/motion_detection/motion_detection.cpp \
        src/motion_detection/filters.cpp \
        src/motion_detection/storage.cpp

SRC_STUB:=src/cli/console_eeprom_stubs.cpp \
          src/motion_detection/motionfx_wrapper_stub.cpp \
          src/cli/motion_runner.cpp

OBJS:=$(patsubst %.cpp,$(BIN_DIR)/%.o,$(SRC_MD) $(SRC_STUB))

.PHONY: all clean run

all: $(BIN)

$(BIN): $(OBJS)
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

$(BIN_DIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf $(BIN_DIR)

run: $(BIN)
	$(BIN)

