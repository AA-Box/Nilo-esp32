# Host tests for the parts of the board that are plain C++.
#
#   make            build and run every test
#   make clean
#
# No framework and no package manager: a compiler and this file. Everything under test is
# deliberately free of ESP-IDF, FreeRTOS and GPIO, which is what makes that possible — and
# what makes "the robot stops when the link dies" a thing that can be asserted on a laptop.

CXX ?= c++
CXXFLAGS ?= -std=c++20 -O1 -g -Wall -Wextra -Werror -Wshadow -Wconversion -Wsign-conversion
INCLUDES := -I. -I../board
BUILD := build

BOARD_SOURCES := ../board/motion_profile.cc ../board/safety_supervisor.cc \
                 ../board/robot_tool_schema.cc ../board/profiled_drive.cc

TESTS := test_motion_profile test_safety_supervisor test_profiled_drive test_tool_schema

.PHONY: all clean
all: $(addprefix $(BUILD)/,$(TESTS))
	@echo
	@failed=0; for test in $(TESTS); do $(BUILD)/$$test || failed=1; done; \
	 if [ $$failed -ne 0 ]; then echo; echo "host tests failed"; exit 1; fi; \
	 echo; echo "host tests passed"

$(BUILD)/%: %.cc $(BOARD_SOURCES) check.h
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $< $(BOARD_SOURCES)

clean:
	rm -rf $(BUILD)
