# Makefile for the WSPR encoded-message library smoke test.
#
# The library is header-only; only test_main.cc compiles to an object.
#
# Targets:
#   all      - build the smoke test (default)
#   test     - build and run the smoke test
#   debug    - build with AddressSanitizer + UBSan (default for `all`)
#   release  - build with optimization, no sanitizers
#   clean    - remove build artifacts
#
# Variables you can override on the command line:
#   CXX=clang++ make           - use a different compiler
#   BUILD=release make         - release build
#   EXTRA_CXXFLAGS='...' make  - append flags

CXX      ?= g++
BUILD    ?= debug

CXXSTD   := -std=c++11
WARNINGS := -Wall -Wextra -Wpedantic
INCLUDES := -I.

# Per-build flags
ifeq ($(BUILD),release)
    BUILD_FLAGS := -O2 -DNDEBUG
else ifeq ($(BUILD),debug)
    BUILD_FLAGS := -O0 -g -fsanitize=address,undefined
    LDFLAGS     += -fsanitize=address,undefined
else
    $(error BUILD must be 'debug' or 'release', got '$(BUILD)')
endif

CXXFLAGS := $(CXXSTD) $(WARNINGS) $(INCLUDES) $(BUILD_FLAGS) $(EXTRA_CXXFLAGS)

# Sources and outputs
HEADERS  := $(wildcard *.h)
TEST_SRC := test_main.cc
TEST_OBJ := $(TEST_SRC:.cc=.o)
TEST_BIN := test_main
DEP_FILE := $(TEST_OBJ:.o=.d)


.PHONY: all test debug release clean help

all: $(TEST_BIN)

# Convenience aliases
debug:
	@$(MAKE) BUILD=debug all

release:
	@$(MAKE) BUILD=release all

# Build the test binary
$(TEST_BIN): $(TEST_OBJ)
	$(CXX) $(LDFLAGS) -o $@ $^

# Compile with auto-generated header dependency tracking (-MMD -MP)
%.o: %.cc
	$(CXX) $(CXXFLAGS) -MMD -MP -c -o $@ $<

# Run the smoke test
test: $(TEST_BIN)
	./$(TEST_BIN)

clean:
	rm -f $(TEST_BIN) $(TEST_OBJ) $(DEP_FILE)

help:
	@echo "Targets:"
	@echo "  all      - build the smoke test (default, debug build)"
	@echo "  test     - build and run the smoke test"
	@echo "  debug    - debug build with ASan+UBSan"
	@echo "  release  - release build with -O2"
	@echo "  clean    - remove build artifacts"
	@echo ""
	@echo "Variables:"
	@echo "  CXX=...           override compiler (default g++)"
	@echo "  BUILD=debug|release  default debug"
	@echo "  EXTRA_CXXFLAGS=...   appended to compile flags"

# Pull in auto-generated header dependencies so edits to any .h file
# trigger a rebuild of test_main.o
-include $(DEP_FILE)
