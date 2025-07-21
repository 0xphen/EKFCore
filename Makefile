BUILD_DIR = build
SRC_DIR = src
TEST_DIR = test
EXE_NAME = ekfusion2d_sim
TEST_EXE_NAME = ekfusion2d_tests

# Source file extensions that clang-format should process
FORMAT_EXTS = cpp hpp cxx h

.PHONY: all build clean format run test cmake-generate debug-info

all: build

build: $(BUILD_DIR)/Makefile
	@echo "==================================="
	@echo "     Building Project: $(EXE_NAME)"
	@echo "==================================="

$(BUILD_DIR)/Makefile:
	@echo "-----------------------------------"
	@echo " Generating CMake build files in $(BUILD_DIR)/"
	@echo "-----------------------------------"
	mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && cmake ..

clean:
	@echo "==================================="
	@echo "     Cleaning Build Directory"
	@echo "==================================="
	rm -rf $(BUILD_DIR)
	@echo "Clean complete."

format:
	@echo "==================================="
	@echo "     Formatting C++ Code"
	@echo "==================================="
	# The `find` command locates all files with specified extensions.
	# `\( -name ... -o ... \)` creates an OR condition for file names.
	# `-false` at the end handles the trailing `-o` from the loop.
	# `-exec ... {} +` efficiently executes clang-format on batches of files.
	find $(SRC_DIR) $(TEST_DIR) -type f \( $(foreach ext,$(FORMAT_EXTS),-name "*.$(ext)" -o ) -false \) -exec clang-format -i {} +
	@echo "Code formatting complete."

run: build
	@echo "==================================="
	@echo "     Running $(EXE_NAME)"
	@echo "==================================="
	# The executable is typically found inside the build directory.
	# Adjust the path if your executable ends up in a 'Debug', 'Release', etc. subfolder.
	# For example: `./$(BUILD_DIR)/Release/$(EXE_NAME)` on Windows MSVC.
	@./$(BUILD_DIR)/$(EXE_NAME)

test: build
	@echo "==================================="
	@echo "     Running Unit Tests"
	@echo "==================================="
	# `ctest` command is run from the build directory.
	# `--output-on-failure` shows test output only for failed tests.
	cd $(BUILD_DIR) && ctest --output-on-failure

cmake-generate:
	@echo "-----------------------------------"
	@echo " Regenerating CMake build files..."
	@echo "-----------------------------------"
	mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && cmake ..

debug-info: $(BUILD_DIR)/Makefile
	@echo "==================================="
	@echo "   Building Project (Debug Info)"
	@echo "==================================="
	cmake --build $(BUILD_DIR) --config Debug