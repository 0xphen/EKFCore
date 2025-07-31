BUILD_DIR := build

.PHONY: all build configure clean rebuild test format

all: build

build:

	@echo "🔧 Running make -j in $(BUILD_DIR)..."
	@cd $(BUILD_DIR) && make -j

configure:
	@echo "⚙️ Configuring CMake..."
	@mkdir -p $(BUILD_DIR)
	@cd $(BUILD_DIR) && cmake -DCMAKE_BUILD_TYPE=Debug .. 

clean:
	@echo "🧹 Cleaning build directory..."
	rm -rf $(BUILD_DIR)

rebuild: clean configure build

test: build
	@echo "🧪 Running tests..."
	@cd $(BUILD_DIR) && ctest --output-on-failure --verbose 

format:
	@echo "🎨 Formatting all C++ source and header files recursively..."
	find . -type f \( -name "*.cpp" -o -name "*.hpp" -o -name "*.cxx" -o -name "*.h" \) -exec clang-format -i {} +
