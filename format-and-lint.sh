#!/bin/bash

# Format and Lint Script for SensorFusion Project
# This script provides easy access to clang-format and clang-tidy tools

set -e  # Exit on any error

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${PROJECT_ROOT}/build"

print_usage() {
    echo "Usage: $0 [OPTION]"
    echo "Format and lint the SensorFusion codebase"
    echo ""
    echo "Options:"
    echo "  format        Format all source files with clang-format"
    echo "  format-check  Check if files need formatting (dry-run)"
    echo "  format-diff   Show formatting differences"
    echo "  tidy          Run clang-tidy static analysis"
    echo "  tidy-fix      Run clang-tidy with automatic fixes"
    echo "  lint          Run both format and tidy"
    echo "  setup         Install clang-format and clang-tidy"
    echo "  help          Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0 format          # Format all source files"
    echo "  $0 format-check    # Check formatting without changes"
    echo "  $0 tidy            # Run static analysis"
    echo "  $0 lint            # Run both formatting and analysis"
}

check_tools() {
    if ! command -v clang-format &> /dev/null; then
        echo "Error: clang-format not found. Please install it first."
        echo "Run: $0 setup"
        exit 1
    fi

    if ! command -v clang-tidy &> /dev/null; then
        echo "Error: clang-tidy not found. Please install it first."
        echo "Run: $0 setup"
        exit 1
    fi
}

setup_tools() {
    echo "Installing clang-format and clang-tidy..."
    
    if command -v apt-get &> /dev/null; then
        # Ubuntu/Debian
        sudo apt-get update
        sudo apt-get install -y clang-format clang-tidy
    elif command -v yum &> /dev/null; then
        # RHEL/CentOS/Fedora
        sudo yum install -y clang-tools-extra
    elif command -v brew &> /dev/null; then
        # macOS
        brew install clang-format
        brew install llvm  # includes clang-tidy
    else
        echo "Unsupported package manager. Please install clang-format and clang-tidy manually."
        exit 1
    fi
    
    echo "Tools installed successfully!"
}

build_if_needed() {
    if [ ! -d "$BUILD_DIR" ]; then
        echo "Build directory not found. Creating build..."
        mkdir -p "$BUILD_DIR"
        cd "$BUILD_DIR"
        cmake ..
        make -j$(nproc)
        cd "$PROJECT_ROOT"
    elif [ ! -f "$BUILD_DIR/compile_commands.json" ]; then
        echo "compile_commands.json not found. Rebuilding..."
        cd "$BUILD_DIR"
        cmake ..
        make -j$(nproc)
        cd "$PROJECT_ROOT"
    fi
}

format_code() {
    echo "Formatting source files..."
    find src include -name "*.cpp" -o -name "*.h" -o -name "*.cu" | xargs clang-format -i -style=file
    echo "Formatting completed!"
}

format_check() {
    echo "Checking formatting..."
    find src include -name "*.cpp" -o -name "*.h" -o -name "*.cu" | xargs clang-format --dry-run --Werror -style=file
    echo "Format check completed!"
}

format_diff() {
    echo "Showing formatting differences..."
    find src include -name "*.cpp" -o -name "*.h" -o -name "*.cu" | xargs clang-format --dry-run -style=file
}

run_tidy() {
    echo "Running clang-tidy static analysis..."
    build_if_needed
    find src -name "*.cpp" | xargs clang-tidy -p="$BUILD_DIR"
    echo "Static analysis completed!"
}

run_tidy_fix() {
    echo "Running clang-tidy with automatic fixes..."
    build_if_needed
    find src -name "*.cpp" | xargs clang-tidy -p="$BUILD_DIR" -fix
    echo "Static analysis with fixes completed!"
}

run_lint() {
    format_code
    run_tidy
    echo "Linting completed!"
}

# Main script logic
case "${1:-}" in
    format)
        check_tools
        format_code
        ;;
    format-check)
        check_tools
        format_check
        ;;
    format-diff)
        check_tools
        format_diff
        ;;
    tidy)
        check_tools
        run_tidy
        ;;
    tidy-fix)
        check_tools
        run_tidy_fix
        ;;
    lint)
        check_tools
        run_lint
        ;;
    setup)
        setup_tools
        ;;
    help)
        print_usage
        ;;
    *)
        echo "Error: Invalid option '${1:-}'"
        print_usage
        exit 1
        ;;
esac