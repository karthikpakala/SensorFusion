#!/bin/bash

# Install Git Hooks for SensorFusion Project

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GIT_HOOKS_DIR="$PROJECT_ROOT/.git/hooks"
CUSTOM_HOOKS_DIR="$PROJECT_ROOT/git-hooks"

echo "Installing Git hooks for SensorFusion project..."

# Check if we're in a git repository
if [ ! -d "$PROJECT_ROOT/.git" ]; then
    echo "Error: Not in a Git repository. Please run 'git init' first."
    exit 1
fi

# Create git hooks directory if it doesn't exist
mkdir -p "$GIT_HOOKS_DIR"

# Install pre-commit hook
if [ -f "$CUSTOM_HOOKS_DIR/pre-commit" ]; then
    cp "$CUSTOM_HOOKS_DIR/pre-commit" "$GIT_HOOKS_DIR/pre-commit"
    chmod +x "$GIT_HOOKS_DIR/pre-commit"
    echo "✅ Pre-commit hook installed successfully"
else
    echo "❌ Pre-commit hook not found at $CUSTOM_HOOKS_DIR/pre-commit"
    exit 1
fi

echo ""
echo "Git hooks have been installed!"
echo ""
echo "The pre-commit hook will:"
echo "  - Check that all C++ files are properly formatted with clang-format"
echo "  - Prevent commits if formatting issues are found"
echo ""
echo "To bypass the hook (not recommended): git commit --no-verify"
echo "To format your code: ./format-and-lint.sh format"