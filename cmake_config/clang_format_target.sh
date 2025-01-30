#!/bin/bash

echo "Checking for clang-format 18..."

# Use clang-format-18
if ! command -v clang-format-18 > /dev/null 2>&1; then
    echo "clang-format 18 not found, skipping format target"
    exit 0 
fi

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
echo "Searching for .clang-format file in $PROJECT_DIR..."

CLANG_FORMAT_FILE=$(find "$PROJECT_DIR" -name ".clang-format" -print -quit)
if [ -z "$CLANG_FORMAT_FILE" ]; then
    echo "No .clang-format file found in $PROJECT_DIR, skipping format target"
    exit 0
fi

echo "Found .clang-format file at $CLANG_FORMAT_FILE"

echo "Running clang-format on the following files..."

FILES=$(find "$PROJECT_DIR" -type f \( -name "*.cpp" -o -name "*.h" \))

if [ -z "$FILES" ]; then
    echo "No source files found in $PROJECT_DIR, skipping format target"
    exit 0
fi

echo "$FILES"

for FILE in $FILES; do
    clang-format-18 -i --style=file --assume-filename="$CLANG_FORMAT_FILE" "$FILE"
done

echo "clang-format completed successfully."
