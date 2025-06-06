#!/bin/bash

# Source and destination directories
SOURCE_DIR="$HOME/car_exports"
DEST_DIR="extras/sim/models"

# Check if source directory exists
if [ ! -d "$SOURCE_DIR" ]; then
    echo "Error: Source directory '$SOURCE_DIR' not found."
    exit 1
fi

# Check if destination directory exists
if [ ! -d "$DEST_DIR" ]; then
    echo "Error: Destination directory '$DEST_DIR' not found."
    exit 1
fi

# Move files from source to destination
echo "Moving files from $SOURCE_DIR to $DEST_DIR..."
mv "$SOURCE_DIR"/* "$DEST_DIR"/

# Check if the move was successful
if [ $? -ne 0 ]; then
    echo "Error: Failed to move files."
    exit 1
fi

# Run the modify_all_brightness.sh script
echo "Running modify_all_brightness.sh..."
./modify_all_brightness.sh

# Check if the script ran successfully
if [ $? -ne 0 ]; then
    echo "Error: modify_all_brightness.sh encountered an issue."
    exit 1
fi

echo "All tasks completed successfully!"