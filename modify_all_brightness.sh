#!/bin/bash

# Path to the Python script (update this to your actual path)
PYTHON_SCRIPT="modify_brightness.py"

# Root directory containing Gazebo models
MODELS_DIR="extras/sim/models"

# Check if Python script exists
if [ ! -f "$PYTHON_SCRIPT" ]; then
    echo "Error: Python script '$PYTHON_SCRIPT' not found."
    exit 1
fi

# Check if models directory exists
if [ ! -d "$MODELS_DIR" ]; then
    echo "Error: Models directory '$MODELS_DIR' not found."
    exit 1
fi

# Process each model directory
for model_dir in "$MODELS_DIR"/*/; do
    if [ -d "$model_dir/meshes" ]; then
        echo "Processing: $model_dir"
        python "$PYTHON_SCRIPT" "$model_dir"
    else
        echo "Skipping (no meshes/): $model_dir"
    fi
done

echo "Done!"