#!/bin/bash

# Define the base directory for the simulation files
SIM_DIR="extras/sim"

# Generate environment variables for subdirectories
export SIM_LAUNCH_DIR="$SIM_DIR/launch"
export SIM_MODELS_DIR="$SIM_DIR/models"
export SIM_URDF_DIR="$SIM_DIR/urdf"
export SIM_WORLDS_DIR="$SIM_DIR/worlds"
export SIM_CONFIG_DIR="$SIM_DIR/config"

# Define the target directories (set these as environment variables beforehand)
TARGET_LAUNCH_DIR="$HOME/catkin_ws/src/rotors_simulator/rotors_gazebo/launch"
TARGET_MODELS_DIR="$HOME/catkin_ws/src/rotors_simulator/rotors_gazebo/models"
TARGET_URDF_DIR="$HOME/catkin_ws/src/rotors_simulator/rotors_description/urdf"
TARGET_WORLDS_DIR="$HOME/catkin_ws/src/rotors_simulator/rotors_gazebo/worlds"
TARGET_CONFIG_DIR="$HOME/catkin_ws/src/rotors_simulator/rotors_gazebo/config"

# Function to create soft links
create_links() {
    local source_dir=$1
    local target_dir=$2

    if [ -d "$source_dir" ]; then
        mkdir -p "$target_dir"

        # Remove existing symbolic links in the target directory
        find "$target_dir" -type l -exec rm -f {} \;

        for file in "$source_dir"/*; do
            if [ -e "$file" ]; then
                ln -sf "$(realpath "$file")" "$target_dir/"
                echo "Linked $(basename "$file") to $target_dir"
            fi
        done
    else
        echo "Source directory $source_dir does not exist. Skipping."
    fi
}

# Create soft links for each subdirectory
create_links "$SIM_LAUNCH_DIR" "$TARGET_LAUNCH_DIR"
create_links "$SIM_MODELS_DIR" "$TARGET_MODELS_DIR"
create_links "$SIM_URDF_DIR" "$TARGET_URDF_DIR"
create_links "$SIM_WORLDS_DIR" "$TARGET_WORLDS_DIR"
create_links "$SIM_CONFIG_DIR" "$TARGET_CONFIG_DIR"

echo "All files have been linked successfully."