#!/bin/bash
# Simple script to view the octomap periodically

MAP_FILE="../build/global_map.ot"
VISUALIZER="../lib/octomap/bin/octovis"

if [ ! -f "$MAP_FILE" ]; then
    echo "Map file not found: $MAP_FILE"
    echo "Make sure map_server has been running and saving data"
    exit 1
fi

echo "Opening octomap visualization..."
echo "File: $MAP_FILE"
$VISUALIZER $MAP_FILE
