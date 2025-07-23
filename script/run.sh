#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Define paths
# QUERY_PCD_DIR="/home/dxz/LoopClosureDetection/github/dvmm/data/mcd_kthnight01/mid70/Submaps"
# CANDIDATE_PCD_DIR="/home/dxz/LoopClosureDetection/github/dvmm/data/mcd_kthnight01/ost64/submaps"
# CONFIG="/home/dxz/LoopClosureDetection/github/dvmm/config/MCD.yaml"

QUERY_PCD_DIR="/home/dxz/LoopClosureDetection/github/dvmm/data/tiers_road03/ost64/Submaps"
CANDIDATE_PCD_DIR="/home/dxz/LoopClosureDetection/github/dvmm/data/tiers_road03/avia/Submaps"
CONFIG_PATH="/home/dxz/LoopClosureDetection/github/dvmm/config/TIERS.yaml"

# Check if the executable exists
if [ ! -f "../build/dvmm_demo" ]; then
    echo "[Error] dvmm_demo not found. Please build the project first (run cmake and make)."
    exit 1
fi

# Run the program
echo "Running dvmm_demo..."
../build/dvmm_demo "$QUERY_PCD_DIR" "$CANDIDATE_PCD_DIR" "$CONFIG"
