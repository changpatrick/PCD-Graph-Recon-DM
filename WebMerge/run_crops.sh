#!/bin/bash

# Base input files (CHANGE THESE)
PCD1="../../videos_to_pcds/tangle018 255 2025-11-15 18-34-00 T0.65.pcd"
PCD2="../../videos_to_pcds/tangle018r 255 2025-11-15 19-42-48 T0.65.pcd"

# Loop from 0 to 25 in steps of 5
for CROP in 0 5 10 15 20 25; do
  # Calculate decimal value (e.g., 5 -> 0.05, 10 -> 0.10)
  DECIMAL=$(awk "BEGIN {printf \"%.2f\", $CROP/100}")
  
  # Format the output filename based on the crop percentage
  OUTPUT="crop${CROP}.pcd"
  
  echo "---------------------------------------------------------"
  echo "Running with crop percent: ${DECIMAL} -> output: ${OUTPUT}"
  echo "---------------------------------------------------------"
  
  python3 merge_web_scans.py "$PCD1" "$PCD2" --crop-percent "$DECIMAL" --output "$OUTPUT"
done

echo "All crop variations completed!"
