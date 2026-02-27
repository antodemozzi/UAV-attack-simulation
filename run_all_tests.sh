#!/bin/bash
# =============================================================
# Batch simulation runner for DronesFinal
# Runs all combinations of parameters and collects results
# =============================================================

# Parameter arrays - EDIT THESE to change test configurations
DRONES=(2 5 10)
TRUCKS=(2 3 10)
SPEEDS=(1 2 10)
ANCHOR_MODES=("trucks" "drones" "hybrid")

# Output file for results
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
RESULTS_FILE="simulation_results_${TIMESTAMP}.csv"

# CSV header
echo "nDrones,nTrucks,droneSpeedX,anchorMode,TotalMeasurements,AvgError_m" > "$RESULTS_FILE"

# Count total simulations
TOTAL=$(( ${#DRONES[@]} * ${#TRUCKS[@]} * ${#SPEEDS[@]} * ${#ANCHOR_MODES[@]} ))
CURRENT=0

echo "=============================================="
echo "  BATCH SIMULATION RUNNER"
echo "  Total combinations: $TOTAL"
echo "  Anchor modes: ${ANCHOR_MODES[*]}"
echo "  Results file: $RESULTS_FILE"
echo "=============================================="
echo ""

for nd in "${DRONES[@]}"; do
    for nt in "${TRUCKS[@]}"; do
        for sp in "${SPEEDS[@]}"; do
            for am in "${ANCHOR_MODES[@]}"; do
                CURRENT=$((CURRENT + 1))
                echo "----------------------------------------------"
                echo "[$CURRENT/$TOTAL] nDrones=$nd  nTrucks=$nt  droneSpeedX=$sp  anchorMode=$am"
                echo "----------------------------------------------"
                
                # Run simulation and capture output
                OUTPUT=$(./ns3 run "DronesFinal --nDrones=$nd --nTrucks=$nt --droneSpeedX=$sp --anchorMode=$am" 2>&1)
                
                # Extract results from output
                MEASUREMENTS=$(echo "$OUTPUT" | grep "Total measurements:" | awk '{print $NF}')
                AVG_ERROR=$(echo "$OUTPUT" | grep "Average error:" | awk '{print $(NF-1)}')
                
                # Handle case where no measurements were recorded
                if [ -z "$MEASUREMENTS" ]; then
                    MEASUREMENTS="0"
                    AVG_ERROR="N/A"
                fi
                
                echo "  -> Measurements: $MEASUREMENTS | Avg Error: $AVG_ERROR m"
                echo ""
                
                # Append to CSV
                echo "$nd,$nt,$sp,$am,$MEASUREMENTS,$AVG_ERROR" >> "$RESULTS_FILE"
            done
        done
    done
done

echo ""
echo "=============================================="
echo "  ALL $TOTAL SIMULATIONS COMPLETE!"
echo "  Results saved to: $RESULTS_FILE"
echo "=============================================="
echo ""

# Print summary table
echo "=== RESULTS SUMMARY ==="
echo ""
printf "%-10s %-10s %-12s %-12s %-15s %-12s\n" "nDrones" "nTrucks" "Speed(m/s)" "AnchorMode" "Measurements" "AvgError(m)"
printf "%-10s %-10s %-12s %-12s %-15s %-12s\n" "-------" "-------" "----------" "----------" "------------" "-----------"
while IFS=',' read -r nd nt sp am meas err; do
    # Skip header
    if [ "$nd" = "nDrones" ]; then continue; fi
    printf "%-10s %-10s %-12s %-12s %-15s %-12s\n" "$nd" "$nt" "$sp" "$am" "$meas" "$err"
done < "$RESULTS_FILE"
echo ""
