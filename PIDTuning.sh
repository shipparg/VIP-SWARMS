#!/bin/bash

# Define the initial values for PID parameters
KP=1.0
KI=0.1
KD=0.01

# Define the increment values for each PID parameter
KP_INCREMENT=0.1
KI_INCREMENT=0.05
KD_INCREMENT=0.005

# Define the number of iterations
NUM_ITERATIONS=5

# Loop through and run the Python script with updated PID values
for ((i=1; i<=NUM_ITERATIONS; i++))
do
    echo "Running iteration $i with PID values - KP: $KP, KI: $KI, KD: $KD"

    # Run the Python script with the current PID parameters
    python test_PIDwithBash.py $KP $KI $KD

    # Increment the PID values for the next iteration using awk
    KP=$(awk "BEGIN {print $KP + $KP_INCREMENT}")
    KI=$(awk "BEGIN {print $KI + $KI_INCREMENT}")
    KD=$(awk "BEGIN {print $KD + $KD_INCREMENT}")

    # Wait for a while before starting the next iteration
    sleep 5
done

