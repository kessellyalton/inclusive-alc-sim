#!/usr/bin/env python3
import subprocess
import sys
import time
import re

CHECK_SCRIPT = ["python3", "scripts/check_missing_runs.py"]
RUN_SCRIPT   = ["python3", "scripts/run_batch_simulations.py"]

SLEEP_BETWEEN_LOOPS = 5  # seconds


def run_and_capture(cmd):
    result = subprocess.run(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True
    )
    return result.stdout


print("\n==============================")
print("AUTOMATED RUN-UNTIL-COMPLETE")
print("==============================\n")

iteration = 0

while True:
    iteration += 1
    print(f"\n--- ITERATION {iteration} ---\n")

    print("🔍 Checking missing runs...")
    check_output = run_and_capture(CHECK_SCRIPT)
    print(check_output)

    # Look for "Missing runs: X"
    match = re.search(r"Missing runs:\s*(\d+)", check_output)

    if not match:
        print("❌ Could not determine missing runs count. Aborting.")
        sys.exit(1)

    missing = int(match.group(1))

    if missing == 0:
        print("\n✅ ALL REQUIRED RUNS COMPLETED.")
        print("Stopping automatically.\n")
        break

    print(f"🚀 {missing} runs missing — launching simulations...")
    subprocess.run(RUN_SCRIPT)

    print(f"⏳ Waiting {SLEEP_BETWEEN_LOOPS}s before re-checking...")
    time.sleep(SLEEP_BETWEEN_LOOPS)

print("🎉 Coverage complete. Safe to generate final tables.")

