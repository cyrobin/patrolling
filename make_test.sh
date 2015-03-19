#! /bin/bash

N=10
# Run the test n times.
for I in `seq 1 $N`;
do
    # Adapt mission file to your needs
    echo "[TEST SCRIPT] New Caylus test launched ($I/$N)"
    ./patrol_mission.py mission_caylus.json
done

echo "[TEST SCRIPT] Done."
exit 0

