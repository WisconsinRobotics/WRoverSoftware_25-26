#!/bin/bash
# --- CONFIGURATION ---
PORT="/dev/ttyACM0"
BAUD="115200"

# Change these on game day based on what the judges give us and what we measure
POLE_LAT="40.1234567"
POLE_LON="-75.1234567"
POLE_ALT="250.5"        # Altitude in meters
DISTANCE="10.5"         # Measured distance in meters
BEARING="135.0"         # Measured compass bearing (e.g. 135 is SE) - just use mobile phone
# ---------------------

echo "Calculating local offset from competition pole..."

# Pipe the python output into bash variables (Lat, Lon, AltCm)
read -r TENT_LAT TENT_LON TENT_ALT_CM < <(python3 calculate_offset.py \
    --pole-lat "$POLE_LAT" \
    --pole-lon "$POLE_LON" \
    --pole-alt "$POLE_ALT" \
    --dist "$DISTANCE" \
    --bearing "$BEARING")

echo "----------------------------------------"
echo "Calculated Tent Coordinates:"
echo "Latitude:  $TENT_LAT"
echo "Longitude: $TENT_LON"
echo "Altitude:  $TENT_ALT_CM cm"
echo "----------------------------------------"
echo "Injecting Fixed Mode parameters into u-blox module at $PORT..."

# Execute the pyubxutils command using the piped variables
python3 -m pyubxutils.ubxset --port "$PORT" --baud "$BAUD" --layer 7 --key \
    CFG_TMODE_MODE,2 \
    CFG_TMODE_POS_TYPE,1 \
    CFG_TMODE_LAT,"$TENT_LAT" \
    CFG_TMODE_LON,"$TENT_LON" \
    CFG_TMODE_HEIGHT,"$TENT_ALT_CM"

if [ $? -eq 0 ]; then
    echo "SUCCESS: Base station locked to Fixed Mode and configuration saved to Flash!"
else
    echo "ERROR: Failed to communicate with the RTK module. Check USB connection/permissions."
fi
