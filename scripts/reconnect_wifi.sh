#!/bin/bash

SSID="CTRL_LAB_TPLINK_DECO"
THRESHOLD=70

# Get the signal strength for the SSID
SIGNAL=$(nmcli -f SSID,SIGNAL dev wifi list | grep "$SSID" | awk '{print $2}' | tr -d '%')

# Check if connected to the SSID and signal is below threshold
if [[ -n "$SIGNAL" && "$SIGNAL" -lt "$THRESHOLD" ]]; then
    echo "Signal strength ($SIGNAL) below $THRESHOLD. Reconnecting to $SSID..."
    nmcli con down id "$SSID"
    # sleep 1
    nmcli con up id "$SSID"
else
    echo "Signal strength is $SIGNAL, which is above $THRESHOLD or not connected to $SSID."
fi