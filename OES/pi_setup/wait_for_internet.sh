#!/bin/bash

while ! ping -c 1 -W 1 8.8.8.8; do
    echo "Waiting for ping - network interface might be down..."
    sleep 1
done

echo "Connected"

