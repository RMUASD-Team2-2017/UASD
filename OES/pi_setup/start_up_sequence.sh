#!/bin/bash
python /home/pi/workspace/setup/control_io.py pwm_on
python /home/pi/workspace/setup/control_io.py start_up
/home/pi/workspace/setup/wait_for_internet.sh
python /home/pi/workspace/setup/control_io.py network

