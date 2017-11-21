#!/usr/bin/python

import time
import os.path

GPIO_RESET = False  # Whether GPIOs should be re-exported
GPIO_PATH = "/sys/class/gpio"
GPIO_DIR_OUT = "out"
GPIO_VAL_HI = "1"
GPIO_VAL_LO = "0"
GPIO_CHAN_NUM = "46"  # GPIO1 on Apalis T30

BLINK_PERIOD = 500   # Blink period (milliseconds)
BLINK_DUTY = 0.25  # Blink duty cycle (fraction)


def main():
    try:
        # ## Initialize GPIO - optionally reset if already initialized

        # # Note: GPIOs which are already used in the drivers can not be controlled from sysfs,
        # # unless a driver explicitly exported that particular pins GPIO.

        # Open GPIO export & unexport files
        exportFile = open(GPIO_PATH + '/export', 'w')
        unexportFile = open(GPIO_PATH + '/unexport', 'w')

        # Unexport GPIO if it exists and GPIO_RESET is enabled
        exportExists = os.path.isdir(GPIO_PATH + '/gpio' + GPIO_CHAN_NUM)
        if exportExists and GPIO_RESET:
            unexportFile.write(GPIO_CHAN_NUM)
            unexportFile.flush()

        # Export GPIO
        if not exportExists or GPIO_RESET:
            exportFile.write(GPIO_CHAN_NUM)
            exportFile.flush()

        # Open GPIO direction file to set direction
        directionFile = open(GPIO_PATH + '/gpio' + GPIO_CHAN_NUM + '/direction', 'w')

        # Set GPIO direction to "out"
        directionFile.write(GPIO_DIR_OUT)
        directionFile.flush()

        # Open GPIO value file to set value
        valueFile = open(GPIO_PATH + '/gpio' + GPIO_CHAN_NUM + '/value', 'w')

        # Loop indefinitely
        while True:

            # Set GPIO value to HI
            valueFile.write(GPIO_VAL_HI)
            valueFile.flush()

            # Sleep for blink on duration
            time.sleep(BLINK_PERIOD * BLINK_DUTY / 1000.0)

            # Set GPIO value to LO
            valueFile.write(GPIO_VAL_LO)
            valueFile.flush()

            # Sleep for blink off duration
            time.sleep(BLINK_PERIOD * (1.0 - BLINK_DUTY) / 1000.0)

    except Exception as e:
        e.printStackTrace()
    return


if __name__ == "__main__":
    main()
