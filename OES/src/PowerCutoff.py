#!/usr/bin/python

# import time
import os.path

GPIO_RESET = False  # Whether GPIOs should be re-exported
GPIO_PATH = "/sys/class/gpio"
GPIO_DIR_OUT = "out"
GPIO_VAL_HI = "1"
GPIO_VAL_LO = "0"
# pin 1 on the cutoff board
GPIO_CHAN_PIN19 = "49"  # GPIO19 on the Viola board/ GPIO14 on the Iris board
# pin 3 on the cutoff board
GPIO_CHAN_PIN20 = "41"  # GPIO20 on the Viola board/ GPIO14 on the Iris board
cutOff = False


def main():
    try:
        exportFile = open(GPIO_PATH + '/export', 'w')
        unexportFile = open(GPIO_PATH + '/unexport', 'w')

        # Unexport GPIO if it exists and GPIO_RESET is enabled
        exportExists = os.path.isdir(GPIO_PATH + '/gpio' + GPIO_CHAN_PIN19)
        exportExists = os.path.isdir(GPIO_PATH + '/gpio' + GPIO_CHAN_PIN20)
        if exportExists and GPIO_RESET:
            unexportFile.write(GPIO_CHAN_PIN19)
            unexportFile.write(GPIO_CHAN_PIN20)
            unexportFile.flush()

        # Export GPIO
        if not exportExists or GPIO_RESET:
            exportFile.write(GPIO_CHAN_PIN19)
            exportFile.write(GPIO_CHAN_PIN20)
            exportFile.flush()

        # Open GPIO direction file to set direction
        directionFile19 = open(GPIO_PATH + '/gpio' + GPIO_CHAN_PIN19 + '/direction', 'w')
        directionFile20 = open(GPIO_PATH + '/gpio' + GPIO_CHAN_PIN20 + '/direction', 'w')

        # Set GPIO direction to "out"
        directionFile19.write(GPIO_DIR_OUT)
        directionFile19.flush()
        directionFile20.write(GPIO_DIR_OUT)
        directionFile20.flush()

        # Open GPIO value file to set value
        valueFile19 = open(GPIO_PATH + '/gpio' + GPIO_CHAN_PIN19 + '/value', 'w')
        valueFile20 = open(GPIO_PATH + '/gpio' + GPIO_CHAN_PIN20 + '/value', 'w')

        # Loop indefinitely
        while not cutOff:

            # Set GPIO value to HI
            valueFile19.write(GPIO_VAL_HI)
            valueFile19.flush()
            valueFile20.write(GPIO_VAL_LO)
            valueFile20.flush()

        valueFile19.write(GPIO_VAL_LO)
        valueFile19.flush()
        valueFile20.write(GPIO_VAL_HI)
        valueFile20.flush()

    except Exception as e:
        e.printStackTrace()
    return


if __name__ == "__main__":
    main()
