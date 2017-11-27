import RPi.GPIO as GPIO
import time

class io:
    def __init__(self):
        self.pwm_cut_low_pin = 3
        self.pwm_cut_high_pin = 5
        self.human_interaction_pin0 = 7
        self.human_interaction_pin1 = 11
        self.human_interaction_pin2 = 13
        self.human_interaction_pin3 = 15
        pins = [self.pwm_cut_low_pin, self.pwm_cut_high_pin, self.human_interaction_pin0, self.human_interaction_pin1, self.human_interaction_pin2, self.human_interaction_pin3 ]

        GPIO.setmode(GPIO.BOARD)  # Use standard pin numbers
        GPIO.setwarnings(False)  # Disable warning of multiple read input ports
        for i in pins:
            GPIO.setup(i, GPIO.OUT)

    def on(self, pin):
        "Turn on specified pin"
        GPIO.output(pin, 1)  # 1 for sourcing, 0 for sinking

    def off(self, pin):
        "Turn off specified pin"
        GPIO.output(pin, 0)  # 1 for sinking, 0 for sourcing

    def enable_pwm(self):
        self.off(self.pwm_cut_low_pin)
        self.on(self.pwm_cut_high_pin)

    def disable_pwm(self):
        self.on(self.pwm_cut_low_pin)
        self.off(self.pwm_cut_high_pin)

    def human_interaction_control(self,pin0,pin1,pin2,pin3):
        if pin0:
            self.on(self.human_interaction_pin0)
        else:
            self.off(self.human_interaction_pin0)
        if pin1:
            self.on(self.human_interaction_pin1)
        else:
            self.off(self.human_interaction_pin1)
        if pin2:
            self.on(self.human_interaction_pin2)
        else:
            self.off(self.human_interaction_pin2)
        if pin3:
            self.on(self.human_interaction_pin3)
        else:
            self.off(self.human_interaction_pin3)

    def hi_off(self):
        self.human_interaction_control(False,False,False,False)

    def hi_flash_rotation(self):
        self.human_interaction_control(False, False, False, True)

    def hi_indicators(self):
        self.human_interaction_control(False, False, True, False)

    def hi_flash_rotation_indicators(self):
        self.human_interaction_control(False, False, True, True)

    def hi_siren(self):
        self.human_interaction_control(False,True,False,False)

    def hi_flash_rotation_siren(self):
        self.human_interaction_control(False,True,False,True)

    def hi_flash_rotation_indicators_siren(self):
        self.human_interaction_control(False,True,True,False)

    def hi_safe(self):
        self.human_interaction_control(False,True,True,True)

    def hi_landing(self):
        self.human_interaction_control(True,False,False,False)

    def hi_landing_siren(self):
        self.human_interaction_control(True,False,False,True)

    def read_hi_pins(self):
        return self.read_pin(self.human_interaction_pin0), self.read_pin(self.human_interaction_pin1), self.read_pin(self.human_interaction_pin2), self.read_pin(self.human_interaction_pin3)

    def read_pwm_pins(self):
        return self.read_pin(self.pwm_cut_low_pin), self.read_pin(self.pwm_cut_high_pin)

    def read_pin(self,pin):
        return GPIO.input(pin)


def main():
    gpio = io()

    gpio.enable_pwm()
    print gpio.read_pwm_pins()
    time.sleep(1)

    gpio.disable_pwm()
    print gpio.read_pwm_pins()
    time.sleep(1)

    gpio.hi_off()
    print gpio.read_hi_pins()
    time.sleep(1)

    gpio.hi_flash_rotation()
    print gpio.read_hi_pins()
    time.sleep(1)

    gpio.hi_indicators()
    print gpio.read_hi_pins()
    time.sleep(1)

    gpio.hi_flash_rotation_indicators()
    print gpio.read_hi_pins()
    time.sleep(1)

    gpio.hi_siren()
    print gpio.read_hi_pins()
    time.sleep(1)

    gpio.hi_flash_rotation_siren()
    print gpio.read_hi_pins()
    time.sleep(1)

    gpio.hi_flash_rotation_indicators_siren()
    print gpio.read_hi_pins()
    time.sleep(1)

    gpio.hi_safe()
    print gpio.read_hi_pins()
    time.sleep(1)

    gpio.hi_landing()
    print gpio.read_hi_pins()
    time.sleep(1)

    gpio.hi_landing_siren()
    print gpio.read_hi_pins()
    time.sleep(1)

if __name__ == "__main__":
    main()