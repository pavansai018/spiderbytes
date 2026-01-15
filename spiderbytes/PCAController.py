import board
import busio
from adafruit_pca9685 import PCA9685
import time

class PCA9685Controller:
    def __init__(self, frequency=50):
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(self.i2c)
        self.pca.frequency = frequency

        self.period_us = 1_000_000 / frequency
        self.us_per_step = self.period_us / 4096  # ~4.88

    def set_pulse_us(self, channel: int, pulse_us: float):
        steps = int(pulse_us / self.us_per_step)
        steps = max(0, min(4095, steps))
        self.pca.channels[channel].duty_cycle = steps * 16

    def relax(self, channel: int):
        self.pca.channels[channel].duty_cycle = 0

    def shutdown(self):
        for ch in range(16):
            self.relax(ch)
        self.pca.deinit()

class Servo:
    def __init__(
        self,
        controller: PCA9685Controller,
        channel: int,
        min_pulse_us: int = 450,
        max_pulse_us: int = 2550,
        min_angle: float = 0.0,
        max_angle: float = 180.0,
    ):
        self.ctrl = controller
        self.channel = channel

        self.min_pulse = min_pulse_us
        self.max_pulse = max_pulse_us
        self.min_angle = min_angle
        self.max_angle = max_angle

    def set_angle(self, angle: float):
        # Clamp angle
        angle = max(self.min_angle, min(self.max_angle, angle))

        # Linear mapping
        pulse = self.min_pulse + (
            (angle - self.min_angle)
            / (self.max_angle - self.min_angle)
        ) * (self.max_pulse - self.min_pulse)

        self.ctrl.set_pulse_us(self.channel, pulse)

    def relax(self):
        self.ctrl.relax(self.channel)
def stand_position(pwm_connection_pins, controller):
    servos = {f'servo_{i}': Servo(controller, channel=i) for i in pwm_connection_pins}
    
    servos['servo_8'].set_angle(45)
    
    servos['servo_12'].set_angle(135)
    print(f'Hip Changed')
    time.sleep(2)
    servos['servo_10'].set_angle(30)
    servos['servo_14'].set_angle(30)
    print(f'Legs Changed')
    time.sleep(5)

    servos['servo_9'].set_angle(135)
    servos['servo_13'].set_angle(135)
    print(f'Limb Changed')
    time.sleep(2)
    servos['servo_10'].set_angle(90)
    servos['servo_14'].set_angle(90)
    print(f'Legs Changed')

    time.sleep(1)
    servos['servo_9'].set_angle(90)
    servos['servo_13'].set_angle(90)
    print(f'Limb Changed')


def home_position(controller, home_angle=90, sleep_time=1, pwm_connection_pins=[0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14]):
    
    servos = [
        Servo(controller, channel=i)
        for i in pwm_connection_pins
    ]
    for idx, servo in enumerate(servos):
        servo.set_angle(home_angle)
        print(f'{idx} Motor Completed')
        time.sleep(sleep_time)
def main():
    pwm_connection_pins = [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14]
    controller = PCA9685Controller(frequency=50)
    home_position(controller=controller)
    # time.sleep(5)
    # stand_position(pwm_connection_pins=pwm_connection_pins, controller=controller)
    

# home_position = {i: 90 for i in pwm_connection_pins}

# for channel, angle in home_position.items():
#     servos[channel].set_angle(angle)
if __name__ == '__main__':
    main()
