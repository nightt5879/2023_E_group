# The servo module, which is used to control the servo motor
# the class HalfCircleServo for the 180° servo and class FullCircleServo for the 360° servo

# import the necessary modules
# create three threads to control the servo and read the angle
import threading
import RPi.GPIO as GPIO
import time


class Servo:
    """
    The servo class, which is used to control the servo motor

    :param pin: the servo's gpio pin
    """

    def __init__(self, pin):
        self.pwm = None
        self.pin = pin
        self.target = 0

    def set_mode(self):
        GPIO.setup(self.pin, GPIO.OUT, initial=False)
        self.pwm = GPIO.PWM(self.pin, 50)
        self.pwm.start(0)

    @staticmethod
    def clamp_number(num, min_number, max_number):
        return max(min(num, max(min_number, max_number)), min(min_number, max_number))


class HalfCircleServo(Servo):
    """
    The half circle servo class, which is used to control the 180° servo motor, can directly set the angle

    :param pin: the servo's gpio pin
    """

    def __init__(self, pin):
        super().__init__(pin)
        self.angle = 0
        self.set_mode()

    def set_mode(self):
        super().set_mode()
        # initialize the servo's angle
        self.set_angle(self.angle)

    def set_angle(self, angle):
        angle = self.clamp_number(angle, 0, 180)
        if self.target != angle:
            self.target = angle
        self.pwm.ChangeDutyCycle(2.5 + 10 * angle / 180)
        # if the time too long, you can not use the two code below
        time.sleep(0.8)
        self.pwm.ChangeDutyCycle(0)

        # update the servo's angle
        self.angle = angle

    def get_angle(self):
        return self.angle


if __name__ == '__main__':
    """
    the process of the create the thread function
    """
    rotate_angle = 0


    def thread_nodding():
        while True:
            if servo1.target != servo1.angle:
                servo1.set_angle(servo1.target)
            time.sleep(0.5)


    def thread_rotating():
        global rotate_angle
        while True:
            if rotate_angle != servo2.angle + servo3.angle:
                rotate_angle = servo2.clamp_number(rotate_angle, 0, 360)
                if 0 <= rotate_angle <= 180:
                    # check the servo3's angle
                    if servo3.angle != 0:
                        servo3.set_angle(0)
                    servo2.set_angle(rotate_angle)
                elif 180 < rotate_angle <= 360:
                    servo2.set_angle(180)
                    print(rotate_angle)
                    servo3.set_angle(rotate_angle - 180)
            time.sleep(0.5)


    """
    the process of the initialization
    """
    # set the gpio mode to BCM
    GPIO.setmode(GPIO.BCM)

    # define the three servo
    servo1 = HalfCircleServo(24)
    servo2 = HalfCircleServo(23)
    servo3 = HalfCircleServo(18)

    # set the servo's rotate angle
    servo1.target = 90
    servo2.target = 0
    servo3.target = 0

    # create two threads to control the servo and read the angle
    t1 = threading.Thread(target=thread_rotating)
    t2 = threading.Thread(target=thread_nodding)
    t1.start()
    t2.start()

    """
    the process of the control other parts and set the servo's angle
    """
    # the function is defined in main file
    # control_servo(servo1, 90, 90)
    while True:
        # set the servo's vertical angle
        head_angle = int(input("please input the head angle: "))
        # only set the servo1's angle, and remain the rotate angle unchanged
        # control_servo(servo1, head_angle, rotate_angle)
        # set the servo's rotate angle
        rotate_angle = int(input("please input the rotate angle: "))
        # only set the servo2 and servo3's angle, and remain the servo1's angle unchanged
        # control_servo(servo1, servo1.angle, rotate_angle)
