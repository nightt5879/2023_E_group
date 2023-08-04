import raspberry_king
import time
key = raspberry_king.KeyInput(pin_set=21)
servo_control = raspberry_king.ServoSTM32()
if __name__ == "__main__":
    servo_control.control_servo(0, 20)
    while True:
        # print(key.read_input())
        pass
        # servo_control.control_servo(-20, 0)
        # time.sleep(1)

