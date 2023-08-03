import raspberry_king
import time

if __name__ == "__main__":
    servo_control = raspberry_king.ServoSTM32()
    servo_control.control_servo(17, 19)
    time.sleep(3)
    servo_control.control_servo(0, 0)
    time.sleep(3)
    servo_control.control_servo(-17, -19)
    time.sleep(3)
    servo_control.control_servo(0, 0)
    print("test done")
    while True:
        pass
        # servo_control.control_servo(-20, 0)
        # time.sleep(1)
    print("test done")
