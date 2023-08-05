import raspberry_king
import time
key = raspberry_king.KeyInput(pin_set=21)
servo_control = raspberry_king.ServoSTM32()
six_key = raspberry_king.SixKeyInput()  # 用于选择功能的面包板按键，总计6个对应关系如下：
if __name__ == "__main__":
    move_x = 0
    move_y = 0
    servo_control.control_servo(0,10)
    while True:
        six_key.read_input()
        # if six_key.pin_pressed[0]:  # 1号按键已经被按下
        #     six_key.flash_all_key()  # 清空所有的值
        #     move_x += 5
        # elif six_key.pin_pressed[1]:  # 2号按键已经被按下
        #     six_key.flash_all_key()  # 清空所有的值
        #     move_x -= 5
        # elif six_key.pin_pressed[2]:  # 3号按键已经被按下
        #     six_key.flash_all_key()  # 清空所有的值
        #     move_y -= 5
        # elif six_key.pin_pressed[3]:  # 4号按键已经被按下
        #     six_key.flash_all_key()  # 清空所有的值
        #     move_y += 5
        # servo_control.control_servo(move_y, move_x)
        # print(move_x, move_y)
        # time.sleep(1)

