import RPi.GPIO as GPIO
import time

# 设置BCM引脚编号模式
GPIO.setmode(GPIO.BCM)

# 定义引脚编号
PWM_PIN = 25

# 设置引脚为输出模式
GPIO.setup(PWM_PIN, GPIO.OUT)

# 创建PWM实例，频率为2KHz
pwm = GPIO.PWM(PWM_PIN, 2000)

# 启动PWM，初始占空比为50%
pwm.start(50)
pwm.ChangeDutyCycle(50)
