#关闭警告  值得一提的是cv2没更新会有[WRN:0]无法关闭的报错。。。
import warnings
warnings.filterwarnings("ignore")

import sys
import cv2
import numpy as np

#判断在Win还是在Linux  如果是Win就1ms延时，Linux 40ms延时
system_platform = sys.platform
if 'win' in system_platform:
    time = 1
else:
    time = 40
try:
    import Tkinter as tk
except:
    import tkinter as tk

result = None
class Test():
    def __init__(self):
        result = ''
        self.root = tk.Tk()
        self.root.title('逐帧掩膜')
        self.root.geometry('500x500')
        button = tk.Button(self.root,
                           text='运行',
                           height=4,
                           width=20,
                           command=self.quit)
        button.pack(side='bottom')

        self.textExample = tk.Text(self.root, height=3)  # 创建文本输入框
        self.textExample.pack()  # 把Text放在window上面，显示Text这个控件

        test = """
    这是一个逐帧掩膜的调参分析的程序
    请在上方空白区域输入视频的 绝对路径
    之后点击运行（未知问题较为卡顿）
    操作方法:(按键可长按）
            Q:后退5帧 W:后退1帧 
            E:前进1帧 R:前进5帧
            ESC:退出程序
丝滑版本请打开同目录下.py更改77行运行
    YYJ -22/5/4
                """

        lb = tk.Label(self.root, text=test,  # 设置文本内容
                      width=30,  # 设置label的宽度：30
                      height=10,  # 设置label的高度：10
                      justify='left',  # 设置文本对齐方式：左对齐
                      anchor='nw',  # 设置文本在label的方位：西北方位
                      font=('微软雅黑', 18),  # 设置字体：微软雅黑，字号：18
                      fg='white',  # 设置前景色：白色
                      bg='grey',  # 设置背景色：灰色
                      padx=20,  # 设置x方向内边距：20
                      pady=10)
        lb.pack()

        self.root.mainloop()

    def quit(self):
        global result
        result = self.textExample.get("1.0", "end")  # 获取文本输入框的内容
        self.root.destroy()




# app = Test()
# #print(app.result)
#
#
# del app  #删除实例化对象
# a = result   #'D:\\robot\job\FindBall\output.avi'
cap = cv2.VideoCapture('output.avi')  #读取视频文件


Str = 'Lower: 0 0 0 \n Upper: 0 0 0'  #用于保存阈值的字符串

def Onchange(x):  #滑块的回调函数
    global Str
    H_Max = cv2.getTrackbarPos("H-Max", "Trackbar")
    H_Min = cv2.getTrackbarPos("H-Min", "Trackbar")
    S_Max = cv2.getTrackbarPos("S-Max", "Trackbar")
    S_Min = cv2.getTrackbarPos("S-Min", "Trackbar")
    V_Max = cv2.getTrackbarPos("V-Max", "Trackbar")
    V_Min = cv2.getTrackbarPos("V-Min", "Trackbar")
    """
    将这六个值保存为字符串后续放入TXT
    格式为：
        Lower: H_Min S_Min V_Min
        Upper: H_Max S_Max V_Max
    方便后续的直接字符串转list
    """
    Str = 'Lower: ' + str(H_Min) + ' ' + str(S_Min) + ' ' + str(V_Min) + '\n' \
        + 'Upper: ' + str(H_Max) + ' ' + str(S_Max) + ' ' + str(V_Max)


#cap = cv2.VideoCapture(0)  #设置摄像头为默认 0 号

#创建一个滑块窗口
cv2.namedWindow('Trackbar',cv2.WINDOW_FREERATIO)
cv2.createTrackbar("H-Min", "Trackbar", 0, 180, Onchange)
cv2.createTrackbar("H-Max", "Trackbar", 0, 180, Onchange)
cv2.createTrackbar("S-Min", "Trackbar", 0, 255, Onchange)
cv2.createTrackbar("S-Max", "Trackbar", 0, 255, Onchange)
cv2.createTrackbar("V-Min", "Trackbar", 0, 255, Onchange)
cv2.createTrackbar("V-Max", "Trackbar", 0, 255, Onchange)

font = cv2.FONT_HERSHEY_COMPLEX  #导入cv2字体
cv2.namedWindow('img',cv2.WINDOW_FREERATIO)  #创建新的img窗口用于显示摄像头图像

i = 1

while True:
    cap.set(cv2.CAP_PROP_POS_FRAMES, i)  # 设置要获取的帧号
    _, frame = cap.read()  #读取摄像头

    list = Str.split()  #将字符串切片（以空格为间隔保存为list）
    Lower = np.array([int(list[1]), int(list[2]), int(list[3])])  #Lower 的值（包含强转 int， list变Numpy）
    Upper = np.array([int(list[5]), int(list[6]), int(list[7])])  #Upper 的值（包含强转 int， list变Numpy）
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, Lower, Upper)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel)   #HSV掩膜后的结果
    #下面是寻找轮廓
    contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
        x = approx.ravel()[0]
        y = approx.ravel()[1]

        if area > 400:

            cv2.drawContours(frame, [approx], 0, (0, 0, 0), 5)

            if len(approx) == 3:
                cv2.putText(frame, "Triangle", (x, y), font, 1, (0, 0, 0))

            elif len(approx) == 4:

                cv2.putText(frame, "Rectangle", (x, y), font, 1, (0, 0, 0))
            elif 10 < len(approx) < 20:
                cv2.putText(frame, "Circle", (x, y), font, 1, (0, 0, 0))

    cv2.imshow('HSV', mask)  #显示掩膜
    cv2.imshow('img',frame)  #显示原图像

    key = cv2.waitKey(time)
    """
    下面是快进快退切换帧数，er分别快进1，5帧  qw分别退后1，5帧
    if 判断是防止超出了这个视频的长度下限与上限。
    """
    if key == ord('e'):
        if i <cap.get(7) - 1:
            i += 1
        else:
            i = i
        # print(i)
        # print(cap.get(7))
    if key == ord('r'):
        if i < cap.get(7) - 5:
            i += 5
        else:
            i = i
        # print(i)
        # print(cap.get(7))
    if key == ord('w'):
        if i >0:
            i -= 1
    if key == ord('q'):
        if i >0:
            i -= 5

    if key == 27:
        break


cv2.destroyAllWindows()

