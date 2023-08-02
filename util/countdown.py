#!/usr/local/bin/python3
# -*- coding: utf-8 -*-

"""
@File    : countdown.py
@Author  : Gyanano
@Time    : 2023/7/18 10:19
"""

import os
import time

import cv2

from .get_map import show_lcd

path = os.path.join(os.getcwd(), "util/numbers")
# print(path)


def countdown(n: int):
    if n < 1:
        n = 1
    elif n > 10:
        n = 10
    for i in range(n):
        img_path = os.path.join(path, str(n - i) + ".jpg")
        # print("path: ", os.path.join(path, str(n - i) + ".jpg"))
        show_lcd(cv2.imread(img_path))
        time.sleep(1)

