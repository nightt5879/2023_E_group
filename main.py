import raspberry_king


# 全局变量
key = raspberry_king.KeyInput()

if __name__ == '__main__':
    while True:
        print(key.read_input())
