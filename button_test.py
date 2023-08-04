import raspberry_king

if __name__ == '__main__':
    six_key = raspberry_king.SixKeyInput()
    while True:
        six_key.read_input()
        print(six_key.pin_pressed)
