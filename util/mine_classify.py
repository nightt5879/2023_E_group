import numpy as np
import paddlelite.lite
import os
import cv2


class MinesClassifier:
    def __init__(self, paddle_model="./model/MobileNet_small.nb"):
        
        config = paddlelite.lite.MobileConfig()
        config.set_model_from_file(paddle_model)
        self.predictor = paddlelite.lite.create_paddle_predictor(config)
        self.input_tensor0 = self.predictor.get_input(0)
        result_list = []
        true_result = 0
        Dict = {
            0:"blue_false",
            1:"blue_true",
            2:"red_false",
            3:"red_true"
            }

    def process_image(self,image_data, shape=224):
        """
        对图片进行预处理
        """
        img_mean = [0.485, 0.456, 0.406]
        img_std = [0.229, 0.224, 0.225]
        image_data = cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)
        image_data = cv2.resize(image_data, (shape, shape))
        image_data = image_data.transpose((2, 0, 1)) / 255.0

        image_data = (image_data - np.array(img_mean).reshape(
            (3, 1, 1))) / np.array(img_std).reshape((3, 1, 1))
        image_data = image_data.reshape([1, 3, shape, shape]).astype('float32')
        return image_data
    def recognize_img(self,img):
        image_data = self.process_image(img)
        self.input_tensor0.from_numpy(image_data)
        self.predictor.run()
        output_tensor = self.predictor.get_output(0)
        output_tensor = output_tensor.numpy().squeeze()
        return np.argmax(output_tensor),output_tensor



if __name__ == "__main__":
    mine_classifier = MinesClassifier(paddle_model="../model/MobileNetV2_728.nb")
    #for class_num in range(4):
    # img = cv2.imread(f"../img/测试图片/blue_true/0.png")
    cam = cv2.VideoCapture(-1)
    cam.set(3, 320)
    cam.set(4, 240)
    while True:
        success, img = cam.read()
        if success:
            result = mine_classifier.recognize_img(img)
            cv2.imshow("img", img)
            print(result)
        cv2.waitKey(1)




