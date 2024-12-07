import cv2
import numpy as np
import onnxruntime as ort

ort_session = ort.InferenceSession("saved_model/my_model.onnx")

# 使用 OpenCV 处理单张图片
img = cv2.imread('./48.jpg', cv2.IMREAD_GRAYSCALE)  # 使用 OpenCV 读取灰度图
cv2.imshow("1",img);
cv2.waitKey(1000);
img = cv2.resize(img, (28, 20))  # 调整大小 (宽28, 高20)
img = img.astype('float32') / 255.0  # 归一化
sample_image = img.flatten().reshape(1, 20 * 28).astype('float32')  # 展平并调整为模型输入形状

# 进行预测
outputs = ort_session.run(None, {"img": sample_image})
print("ONNX 模型预测结果：", np.argmax(outputs))