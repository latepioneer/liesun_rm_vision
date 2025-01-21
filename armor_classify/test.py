import tensorflow as tf
import numpy as np
import os
import cv2
from sklearn.model_selection import train_test_split
import tf2onnx

# 数据集路径
dataset_path = os.path.join(os.path.dirname(__file__), '..', 'imgs')

# 加载图片和标签
def load_images_from_folder(folder_path):
    images = []
    labels = []
    for label in os.listdir(folder_path):
        label_path = os.path.join(folder_path, label)
        if not os.path.isdir(label_path):
            continue
        for img_file in os.listdir(label_path):
            img_path = os.path.join(label_path, img_file)
            img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)  # 使用 OpenCV 读取灰度图
            img = cv2.resize(img, (28, 20))  # 调整大小 (宽28, 高20)
            img = img.astype('float32') / 255.0  # 归一化
            images.append(img.flatten())  # 展平图像为一维数组
            labels.append(int(label))
    return np.array(images), np.array(labels)

# 使用类别 0~7
valid_labels = list(range(7))
images, labels = load_images_from_folder(dataset_path)

# One-hot 编码标签
labels = tf.keras.utils.to_categorical(labels, len(valid_labels) + 1)  # 加一个额外类别表示未知

# 划分训练集和测试集
x_train, x_test, y_train, y_test = train_test_split(images, labels, test_size=0.1, random_state=42)

# 构建模型
model = tf.keras.Sequential([
    tf.keras.layers.Dense(512, activation='relu', input_shape=(20 * 28,)),
    tf.keras.layers.Dropout(0.2),
    tf.keras.layers.Dense(len(valid_labels) + 1, activation='softmax')  # 输出节点数量为 8 + 1（未知类别）
])

# 编译模型
model.compile(optimizer='adam',
              loss='categorical_crossentropy',
              metrics=['accuracy'])

# 训练模型
history = model.fit(x_train, y_train,
                    epochs=10,
                    batch_size=128,
                    validation_data=(x_test, y_test))

# 评估模型
test_loss, test_acc = model.evaluate(x_test, y_test)
print('Test accuracy:', test_acc)

# 保存 TensorFlow 模型
model.save('saved_model/my_model')

# 转换为 ONNX 格式
onnx_model_path = "saved_model/my_model.onnx"
spec = (tf.TensorSpec((None, 20 * 28), tf.float32, name="img"),)
model_proto, _ = tf2onnx.convert.from_keras(model, input_signature=spec, opset=13)
with open(onnx_model_path, "wb") as f:
    f.write(model_proto.SerializeToString())
