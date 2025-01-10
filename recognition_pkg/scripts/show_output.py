import torch
from torchvision import models
from torchvision.models.segmentation import DeepLabV3_ResNet50_Weights

# デバイスの設定
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# モデルの準備
model = models.segmentation.deeplabv3_resnet50(pretrained=False, num_classes=21)  # num_classesを学習時と一致させる
model.load_state_dict(torch.load("segmentation_model.pth", map_location=device))  # 重みをロード
model = model.to(device)
model.eval()  # 評価モードに設定

from torchvision import transforms
from PIL import Image

# 入力画像の前処理
image_transform = transforms.Compose([
    transforms.Resize((256, 256)),  # 学習時のサイズに合わせる
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
])

def main(image_path):
# 画像を読み込んで前処理
# image_path = "dataset/images/image_1.jpg"  # 推論したい画像のパス
    input_image = Image.open(image_path).convert("RGB")
    input_tensor = image_transform(input_image).unsqueeze(0).to(device)  # バッチ次元を追加

    # 推論
    with torch.no_grad():
        output = model(input_tensor)["out"]
        predicted_mask = torch.argmax(output, dim=1).squeeze(0).cpu().numpy()


    import matplotlib.pyplot as plt
    import numpy as np

    # カラーマップを定義（例としてランダムに色を割り当て）
    colormap = np.random.randint(0, 255, size=(21, 3))  # クラスごとの色を生成

    def apply_colormap(mask, colormap):
        """
        クラスインデックスをカラーマップに変換
        """
        color_mask = np.zeros((mask.shape[0], mask.shape[1], 3), dtype=np.uint8)
        for class_idx in range(len(colormap)):
            color_mask[mask == class_idx] = colormap[class_idx]
        return color_mask

    # カラーマップを適用
    colored_mask = apply_colormap(predicted_mask, colormap)

    # 入力画像と予測マスクを並べて表示
    plt.figure(figsize=(12, 6))

    # 入力画像
    plt.subplot(1, 2, 1)
    plt.imshow(input_image)
    plt.title("Input Image")
    plt.axis("off")

    # 予測マスク
    plt.subplot(1, 2, 2)
    plt.imshow(colored_mask)
    plt.title("Predicted Segmentation Mask")
    plt.axis("off")

    plt.show()

import os
import sys
import glob
if __name__ == '__main__':
    images = glob.glob("dataset/images/*.jpg")
    for image in images:
        main(image)
