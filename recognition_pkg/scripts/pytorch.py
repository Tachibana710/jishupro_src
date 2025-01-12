import torch
from torchvision import models
import torch.nn as nn
import torch.optim as optim

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
if torch.cuda.is_available():
    print("GPU is available.")
else:
    print("GPU is not available.")

from torch.utils.data import DataLoader
import os
from PIL import Image
from torch.utils.data import Dataset
import numpy as np

# CustomSegmentationDatasetクラスの定義
class CustomSegmentationDataset(Dataset):
    def __init__(self, image_dir, mask_dir, transform=None, mask_transform=None):
        self.image_dir = image_dir
        self.mask_dir = mask_dir
        self.transform = transform
        self.mask_transform = mask_transform
        # self.image_names = os.listdir(image_dir)  # ディレクトリ内の画像ファイル名リスト

        self.image_names = [
            image_name for image_name in os.listdir(image_dir)
            if os.path.exists(os.path.join(mask_dir, image_name.replace('.jpg', '.png')))
        ]

    def __len__(self):
        return len(self.image_names)

    def __getitem__(self, idx):
        # 対応する画像とマスクのファイルパス
        image_path = os.path.join(self.image_dir, self.image_names[idx])
        mask_path = os.path.join(self.mask_dir, self.image_names[idx].replace('.jpg', '.png'))

        # 画像とマスクを読み込み
        image = Image.open(image_path).convert("RGB")
        mask = Image.open(mask_path)

        # 前処理を適用
        if self.transform:
            image = self.transform(image)
        if self.mask_transform:
            mask = self.mask_transform(mask)

        return image, mask

import torchvision.transforms as transforms

image_transform = transforms.Compose([
    transforms.Resize((256, 256)),  # サイズ変更
    transforms.ToTensor(),         # テンソル化
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])  # 正規化
])

class MaskToTensor(object):
    def __call__(self, mask):
        arr = np.array(mask)
        # class0_num = (arr == 0).sum()
        # class1_num = (arr == 1).sum()
        # class2_num = (arr == 2).sum()
        # class3_num = (arr == 3).sum()
        # class4_num = (arr == 4).sum()
        
        # print(f"Class 0: {class0_num}, Class 1: {class1_num}, Class 2: {class2_num}, Class 3: {class3_num}, Class 4: {class4_num}")
        return torch.as_tensor(arr, dtype=torch.int64)

# マスク画像の前処理
mask_transform = transforms.Compose([
    transforms.Resize((256, 256)),  # サイズ変更
    MaskToTensor()  # テンソル化
])

# データセットの作成
dataset = CustomSegmentationDataset(
    image_dir="dataset/images",
    mask_dir="dataset/masks",
    transform=image_transform,
    mask_transform=mask_transform,
)



# DataLoaderの作成
train_loader = DataLoader(dataset, batch_size=8, shuffle=True, num_workers=4)

test_loader = DataLoader(dataset, batch_size=8, shuffle=False, num_workers=4)

model = models.segmentation.deeplabv3_resnet50(pretrained=False, num_classes=5)
model = model.to(device)

class_counts = [65433, 45, 28, 19, 11]
total_pixels = sum(class_counts)

# 各クラスの重みを逆頻度で計算
# weights = [total_pixels / count if count > 0 else 0 for count in class_counts]
weights = [1.0/20, 1.0/5, 1, 1, 1.0]
weights = torch.tensor(weights, dtype=torch.float32).to(device)

weights = weights / weights.sum()  # 正規化

print("Class Weights:", weights)

criterion = nn.CrossEntropyLoss(weight=weights)  # ピクセルごとの分類用
optimizer = optim.Adam(model.parameters(), lr=1e-4)

num_epochs = 50

for epoch in range(num_epochs):
    model.train()
    running_loss = 0.0

    for images, masks in train_loader:
        images, masks = images.to(device), masks.squeeze(1).long().to(device)  # 次元調整

        # 勾配をリセット
        optimizer.zero_grad()

        # フォワードパス
        outputs = model(images)["out"]

        # 損失計算
        loss = criterion(outputs, masks)

        # バックプロパゲーション
        loss.backward()
        optimizer.step()

        running_loss += loss.item()

    print(f"Epoch [{epoch+1}/{num_epochs}], Loss: {running_loss/len(train_loader):.4f}")

model.eval()
with torch.no_grad():
    for images, masks in test_loader:
        images = images.to(device)
        outputs = model(images)["out"]
        predictions = torch.argmax(outputs, dim=1).cpu().numpy()

torch.save(model.state_dict(), "segmentation_model.pth")


