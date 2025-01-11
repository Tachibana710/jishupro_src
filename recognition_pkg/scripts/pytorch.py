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

# マスク画像の前処理
mask_transform = transforms.Compose([
    transforms.Resize((256, 256)),  # サイズ変更
    transforms.ToTensor()           # テンソル化
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

model = models.segmentation.deeplabv3_resnet50(pretrained=False, num_classes=21)
model = model.to(device)

criterion = nn.CrossEntropyLoss()  # ピクセルごとの分類用
optimizer = optim.Adam(model.parameters(), lr=1e-4)

num_epochs = 30

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


