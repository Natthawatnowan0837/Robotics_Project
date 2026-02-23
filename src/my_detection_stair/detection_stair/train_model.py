import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
from torch.utils.data import DataLoader, Dataset
import numpy as np
import os
import glob
from sklearn.model_selection import train_test_split
from sklearn.metrics import confusion_matrix, classification_report
import matplotlib.pyplot as plt
import seaborn as sns

# ==========================================
# ส่วนที่ 1: การเตรียมข้อมูล (Data Loader)
# ==========================================
class StairDataset(Dataset):
    def __init__(self, file_paths, labels):
        self.file_paths = file_paths
        self.labels = labels

    def __len__(self):
        return len(self.file_paths)

    def __getitem__(self, idx):
        point_cloud = np.load(self.file_paths[idx])
        # Transpose จาก (1000, 3) เป็น (3, 1000) สำหรับ Conv1d
        point_cloud = point_cloud.transpose(1, 0)
        return torch.from_numpy(point_cloud).float(), self.labels[idx]

def load_dataset(root_dir="dataset_stair"):
    classes = {"downstairs": 0, "upstairs": 1, "others": 2}
    files = []
    labels = []

    for class_name, label in classes.items():
        class_path = os.path.join(root_dir, class_name)
        if not os.path.exists(class_path):
            print(f"Warning: Folder {class_path} not found!")
            continue
            
        class_files = glob.glob(os.path.join(class_path, "*.npy"))
        files.extend(class_files)
        labels.extend([label] * len(class_files))
        print(f"Loaded {len(class_files)} files for class '{class_name}'")

    return files, labels

# ==========================================
# ส่วนที่ 2: โครงสร้างโมเดล (PointNet)
# ==========================================
class TNet(nn.Module):
    def __init__(self, k=3):
        super(TNet, self).__init__()
        self.k = k
        self.conv1 = nn.Conv1d(k, 64, 1)
        self.conv2 = nn.Conv1d(64, 128, 1)
        self.conv3 = nn.Conv1d(128, 1024, 1)
        self.fc1 = nn.Linear(1024, 512)
        self.fc2 = nn.Linear(512, 256)
        self.fc3 = nn.Linear(256, k*k)

        self.bn1 = nn.BatchNorm1d(64)
        self.bn2 = nn.BatchNorm1d(128)
        self.bn3 = nn.BatchNorm1d(1024)
        self.bn4 = nn.BatchNorm1d(512)
        self.bn5 = nn.BatchNorm1d(256)

    def forward(self, x):
        batch_size = x.size(0)
        x = F.relu(self.bn1(self.conv1(x)))
        x = F.relu(self.bn2(self.conv2(x)))
        x = F.relu(self.bn3(self.conv3(x)))
        x = torch.max(x, 2, keepdim=True)[0]
        x = x.view(-1, 1024)

        x = F.relu(self.bn4(self.fc1(x)))
        x = F.relu(self.bn5(self.fc2(x)))
        x = self.fc3(x)

        iden = torch.eye(self.k, requires_grad=True).repeat(batch_size, 1, 1).to(x.device)
        x = x.view(-1, self.k, self.k) + iden
        return x

class PointNet(nn.Module):
    def __init__(self, classes=3):
        super(PointNet, self).__init__()
        self.stn = TNet(k=3)
        self.conv1 = nn.Conv1d(3, 64, 1)
        self.bn1 = nn.BatchNorm1d(64)
        self.fstn = TNet(k=64)
        
        self.conv2 = nn.Conv1d(64, 64, 1)
        self.bn2 = nn.BatchNorm1d(64)
        self.conv3 = nn.Conv1d(64, 64, 1)
        self.bn3 = nn.BatchNorm1d(64)
        self.conv4 = nn.Conv1d(64, 1024, 1)
        self.bn4 = nn.BatchNorm1d(1024)

        self.fc1 = nn.Linear(1024, 512)
        self.bn5 = nn.BatchNorm1d(512)
        self.fc2 = nn.Linear(512, 256)
        self.bn6 = nn.BatchNorm1d(256)
        self.fc3 = nn.Linear(256, classes)
        self.dropout = nn.Dropout(p=0.3)

    def forward(self, x):
        B, D, N = x.size()
        
        trans = self.stn(x)
        x = x.transpose(2, 1)
        x = torch.bmm(x, trans)
        x = x.transpose(2, 1)
        
        x = F.relu(self.bn1(self.conv1(x)))
        
        trans_feat = self.fstn(x)
        x = x.transpose(2, 1)
        x = torch.bmm(x, trans_feat)
        x = x.transpose(2, 1)

        x = F.relu(self.bn2(self.conv2(x)))
        x = F.relu(self.bn3(self.conv3(x)))
        x = self.bn4(self.conv4(x))
        
        x = torch.max(x, 2, keepdim=True)[0]
        x = x.view(-1, 1024)

        x = F.relu(self.bn5(self.fc1(x)))
        x = F.relu(self.bn6(self.fc2(x)))
        x = self.dropout(x)
        x = self.fc3(x)

        return F.log_softmax(x, dim=1), trans_feat

# ==========================================
# ส่วนที่ 4: การประเมินผล
# ==========================================
def plot_confusion_matrix(model, val_loader, device, class_names):
    all_preds = []
    all_labels = []
    
    model.eval()
    with torch.no_grad():
        for data, target in val_loader:
            data, target = data.to(device), target.to(device)
            output, _ = model(data)
            pred = output.argmax(dim=1)
            all_preds.extend(pred.cpu().numpy())
            all_labels.extend(target.cpu().numpy())

    cm = confusion_matrix(all_labels, all_preds)
    plt.figure(figsize=(8, 6))
    sns.heatmap(cm, annot=True, fmt='d', cmap='Blues',
                xticklabels=class_names, yticklabels=class_names)
    plt.xlabel('Predicted')
    plt.ylabel('Actual')
    plt.title('Confusion Matrix')
    plt.show()

    print("\nClassification Report:")
    print(classification_report(all_labels, all_preds, target_names=class_names))

# ==========================================
# Main execution
# ==========================================
def main():
    # Configuration
    BATCH_SIZE = 32
    EPOCHS = 50
    LR = 0.0001
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    
    # 1. Prepare Data
    files, labels = load_dataset()
    if not files:
        print("Error: No data found.")
    else:
        X_train, X_val, y_train, y_val = train_test_split(files, labels, test_size=0.3, random_state=42)
        
        train_loader = DataLoader(StairDataset(X_train, y_train), batch_size=BATCH_SIZE, shuffle=True)
        val_loader = DataLoader(StairDataset(X_val, y_val), batch_size=BATCH_SIZE, shuffle=False)

        # 2. Setup Model
        model = PointNet(classes=3).to(device)
        optimizer = optim.Adam(model.parameters(), lr=LR)
        criterion = nn.NLLLoss()
        best_acc = 0

        # 3. Training Loop
        for epoch in range(EPOCHS):
            model.train()
            total_loss, correct, total = 0, 0, 0
            for data, target in train_loader:
                data, target = data.to(device), target.to(device)
                optimizer.zero_grad()
                output, _ = model(data)
                loss = criterion(output, target.long())
                loss.backward()
                optimizer.step()
                
                total_loss += loss.item()
                correct += (output.argmax(1) == target).sum().item()
                total += target.size(0)

            # Validation
            model.eval()
            val_correct = 0
            with torch.no_grad():
                for data, target in val_loader:
                    data, target = data.to(device), target.to(device)
                    output, _ = model(data)
                    val_correct += (output.argmax(1) == target).sum().item()
            
            val_acc = 100. * val_correct / len(X_val)
            print(f"Epoch {epoch+1:02d} | Loss: {total_loss/len(train_loader):.4f} | Val Acc: {val_acc:.2f}%")

            if val_acc > best_acc:
                best_acc = val_acc
                torch.save(model.state_dict(), "pointnet_stairs4.pth")

        # 4. Final Evaluation
        print(f"\nTraining Complete. Best Accuracy: {best_acc:.2f}%")
        model.load_state_dict(torch.load("pointnet_stairs4.pth"))
        plot_confusion_matrix(model, val_loader, device, ["downstairs", "upstairs", "others"])

if __name__ == '__main__':
    main()