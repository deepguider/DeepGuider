import os
from PIL import Image
from torch.utils.data import Dataset, DataLoader
import torchvision.transforms as transforms
from ipdb import set_trace as bp

class CustomImageDataset(Dataset):

    def read_data_set(self):
        all_img_files = []
        all_labels = []
        class_names = os.walk(self.data_set_path).__next__()[1]
        class_names.sort()
        if len(class_names) == 0:
            class_names.append('.')
        filename_only = [] 
        filename_with_path =[] 
        #print('Sub-Dirname used by class labels in train/test : ', class_names)
        for index, class_name in enumerate(class_names):
            label = index
            img_dir = os.path.join(self.data_set_path, class_name)
            img_files = os.walk(img_dir).__next__()[2]
            img_files.sort()
            for img_file in img_files:
                img_file = os.path.join(img_dir, img_file)
                try:
                    img = Image.open(img_file)
                except:  # not image file
                    continue
                if img is not None:
                    all_img_files.append(img_file)
                    all_labels.append(label)
        return all_img_files, all_labels, len(all_img_files), len(class_names)

    def __init__(self, data_set_path, transform=None):
        self.data_set_path = data_set_path
        self.image_files_path, self.labels, self.length, self.num_classes = self.read_data_set()
        self.transform_base = transforms.Compose([transforms.ToTensor()])

        if transform == None:
            self.transform = self.transform_base
        else:
            self.transform = transform

    def __getitem__(self, index):
        image = Image.open(self.image_files_path[index])
        image = image.convert("RGB")
        if self.transform is not None:
            image = self.transform(image)

        return {'image': image, 'label': self.labels[index]}

    def __len__(self):
        return self.length
