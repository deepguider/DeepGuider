# import src.intersection_cls
import cv2
import torch
from torchvision import transforms
from lib_intersection3camera_cls.resnet_network import ResNetDropblock
from lib_intersection3camera_cls.convert_ricohtheta_to_3camera import RicohthetaTo3CameraConverter
from PIL import Image
import torch.nn as nn
import time

class Intersection3CameraClassifier:
    def __init__(self):
        self.cls = -1  # road direction (radian)
        self.prob = -1  # reliablity of the result. 0: fail ~ 1: success

    ##### Time-consuming pre-initialization code here (e.g. network load)
    def initialize(self):
        self.cls = 0
        self.prob = 1.0

        # gpu or cpu
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # initialize network
        network_type = 'resnet18'
        self.network = ResNetDropblock(resnet_type=18, from_scratch=True)

        # load network
        resume_load = torch.load('./data_intersection3camera_cls/outdoor_3camera_bestrecall.pth')
        self.network.load_state_dict(resume_load['model_state_dict'])

        self.network.eval()

        self.softmax = nn.Softmax(dim=1)

        # define data transforms
        self.data_transforms = transforms.Compose([transforms.Resize((224, 224)),
                                                   transforms.ToTensor()  # 0 to 1
                                                  ])

        self.ricohto3cameraconverter = RicohthetaTo3CameraConverter()

        return True

    ##### Process one frame
    def apply(self, image360, timestamp):
        self.image360 = image360

        ##### Convert 360 image to 3 camera #####
        self.imageleft, self.imagecenter, self.imageright = self.ricohto3cameraconverter.convert_ricohtheta_to_3camera(self.image360)


        ##### Process Input #####
        # convert cv::Mat image to PIL image (https://stackoverflow.com/a/43234001)
        img_left = cv2.cvtColor(self.imageleft, cv2.COLOR_BGR2RGB)
        imleft_pil = Image.fromarray(img_left)
        img_center = cv2.cvtColor(self.imagecenter, cv2.COLOR_BGR2RGB)
        imcenter_pil = Image.fromarray(img_center)
        img_right = cv2.cvtColor(self.imageright, cv2.COLOR_BGR2RGB)
        imright_pil = Image.fromarray(img_right)

        # transform the data
        inputs_left = self.data_transforms(imleft_pil)
        inputs_left = inputs_left.unsqueeze(0)
        inputs_center = self.data_transforms(imcenter_pil)
        inputs_center = inputs_center.unsqueeze(0)
        inputs_right = self.data_transforms(imright_pil)
        inputs_right = inputs_right.unsqueeze(0)

        # forward to the network
        with torch.no_grad():
            outputs = self.network(inputs_left, inputs_center, inputs_right)
            outputs = self.softmax(outputs)

        conf, preds = torch.max(outputs, 1)

        self.cls = preds[0].cpu().detach().item()
        self.prob = conf[0].cpu().detach().item()

        ##### Results #####
        return self.cls, self.prob


if __name__ == "__main__":
    nonintersection_img = cv2.imread("data_intersection3camera_cls/nonintersection_omni.jpg")
    intersection_img = cv2.imread("data_intersection3camera_cls/intersection_omni.jpg")

    # initialize classifier
    classifier = Intersection3CameraClassifier()
    classifier.initialize()

    total_time = 0

    # for _ in range(20):
    # run classifier
    tic = time.perf_counter()
    cls, prob = classifier.apply(nonintersection_img, 1)
    toc = time.perf_counter()
    print(f'processing time: {toc - tic:0.4f} seconds')
    print('nonintersection_img demo: class ', cls, ' confidence ', prob)

    # total_time += (toc-tic)

    tic = time.perf_counter()
    cls, prob = classifier.apply(intersection_img, 1)
    toc = time.perf_counter()
    print(f'processing time: {toc - tic:0.4f} seconds')
    print('intersection_img demo: class ', cls, ' confidence ', prob)
