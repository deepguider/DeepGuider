# import src.intersection_cls
import cv2
import torch
from torchvision import transforms
from lib_intersection_cls.initialize_network import initialize_network
from PIL import Image
import torch.nn as nn
import time

class IntersectionClassifier:
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
        self.network, _ = initialize_network(network_type, output_class=2)

        # load network
        resume_load = torch.load('./data_intersection_cls/v0.14_binary_resnet18_dropprob0.75_dropblocksize3_3_best.pth')
        self.network.load_state_dict(resume_load['model_state_dict'])

        self.network.eval()

        self.softmax = nn.Softmax(dim=1)

        # define data transforms
        self.data_transforms = transforms.Compose([transforms.Resize(224),
                                                   transforms.CenterCrop(224),
                                                   transforms.ToTensor(),
                                                   transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
                                                   ])

        return True

    ##### Process one frame
    def apply(self, image):
        self.image = image

        ##### Process Input #####
        # convert cv::Mat image to PIL image (https://stackoverflow.com/a/43234001)
        img = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
        im_pil = Image.fromarray(img)

        # transform the data
        inputs = self.data_transforms(im_pil)
        inputs = inputs.unsqueeze(0)

        # forward to the network
        with torch.no_grad():
            outputs = self.network(inputs)
            outputs = self.softmax(outputs)

        conf, preds = torch.max(outputs, 1)

        self.cls = preds[0].cpu().detach().item()
        self.prob = conf[0].cpu().detach().item()

        ##### Results #####
        return self.cls, self.prob


if __name__ == "__main__":
    nonintersection_img = cv2.imread("data_intersection_cls/nonintersection_demoimg.jpg")
    intersection_img = cv2.imread("data_intersection_cls/intersection_demoimg.jpg")

    # initialize classifier
    classifier = IntersectionClassifier()
    classifier.initialize()

    # run classifier
    tic = time.perf_counter()
    cls, prob = classifier.apply(nonintersection_img)
    toc = time.perf_counter()
    print(f'processing time: {toc - tic:0.4f} seconds')
    print('nonintersection_img demo: class ', cls, ' confidence ', prob)

    tic = time.perf_counter()
    cls, prob = classifier.apply(intersection_img)
    toc = time.perf_counter()
    print(f'processing time: {toc - tic:0.4f} seconds')
    print('intersection_img demo: class ', cls, ' confidence ', prob)
