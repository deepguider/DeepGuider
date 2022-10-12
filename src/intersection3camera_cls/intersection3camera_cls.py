# import src.intersection_cls
import cv2
import torch
from torchvision import transforms
from lib_intersection3camera_cls.resnet_network import ResNetDropblock
from lib_intersection3camera_cls.convert_ricohtheta_to_3camera import RicohthetaTo3CameraConverter
from PIL import Image
import torch.nn as nn
import time
import os
import glob

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
        resume_load = torch.load('./data_intersection3camera_cls/weight.pth')
        self.network.load_state_dict(resume_load['model_state_dict'])

        self.network.eval()

        self.softmax = nn.Softmax(dim=1)

        # define data transforms
        self.data_transforms = transforms.Compose([transforms.Resize((224, 224)),
                                                   transforms.ToTensor()  # 0 to 1
                                                  ])

        return True

    ##### Process one frame
    def apply(self, image, timestamp):
        self.image = image # image is concatenated left, center, right view

        ##### size of one view        
        width = self.image.shape[1] // 3
        height = self.image.shape[0]
        channel = self.image.shape[2]

        ##### Separate 3camera
        self.imageleft = image[:, 0:width, :]
        self.imagecenter = image[:, width:width*2, :]  
        self.imageright =  image[:, width*2:width*3, :]      

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

        #combined_image = cv2.hconcat([self.imageleft, self.imagecenter, self.imageright])
        #text = 'class {:d}, confidence {:.2f}'.format(self.cls, self.prob) 
        #combined_image = cv2.putText(combined_image, text, (50,80), cv2.FONT_HERSHEY_SIMPLEX, 3, (0, 0, 0), 3)
        #if self.cls == 1:
        #    print(combined_image.shape)
        #    combined_image = cv2.rectangle(combined_image, (0, 0), (3840, 960), (0, 255, 0), 20)
        #cv2.imwrite("/home/dg/Documents/combined3camera_image_withconfidence/{:06d}.jpg".format(timestamp), combined_image)

        ##### Results #####
        return self.cls, self.prob


if __name__ == "__main__":
    # read omni frame
    nonintersection_img360 = cv2.imread("data_intersection3camera_cls/nonintersection_omni.jpg")
    intersection_img360 = cv2.imread("data_intersection3camera_cls/intersection_omni.jpg")

    # initialize classifier
    classifier = Intersection3CameraClassifier()
    classifier.initialize()

    # Convert omni to concatenated 3 camera
    ricohto3cameraconverter = RicohthetaTo3CameraConverter()

    imageleft, imagecenter, imageright = ricohto3cameraconverter.convert_ricohtheta_to_3camera(nonintersection_img360)
    nonintersection_img = cv2.hconcat([imageleft, imagecenter, imageright])

    imageleft, imagecenter, imageright = ricohto3cameraconverter.convert_ricohtheta_to_3camera(intersection_img360)
    intersection_img = cv2.hconcat([imageleft, imagecenter, imageright])

    total_time = 0

    # run classifier
    tic = time.perf_counter()
    cls, prob = classifier.apply(nonintersection_img, 1)
    toc = time.perf_counter()
    print(f'processing time: {toc - tic:0.4f} seconds')
    print('nonintersection_img demo: class ', cls, ' confidence ', prob)

    tic = time.perf_counter()
    cls, prob = classifier.apply(intersection_img, 1)
    toc = time.perf_counter()
    print(f'processing time: {toc - tic:0.4f} seconds')
    print('intersection_img demo: class ', cls, ' confidence ', prob)

    ########## For test with extracted omni camera frames
    #test_select = 1  # select the number here (0: etri to 119, 1: coex indoor, 2: 119 to etri)

    #datasetfolder_list = ['/home/dg/Downloads/2022-08-08-13-38-04_etri_to_119/omni_image/', '/home/dg/Downloads/220628_COEX_Indoor_workshop/rosbag/_2022-06-28-17-03-29_pathA1_back/omni_image/', '/home/dg/Downloads/2022-08-08-14-17-06_119_to_etri/omni_image/']
    #num_frames_list = [3396, 777, 3682]

    ## create result folder is not exist
    #if not os.path.exists('/home/dg/Documents/combined3camera_image_withconfidence'):
    #    os.makedirs('/home/dg/Documents/combined3camera_image_withconfidence')
    ## remove results of previous run
    #files = glob.glob('/home/dg/Documents/combined3camera_image_withconfidence/*')
    #for f in files:
    #    os.remove(f)

    #ricohto3cameraconverter = RicohthetaTo3CameraConverter()  

    #for idx in range(num_frames_list[test_select]):
    #    filename = datasetfolder_list[test_select] + "{:06d}.jpg".format(idx)
    #    img360 = cv2.imread(filename)
    #    imageleft, imagecenter, imageright = ricohto3cameraconverter.convert_ricohtheta_to_3camera(img360)
    #    img = cv2.hconcat([imageleft, imagecenter, imageright])
        
    #    cls, prob = classifier.apply(img, idx)
    #    print('frame ', idx, ': class: ', cls, ' confidence ', prob)
