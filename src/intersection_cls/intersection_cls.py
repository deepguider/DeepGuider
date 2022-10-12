# import src.intersection_cls
import cv2
import torch
from torchvision import transforms
from lib_intersection_cls.initialize_network import initialize_network
from load_cv2_yaml import load_cv2_yaml

from lib_intersection_cls.modules.resnet_network_3cam import ResNetDropblock, ResNetDropblockOneInput
from lib_intersection_cls.convert_ricohtheta_to_3camera import RicohthetaTo3CameraConverter

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

        self.dg_ros_yml = load_cv2_yaml("dg_ros.yml")
        self.parsing_dg_ros_yml()

        if self.enable_360cam_crop == 0: # if image is coming from web cam
            print('intersection using 1 camera')
            self.init_1camera()
        else:
            if self.intersection_cam != 3:
                print('intersection using 1 camera')
                self.init_1camera()
            else:
                print('intersection using 3 camera')
                self.init_3camera()

            
        return True

    def init_1camera(self):

        # initialize network
        network_type = 'densenet121'
        self.network, _ = initialize_network(network_type, output_class=2)

        # load network
        resume_load = torch.load('./data_intersection_cls/weight_1camera.pth')

        self.network.load_state_dict(resume_load['model_state_dict'])

        self.network.eval()

        self.softmax = nn.Softmax(dim=1)

        # define data transforms
        self.data_transforms = transforms.Compose([transforms.Resize(230),
                                                   transforms.CenterCrop(224),
                                                   transforms.ToTensor(),
                                                   transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
                                                   ])

    def init_3camera(self):
        # initialize network
        network_type = 'resnet18'
        self.network_selection = 0  # 0: feature fusion, 1: late fusion

        if self.network_selection == 0:        
            self.network = ResNetDropblock(resnet_type=18, from_scratch=True)
        else:
            self.network = ResNetDropblockOneInput(resnet_type=18, from_scratch=True)

        # load network
        resume_load = torch.load('./data_intersection_cls/weight_3camera.pth')
        self.network.load_state_dict(resume_load['model_state_dict'])

        self.network.eval()

        self.softmax = nn.Softmax(dim=1)

        # define data transforms
        self.data_transforms = transforms.Compose([transforms.Resize((224, 224)),
                                                   transforms.ToTensor()  # 0 to 1
                                                  ])


    def parsing_dg_ros_yml(self):
        ## You can access dg_ros.yml directly with follwing yml API
        self.intersection_cam = self.dg_ros_yml.read("intersection_cam")
        self.enable_360cam_crop = self.dg_ros_yml.read("enable_360cam_crop")
        #self.intersection_cam = 3
        #self.enable_360cam_crop = 1
        
    def run_1camera(self):
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

    def run_3camera(self):
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
            if self.network_selection == 0:
                outputs = self.network(inputs_left, inputs_center, inputs_right)
                outputs = self.softmax(outputs)

                conf, preds = torch.max(outputs, 1)

                self.cls = preds[0].cpu().detach().item()
                self.prob = conf[0].cpu().detach().item()
            else:
                combined_inputs = torch.cat([inputs_left, inputs_center, inputs_right], 0)
                outputs = self.network(combined_inputs)
                outputs = self.softmax(outputs)
                left_outputs = outputs[0]
                center_outputs = outputs[1]
                right_outputs = outputs[2] 
                left_conf, left_preds = torch.max(left_outputs, 0)
                center_conf, center_preds = torch.max(center_outputs, 0)
                right_conf, right_preds = torch.max(right_outputs, 0)

                num_roads = left_preds.cpu().detach().item() + center_preds.cpu().detach().item() + right_preds.cpu().detach().item() 
                mean_conf = (left_conf.cpu().detach().item() + center_conf.cpu().detach().item() + right_conf.cpu().detach().item()) / 3

                self.cls = 1 if num_roads >= 2 else 0
                self.prob = mean_conf


    ##### Process one frame
    def apply(self, image, timestamp):
        #print('intersection_cam: ', str(self.intersection_cam))
        #print('360camcrop: ', str(self.enable_360cam_crop))
        #print("intersection image size", image.shape)
        self.image = image


        if self.enable_360cam_crop == 0: # if image is coming from web cam
            self.run_1camera()
        else:
            ##### size of one view        
            width = self.image.shape[1] // 3
            height = self.image.shape[0]
            channel = self.image.shape[2]

            ##### Separate 3camera
            self.imageleft = image[:, 0:width, :]
            self.imagecenter = image[:, width:width*2, :]  
            self.imageright =  image[:, width*2:width*3, :]
            
            if self.intersection_cam != 3:
                self.run_1camera()
            else:
                self.run_3camera()


        #cv2.imshow('image', image) 
        #cv2.waitKey(1)

       

        ##### Results #####
        return self.cls, self.prob


if __name__ == "__main__":
    # read omni frame
    nonintersection_img360 = cv2.imread("data_intersection_cls/nonintersection_omni.jpg")
    intersection_img360 = cv2.imread("data_intersection_cls/intersection_omni.jpg")

    # initialize classifier
    classifier = IntersectionClassifier()
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

