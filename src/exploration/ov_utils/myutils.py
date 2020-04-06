import numpy as np
import cv2
from PIL import Image
from sklearn.metrics.pairwise import cosine_similarity
import torchvision.transforms as transforms
from torchvision import models
from torch.autograd import Variable
import matplotlib.image as mpimg
import ov_utils.imgproc as imgproc

model = models.resnet18(pretrained=True).cuda()
model.eval()
scaler = transforms.Scale((224, 224))
normalize = transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                 std=[0.229, 0.224, 0.225])
to_tensor = transforms.ToTensor()


def make_mask(bbox, shape):
    x1, y1, x2, y2, x3, y3, x4, y4 = np.reshape(bbox, [-1])
    mask = np.ones(shape)
    a1 = -(y1 - y4) / (x1 - x4+0.0001)
    b1 = -(y1 - (y1 - y4) / (x1 - x4+0.0001) * x1)
    a2 = -(y1 - y2) / (x1 - x2+0.0001)
    b2 = -(y1 - (y1 - y2) / (x1 - x2+0.0001) * x1)
    a3 = -(y2 - y3) / (x2 - x3+0.0001)
    b3 = -(y2 - (y2 - y3) / (x2 - x3+0.0001) * x2)
    a4 = -(y3 - y4) / (x3 - x4+0.0001)
    b4 = -(y3 - (y3 - y4) / (x3 - x4+0.0001) * x3)
    for i in range(shape[1]):
        for j in range(shape[0]):
            if a1 > 0:
                if (j < -(a1 * i + b1)):
                    mask[j, i] = 0
            else:
                if (j > -(a1 * i + b1)):
                    mask[j, i] = 0
            if (j < -(a2 * i + b2)):
                mask[j, i] = 0
            if a3 > 0:
                if (j > -(a3 * i + b3)):
                    mask[j, i] = 0
            else:
                if (j < -(a3 * i + b3)):
                    mask[j, i] = 0
            if (j > -(a4 * i + b4)):
                mask[j, i] = 0
    return mask


def template_matching(image, bbox, templates, main_template):
    template_matched = False
    sizes = np.zeros((len(templates), len(bbox)))
    for i, template in enumerate(templates):
        template = cv2.resize(template, (224, 224))
        template = cv2.cvtColor(template, cv2.COLOR_BGR2RGB)
        template = Image.fromarray(template)
        t_img = Variable(normalize(to_tensor(scaler(template))).unsqueeze(0)).cuda()
        t_h = model(t_img)
        for j in range(len(bbox)):
            try:
                im_bbox = image[np.maximum(int(bbox[j, 0, 1]), 0):int(bbox[j, 2, 1]), np.maximum(int(bbox[j, 0, 0]), 0):int(bbox[j, 2, 0])]
                im_bbox = cv2.resize(im_bbox, (224, 224))
                im_bbox = cv2.cvtColor(im_bbox, cv2.COLOR_BGR2RGB)
                im_bbox = Image.fromarray(im_bbox)
                im_bbox = Variable(normalize(to_tensor(scaler(im_bbox))).unsqueeze(0)).cuda()
                image_h = model(im_bbox)
                sizes[i][j] = cosine_similarity(np.reshape(image_h.cpu().detach().numpy(), [1, -1]), np.reshape(t_h.cpu().detach().numpy(), [1, -1]))[0][0]
            except:
                sizes[i][j] = 0
    matched_bbox = np.argmax(np.max(sizes, 0))
    if np.max(np.max(sizes, 0)) > 0.2:
        template_matched = True

    sizes = np.zeros(len(bbox))
    template = cv2.resize(main_template, (224, 224))
    template = cv2.cvtColor(template, cv2.COLOR_BGR2RGB)
    template = Image.fromarray(template)
    t_img = Variable(normalize(to_tensor(scaler(template))).unsqueeze(0)).cuda()
    t_h = model(t_img)
    for j in range(len(bbox)):
        try:
            im_bbox = image[np.maximum(int(bbox[j, 0, 1]), 0):int(bbox[j, 2, 1]), np.maximum(int(bbox[j, 0, 0]), 0):int(bbox[j, 2, 0])]
            im_bbox = cv2.resize(im_bbox, (224, 224))
            im_bbox = cv2.cvtColor(im_bbox, cv2.COLOR_BGR2RGB)
            im_bbox = Image.fromarray(im_bbox)
            im_bbox = Variable(normalize(to_tensor(scaler(im_bbox))).unsqueeze(0)).cuda()
            image_h = model(im_bbox)
            sizes[j] = cosine_similarity(np.reshape(image_h.cpu().detach().numpy(), [1, -1]), np.reshape(t_h.cpu().detach().numpy(), [1, -1]))[0][0]
        except:
            sizes[j] = 0
    return template_matched, bbox[matched_bbox], matched_bbox, np.max(sizes)


def get_img(data_dir, img_path):
    img = mpimg.imread(data_dir + 'image/' + img_path + '.jpg')
    return img


def get_surfacenormal(data_dir, img_path):
    sf = imgproc.loadImage(data_dir + 'surface_normal/' + img_path + '_sf.jpg')
    sf = sf/255.
    return sf


def get_bbox(data_dir, img_path):
    with open(data_dir + 'det/' + img_path + '.txt', "r") as det_file:
        bbox = det_file.readlines()
    return bbox


def get_depth(data_dir, img_path):
    depth = imgproc.loadImage(data_dir + 'depth/' + img_path + '_depth.jpg')
    depth = depth / 255.
    return depth
