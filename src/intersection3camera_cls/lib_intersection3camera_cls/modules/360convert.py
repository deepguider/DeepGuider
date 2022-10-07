import cv2
import numpy as np
from scipy.ndimage import map_coordinates
import py360convert


# unit test
if __name__ == '__main__':

    import argparse
    parser = argparse.ArgumentParser("Crop image")
    parser.add_argument("--image", type=str, default="test.jpg", help='target image')
    parser.add_argument("--tilt", type=float, default=0.0, help='tilt angle [deg]')
    parser.add_argument("--pan", type=float, default=0.0, help='pan angle [deg]')
    parser.add_argument("--fov", type=float, default=70.0, help='fov [deg]')
    parser.add_argument("--width", type=int, default=1280, help='width [px]')
    parser.add_argument("--height", type=int, default=960, help='height [px]')
    args = parser.parse_args()

    img = dict()

    img['top'] = cv2.imread('/home/dg/Downloads/220628_COEX_Indoor_workshop/rosbag/_2022-06-28-17-03-29_pathA1_back/top_000000.jpg')
    img['bottom'] = cv2.imread('/home/dg/Downloads/220628_COEX_Indoor_workshop/rosbag/_2022-06-28-17-03-29_pathA1_back/bottom_000000.jpg')
    #img['back'] = cv2.imread('/home/dg/Downloads/220628_COEX_Indoor_workshop/rosbag/_2022-06-28-17-11-38_pathA1_both/back_000100.jpg')
    #img['left'] = cv2.imread('/home/dg/Downloads/220628_COEX_Indoor_workshop/rosbag/_2022-06-28-17-03-29_pathA1_back/3camera_image/center_000389.jpg')
    #img['front'] = cv2.imread('/home/dg/Downloads/220628_COEX_Indoor_workshop/rosbag/_2022-06-28-17-03-29_pathA1_back/3camera_image/left_000460.jpg')
    #img['right'] = cv2.imread('/home/dg/Downloads/220628_COEX_Indoor_workshop/rosbag/_2022-06-28-17-03-29_pathA1_back/3camera_image/right_000581.jpg')
    img['back'] = cv2.imread('/home/dg/Downloads/220628_COEX_Indoor_workshop/rosbag/_2022-06-28-17-03-29_pathA1_back/back_000000.jpg')
    img['left'] = cv2.imread('/home/dg/Downloads/220628_COEX_Indoor_workshop/rosbag/_2022-06-28-17-03-29_pathA1_back/3camera_image/left_000000.jpg')
    img['front'] = cv2.imread('/home/dg/Downloads/220628_COEX_Indoor_workshop/rosbag/_2022-06-28-17-03-29_pathA1_back/3camera_image/center_000000.jpg')
    img['right'] = cv2.imread('/home/dg/Downloads/220628_COEX_Indoor_workshop/rosbag/_2022-06-28-17-03-29_pathA1_back/3camera_image/right_000000.jpg')

    img['top'] = cv2.resize(img['top'], (img['top'].shape[0], img['top'].shape[0]))
    img['bottom'] = cv2.resize(img['bottom'], (img['top'].shape[0], img['top'].shape[0]))
    img['back'] = cv2.resize(img['back'], (img['top'].shape[0], img['top'].shape[0]))
    img['left'] = cv2.resize(img['left'], (img['top'].shape[0], img['top'].shape[0]))
    img['front'] = cv2.resize(img['front'], (img['top'].shape[0], img['top'].shape[0]))
    img['right'] = cv2.resize(img['right'], (img['top'].shape[0], img['top'].shape[0]))

    cv2.imshow('top img', img['top'])

    assert img['top'].shape == img['bottom'].shape == img['back'].shape == img['left'].shape == img['front'].shape == img['right'].shape 

    one_h, one_w = img['top'].shape[0], img['top'].shape[1]
    cube_img_h, cube_img_w = img['top'].shape[0] * 3, img['top'].shape[1] * 4
    cube_img = np.zeros((cube_img_h, cube_img_w, img['top'].shape[2]), np.uint8)

    cube_img[one_h:2*one_h,0:one_w,:] = img['left']
    cube_img[0:one_h,one_w:2*one_w,:] = img['top']
    cube_img[one_h:2*one_h,one_w:2*one_w,:] = img['front']
    cube_img[one_h*2:3*one_h,one_w:2*one_w,:] = img['bottom']
    cube_img[one_h:2*one_h,one_w*2:3*one_w,:] = img['right']
    cube_img[one_h:2*one_h,one_w*3:4*one_w,:] = img['back']

    cv2.imshow('cubeimg', cube_img)

    w = cube_img.shape[0] // 3
    print('cube img shape: ', cube_img.shape, ' w*3=', w*3, ' w*4', w*4)

    pano_image = py360convert.c2e(cube_img, h=960, w= 1920, mode='bilinear')
    pano_image = pano_image.astype(np.uint8)
    cv2.imshow('panoimg', pano_image)
    cv2.imwrite('match_cubeimg.jpg', cube_img)
    cv2.imwrite('match_panoimg.jpg', pano_image)
    cv2.waitKey(0)

