from ipdb import set_trace as bp
import numpy as np
import math
import cv2
import copy
import glob, os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
#from noise_filter import noise_filter

class relativePose:
    def __init__(self, mode='normal', Tx=6.0, swap_input=False):
        self.n_features = 0
        self.lk_params = dict(winSize=(21, 21),
                     criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.03))
        self.feature_detector = cv2.FastFeatureDetector_create(threshold=25, nonmaxSuppression=True)
        self.image0 = None
        self.init_vo()
        self.mode = mode
        self.swap_input = swap_input   # swap input1 and input2
        self.Tx = Tx  # Distance from cam1 to cam2 in x-direction in meter.

        if self.swap_input == True:
            self.set_camera_matrix_1(self.get_c930e_camera_matrix())
            self.set_camera_matrix_2(self.get_roadview_camera_matrix())
        else:
            self.set_camera_matrix_1(self.get_roadview_camera_matrix())
            self.set_camera_matrix_2(self.get_c930e_camera_matrix())

        if 'test' in self.mode.lower():
            self.set_camera_matrix_1(self.get_c930e_camera_matrix())
            self.set_camera_matrix_2(self.get_c930e_camera_matrix())

    def get_Rt(self, img1, img2):
        '''
            defaut :
             - img1 for db (roadview)
             - img2 for q  (c930e)
            swap:
             - img1 for q  (c930e)
             - img2 for db (roadview)
            test:
             - img1 for q  (c930e)
             - img2 for q  (c930e)
        '''
        R, t = self.get_zero_Rt()

        if self.swap_input == True:
            img2_tmp = img2
            img2 = img1
            img1 = img2_tmp

        if 'opticalflow' in self.mode.lower():   # Normal, compare (db, q)
            R, t = self.get_relativePose_by_vo(img1, img2)  # Input are image path or np.ndarray(cv2 image)
        else:
            R, t = self.get_relativePose(img1, img2)  # Input are image path or np.ndarray(cv2 image)

        return R, t

    def get_absolute_scale(self, t, Tx=None):
        if Tx is None:
            Tx = self.Tx
        eps = 1e-6
        (tx, ty, tz) = t 
        scale = np.abs(Tx) / (np.abs(tx) + eps)
        return scale

    def get_cam2origin_on_cam1coordinate(self, R, t, Tx=None):
        '''
            Tx = 6.0  # scale* tx = Tx meter. Metric distance between center of cam0 and cam1 in x-direction (left to right)
            From https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html
            Let, 
                P1 : 3D point on camera 1 (Cam1) coordinate
                P2 : 3D point on camera 2 (Cam2) coordinate
                P2 = [R|t]P1 = R*P1 + t
            Then,
                For Cam1's origin in Cam2 coordinate (P2_cam1origin) : 
                    P2_cam1origin = R*P1_cam1origin + t
                    P2_cam1origin = R*(0,0,0)       + t = t = scale*(tx,ty,tz)
                For Cam2's origin in Cam1 coordinate (P1_cam2origin): 
                    P2_cam2origin = R*P1_cam2origin + t
                    P2_cam2origin = (0,0,0)
                               0  = R*P1_cam2origin + t
                               -t = R*P1_cam2origin
                               P1_cam2origin = -R_inv*t = -R.T*t, because R*(R.T) = I
            We will return Cam2's origin on Cam coordinate as pos
        '''
        if Tx is None:
            Tx = self.Tx
        scale = self.get_absolute_scale(t, Tx)
        t = scale * t

        # next two will be used as prevR and prevPos at next call
        P1_cam2origin = - (R.T).dot(t)  # cam2's origin on cam1's coordinate

        return P1_cam2origin

    @staticmethod
    def check_cam2_pos(cam2_pos):
        '''
        C1: centre of cam1, C2: centre of cam2, t (tx,ty,tz) is tralslation from C1 to C2
    
             Z (optical axis)                    Z (optical axis) 
            /                                   /
           /                                   /
          /                --------->         /
        C1------- X        (tx,ty,tz)       C2------- X
         |                                   |
         |                                   |
         |                                   |
         Y                                   Y
        '''
        valid_t = False
        (tx, ty, tz) = np.abs(cam2_pos)
        if tx > 0 :  # cam2 should be located to the right of of cam1. tx > 0
            if tx > ty*2 :  # |tx| (movement in left or right) should be at least 2 times greater than |ty| (altitude)
                if tz > ty*2:  # |tz| (movement in front or backward) should be at least 2 times greater than |ty| (altitude)
                    if tz < tx*5: # |tz| is three times smaller than |tx|
                        valid_t = True
        return valid_t

    def get_roadview_camera_matrix(self):
        # Camera intrinsic matrix K = [[fx, skew, cx], [0, fy, cy], [0, 0, 1]]
        # Distortion parameter : k1, k2, p1, p2, k3
        #   - k1, k2, k3 : radial distortion coefficient
        #   - p1, p2 : tangential distortion coefficient).

        # Roadview camera model (FOV=90)
        ''' simple estimation by fov_diag(90) and w(1024), h(1024)
            Assume that fx = fy = f
            w, h = 1024, 1024
            cx, cy = w/2, h/2
            r=w; fov=90; fov=np.deg2rad(fov/2); f = (r/2) / np.tan(fov)  # 512.0
            print("[[{},0,{}],[0,{},{}],[0,0,1]]".format(f,cx,f,cy))
            # [[512.0000000000001,0,512.0],[0,512.0000000000001,512.0],[0,0,1]]
        '''
        camera_matrix = np.array([[512, 0, 512], [0, 512, 512], [0, 0, 1]])   # Streetview of frontal image, 1024*1024, FOV_hori 90
        distCoeffs = None

        return (camera_matrix, distCoeffs)

    def get_c930e_camera_matrix(self):
        # Camera intrinsic matrix K = [[fx, skew, cx], [0, fy, cy], [0, 0, 1]]
        # Distortion parameter : k1, k2, p1, p2i, k3
        #   - k1, k2, k3 : radial distortion coefficient
        #   - p1, p2 : tangential distortion coefficient).

        # Query camera model is c930e of Logitech
        ''' simple estimation by fov_diag(90) and w(1280), h(720)
            Assume that fx = fy = f
            w, h = 1280, 720
            diag = np.sqrt(w**2 + h**2)
            r=diag; fov=90; fov=np.deg2rad(fov/2); f = (r/2) / np.tan(fov);print(f) # 734.3023900274329
        '''
        if False:  # by jylee, c930e, 1280*720
            camera_matrix = np.array([[770.759827, 0, 619.924535], [0, 770.759827, 379.503169], [0, 0, 1]])
            k1 = 0.079077
            k2 = -0.154798
            p1 = 0.000260
            p2 = -0.000010
            distCoeffs = np.array([k1, k2, p1, p2])
        else:  #  by ccsmm, c930e, 1280*720
            camera_matrix = np.array(
                    [[732.27093506,   0.,         619.40541797],
                     [  0.        , 733.39678955,  405.92213755],
                     [  0.        ,   0.,           1.        ]])
            [k1, k2, p1, p2, k3] = [ 0.09470587, -0.24501318,  0.0018614,   0.00108221,  0.11442398]
            distCoeffs = np.array([k1, k2, p1, p2, k3])

        return (camera_matrix, distCoeffs)

    def set_camera_matrix_1(self, camera_matrix_distCoeffs):
        self.camera_matrix_1 = copy.deepcopy(camera_matrix_distCoeffs[0])
        self.distCoeffs_1 = copy.deepcopy(camera_matrix_distCoeffs[1])

    def set_camera_matrix_2(self, camera_matrix_distCoeffs):
        self.camera_matrix_2 = copy.deepcopy(camera_matrix_distCoeffs[0])
        self.distCoeffs_2 = copy.deepcopy(camera_matrix_distCoeffs[1])

    def get_img(self, path_or_cv2data, gray_enable=True):
        image = None
        if type(path_or_cv2data) in [str, np.str_]  :
            if os.path.exists(path_or_cv2data):
                if gray_enable == True:
                    image = cv2.imread(path_or_cv2data, cv2.IMREAD_GRAYSCALE)
                else:
                    image = cv2.imread(path_or_cv2data)  # BGR
        elif type(path_or_cv2data) == np.ndarray:
            image = path_or_cv2data.copy()
            if (gray_enable == True) and (image.shape[-1] == 3):
                image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            print("[vps] Unknown type of image : ", type(path_or_cv2data))

        #if image is not None:
        #    try:
        #        image = cv2.resize(image, (640, 480))
        #    except:
        #        image = None
        return image  # ndarray [h,w,c] or None

    def visual_odometry(self, image):
        self.R, self.t = self.get_zero_Rt()

        self.image1 = self.get_img(image)
        if len(self.image1) > 0:
            self.keypoint1 = self.feature_detector.detect(self.image1, None)
            if self.image0 is None:  # Initialize
                self.image0 = self.image1.copy()
                self.keypoint0 = self.keypoint1.copy()
    
            #pts0 = np.array(map(lambda x: [x.pt], self.keypoint0), dtype=np.float32)
            pts0 = np.array([i for i in map(lambda x: [x.pt], self.keypoint0)], dtype=np.float32).squeeze()
            try:
                pts1, st, err = cv2.calcOpticalFlowPyrLK(self.image0, self.image1, pts0, None, **self.lk_params)
    
                # Save the good points from the optical flow
                st = st.squeeze()
                self.good0 = pts0[st == 1]
                self.good1 = pts1[st == 1]
    
                E, mask = cv2.findEssentialMat(self.good1, self.good0, self.camera_matrix_1, cv2.RANSAC, 0.999, 1.0, None)
    
                _, self.R, self.t, mask = cv2.recoverPose(E, self.good0, self.good1, self.current_rot.copy(), self.current_pos.copy(), self.camera_matrix_1)
                self.image0 = self.image1.copy()
                self.keypoint0 = self.keypoint1.copy()
            except:
                self.R, self.t = self.get_zero_Rt()

        return self.R, self.t

    def init_vo(self):
        # params for ShiTomasi corner detection
        self.feature_params1 = dict( maxCorners = 100, 
                               qualityLevel = 0.3,
                               minDistance = 7,
                               blockSize = 7 ) 
	# Parameters for lucas kanade optical flow
        self.lk_params1 = dict( winSize  = (15,15),
                                maxLevel = 2,
                                criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

        self.vo_result = []
	

    def draw_vo(self, color_old, color_new, good_old, good_new):
        # Create some random colors
        color = np.random.randint(0,255,(10000,3))
        # draw the tracks
        mask = np.zeros_like(color_old)
        for i,(new,old) in enumerate(zip(good_new,good_old)):
            a,b = new.ravel()
            c,d = old.ravel()
            mask = cv2.line(mask, (a,b),(c,d), color[i].tolist(), 2)
            frame = cv2.circle(color_new,(a,b),5,color[i].tolist(),-1)
        img = cv2.add(frame,mask)
        return img

    def get_relativePose_by_vo(self, img1, img2):  # Find feature from visual odometry
        self.img1 = self.get_img(img1)  # old frame (reference)
        self.img2 = self.get_img(img2)  # new frame (target)
        self.R, self.t = self.get_zero_Rt()
        self.pts1 = []
        self.pts2 = []
        if (self.img1 is not None) and (self.img2 is not None):
            try:
                # calculate optical flow
                if False:  # fast with no accuracy
                    self.pts1 = cv2.goodFeaturesToTrack(self.img1, mask = None, **self.feature_params1)  # fast with no accuracy
                else:
                    self.kps1 = self.feature_detector.detect(self.img1, None)
                    self.pts1 = np.array([i for i in map(lambda x: [x.pt], self.kps1)], dtype=np.float32).squeeze()

                self.pts2, st, err = cv2.calcOpticalFlowPyrLK(self.img1, self.img2, self.pts1, None, **self.lk_params)
            except:
                bp()
                return self.R, self.t.squeeze()

            try:
                # Save the good points from the optical flow
                st = st.squeeze()
                self.good_pts1 = np.int32(self.pts1[st==1].squeeze())
                self.good_pts2 = np.int32(self.pts2[st==1].squeeze())
                if True:
                    color_img1 = self.get_img(img1, gray_enable=False)
                    color_img2 = self.get_img(img2, gray_enable=False)
                    self.vo_result = self.draw_vo(color_img1, color_img2, self.good_pts1, self.good_pts2)
            except:
                bp()
                return self.R, self.t.squeeze()

            if len(self.good_pts1) > 10:
                _, self.R, self.t = self.estimate_relative_pose_from_correspondence(self.good_pts1, self.good_pts2, self.camera_matrix_1, self.camera_matrix_2, self.distCoeffs_1, self.distCoeffs_2)

        return self.R, self.t.squeeze()

    def get_relativePose(self, img1, img2):
        self.img1 = self.get_img(img1)
        self.img2 = self.get_img(img2)
        self.R, self.t = self.get_zero_Rt()
        self.pts1 = []
        self.pts2 = []
        self.kps1 = []
        self.kps2 = []
        if (self.img1 is not None) and (self.img2 is not None):
            #self.F, self.mask, self.pts1, self.pts2, self.kps1, self.kps2 = self.estimate_fundamental_matrix(self.img0, self.img1)
            self.pts1, self.pts2, self.kps1, self.kps2 = self.detect_matching_points(self.img1, self.img2)
            if len(self.pts1) > 10:
                _, self.R, self.t = self.estimate_relative_pose_from_correspondence(self.pts1, self.pts2, self.camera_matrix_1, self.camera_matrix_2, self.distCoeffs_1, self.distCoeffs_2)
                #_, self.R, self.t = self.estimate_relative_pose_from_correspondence(self.pts1, self.pts2, self.camera_matrix_1, self.camera_matrix_2, self.distCoeffs_1)
            else:
                self.R, self.t = self.get_zero_Rt()

        return self.R, self.t.squeeze()

    def get_pan_tilt(self, R):
        ''' Get pan(yaw) and tile(pitch) of unit vector on z-axis(to proceeding direction)
            Eq. 4 at  https://darkpgmr.tistory.com/122
            R : 3x3 rotation matrix

            Refer to ./dg_pan_tilt.png
            Zc is Z axis of cam2 in cam2's coordinate.
            Zw is Z axis of cam2 in cam1's coordinate.
            Z0 is Z axis of cam1 in cam1's coordinate.
            pan : angle between Z0 and Zw on x-z plane in cam1's coordinate. Rotation axis is y-axis.
                positive value : turn right
                negative value : turn left
                pan is angle of Zw from z-axis to x-axis.
                theta is angle of Zw from x-axis to z-axis.
                pan = pi/2 - theta
                tan(theta) = zz / zx
                theta = arctan2(zz, zx)
                pan = pi/2 - arctan2(zz, zx)
                or
                pan = arctan2(zx, zz)
            tilt : Rotation axis is x-axis.
                positive value : turn down
                negative value : turn up
                tan(tilt) = zy / d, where d = sqrt(zx^2 + zz^2)
                tilt = arctan2(zy, d)
        '''
        unit_z = [0,0,1]
        Zc = unit_z  # Z axis of camera 2
        Zw = np.matmul(R.T, Zc) + 0  # Translation is not requried to calculate angle, so it is zero. R_inv is R.T in case of rotation matrix.
        (zx, zy, zz) = Zw
        if False:  # darkpgmr
            pan = math.atan2(Zw[1], Zw[0]) - np.pi/2
            tilt = math.atan2(Zw[2], np.sqrt(Zw[0]*Zw[0] + Zw[1]*Zw[1]))
        else:  # ccsmm
            #pan = np.pi/2 - np.arctan2(zz, zx)  # negative angle : turn left, positive : turn right
            pan  = np.arctan2(zx, zz)  # negative angle : turn left, positive : turn right
            tilt = np.arctan2(zy, np.sqrt(zx**2 + zz**2))  # negative angle : turn up, postive angle : turn down
        return pan, tilt

    @staticmethod
    def estimate_relative_pose_from_correspondence(pts1, pts2, camera_matrix_1, camera_matrix_2, distCoeffs_1=None, distCoeffs_2=None):
        # https://github.com/zju3dv/GIFT/blob/master/train/evaluation.py
        f_avg = (camera_matrix_1[0, 0] + camera_matrix_2[0, 0]) / 2
        pts1, pts2 = np.ascontiguousarray(pts1, np.float32), np.ascontiguousarray(pts2, np.float32)
        pts1_norm = cv2.undistortPoints(np.expand_dims(pts1, axis=1), cameraMatrix=camera_matrix_1, distCoeffs=distCoeffs_1)
        pts2_norm = cv2.undistortPoints(np.expand_dims(pts2, axis=1), cameraMatrix=camera_matrix_2, distCoeffs=distCoeffs_2)
    
        # pp : principal point of the camera, focal : focal length of the camera.
        # https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html
        E, mask = cv2.findEssentialMat(pts1_norm, pts2_norm, focal=1.0, pp=(0., 0.), method=cv2.RANSAC, prob=0.999, threshold=1.5/f_avg)  # ori : 3.0/f_avg
        points, R_est, t_est, mask_pose = cv2.recoverPose(E, pts1_norm, pts2_norm)
        return mask[:,0].astype(np.bool), R_est, t_est

    def detect_matching_points(self, img1, img2):  # Get fundamental matrix, input is cv2 img
        try:
            sift = cv2.xfeatures2d.SIFT_create()  # cv2.__version__ < 4.4.0
        except:
            sift = cv2.SIFT_create()  # cv2.__version__ >= 4.4.0

        # find the keypoints and descriptors with SIFT
        kp1, des1 = sift.detectAndCompute(img1,None)
        kp2, des2 = sift.detectAndCompute(img2,None)

        # FLANN parameters
        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks=50)
        flann = cv2.FlannBasedMatcher(index_params,search_params)
        matches = flann.knnMatch(des1,des2,k=2)
        good = []
        pts1 = []
        pts2 = []
        kps1 = []
        kps2 = []
    
        # ratio test as per Lowe's paper
        for i,(m,n) in enumerate(matches):  # m is for top-1. n is for top-2, k=2
            if m.distance < 0.75*n.distance:  # ori. 0.8
                good.append(m)
                pts2.append(kp2[m.trainIdx].pt)  # trainIdx : matched point's idx of img2
                pts1.append(kp1[m.queryIdx].pt)  # queryIdx : reference idx of img1
                kps2.append(kp2[m.trainIdx])  # for display
                kps1.append(kp1[m.queryIdx])  # for display
                
        self.raw_kps1 = kp1
        self.raw_kps2 = kp2
        self.raw_matches = matches
        self.good_matches = good
    
        pts1 = np.int32(pts1)
        pts2 = np.int32(pts2)
    
        return pts1, pts2, kps1, kps2


    def estimate_fundamental_matrix(self, img0, img1):  # Get fundamental matrix, input is cv2 img
        pts1, pts2, kps1, kps2 = self.detect_matching_points(img0, img1)
    
        F, mask = cv2.findFundamentalMat(pts1, pts2, cv2.FM_LMEDS)
        # We select only inlier points
        try:
            pts1 = pts1[mask.ravel()==1]
            pts2 = pts2[mask.ravel()==1]
        except:
            pass
        
        return F, mask, pts1, pts2, kps1, kps2

    def get_zero_Rt(self):
        R0 = np.eye(3)
        #t0 = np.zeros((3, 1))
        t0 = np.zeros(3)
        return R0, t0

    def euler_to_rotMat(yaw, pitch, roll):
        Rz_yaw = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw),  np.cos(yaw), 0],
            [          0,            0, 1]])
        Ry_pitch = np.array([
            [ np.cos(pitch), 0, np.sin(pitch)],
            [             0, 1,             0],
            [-np.sin(pitch), 0, np.cos(pitch)]])
        Rx_roll = np.array([
            [1,            0,             0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll),  np.cos(roll)]])
        # R = RzRyRx
        rotMat = np.dot(Rz_yaw, np.dot(Ry_pitch, Rx_roll))
        return rotMat

    def display_init_pose(self, buf_size=500):
        #self.current_pos = np.zeros((3, 1))
        #self.current_rot = np.eye(3)
        self.current_rot, self.current_pos = self.get_zero_Rt()
        self.pos_x = np.zeros(buf_size)
        self.pos_y = np.zeros(buf_size)
        self.pos_z = np.zeros(buf_size)
        self.scale = 1  # depth to z direction
        #self.pos_x = []
        #self.pos_y = []
        #self.pos_z = []

        # create graph.
        self.position_figure = plt.figure()
        self.position_axes = self.position_figure.add_subplot(1, 1, 1)

    def update_buffer(self, buf, newdata):
        buf[0:-1] = buf[1:]
        buf[-1] = newdata 
        return buf

    def display_update_pose(self, R=[], t=[], scale=0, accumulated_pose_display=False):
        if len(R) == 0:
            R = self.R
        if len(t) == 0:
            t = self.t
        if scale == 0:
            scale = self.scale

        ## Accumulate trace of position
        self.current_pos += self.current_rot.dot(t) * scale
        self.current_rot = R.dot(self.current_rot)
        self.update_buffer(self.pos_x, self.current_pos[0])
        self.update_buffer(self.pos_y, self.current_pos[1])
        self.update_buffer(self.pos_z, self.current_pos[2])

        ## Relative position from origion for every frame
        _, t0 = self.get_zero_Rt()
        relPos = (R.dot(t0) + R.dot(t))*scale 

        if accumulated_pose_display == True:  # display accumulated pose
            self.position_axes.cla()
            self.position_axes.plot(self.pos_x, self.pos_z)  # x : horizontal, z : cam's optical direction, from cam to world direction
            self.position_axes.plot(self.pos_x[-1], self.pos_z[-1],"r*")  # x : horizontal, z : cam's optical direction, from cam to world direction
            self.position_axes.set_aspect('equal', adjustable='box')
            lim_max = np.abs(np.max([np.max(np.abs(self.pos_x)), np.max(np.abs(self.pos_z)), 1],))
        else:  # display current relative pose
            self.position_axes.cla()
            self.position_axes.plot([t0[0], relPos[0]], [t0[2],relPos[2]],"g")  # x : horizontal, z : cam's optical direction, from cam to world direction
            self.position_axes.plot(relPos[0], relPos[2],"r*")  # x : horizontal, z : cam's optical direction, from cam to world direction
            self.position_axes.set_aspect('equal', adjustable='box')
            lim_max = scale * 1.5

        plt.xlim(-lim_max, lim_max)
        plt.ylim(-lim_max, lim_max)
        plt.xlabel("x")
        plt.ylabel("z")
        plt.title("relativePose of Query")

        plt.pause(0.01)

def run_kitti():
    import pykitti
    basedir = '/mnt/data1/workdir/DB_Repo2/kitti'
    date = '2011_09_26'
    drive = '0022'
    
    data = pykitti.raw(basedir, date, drive, frames=range(0, 480, 1))
    img0 = []
    img1 = []
    camera_matrix_1 = data.calib.K_cam0
    camera_matrix_2 = data.calib.K_cam0
    
    self = relativePose()
    img_count = 0

    self.display_init_pose()
    for cam0_image in data.cam0:
        img1 = np.asarray(cam0_image)  # Get current image
        print(img_count)
        img_count+=1
        if True:  # no tracking
            self.set_camera_matrix_1(camera_matrix_1)
            self.set_camera_matrix_2(camera_matrix_1)
            if len(img0) == 0:
                img0 = copy.deepcopy(img1)  # Update img0 with new one, img1
                continue
    
            R, t = self.get_relativePose(img0, img1)
            self.display_update_pose(R, t)
            #print(R)
            #print(t)
    
            img = cv2.drawKeypoints(img1, self.kps2, None)
            cv2.imshow('feature', img)
            cv2.waitKey(1)
        
            img1 = copy.deepcopy(img1)  # Update img0 with new one, img1
        else:  # with tracking
            self.set_camera_matrix_1(camera_matrix_1)
            self.set_camera_matrix_2(camera_matrix_1)
            R, t = self.visual_odometry(img1)
            self.display_update_pose(R, t, True)

            img = cv2.drawKeypoints(img1, self.keypoint0, None)
            cv2.imshow('feature', img)
            cv2.waitKey(1)

def draw_vector(vec_start=[0,0,0], vec_end=[0.58, 0.58, 0.58]):
    global draw_vector_fig, draw_vector_ax
    if draw_vector_fig is None:
        draw_vector_fig = plt.figure()
    if draw_vector_ax is None:
        draw_vector_ax = draw_vector_fig.add_subplot(1,1,1, projection='3d')

    plt.cla()
    draw_vector_ax.set_xlim(-1,30)
    draw_vector_ax.set_ylim(-1,30)
    draw_vector_ax.set_zlim(-1,30)
    draw_vector_ax.set_xlabel('X')
    draw_vector_ax.set_ylabel('Y')
    draw_vector_ax.set_zlabel('Z')
    draw_vector_ax.view_init(elev=0, azim=-90) # (0, -90) means topview of xz-plane, viewangle, elev = virtical,  azim = horizontal
    draw_vector_ax.grid()

    draw_vector_ax.set_axisbelow(True)        
    draw_vector_ax.scatter(vec_start[0], vec_start[1], vec_start[2], color='r', s=20)
    draw_vector_ax.quiver(vec_start[0], vec_start[1], vec_start[2], vec_end[0], vec_end[1], vec_end[2], color='black')
    plt.pause(0.1)

def run_usbcam(video_src=0, feature_mode='normal', Tx=1.0, skip_frame=0, feature_display=True, vector_display=True, interlaced=False):
    '''
        feature_mode = 'normal' : two images ==> SIFT desc ==> matching ==> EssentialMatrix ==> R,t from recoverPose
        feature_mode = 'opticalflow' : two images ==> SIFT desc ==> matching ==> EssentialMatrix ==> R,t from recoverPose
    '''
    if ".avi" in video_src.lower():
        cap = cv2.VideoCapture(video_src)
    elif 0 == video_src.lower():
        cap = cv2.VideoCapture(video_src)
    else:
        from simple_VideoCapture import simple_VideoCapture
        cap = simple_VideoCapture(video_src, "jpg")

    if interlaced ==False:
        while(True):
            ret, frame = cap.read()
            skip_frame = skip_frame - 1
            print("Skipping {}    \r".format(skip_frame), end='')
            if skip_frame < 0:
                break
        ref_img = frame # first image is query image.

    fig, ax = None, None
    mod_rPose = None
    t = [0.57735027, 0.57735027,  0.57735027]

    global draw_vector_fig, draw_vector_ax
    draw_vector_fig = None
    draw_vector_ax = None

    fig_num = 100
    frame_num = 0
    mod_rPose = relativePose(mode=feature_mode, Tx=Tx)

    while(True):
        if interlaced == True:
            ret1, img1 = cap.read()
            ret2, img2 = cap.read()
            ret = ret1 and ret2
        else:
            img1 = ref_img.copy()
            ret, img2 = cap.read()

        if not ret:
            break

        print("Press [esc] to exit.")
        print(" Frame : {}".format(frame_num))

        frame_num = frame_num + 1

        R, t = mod_rPose.get_Rt(img1, img2)  # Swapping input works fine. why?
        pan, tilt = mod_rPose.get_pan_tilt(R)
        cam2_pos = mod_rPose.get_cam2origin_on_cam1coordinate(R, t)

        if mod_rPose.check_cam2_pos(cam2_pos) is False:
            print("\033[F", end='') # put the cursor to the previous line
            print("\033[F", end='') # put the cursor to the previous line
            continue

        print("   >> (tx,ty,tz)        : ({0:02.3f}, {1:02.3f}, {2:02.3f})   ".format(t[0], t[1], t[2]))
        print("   >> Cam2's origin [m] : ({0:02.3f}, {1:02.3f}, {2:02.3f})   ".format(cam2_pos[0], cam2_pos[1], cam2_pos[2]))
        print("   >> pan[degree]: {0:03.3f}, tilt: {1:03.3f}    ".format(np.rad2deg(pan), np.rad2deg(tilt)))
        print("\033[F", end='') # put the cursor to the previous line
        print("\033[F", end='') # put the cursor to the previous line
        print("\033[F", end='') # put the cursor to the previous line
        print("\033[F", end='') # put the cursor to the previous line
        print("\033[F", end='') # put the cursor to the previous line

        if vector_display:
            cam1_pos = [0,0,0]
            draw_vector(cam1_pos, cam2_pos)

        if feature_display:
            if feature_mode.lower() in 'opticalflow':
                img = mod_rPose.vo_result
            else:
                img = cv2.drawMatches(mod_rPose.img1, mod_rPose.raw_kps1, mod_rPose.img2, mod_rPose.raw_kps2, mod_rPose.good_matches, None, flags=cv2.DRAW_MATCHES_FLAGS_NOT_DRAW_SINGLE_POINTS )
            img_re = img_resize(img, w_resize=1000)
            cv2.imshow('features', img_re)
        else:
            img1_re = img_resize(img1, w_resize=500)
            cv2.imshow('img1', img1_re)
            if interlaced == True:
                img2_re = img_resize(img2, w_resize=500)
                cv2.imshow('img2', img2_re)

        if cv2.waitKey(1000) & 0xFF == 27:  # Esc key to stop
            break

    print("\n\n\n\n\n")
    cap.release()
    cv2.destroyAllWindows()

def img_resize(img, w_resize=None, h_resize=None):
    H,W,C = img.shape

    if w_resize is not None:
        ratio = w_resize / W
        h_resize = int(H*ratio)

    if h_resize is not None:
        ratio = h_resize / H
        w_resize = int(W*ratio)

    if (w_resize == None) or (h_resize ==  None):
        w_resize = W
        h_resize = H

    return cv2.resize(img, (w_resize, h_resize))

            
def run_usbcam_simple(video_src=0, feature_mode='normal', Tx=1.0):
    cap = cv2.VideoCapture(video_src)
    ret, frame = cap.read()
    ref_img = frame # first image is query image.
    H,W,C = frame.shape
    print("H,W,C = ", H,W,C)

    fig, ax = None, None
    mod_rPose = None

    global draw_vector_fig, draw_vector_ax
    draw_vector_fig = None
    draw_vector_ax = None

    fig_num = 100
    frame_num = 0
    mod_rPose = relativePose(mode=feature_mode, Tx=Tx)
    while(True):
        ret, img2 = cap.read()
        if not ret:
            break
        img1 = ref_img.copy()

        R, t = mod_rPose.get_Rt(img1, img2)  # Swapping input works fine. why?
        pan, tilt = mod_rPose.get_pan_tilt(R)
        cam2_pos = mod_rPose.get_cam2origin_on_cam1coordinate(R, t)

        print("   >> (tx,ty,tz)        : ({0:02.3f}, {1:02.3f}, {2:02.3f})   ".format(t[0], t[1], t[2]))
        print("   >> Cam2's origin [m] : ({0:02.3f}, {1:02.3f}, {2:02.3f})   ".format(cam2_pos[0], cam2_pos[1], cam2_pos[2]))
        print("   >> pan[degree]: {0:03.3f}, tilt: {1:03.3f}    ".format(np.rad2deg(pan), np.rad2deg(tilt)))
        print("==================================================")

        w_re = 500
        ratio = w_re / W
        h_re = int(H*ratio)
        cv2.imshow('img', cv2.resize(img2, (w_re, h_re)))

        if cv2.waitKey(1000) & 0xFF == 27:  # Esc key to stop
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    ## Prepare dataset
    #interlaced = False # video sequence : ref, tar1, tar2, tar2, tar3, ...
    #interlaced = True  # video sequence : db0, q0, db1, q1, ... , dbn, qn
    
    #feature_mode = "test"  # Same camera for cam1, cam2  
    #feature_mode = "normal" # cam1 is roadview, cam2 is logitech c903e

    feature_display=True
    vector_display=True

    ############################
    ## Choose one in following :
    #video_src = 0  # usb cam
    #video_src = "./video_indoor.avi"; Tx=1.0; feature_mode = "test"; interlaced = False
    video_src = "./test_relativePose_outdoor.avi"; Tx=6.0; feature_mode = "test"; interlaced = False
    #video_src = "../../bin/vps_img"; Tx=6.0; feature_mode = "normal"; interlaced = True
    ############################

    ## Run
    run_usbcam(video_src, feature_mode=feature_mode, Tx=Tx, skip_frame=0, feature_display=feature_display, vector_display=vector_display, interlaced=interlaced)
    #run_kitti()
