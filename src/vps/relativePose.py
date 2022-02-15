from ipdb import set_trace as bp
import numpy as np
import cv2
import copy
import glob, os
import matplotlib.pyplot as plt

class relativePose:
    def __init__(self):
        self.n_features = 0
        self.lk_params = dict(winSize=(21, 21),
                     criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.03))
        self.feature_detector = cv2.FastFeatureDetector_create(threshold=25, nonmaxSuppression=True)
        self.image0 = None
        self.get_dg_camera_matrix()

    def get_dg_camera_matrix(self):
        # Camera intrinsic matrix K = [[fx, skew, cx], [0, fy, cy], [0, 0, 1]]
        # Distortion parameter : k1, k2, k3, p1, p2
        #   - k1, k2, k3 : radial distortion coefficient
        #   - p1, p2 : tangential distortion coefficient).

        # Query camera model is c930e of Logitech
        camera_matrix_1 = np.array([[770.759827, 0, 379.503169], [0, 770.759827, 379.503169], [0, 0, 1]])  # c930e, 1280*720
        k1 = 0.079077
        k2 = -0.154798
        p1 = 0.000260
        p2 = -0.000010
        distCoeffs_1 = np.array([k1, k2, p1, p2])

        # Roadview camera model (FOV=90)
        #camera_matrix_2 = np.array([[731.2117795, 0, 512], [0, 731.2117795, 512], [0, 0, 1]])  #  Streetvoew of frontal image, 1024*1024, FOV 70
        camera_matrix_2 = np.array([[512, 0, 512], [0, 512, 512], [0, 0, 1]])   # Streetview of frontal image, 1024*1024, FOV 90
        distCoeffs_2 = None

        self.set_camera_matrix_1(camera_matrix_1, distCoeffs_1)
        self.set_camera_matrix_2(camera_matrix_2, distCoeffs_2)

        return camera_matrix_1, camera_matrix_2, distCoeffs_1, distCoeffs_2

    def set_camera_matrix_1(self, camera_matrix=None, distCoeffs=None):
        if camera_matrix is not None:
            self.camera_matrix_1 = camera_matrix
        if distCoeffs is not None:
            self.distCoeffs_1 = distCoeffs

    def set_camera_matrix_2(self, camera_matrix=None, distCoeffs=None):
        if camera_matrix is not None:
            self.camera_matrix_2 = camera_matrix
        if distCoeffs is not None:
            self.distCoeffs_2 = distCoeffs

    def get_img(self, path_or_cv2data, gray_enable=True):
        image = []
        if type(path_or_cv2data) in [str, np.str_]  :
            if gray_enable == True:
                image = cv2.imread(path_or_cv2data, cv2.IMREAD_GRAYSCALE)
            else:
                image = cv2.imread(path_or_cv2data)  # BGR
            image = cv2.resize(image, (640, 480))
            if image is None:
                image = []
        elif type(path_or_cv2data) == np.ndarray:
            image = path_or_cv2data
        else:
            print("Unknown type of image : ", type(path_or_cv2data))
        return image

    def visual_odometry(self, image):
        self.image1 = self.get_img(image)

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

    def get_relativePose(self, img0, img1):
        self.img0 = self.get_img(img0)
        self.img1 = self.get_img(img1)
        self.R, self.t = self.get_zero_Rt()
        self.pts1 = []
        self.pts2 = []
        self.kps1 = []
        self.kps2 = []
        if (len(self.img0) != 0) and (len(self.img1) != 0):
            #self.F, self.mask, self.pts1, self.pts2, self.kps1, self.kps2 = self.estimate_fundamental_matrix(self.img0, self.img1)
            self.pts1, self.pts2, self.kps1, self.kps2 = self.detect_matching_points(self.img0, self.img1)
            if len(self.pts1) > 10:
                _, self.R, self.t = self.estimate_relative_pose_from_correspondence(self.pts1, self.pts2, self.camera_matrix_1, self.camera_matrix_2)
                #_, self.R, self.t = self.estimate_relative_pose_from_correspondence(self.pts1, self.pts2, self.camera_matrix_1, self.camera_matrix_2, self.distCoeffs_1)
            else:
                self.R, self.t = self.get_zero_Rt()

        return self.R, self.t.squeeze()

    def estimate_relative_pose_from_correspondence(self, pts1, pts2, camera_matrix_1, camera_matrix_2, distCoeffs_1=None, distCoeffs_2=None):
        # https://github.com/zju3dv/GIFT/blob/master/train/evaluation.py
        f_avg = (camera_matrix_1[0, 0] + camera_matrix_2[0, 0]) / 2
        pts1, pts2 = np.ascontiguousarray(pts1, np.float32), np.ascontiguousarray(pts2, np.float32)
        pts_l_norm = cv2.undistortPoints(np.expand_dims(pts1, axis=1), cameraMatrix=camera_matrix_1, distCoeffs=distCoeffs_1)
        pts_r_norm = cv2.undistortPoints(np.expand_dims(pts2, axis=1), cameraMatrix=camera_matrix_2, distCoeffs=distCoeffs_2)
    
        # pp : principal point of the camera, focal : focal length of the camera.
        # https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html
        E, mask = cv2.findEssentialMat(pts_l_norm, pts_r_norm, focal=1.0, pp=(0., 0.),
                                   method=cv2.RANSAC, prob=0.999, threshold=3.0 / f_avg)
        points, R_est, t_est, mask_pose = cv2.recoverPose(E, pts_l_norm, pts_r_norm)
        return mask[:,0].astype(np.bool), R_est, t_est

    def detect_matching_points(self, img0, img1):  # Get fundamental matrix, input is cv2 img
    
        #sift = cv2.xfeatures2d.SIFT_create()
        sift = cv2.SIFT_create()
        # find the keypoints and descriptors with SIFT
        kp1, des1 = sift.detectAndCompute(img0,None)
        kp2, des2 = sift.detectAndCompute(img1,None)

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
        for i,(m,n) in enumerate(matches):
            if m.distance < 0.8*n.distance:
                good.append(m)
                pts2.append(kp2[m.trainIdx].pt)
                pts1.append(kp1[m.queryIdx].pt)
                kps2.append(kp2[m.trainIdx])  # for display
                kps1.append(kp1[m.queryIdx])  # for display
    
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
        t0 = np.zeros((3, 1))
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

    def display_update_pose(self, R=[], t=[], accumulated_pose_display=False):
        if len(R) == 0:
            R = self.R
        if len(t) == 0:
            t = self.t

        ## Accumulate trace of position
        self.current_pos += self.current_rot.dot(t) * self.scale
        self.current_rot = R.dot(self.current_rot)
        self.update_buffer(self.pos_x, self.current_pos[0][0])
        self.update_buffer(self.pos_y, self.current_pos[1][0])
        self.update_buffer(self.pos_z, self.current_pos[2][0])

        ## Relative position from origion for every frame
        _, t0 = self.get_zero_Rt()
        relPos = (R.dot(t0) + R.dot(t))*self.scale 

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
            lim_max = self.scale * 1.5

        plt.xlim(-lim_max, lim_max)
        plt.ylim(-lim_max, lim_max)
        plt.xlabel("x")
        plt.ylabel("z")
        plt.title("relativePose of Query")

        plt.pause(0.01)

if __name__ == "__main__":
    import pykitti
    
    basedir = '/mnt/data1/workdir/DB_Repo2/kitti'
    date = '2011_09_26'
    drive = '0022'
    
    data = pykitti.raw(basedir, date, drive, frames=range(0, 480, 1))
    img0 = []
    img1 = []
    camera_matrix_1 = data.calib.K_cam0
    camera_matrix_2 = data.calib.K_cam0
    
    mod_rPose = relativePose()
    img_count = 0

    mod_rPose.display_init_pose()
    for cam0_image in data.cam0:
        img1 = np.asarray(cam0_image)  # Get current image
        print(img_count)
        img_count+=1
        if False:  # no tracking
            mod_rPose.set_camera_matrix_1(camera_matrix_1)
            mod_rPose.set_camera_matrix_2(camera_matrix_1)
            if len(img0) == 0:
                img0 = copy.deepcopy(img1)  # Update img0 with new one, img1
                continue
    
            R, t = mod_rPose.get_relativePose(img0, img1)
            mod_rPose.display_update_pose(R, t)
            #print(R)
            #print(t)
    
            img = cv2.drawKeypoints(img1, mod_rPose.kps2, None)
            cv2.imshow('feature', img)
            cv2.waitKey(1)
        
            img1 = copy.deepcopy(img1)  # Update img0 with new one, img1
        else:  # with tracking
            mod_rPose.set_camera_matrix_1(camera_matrix_1)
            mod_rPose.set_camera_matrix_2(camera_matrix_1)
            R, t = mod_rPose.visual_odometry(img1)
            bp()
            mod_rPose.display_update_pose(R, t, True)

            img = cv2.drawKeypoints(img1, mod_rPose.keypoint0, None)
            cv2.imshow('feature', img)
            cv2.waitKey(1)
