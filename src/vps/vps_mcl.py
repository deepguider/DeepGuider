## Code from https://ros-developer.com/2019/04/10/parcticle-filter-explained-with-python-code-from-scratch/

import numpy as np
import scipy
from numpy.random import uniform
import scipy.stats
import cv2

import time
import matplotlib.pylab as plt
from ipdb import set_trace as bp

import mcl_config 
from DispMap import DispMap

class MCL():
    def __init__(self, cv_img, ltop, rbottom, incoord="utm",  n_particle=400,
            motion_err_mu=0, motion_err_std=0.1, sensor_vps_err_mu=0, sensor_vps_err_std=0.1, pdf_std=10):
        '''
        observation_mode is one of the ["distance", "angle"]
        pdf_std = 0.1 for angle metric, 10 for distance metric
        width, height are width and height of a map, respectively.
        n_particle is a number of particles
        n_landmark is a number of landmarks. If it is zero, it will be a random integer value between 1 and 10 every loop.
        landmark positions change at every landmark_update_count loop
        '''
        # You can make empyth image with : np.zeros((self.HEIGHT, self.WIDTH, 3), np.uint8)
        self.height, self.width, ch = cv_img.shape
        self.incoord = incoord

        self.mMap = DispMap(cv_img, ltop, rbottom, incoord)

        self.WINDOW_NAME="Particle Filter"
    
        self.x_range=np.array([0, self.width])
        self.y_range=np.array([0, self.height])
        
        #MCL parameters
        self.n_particle = n_particle
        self.pdf_std = pdf_std
        self.callback_count = 0
        self.sensor_std_err = 5  # 0.1 for cosine sim
        self.motion_err_mu = motion_err_mu
        self.motion_err_std = motion_err_std
        self.sensor_vps_err_mu = sensor_vps_err_mu
        self.sensor_vps_err_std = sensor_vps_err_std

        # Color (B,G,R)
        self.color_landmark = (128,0,0)
        self.color_particle = (0,255,0)
        self.color_trajectory_odo = (0,0,255)
        self.color_trajectory_vps = (255,0,0)
        self.color_trajectory_mcl = (0,255,0)

    def initialize(self, disp=False):
        self.particles = self.create_uniform_particles()
        self.weights = np.array([1.0]*self.n_particle)

        self.previous_x = -1
        self.previous_y = -1
        
        # Create a black image, a window and bind the function to window
        self.disp = disp

        if self.disp == True:
            cv2.namedWindow(self.WINDOW_NAME)
        
        self.trajectory_odo = np.zeros(shape=(0,2))
        self.trajectory_vps = np.zeros(shape=(0,2))
        self.trajectory_mcl = np.zeros(shape=(0,2))
        self.robot_pos = np.zeros(shape=(0,2))

        self.DELAY_MSEC = 10
        self.loop_count = 0

        self.draw_legend()
        return 0

    #def run_step(self, odo_x, odo_y, heading, rPose_tx, rPose_ty, landmarks):
    def run_step(self, odo_x, odo_y, odo_heading, rPose_tilt, landmarks):
        '''
        (odo_x, odo_y) is utm position for odometry-based motion model
        heading is the robot heading theta from x-axis of the map.
        (rPose_tx, rpose_ty) is a tralation of the relative pose between query and database images on query camera coodinate.
        landmarks are np.array([[x,y]]) of best matched top-N
        '''
        ## Given, robot movement
        #print("*********************************VPS_MCL : odo_x, odo_y, odo_theta : {}, {}, {}".format(odo_x, odo_y, heading)) # debug
        self.odo_x = odo_x
        self.odo_y = odo_y
        self.odo_heading = odo_heading

        x, y = odo_x, odo_y
        center=np.array([[x,y]])

        ## Given Landmarks, (x,y) of top-1 database from VPS
        self.landmarks = landmarks
        NL = len(landmarks)

        if self.previous_x > 0:
            distance=np.linalg.norm(np.array([[self.previous_x, self.previous_y]])-np.array([[x, y]]) ,axis=1)
            if distance > 0.1 : # 0.1 meters
                heading=np.arctan2(np.array([y-self.previous_y]), np.array([self.previous_x-x ]))
                if heading>0:
                    heading=-(heading-np.pi)
                else:
                    heading=-(np.pi+heading)
                u=np.array([heading,distance])

                ## Move previous particle to current particles with current odometry.
                self.particles = self.predict(self.particles, u, dt=1.)

                ## Get robot's observation
                #zs = np.array([[rPose_tilt]])
                #zs = (np.linalg.norm(landmarks - center, axis=1) + (np.random.randn(NL) * sensor_std_err))
                zs = (np.linalg.norm(landmarks - center, axis=1) + np.random.normal(self.sensor_vps_err_mu, self.sensor_vps_err_std, NL))

                if len(zs) > 0:
                    ## Get particle sample's observation
                    #self.weights = self.update(self.particles, self.odo_heading, self.weights, z=zs, R=self.pdf_std, landmarks=self.landmarks)  # 50, The smaller the R, the better the particles are gathered.
                    self.weights = self.update(self.particles, self.weights, z=zs, R=self.pdf_std, landmarks=landmarks)  # 50, The smaller the R, the better the particles are gathered.

                    ## Re-sampling
                    indexes = self.systematic_resample(self.weights)
                    self.particles, self.weights = self.resample_from_index(self.particles, self.weights, indexes)

                    self.draw_result()

        self.previous_x = x
        self.previous_y = y
        
    def create_uniform_particles(self):
        self.particles = np.empty((self.n_particle, 2))
        self.particles[:, 0] = uniform(self.x_range[0], self.x_range[1], size=self.n_particle)
        self.particles[:, 1] = uniform(self.y_range[0], self.y_range[1], size=self.n_particle)
        return self.particles
    
    def predict(self, particles, u, dt=1.):
        ''' Move particles by current odometry difference
            [odo_x, odo_y] = u
            https://numpy.org/doc/stable/reference/random/generated/numpy.random.normal.html
        '''
        N = len(particles)
        #dist = (u[1] * dt) + (np.random.randn(N) * std[1])
        dist = (u[1] * dt) + np.random.normal(self.motion_err_mu, self.motion_err_std, N)
        particles[:, 0] += np.cos(u[0]) * dist
        particles[:, 1] += np.sin(u[0]) * dist
        #particles[:, 0] += u[0] + np.random.normal(self.motion_err_mu, self.motion_err_std, N)  # Gaussian Normal Distribution
        #particles[:, 1] += u[1] + np.random.normal(self.motion_err_mu, self.motion_err_std, N)  # Gaussian Normal Distribution
        return particles

    def update_single_landmark(self, particles, heading, weights, z, R, landmark):
        vec1 = self.get_heading_vector(particles, heading)
        vec2 = landmark - particles
        phi2 = np.arccos(self.cosine_sim(vec1, vec2))
        weights *= scipy.stats.norm.pdf(x=z, loc=phi2, scale=R)  # probability density function for Gaussian distribution with loc means ans scale std value
        return weights
    
    def update_new(self, particles, heading, weights, z, R, landmarks):
        weights.fill(1.)
        if landmarks is not None:
            for i, landmark in enumerate(landmarks):
                weights = self.update_single_landmark(particles, heading, weights, z[i], R, landmark)
    
        weights += 1.e-300 # avoid round-off to zero
        weights /= sum(weights)
        return weights

    def check_valid_landmark(self, landmark):
        valid = False
        if np.sum(landmark - [-1, -1]) > 0:
            valid = True
        return valid

    def update(self, particles, weights, z, R, landmarks):
        weights.fill(1.)
        for i, landmark in enumerate(landmarks):
            if self.check_valid_landmark(landmark) == True:
                distance=np.power((particles[:,0] - landmark[0])**2 +(particles[:,1] - landmark[1])**2,0.5)
                #weights *= scipy.stats.norm(distance, R).pdf(z[i])  # Gaussian distribution : Norm(dist, sigma^2).pdf(input)
                weights *= scipy.stats.norm.pdf(x=z[i], loc=distance, scale=R)  # probability density function for Gaussian distribution with loc means ans scale sigma value
        weights += 1.e-300 # avoid round-off to zero
        weights /= sum(weights)
        return weights
    
    def systematic_resample(self, weights):
        N = len(weights)
        positions = (np.arange(N) + np.random.random()) / N
    
        indexes = np.zeros(N, 'i')  # 'i' means dtype=int32
        cumulative_sum = np.cumsum(weights)
        #self.draw_weight(weights, positions)  # for debug
        i, j = 0, 0
        while i < N and j<N:
            if positions[i] < cumulative_sum[j]:
                indexes[i] = j
                i += 1
            else:
                j += 1
        return indexes
    
    def estimate(self, particles, weights):
        pos = particles[:, 0:1]
        mean = np.average(pos, weights=weights, axis=0)
        var = np.average((pos - mean)**2, weights=weights, axis=0)
        return mean, var
    
    def resample_from_index(self, particles, weights, indexes):
        particles[:] = particles[indexes]
        weights[:] = weights[indexes]
        weights /= np.sum(weights)
        return particles, weights

    def refresh_img(self):
        self.mMap.refresh_img()

    def get_img(self):
        return self.mMap.get_img()

    def set_img(self, img):
        return self.mMap.set_img(img)

    def draw_result(self):
        ## Get clean map image
        self.refresh_img()

        ## Draw legened
        self.draw_legend()

        ## Draw landmarks (vps)
        if self.landmarks is not None:
            self.mMap.draw_points_on_map(xys=self.landmarks, incoord=self.incoord, radius=3, color=self.color_landmark, thickness=1)  # BGR

        ## Draw particles
        self.mMap.draw_points_on_map(xys=self.particles, incoord="img", radius=3, color=self.color_particle, thickness=1)  # BGR

        ## Draw trajectory
        self.draw_trajectory()

        #self.drawCross(self.img, self.robot_pos_gt, r=255, g=0, b=0)

        cv2.imshow(self.WINDOW_NAME, self.get_img())
        if cv2.waitKey(self.DELAY_MSEC) & 0xFF == 27: 
            cv2.destroyAllWindows()

    def update_trajectory(self, x, y, trajectory, incoord):
        if "img" in incoord:
            img_x, img_y = x, y
        else:
            img_x, img_y = self.mMap.get_img_coord(xy=(x, y), incoord=incoord)
        trajectory = np.vstack((trajectory, np.array([img_x, img_y])))
        return trajectory

    def draw_trajectory(self):
        img = self.get_img()

        self.trajectory_odo = self.update_trajectory(self.odo_x, self.odo_y, self.trajectory_odo, "utm")

        if self.landmarks is not None:
            landmark = self.landmarks[0]
            if self.check_valid_landmark(landmark) == True:
                self.trajectory_vps = self.update_trajectory(landmark[0], landmark[1], self.trajectory_vps, "utm")

        self.drawLines(img, self.trajectory_odo, self.color_trajectory_odo)
        self.drawLines(img, self.trajectory_vps, self.color_trajectory_vps)

        if False:
            self.trajectory_mcl = self.update_trajectory(self.odo_utm_x, self.odo_utm_y, self.trajectory_mcl, "utm")
            self.drawLines(img, self.trajectory_mcl, self.color_trajectory_mcl)
        self.set_img(img)

    def draw_legend(self):
        img = self.get_img()
        cv2.circle(img, (10,10), 10, self.color_landmark, -1)
        cv2.putText(img, "Landmarks", (30,20), 1, 1.0, self.color_landmark)

        cv2.circle(img, (10,30), 3, self.color_particle,-1)
        cv2.putText(img, "Particles", (30,40), 1,1.0, self.color_particle)
        cv2.putText(img, "Odometry", (30,60), 1, 1.0, self.color_trajectory_odo)
        cv2.putText(img, "VPR", (30,80), 1, 1.0, self.color_trajectory_vps)
        cv2.putText(img, "MCL", (30,100), 1, 1.0, self.color_trajectory_mcl)
        #self.drawLines(img, np.array([[10,55],[25,55]]), self.color_trajectory)
        self.set_img(img)

    def get_robot_observation(self, rPose_tx, rPose_ty):
        phi = np.arctan2(rPose_tx, rPose_ty)  # angle from robot heading to landmark(top-N) in CW direction : tan^-1(dx/dy)
        u = np.array([phi])
        # print("x,y, heading(deg), distance = {}, {}, {}, {}".format(x,y, np.rad2deg(heading), distance))  # debugging
        return u

    def get_heading_vector(self, pos, heading, delta_x=1):
        ## Heading Vector : Make a vecter on a line passing through the robot position with the slope of the heading angle from the X-axis
        pos1 = pos # start point. each particle point
        pos2 = np.array([[pos1[0][0] + delta_x, pos1[0][1] + delta_x*np.tan(heading)]])  # end point
        heading_vector = pos2 - pos1
        return heading_vector

    def cosine_sim(self, vec1, vec2):
        ## Calculate the angle between two vectors. theta = cos^(-1)((vec1 . vec2)/(|vec1||vec2|)
        cosine = np.inner(vec1,vec2)/(np.linalg.norm(vec1,axis=1)*np.linalg.norm(vec2,axis=1))
        cosine = cosine[0]
        theta = np.arccos(cosine)  # array([[2.69308044, 1.16742907, 1.64356718, 1.33917604, 0.3524596 ]])
        #zs = theta  # array([1.00941928, 1.87394168, 1.85178042, 2.45739235, 0.69551047])
        zs = cosine  # array([1.00941928, 1.87394168, 1.85178042, 2.45739235, 0.69551047])
        ratio = self.sensor_std_err
        noise_ratio = (0.5-np.random.random(len(zs))) * ratio # (-0.5 ~ 0.5) * self.sensor_std_err
        #print("[zs] original, with noise : {} ==> {} with noise {}".format(zs, zs + zs*noise_ratio, zs*noise_ratio))
        zs = zs + zs*noise_ratio
        return zs

    def observation_angle(self):  # Angle metric between me (robot) and landmarks. Up-to-scale tranlation angle to heading in CCW direction.
        ## Vector 1 : Make a vecter 1 on a line passing through the robot position with the slope of the heading angle from the X-axis
        vec1 = self.get_vector1()
        ## Vector 2
        vec2 = self.landmarks - self.robot_pos_gt
        zs = self.cosine_sim(vec1, vec2)
        return zs

    def get_landmarks(self):
        return [50, 50] # utm_x, utm_y

    
    def draw_pdf(self, mean=0, std=1):
        '''
        x = (x-mean)/std
        pdf(x) = np.exp(-x**2/2)/np.sqrt(2*np.pi)
    
        '''
        #xx = np.linspace(0, mean*2, 100)
        xx = np.linspace(0, 20, 100)
        plt.plot(xx, scipy.stats.norm.pdf(x=xx, loc=mean, scale=std))
        plt.draw()
        plt.pause(1)

    def neff(self, weights):
        return 1. / np.sum(np.square(weights))
    
    def draw_weight(self, weights, positions):
        plt.clf()
        plt.plot(weights, 'gray')
        plt.plot(np.cumsum(weights),'r:')
        plt.plot(positions, 'blue')
        plt.draw()
        plt.pause(0.001)

    def drawLines(self, img, points, color):  # color=(b,g,r)
        cv2.polylines(img, [np.int32(points)], isClosed=False, color=color)

    def drawCross(self, img, robot_pos_gt, r, g, b):
        d = 5
        t = 2
        LINE_AA = cv2.LINE_AA if cv2.__version__[0] in ['3', '4'] else cv2.CV_AA
        color = (r, g, b)
        ctrx = robot_pos_gt[0,0]
        ctry = robot_pos_gt[0,1]
        cv2.line(img, (ctrx - d, ctry - d), (ctrx + d, ctry + d), color, t, LINE_AA)
        cv2.line(img, (ctrx + d, ctry - d), (ctrx - d, ctry + d), color, t, LINE_AA)


if __name__ == "__main__":
    img_path = "map.png"
    cv_img = cv2.imread(img_path)
    mMCL = MCL(cv_img=cv_img, n_particle=400, pdf_std=0.1)  # angel metric
    mMCL.initialize(disp=True)
    #mMCL.run_step(odo_x, odo_y, heading, self.tilt, landmarks)

    cv2.destroyAllWindows()
