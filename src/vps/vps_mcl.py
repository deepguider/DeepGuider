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

class MCL():
    def __init__(self, img_path=None, width=800, height=600, n_particle=400, pdf_sigma=0.1):
        '''
        observation_mode is one of the ["distance", "angle"]
        pdf_sigma = 0.1 for angle metric, 10 for distance metric
        width, height are width and height of a map, respectively.
        n_particle is a number of particles
        n_landmark is a number of landmarks. If it is zero, it will be a random integer value between 1 and 10 every loop.
        landmark positions change at every landmark_update_count loop
        '''
        self.img_path = img_path
        self.img_ori = None

        if img_path is not None:
            self.img_ori = cv2.imread(img_path)

        if self.img_ori is not None:
            height, width, ch = self.img_ori.shape

        self.WIDTH = width
        self.HEIGHT = height
        self.WINDOW_NAME="Particle Filter"
    
        self.x_range=np.array([0, self.WIDTH])
        self.y_range=np.array([0, self.HEIGHT])
        
        #Number of partciles
        self.n_particle = n_particle
        self.pdf_sigma = pdf_sigma
        self.callback_count = 0
        self.sensor_std_err = 0.1

    def initialize(self, disp=False):
        self.particles = self.create_uniform_particles()
        self.weights = np.array([1.0]*self.n_particle)
        
        # Create a black image, a window and bind the function to window
        self.img = self.get_img()
        self.disp = disp

        if self.disp == True:
            cv2.namedWindow(self.WINDOW_NAME)
        
        self.trajectory = np.zeros(shape=(0,2))
        self.robot_pos = np.zeros(shape=(0,2))

        self.DELAY_MSEC = 10
        self.loop_count = 0
        return 0

    #def run_step(self, odo_x, odo_y, heading, rPose_tx, rPose_ty, landmarks):
    def run_step(self, odo_x, odo_y, heading, rPose_tilt, landmarks):
        '''
        (odo_x, odo_y) is odometry difference of the robot
        heading is the robot heading theta from x-axis of the map.
        (rPose_tx, rpose_ty) is a tralation of the relative pose between query and database images on query camera coodinate.
        landmarks are np.array([[x,y]]) of best matched top-N
        '''
        ## Given, robot movement
        #print("*********************************VPS_MCL : odo_x, odo_y, odo_theta : {}, {}, {}".format(odo_x, odo_y, heading)) # debug
        u = np.array([odo_x, odo_y])
        movement = u[0]**2 + u[1]**2
        
        ## Given Landmarks, (x,y) of top-1 database from VPS
        self.landmarks = landmarks

        if movement > 0.1 : # 0.1 meters
            ## Move previous particle to current particles with current odometry.
            self.particles = self.predict(self.particles, u, dt=1.)

            ## Get robot's observation
            zs = np.array([[rPose_tilt]])

            if len(zs) > 0:
                ## Get particle sample's observation
                self.weights = self.update(self.particles, heading, self.weights, z=zs, R=self.pdf_sigma, landmarks=self.landmarks)  # 50, The smaller the R, the better the particles are gathered.

                ## Re-sampling
                indexes = self.systematic_resample(self.weights)
                self.particles, self.weights = self.resample_from_index(self.particles, self.weights, indexes)

                self.draw_result()
        
    def create_uniform_particles(self):
        self.particles = np.empty((self.n_particle, 2))
        self.particles[:, 0] = uniform(self.x_range[0], self.x_range[1], size=self.n_particle)
        self.particles[:, 1] = uniform(self.y_range[0], self.y_range[1], size=self.n_particle)
        return self.particles
    
    def predict(self, particles, u, dt=1.):
        ''' Move particles by current odometry difference
        '''
        N = len(particles)
        [odo_x, odo_y] = u
        particles[:, 0] += u[0]  # odo_x
        particles[:, 1] += u[1]  # odo_y
        return particles

    def update_single_landmark(self, particles, heading, weights, z, R, landmark):
        vec1 = self.get_heading_vector(particles, heading)
        vec2 = landmark - particles
        phi2 = np.arccos(self.cosine_sim(vec1, vec2))
        weights *= scipy.stats.norm.pdf(x=z, loc=phi2, scale=R)  # probability density function for Gaussian distribution with loc means ans scale sigma value
        return weights
    
    def update(self, particles, heading, weights, z, R, landmarks):
        weights.fill(1.)
        for i, landmark in enumerate(landmarks):
            weights = self.update_single_landmark(particles, heading, weights, z[i], R, landmark)
    
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



    def get_img(self):
        if self.img_ori is not None:
            img = self.img_ori.copy()
        else:
            img = np.zeros((self.HEIGHT, self.WIDTH, 3), np.uint8)
        return img

    def draw_result(self):
        self.img = self.get_img()

        #self.drawLines(self.img, self.trajectory,   0,   255, 0)
        #self.drawCross(self.img, self.robot_pos_gt, r=255, g=0, b=0)

        for landmark in self.landmarks:
            x,y =  [328533., 4153478] - landmark  # debug
            if (x is not None) and (y is not None):
                if x >= 0 and y >= 0:
                    cv2.circle(self.img,tuple(landmark),10,(255,0,0),-1)

        #draw_particles:
        for particle in self.particles:
            cv2.circle(self.img,tuple((int(particle[0]),int(particle[1]))),3,(25,25,255),-1)

        cv2.circle(self.img,(10,10),10,(255,0,0),-1)
        cv2.circle(self.img,(10,30),3,(255,255,255),-1)
        cv2.putText(self.img,"Landmarks",(30,20),1,1.0,(255,0,0))
        cv2.putText(self.img,"Particles",(30,40),1,1.0,(25,25,255))
        #cv2.putText(self.img,"Robot Trajectory(Ground truth)",(30,60),1,1.0,(0,255,0))

        self.drawLines(self.img, np.array([[10,55],[25,55]]), 0, 255, 0)

        cv2.imshow(self.WINDOW_NAME, self.img)
        if cv2.waitKey(self.DELAY_MSEC) & 0xFF == 27:  # esc
            exit(0)

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

    def drawLines(self, img, points, r, g, b):
        cv2.polylines(img, [np.int32(points)], isClosed=False, color=(r, g, b))

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
    mMCL = vps_mcl.MCL(img_path=img_path, n_particle=400, pdf_sigma=0.1)  # angel metric
    mMCL.vps_mcl_initialize(disp=True)
    #mMCL.run_step(odo_x, odo_y, heading, self.tilt, landmarks)

    cv2.destroyAllWindows()
