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
from vps_filter import vps_filter

class MCL():
    def __init__(self, cv_img, ltop, rbottom, incoord="utm", map_site="dg", n_particle=400,
            motion_err_mu=0, motion_err_std=10, sensor_vps_err_mu=0, sensor_vps_err_std=7, sensor_vps_importance_pdf_std=5):
        '''
        observation_mode is one of the ["distance", "angle"]
        sensor_vps_importance_pdf_std = 0.1 for angle metric, 10 for distance metric, smaller means more important sensor
        width, height are width and height of a map, respectively.
        n_particle is a number of particles
        n_landmark is a number of landmarks. If it is zero, it will be a random integer value between 1 and 10 every loop.
        landmark positions change at every landmark_update_count loop
        '''
        # You can make empyth image with : np.zeros((self.HEIGHT, self.WIDTH, 3), np.uint8)
        self.height, self.width, ch = cv_img.shape
        self.incoord = incoord

        self.mMap = DispMap(cv_img, ltop, rbottom, incoord)
        self.map_site = map_site

        self.WINDOW_NAME="Particle Filter"
    
        self.x_range=np.array([ltop[0], rbottom[0]])
        self.y_range=np.array([rbottom[1], ltop[1]])
        #self.x_range=np.array([0, self.width])
        #self.y_range=np.array([0, self.height])
        
        #MCL parameters
        self.n_particle = n_particle
        self.sensor_vps_importance_pdf_std = sensor_vps_importance_pdf_std  # Low value : heigh weight for particle with excatly same observation, High std : relative uniform weight for all particle
        self.callback_count = 0
        self.sensor_std_err = 5  # 0.1 for cosine sim
        self.motion_err_mu = motion_err_mu  # 0
        self.motion_err_std = motion_err_std  # Low std for accurate encoder, High std for noisy encoder
        self.sensor_vps_err_mu = sensor_vps_err_mu  #0
        self.sensor_vps_err_std = sensor_vps_err_std  # Low std for an accurate sensor(vpr results), High std for a noisy vps

        # Color (B,G,R)
        self.color_landmark = (128,0,0)
        self.color_particle = (0,255,0)
        self.color_trajectory_odo = (0,0,255)
        self.color_trajectory_vps = (0,0,0)  # black
        self.color_trajectory_mcl = (255,0,0)
        self.color_robot_pred = (255,0,0)
        self.color_green = (0,255,0)

        # Crop image to save result into avi
        
    def crop(self, img):
        return self.crop_img(img, x=self.avi_x0, y=self.avi_y0, w=self.avi_w, h=self.avi_h)

    def crop_img(self, img, x=0, y=0, w=0, h=0):
        if w == 0:
            w = self.width
        if h == 0:
            h = self.height
        x1 = min(x + w, self.width)
        y1 = min(y + h, self.height)
        crop = img[y:y1, x:x1]
        return crop

    def initialize(self, disp=False):
        self.particles = self.create_uniform_particles()
        self.weights = np.array([1.0]*self.n_particle)

        self.previous_x = -1
        self.previous_y = -1
        self.previous_timestamp = -1 # timestamp

        self.previous_lm_x = -1  # landmark 
        self.previous_lm_y = -1  # landmark 
        
        # Create a black image, a window and bind the function to window
        self.disp = disp

        if self.disp == True:
            cv2.namedWindow(self.WINDOW_NAME)

        self.mcl_pose = [-1, -1]
        
        self.trajectory_odo = np.zeros(shape=(0,2))
        self.trajectory_vps = np.zeros(shape=(0,2))
        self.trajectory_mcl = np.zeros(shape=(0,2))
        self.robot_pos = np.zeros(shape=(0,2))

        self.draw_result_count = 0
        self.DELAY_MSEC = 10
        self.loop_count = 0

        self.mSimple_filter = vps_filter(5, mode="mean") 

        self.write_result_init()

        return 0

    #def run_step(self, odo_x, odo_y, heading, rPose_tx, rPose_ty, landmarks):
    def run_step(self, odo_x, odo_y, odo_heading, rPose_tilt, landmarks, timestamp):
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
        self.timestamp = timestamp

        x, y = odo_x, odo_y
        center=np.array([[x,y]])

        ## Given Landmarks, (x,y) of top-1 database from VPS
        self.landmarks = landmarks
        NL = len(landmarks)

        mcl_pose_valid = False

        if self.previous_x > 0:
            dt = self.timestamp - self.previous_timestamp
            distance = np.linalg.norm(np.array([[self.previous_x, self.previous_y]])-np.array([[x, y]]) ,axis=1)
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

                #zs = (np.linalg.norm(landmarks - center, axis=1) + np.random.normal(self.sensor_vps_err_mu, self.sensor_vps_err_std, NL))
                ## Landmark is postion
                zs = np.random.normal(self.sensor_vps_err_mu, self.sensor_vps_err_std, NL)

                if len(zs) > 0:
                    ## Get particle sample's observation
                    #self.weights = self.update(self.particles, self.odo_heading, self.weights, z=zs, R=self.sensor_vps_importance_pdf_std, landmarks=self.landmarks)  # 50, The smaller the R, the better the particles are gathered.
                    self.weights = self.update(self.particles, self.weights, z=zs, R=self.sensor_vps_importance_pdf_std, landmarks=landmarks)  # 50, The smaller the R, the better the particles are gathered.

                    ## Re-sampling
                    indexes = self.systematic_resample(self.weights)
                    self.particles, self.weights = self.resample_from_index(self.particles, self.weights, indexes)

                    ## Get mean position by averaging particles position
                    mcl_pose = self.averaging_particle_position(self.particles, self.weights)
                    mcl_time_pose, mcl_time_std = self.mSimple_filter.get_mean(mcl_pose[0], mcl_pose[1])
                    self.mcl_pose = mcl_time_pose

                    self.draw_result()

        self.previous_x = x
        self.previous_y = y
        self.previous_timestamp = self.timestamp
        if self.get_particles_std() < self.initial_particles_std*0.3:
            mcl_pose_valid = True

        return self.get_mcl_pose(), mcl_pose_valid

    def get_mcl_pose(self):
        return self.mcl_pose

    def get_particles_std(self):
        std = np.linalg.norm(np.std(self.particles, axis=0))
        return std

    def averaging_particle_position(self, particles, weights):
        w = np.tile(weights, (2,1)).T  # w : (400, 2), particles : (400,2)
        mcl_pose = np.sum(w*particles, axis=0)  # [x, y] in utm coord.
        return mcl_pose
        
    def create_uniform_particles(self):
        self.particles = np.empty((self.n_particle, 2))
        self.particles[:, 0] = uniform(self.x_range[0], self.x_range[1], size=self.n_particle)
        self.particles[:, 1] = uniform(self.y_range[0], self.y_range[1], size=self.n_particle)
        self.initial_particles_std = self.get_particles_std()
        return self.particles
    
    def predict(self, particles, u, dt=1.):
        ''' Move particles by current odometry difference
            [odo_x, odo_y] = u
            https://numpy.org/doc/stable/reference/random/generated/numpy.random.normal.html
        '''
        N = len(particles)
        #dist = (u[1] * dt) + (np.random.randn(N) * std[1])
        if True:  # Add Noise in heading direction
            dist = (u[1] * dt) + np.random.normal(self.motion_err_mu, self.motion_err_std, N)
            heading = u[0]+np.random.normal(0, np.deg2rad(10), N)
            cos = np.cos(heading)
            sin = np.sin(heading)
            particles[:, 0] += cos * dist
            particles[:, 1] += sin * dist
        else: # Add Noise in 2D space
            dist = (u[1] * dt)
            particles[:, 0] += np.cos(u[0]) * dist + np.random.normal(self.motion_err_mu, self.motion_err_std, N)
            particles[:, 1] += np.sin(u[0]) * dist + np.random.normal(self.motion_err_mu, self.motion_err_std, N)
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
                distance = np.linalg.norm(landmarks - particles, axis=1)
                #distance=np.power((particles[:,0] - landmark[0])**2 +(particles[:,1] - landmark[1])**2,0.5)
                #weights *= scipy.stats.norm(distance, R).pdf(z[i])  # Gaussian distribution : Norm(dist, sigma^2).pdf(input)
                weights *= scipy.stats.norm.pdf(x=z[i], loc=distance, scale=R)  # probability density function for Gaussian distribution with loc means and scale is sigma value
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
        self.draw_result_count += 1

        ## Draw legened
        self.draw_legend()

        ## Draw particles
        self.mMap.draw_points_on_map(xys=self.particles, incoord=self.incoord, radius=1, color=self.color_particle, thickness=-1)  # BGR

        ## Draw landmarks (vps)
        if self.landmarks is not None:
            self.mMap.draw_points_on_map(xys=self.landmarks, incoord=self.incoord, radius=3, color=self.color_landmark, thickness=2)  # BGR

        ## Draw trajectory
        if self.draw_result_count > 40:  # It's noisy before stablizing
            self.draw_trajectory()

        ## Draw predicted robot position
        self.mMap.draw_point_on_map(xy=self.mcl_pose, incoord=self.incoord, radius=10, color=self.color_robot_pred, thickness=1)  # BGR

        ## Show result image
        cv2.imshow(self.WINDOW_NAME, self.get_img())
        self.fp_avi.write(self.crop(self.get_img()))
        if cv2.waitKey(self.DELAY_MSEC) & 0xFF == 27: 
            self.write_result_close()
            cv2.destroyAllWindows()

    def update_trajectory_img_coord(self, x, y, trajectory, incoord):  # image coordinate trajectory
        if "img" in incoord:
            img_x, img_y = x, y
        else:
            img_x, img_y = self.mMap.get_img_coord(xy=(x, y), incoord=incoord)
        trajectory = np.vstack((trajectory, np.array([img_x, img_y])))
        return trajectory, img_x, img_y

    def update_trajectory(self, x, y, trajectory, incoord):  # utm coordinate trajectory
        if "img" in incoord:
            utm_x, utm_y = self.mMap.get_map_coord(xy=(x, y), outcoord="utm")
        else:
            utm_x, utm_y = x, y
        trajectory = np.vstack((trajectory, np.array([utm_x, utm_y])))
        return trajectory, utm_x, utm_y

    def write_result_init(self):
        ## Write trajectory into csv
        timestr = time.strftime("%Y%m%d_%H%M%S")
        prefix = "vps_mcl_{}_mostd{}_sestd{}_pdfstd{}".format(self.map_site, timestr, self.motion_err_std, self.sensor_vps_err_std, self.sensor_vps_importance_pdf_std)

        fname_odo = "{}_trajectory_odo.csv".format(prefix)
        fname_vps = "{}_trajectory_vps.csv".format(prefix)
        fname_mcl = "{}_trajectory_mcl.csv".format(prefix)
        self.fp_odo = open(fname_odo, "w", newline="")
        self.fp_vps = open(fname_vps, "w", newline="")
        self.fp_mcl = open(fname_mcl, "w", newline="")
        first_line = "Number,img_x,img_y,utm_x,utm_y\n"
        self.fp_odo.write(first_line)
        self.fp_vps.write(first_line)
        self.fp_mcl.write(first_line)
        self.write_idx = 0

        ## Write result map image into avi
        fname_avi = "{}.avi".format(prefix)
        fps = 15

        #self.avi_w = 1280
        #self.avi_h = 720
        self.avi_w = 640
        self.avi_h = 480

        if "bucheon" in self.map_site.lower():
            self.avi_x0 = 200
            self.avi_y0 = 50
            self.avi_x1 = self.avi_x0 + self.avi_w
            self.avi_y1 = self.avi_y0 + self.avi_h

        if "coex" in self.map_site.lower():
            self.avi_x0 = 500
            self.avi_y0 = 300
            self.avi_x1 = self.avi_x0 + self.avi_w
            self.avi_y1 = self.avi_y0 + self.avi_h

        self.fp_avi = cv2.VideoWriter(fname_avi, 0x7634706d, fps, (self.avi_w, self.avi_h))  # write as mp4

        if not self.fp_avi.isOpened():
            print('{} File open failed!'.format(fname_avi))

    def write_result_close(self):
        self.fp_odo.close()
        self.fp_vps.close()
        self.fp_mcl.close()
        self.fp_avi.close()

    def draw_trajectory(self):
        img = self.get_img()

        ## Draw and Write odometry position trajectory
        self.trajectory_odo, self.odo_img_x, self.odo_img_y = self.update_trajectory_img_coord(self.odo_x, self.odo_y, self.trajectory_odo, "utm")
        self.drawLines(img, self.trajectory_odo, self.color_trajectory_odo)
        line_odo = "{0:06d},{1:},{2:},{3:},{4:}\n".format(self.write_idx, self.odo_img_x, self.odo_img_y, self.odo_x, self.odo_y)

        ## Draw and Write landmarks (vps results) position trajectory
        if self.landmarks is not None:
            landmark = self.landmarks[0]
            if self.check_valid_landmark(landmark) == True:
                self.trajectory_vps, self.vps_img_x, self.vps_img_y = self.update_trajectory_img_coord(landmark[0], landmark[1], self.trajectory_vps, "utm")
            else:
                self.vps_img_x, self.vps_img_y = -1, -1
        if True:
            self.drawLines(img, self.trajectory_vps, self.color_trajectory_vps)  # Do not display vps results
        line_vps = "{0:06d},{1:},{2:},{3:},{4:}\n".format(self.write_idx, self.vps_img_x, self.vps_img_y, self.landmarks[0][0], self.landmarks[0][1] )

        ## Draw and Write mcl position trajectory
        self.trajectory_mcl, self.mcl_img_x, self.mcl_img_y = self.update_trajectory_img_coord(self.mcl_pose[0], self.mcl_pose[1], self.trajectory_mcl, "utm")
        if True: # display line for mcl result
            self.drawLines(img, self.trajectory_mcl, self.color_trajectory_mcl)
        else:  # display points for mcl result
            self.mMap.draw_points_on_map(xys=self.trajectory_mcl, incoord="img", radius=1, color=self.color_trajectory_mcl, thickness=-1)  # BGR
        line_mcl = "{0:06d},{1:},{2:},{3:},{4:}\n".format(self.write_idx, self.mcl_img_x, self.mcl_img_y, self.mcl_pose[0], self.mcl_pose[1])

        self.fp_odo.write(line_odo)
        self.fp_vps.write(line_vps)
        self.fp_mcl.write(line_mcl)
        self.write_idx += 1
        #self.drawCross(img, self.rebot_pos_pred, r=255, g=0, b=0)
        self.set_img(img)

    def draw_legend(self):
        img = self.get_img()
        font = cv2.FONT_HERSHEY_DUPLEX
        fontScale = 1.0
        cv2.circle(img,               (10, 10), 10, self.color_landmark, -1)
        cv2.putText(img, "Landmarks", (30, 30), font, fontScale, self.color_landmark)
        cv2.circle(img,               (10, 40), 3, self.color_particle,-1)
        cv2.putText(img, "Particles", (30, 60), font, fontScale, self.color_particle)
        cv2.putText(img, "Odometry",  (30, 90), font, fontScale, self.color_trajectory_odo)
        cv2.putText(img, "VPR",       (30, 120), font, fontScale, self.color_trajectory_vps)
        cv2.putText(img, "MCL",       (30, 150), font, fontScale, self.color_trajectory_mcl)
        cv2.putText(img, "Step : {}".format(self.draw_result_count), (30,180), font, fontScale, self.color_green)
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
        vec2 = self.landmarks - self.rebot_pos_pred
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

    def drawCross(self, img, rebot_pos_pred, r, g, b):
        d = 5
        t = 2
        LINE_AA = cv2.LINE_AA if cv2.__version__[0] in ['3', '4'] else cv2.CV_AA
        color = (r, g, b)
        ctrx = rebot_pos_pred[0]
        ctry = rebot_pos_pred[1]
        cv2.line(img, (ctrx - d, ctry - d), (ctrx + d, ctry + d), color, t, LINE_AA)
        cv2.line(img, (ctrx + d, ctry - d), (ctrx - d, ctry + d), color, t, LINE_AA)


if __name__ == "__main__":
    img_path = "map.png"
    cv_img = cv2.imread(img_path)
    mMCL = MCL(cv_img=cv_img, n_particle=400, sensor_vps_importance_pdf_std=0.1)  # angel metric
    mMCL.initialize(disp=True)
    #mMCL.run_step(odo_x, odo_y, heading, self.tilt, landmarks)

    cv2.destroyAllWindows()
