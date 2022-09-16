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
    def __init__(self, observation_mode=0, img_path=None, width=800, height=600, n_particle=400, n_landmark=0, pdf_sigma=10, landmark_update_count=1, noise_std=5, manualPath=True, manualPath_rate=0.1, noise=False):
        '''
        observation_mode is one of the ["distance", "angle"]
        pdf_sigma = 0.1 for angle metric, 10 for distance metric
        width, height are width and height of a map, respectively.
        n_particle is a number of particles
        n_landmark is a number of landmarks. If it is zero, it will be a random integer value between 1 and 10 every loop.
        landmark positions change at every landmark_update_count loop
        '''
        self.MODE = ["distance", "angle"]
        self.observation_mode = observation_mode
        self.img_path = img_path
        self.img_ori = None

        self.manualPath = manualPath
        self.manualPath_rate = manualPath_rate  # second

        if img_path is not None:
            self.img_ori = cv2.imread(img_path)

        if self.img_ori is not None:
            height, width, ch = self.img_ori.shape

        self.WIDTH = width
        self.HEIGHT = height
        self.WINDOW_NAME="Particle Filter"
    
        #sensorMu=0
        #sensorSigma=3
    
        self.sensor_std_err = noise_std  # ori 5
        self.sensor_std_err_manual = 0.0
        self.noise = noise

        self.x_range=np.array([0, self.WIDTH])
        self.y_range=np.array([0, self.HEIGHT])
        
        #Number of partciles
        self.n_particle = n_particle
        self.n_landmark = n_landmark
        self.landmark_update_count = landmark_update_count
        self.pdf_sigma = pdf_sigma
        self.callback_count = 0

    def get_img(self):
        if self.img_ori is not None:
            img = self.img_ori.copy()
        else:
            img = np.zeros((self.HEIGHT, self.WIDTH, 3), np.uint8)
        return img

    def vps_mcl_initialize(self, landmark_radius=50, disp=False):
        self.particles = self.create_uniform_particles()
        self.weights = np.array([1.0]*self.n_particle)
        
        # Create a black image, a window and bind the function to window
        self.img = self.get_img()
        self.disp = disp

        if self.disp == True:
            cv2.namedWindow(self.WINDOW_NAME)
            cv2.setMouseCallback(self.WINDOW_NAME, self.mouseCallback)
        
        self.trajectory = np.zeros(shape=(0,2))
        self.robot_pos = np.zeros(shape=(0,2))
        self.previous_x = -1
        self.previous_y = -1
        ## Following postions will be updated in mouse callback function.
        self.robot_pos_gt = np.array([[self.previous_x, self.previous_y]])  # Center of True Robot position (mouse)
        self.robot_prev_pos_gt = np.array([[self.previous_x, self.previous_y]])  # Center of True Robot previous position (mouse)

        self.DELAY_MSEC = 10
        self.loop_count = 0
        self.landmark_radius= landmark_radius
        return 0

    def vps_mcl_draw_result(self):
        self.img = self.get_img()

        self.drawLines(self.img, self.trajectory,   0,   255, 0)
        self.drawCross(self.img, self.robot_pos_gt, r=255, g=0, b=0)

        for landmark in self.landmarks:
            cv2.circle(self.img,tuple(landmark),10,(255,0,0),-1)

        #draw_particles:
        for particle in self.particles:
            cv2.circle(self.img,tuple((int(particle[0]),int(particle[1]))),3,(25,25,255),-1)

        cv2.circle(self.img,(10,10),10,(255,0,0),-1)
        cv2.circle(self.img,(10,30),3,(255,255,255),-1)
        cv2.putText(self.img,"Landmarks",(30,20),1,1.0,(255,0,0))
        cv2.putText(self.img,"Particles",(30,40),1,1.0,(25,25,255))
        cv2.putText(self.img,"Robot Trajectory(Ground truth)",(30,60),1,1.0,(0,255,0))

        self.drawLines(self.img, np.array([[10,55],[25,55]]), 0, 255, 0)

        cv2.imshow(self.WINDOW_NAME, self.img)
        if cv2.waitKey(self.DELAY_MSEC) & 0xFF == 27:  # esc
            exit(0)
#        elif cv2.waitKey(self.DELAY_MSEC) & 0xFF == 32: # space bar
#             cv2.waitKey(0)
        elif cv2.waitKey(self.DELAY_MSEC) & 0xFF == ord("n"):
            self.sensor_std_err_manual = self.sensor_std_err_manual + 0.1
            print("self.sensor_std_err_manual : {}".format(self.sensor_std_err_manual))
        elif cv2.waitKey(self.DELAY_MSEC) & 0xFF == ord("b"):
            self.sensor_std_err_manual = self.sensor_std_err_manual - 0.1
            print("self.sensor_std_err_manual : {}".format(self.sensor_std_err_manual))

    def vps_mcl_apply(self, x, y, landmarks):
        ## set current landmarks
        self.landmarks = landmarks

        ## run one step of MCL
        self.manualCallback(x, y)  # This is extra option. main is mousecallback
    
        if self.disp == True:
            self.vps_mcl_draw_result()
    
        self.loop_count += 1
        if self.loop_count >= 1e9:
            self.loop_count = 0

    def run(self):
        self.particles = self.create_uniform_particles()
        self.weights = np.array([1.0]*self.n_particle)
        
        # Create a black image, a window and bind the function to window
        img = self.get_img()

        cv2.namedWindow(self.WINDOW_NAME)
        cv2.setMouseCallback(self.WINDOW_NAME, self.mouseCallback)
        
        self.trajectory = np.zeros(shape=(0,2))
        self.robot_pos = np.zeros(shape=(0,2))
        self.previous_x = -1
        self.previous_y = -1
        ## Following postions will be updated in mouse callback function.
        self.robot_pos_gt = np.array([[self.previous_x, self.previous_y]])  # Center of True Robot position (mouse)
        self.robot_prev_pos_gt = np.array([[self.previous_x, self.previous_y]])  # Center of True Robot previous position (mouse)

        DELAY_MSEC = 50
        self.loop_count = 0
        self.landmarks = self.get_landmarks(self.n_landmark, 50)
        stime = time.time()
        [x, y] = self.get_trajectory()[self.callback_count]
        while(1):
            if (self.manualPath==True) and (time.time() - stime >  self.manualPath_rate) : #sec
                if self.callback_count < len(self.get_trajectory()):
                    [x, y] = self.get_trajectory()[self.callback_count]
                    self.manualCallback(x, y)  # This is extra option. main is mousecallback
                    self.callback_count = self.callback_count + 1
                else:  # reset
                    self.callback_count = 0
                    self.trajectory = np.zeros(shape=(0,2))
                stime = time.time()
            cv2.imshow(self.WINDOW_NAME,img)
            img = self.get_img()
            self.drawLines(img, self.trajectory,   0,   255, 0)
            self.drawCross(img, self.robot_pos_gt, r=255, g=0, b=0)
        
            #landmarks
            if (self.loop_count % self.landmark_update_count) == 0:  # Change landmark at every self.landmark_update_count loop
                self.landmarks = self.get_landmarks(self.n_landmark, 50)
        
            for landmark in self.landmarks:
                cv2.circle(img,tuple(landmark),10,(255,0,0),-1)
        
            #draw_particles:
            for particle in self.particles:
                cv2.circle(img,tuple((int(particle[0]),int(particle[1]))),3,(25,25,255),-1)
        
            if cv2.waitKey(DELAY_MSEC) & 0xFF == 27:
                break
        
            cv2.circle(img,(10,10),10,(255,0,0),-1)
            cv2.circle(img,(10,30),3,(255,255,255),-1)
            cv2.putText(img,"Landmarks",(30,20),1,1.0,(255,0,0))
            cv2.putText(img,"Particles",(30,40),1,1.0,(255,255,255))
            cv2.putText(img,"Robot Trajectory(Ground truth)",(30,60),1,1.0,(0,255,0))
        
            self.drawLines(img, np.array([[10,55],[25,55]]), 0, 255, 0)
        
            self.loop_count += 1
            if self.loop_count >= 1e9:
                self.loop_count = 0

    def get_odometry_ori(self, x, y):
        '''
                 -90
                  ^
                  |
                  |
        -180 <---------> 0   heading(deg)
                  |
                  |
                 +90

        '''
        heading=np.arctan2(np.array([y-self.previous_y]), np.array([self.previous_x-x ]))  # tan^-1(dy/dx)
        if heading>0:
            heading=-(heading-np.pi)
        else:
            heading=-(np.pi+heading)
        distance=np.linalg.norm(np.array([[self.previous_x, self.previous_y]])-np.array([[x,y]]) ,axis=1)
        u = np.array([heading,distance])
        print("x,y, heading, distance = {}, {}, {}, {}".format(x,y, np.rad2deg(u[0]), u[1]))  # debugging
        return u

    def get_odometry(self, x, y):
        '''
                 -90  : particle_y' = partivle_y*sin(heading) : upper is decreasing direction in y-coord.
                  ^
                  |
                  |
        -180 <---------> 0  heading(deg) : particle_x' = particle_x*cos(heading) : right is increasing diretion. Left is descrising direction.
                  |
                  |
                 +90  : lower is increading direction in y-coord.

                 heading is used in predict()

        '''
        dx = (x - self.previous_x)
        dy = (y - self.previous_y)
        heading = np.arctan2(dy, dx)  # tan^-1(dy/dx)
        distance=np.linalg.norm(np.array([[self.previous_x, self.previous_y]])-np.array([[x,y]]) ,axis=1)[0]
        u = np.array([heading, distance])
        # print("x,y, heading(deg), distance = {}, {}, {}, {}".format(x,y, np.rad2deg(heading), distance))  # debugging
        return u

    def get_vector1(self, pos1=None, heading=None, delta_x=1):
        ## Vector 1 : Make a vecter 1 on a line passing through the robot position with the slope of the heading angle from the X-axis
        if True:
            vec1 = self.robot_pos_gt - self.robot_prev_pos_gt  # Easy but we can not use this for particle points.
        else:
            if heading is None:
                heading = self.odo[0]
            if pos1 is None:
                pos1 = self.robot_pos_gt
            pos2 = np.array([[pos1[0][0] + delta_x, pos1[0][1] + delta_x*np.tan(heading)]])
            vec1 = pos2 - pos1
        return vec1

    def cosine_sim(self, vec1, vec2):
        ## Calculate the angle between two vectors. theta = cos^(-1)((vec1 . vec2)/(|vec1||vec2|)
        cosine = np.inner(vec1,vec2)/(np.linalg.norm(vec1,axis=1)*np.linalg.norm(vec2,axis=1))
        cosine = cosine[0]
        theta = np.arccos(cosine)  # array([[2.69308044, 1.16742907, 1.64356718, 1.33917604, 0.3524596 ]])
        #zs = theta  # array([1.00941928, 1.87394168, 1.85178042, 2.45739235, 0.69551047])
        zs = cosine  # array([1.00941928, 1.87394168, 1.85178042, 2.45739235, 0.69551047])
        if self.noise == True:
            ratio = self.sensor_std_err + self.sensor_std_err_manual
            noise_ratio = (0.5-np.random.random(len(zs))) * ratio # (-0.5 ~ 0.5) * self.sensor_std_err
            #print("[zs] original, with noise : {} ==> {} with noise {}".format(zs, zs + zs*noise_ratio, zs*noise_ratio))
            zs = zs + zs*noise_ratio
        return zs

    def observation_distance(self):  # distance metric between me (robot) and landmarks
        distance = np.linalg.norm(self.landmarks - self.robot_pos_gt, axis=1)
        zs = distance
        if self.noise == True:
            sigma = self.sensor_std_err
            zs = zs + np.random.randn(len(distance)) * sigma  # distance is mean of gaussian distribution
        return zs

    def observation_angle(self):  # Angle metric between me (robot) and landmarks. Up-to-scale tranlation angle to heading in CCW direction.
        ## Vector 1 : Make a vecter 1 on a line passing through the robot position with the slope of the heading angle from the X-axis
        vec1 = self.get_vector1()
        ## Vector 2
        vec2 = self.landmarks - self.robot_pos_gt
        zs = self.cosine_sim(vec1, vec2)
        return zs

    def get_observation(self):  # Normal distribution, N(mu, sig^2), { 1/(sigma*sqrt(2*pi)) } * exp(-0.5 * ((x-mu)/sig)^2)
        if self.observation_mode == 0:
            zs = self.observation_distance()
        elif self.observation_mode == 1:
            zs = self.observation_angle()
        return zs

    def manualCallback(self, x, y):
        self.robot_pos_gt=np.array([[x,y]])
        self.robot_prev_pos_gt=np.array([[self.previous_x, self.previous_y]])
        self.trajectory=np.vstack((self.trajectory,np.array([x,y])))
        #noise=sensorSigma * np.random.randn(1,2) + sensorMu
        #if self.previous_x > 0 and False:
        u = self.get_odometry(x, y)
        movement = u[0]**2 + u[1]**2
        if self.previous_x > 0 and movement > 1e-3:
            u = self.get_odometry(x, y)
            self.odo = u
            std = np.array([2,4])

            ## Move previous particle to current particles with current odometry.
            self.particles = self.predict(self.particles, u, std, dt=1.)

            ## Get observation
            zs = self.get_observation()

            if len(zs) > 0:
                self.weights = self.update(self.particles, self.weights, z=zs, R=self.pdf_sigma, landmarks=self.landmarks)  # 50, The smaller the R, the better the particles are gathered.
                indexes = self.systematic_resample(self.weights)
                self.particles, self.weights = self.resample_from_index(self.particles, self.weights, indexes)
        
        self.previous_x = x
        self.previous_y = y

    def mouseCallback(self, event, x, y, flags,null):
        if event == cv2.EVENT_LBUTTONDOWN:
        #if True:
            self.robot_pos_gt=np.array([[x,y]])
            self.robot_prev_pos_gt=np.array([[self.previous_x, self.previous_y]])
            self.trajectory=np.vstack((self.trajectory,np.array([x,y])))
            #noise=sensorSigma * np.random.randn(1,2) + sensorMu
            #if self.previous_x > 0 and False:
            if self.previous_x > 0:
                u = self.get_odometry(x, y)
                self.odo = u
                std = np.array([2,4])

                ## Move previous particle to current particles with current odometry.
                self.particles = self.predict(self.particles, u, std, dt=1.)

                ## Get observation
                zs = self.get_observation()

                if len(zs) > 0:
                    self.weights = self.update(self.particles, self.weights, z=zs, R=self.pdf_sigma, landmarks=self.landmarks)  # 50, The smaller the R, the better the particles are gathered.
                    indexes = self.systematic_resample(self.weights)
                    self.particles, self.weights = self.resample_from_index(self.particles, self.weights, indexes)
        
            self.previous_x = x
            self.previous_y = y

    def get_trajectory(self):
        #trajectory = mcl_config.trajectory_short
        #trajectory = mcl_config.trajectory_long
        trajectory = mcl_config.trajectory_long_with_ready
        return trajectory

    def get_landmarks(self, NL=0, radius=0, top_N=0):
        if NL >= 10000:  # Use fixed landmarks
            #landmarks=np.array([ [144,73], [410,13], [336,175], [718,159], [178,484], [665,464]  ])
            landmarks = mcl_config.landmarks
        else:
            if NL < 1:
                NL = np.random.randint(1,10)
            xs = np.random.choice(np.arange(int(self.WIDTH*0.3), int(self.WIDTH*0.7)), NL)
            ys = np.random.choice(np.arange(int(self.HEIGHT*0.3), int(self.HEIGHT*0.7)), NL)
            xys = [ [xs[i], ys[i]] for i in range(NL)]
            landmarks=np.array(xys)
        if radius > 0:
            lm_idx = np.where(np.linalg.norm(self.robot_pos_gt - landmarks, axis=1) <= radius)
            landmarks = landmarks[lm_idx]
        if (top_N != 0) and (len(landmarks) >= abs(top_N)):
            lm_dist = np.linalg.norm(self.robot_pos_gt - landmarks, axis=1)
            lm_dist_sort = np.sort(lm_dist)  # ascending sort
            if top_N > 0 :  # nearest
                lm_idx = np.where(lm_dist <= lm_dist_sort[top_N-1])
            else: # farest
                lm_idx = np.where(lm_dist >= lm_dist_sort[top_N])
            landmarks = landmarks[lm_idx]
        return landmarks
    
    def create_uniform_particles(self):
        self.particles = np.empty((self.n_particle, 2))
        self.particles[:, 0] = uniform(self.x_range[0], self.x_range[1], size=self.n_particle)
        self.particles[:, 1] = uniform(self.y_range[0], self.y_range[1], size=self.n_particle)
        return self.particles
    
    def predict(self, particles, u, std, dt=1.):
        N = len(particles)
        dist = (u[1] * dt) + (np.random.randn(N) * std[1])
        particles[:, 0] += np.cos(u[0]) * dist
        particles[:, 1] += np.sin(u[0]) * dist
        return particles
    
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

    def update_single_landmark(self, particles, weights, z, R, landmark):
        if self.observation_mode == 0:  # For distance metric
            distance=np.power((particles[:,0] - landmark[0])**2 +(particles[:,1] - landmark[1])**2,0.5)
            #weights *= scipy.stats.norm(distance, R).pdf(z)  # Gaussian distribution : Norm(dist, sigma^2).pdf(input)
            weights *= scipy.stats.norm.pdf(x=z, loc=distance, scale=R)  # probability density function for Gaussian distribution with loc means ans scale sigma value
        elif self.observation_mode == 1:  # For angle metric
            vec1 = self.get_vector1()
            vec2 = landmark - particles
            similarity = self.cosine_sim(vec1, vec2)
            weights *= scipy.stats.norm.pdf(x=z, loc=similarity, scale=R)  # probability density function for Gaussian distribution with loc means ans scale sigma value
        else:
            print("Error observation_mode : {}".format(self.observation_mode))
            exit(0)
        return weights
    
    def update(self, particles, weights, z, R, landmarks):
        weights.fill(1.)
        for i, landmark in enumerate(landmarks):
            weights = self.update_single_landmark(particles, weights, z[i], R, landmark)
    
        weights += 1.e-300 # avoid round-off to zero
        weights /= sum(weights)
        return weights

    def neff(self, weights):
        return 1. / np.sum(np.square(weights))
    
    def draw_weight(self, weights, positions):
        plt.clf()
        plt.plot(weights, 'gray')
        plt.plot(np.cumsum(weights),'r:')
        plt.plot(positions, 'blue')
        plt.draw()
        plt.pause(0.001)
    
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

def single_loop(stime, mMCL, sleep_sec=0.1, n_landmark=10000, landmark_radius=50, top_N=-1):
    ## For n_landmark,  a number less than 10000 means that it will use the number of random generated landmark points.
    ##                  a numvber over 10000 means that it will use pre-defined landmarks in mcl_config.landmarks
    if (time.time() - stime >  0.01) : #sec
        if mMCL.callback_count < len(mMCL.get_trajectory()):
            ## Prepare input

            [x, y] = mMCL.get_trajectory()[mMCL.callback_count]
            landmarks = mMCL.get_landmarks(n_landmark, mMCL.landmark_radius, top_N)

            ## Run MCL
            mMCL.vps_mcl_apply(x, y, landmarks)

            mMCL.callback_count = mMCL.callback_count + 1
        else:  # reset
            mMCL.callback_count = 0
            mMCL.trajectory = np.zeros(shape=(0,2))
        stime = time.time()
    return stime

if __name__ == "__main__":
    img_path = "map.png"

    ## For n_landmark,  a number less than 10000 means that it will use the number of random generated landmark points.
    ##                  a numvber over 10000 means that it will use pre-defined landmarks in mcl_config.landmarks
    n_landmark = 10000  # 10000 means pre-defined landmarks
    landmark_radius = 200
    top_N = -1

    #mMCL = MCL(observation_mode=0, n_landmark=1, pdf_sigma=10, noise_std=0.1, noise=False)  # distance metric 
    #mMCL = MCL(observation_mode=1, n_landmark=1, pdf_sigma=0.1, noise_std=0.1, noise=False)  # angel metric
    #mMCL = MCL(observation_mode=1, n_landmark=10000, pdf_sigma=0.1, noise_std=0.1, noise=False)  # angel metric
    #mMCL = MCL(observation_mode=1, img_path=img_path, n_landmark=10000, pdf_sigma=0.1, noise_std=0.001, manualPath=False, noise=False)  # angel metric
    #mMCL = MCL(observation_mode=0, img_path=img_path, n_landmark=10000, pdf_sigma=10, noise_std=0.001, manualPath=True, manualPath_rate=0.1, noise=False)  # angel metric

    if False:  # demo mode
        ## Manually move robot with mouse click
        mMCL = MCL(observation_mode=1, img_path=img_path, n_landmark=n_landmark, pdf_sigma=0.1, noise_std=0.0, manualPath=False, manualPath_rate=0.1, noise=True)  # angel metric
        mMCL.run()
        ## For debugging
        print("landmarks=np.array([")
        for pos in mMCL.trajectory:
            print("\t\t[{}, {}],".format(int(pos[0]), int(pos[1])))
        print("\t\t])")

    if True:  # vps mode
        ## Automatically move robot with saved trasactory
        mMCL = MCL(observation_mode=1, img_path=img_path, n_landmark=n_landmark, pdf_sigma=0.1, noise_std=0.0, manualPath=True, manualPath_rate=0.1, noise=True)  # angel metric

        mMCL.vps_mcl_initialize(disp=True)

        stime = time.time()
        [x, y] = mMCL.get_trajectory()[mMCL.callback_count]
        while(1):
            stime = single_loop(stime, mMCL, sleep_sec=0.1, n_landmark=n_landmark, landmark_radius=landmark_radius, top_N=top_N)

    cv2.destroyAllWindows()
