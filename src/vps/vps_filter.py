import numpy as np
import math
from matplotlib import pyplot as plt
from sklearn import linear_model, datasets
from ipdb import set_trace as bp

from skimage.measure import LineModelND, ransac

class vps_filter:
    def __init__(self, ksize=9, valid_dist_thre=5, outlier_dist_thre=25):  # meters
        self.n_sample_count = 0
        self.n_samples = ksize  # kernel_size
        #self.n_mean_samples = 15  # filter size of points for average filter
        self.n_samples_max = 100  # maximum filter size of points for ransac
        self.n_samples_min = 1   # minimum filter size of points for ransac
        self.dims = 2
        self.utm_xys = np.zeros((self.n_samples_max, self.dims))
        #self.utm_xys[:,0] = np.arange(self.n_samples_max)  # for debugging
        #self.utm_xys[:,1] = np.arange(self.n_samples_max)  # for debugging

        if True:
            self.valid_dist_thre = valid_dist_thre  # Distance in meter between current point and average of points
            self.outlier_dist_thre = outlier_dist_thre
            #self.utm_differential_threshold = 30  # Differential in the previous ponits. If there is large disarity and not back to be small, then it means noisy data.
        else:  # Large number means disabling this filter
            self.ransacline_distance_threshold = 500 # noisy postion difference in meters
            self.valid_dist_thre = 500
            self.utm_differential_threshold = 500

        self.continuous_outlier_count = 0
        self.continuous_outlier_count_min = 0
        self.continuous_outlier_count_max = 30

    def set_valid_dist_thre(self, thre=5):  # Large number, 500 meter, means filter off.
        self.valid_dist_thre = thre

    def set_outlier_dist_thre(self, thre=25):  # Large number, 500 meter, means filter off.
        self.outlier_dist_thre = thre

    def get_valid_dist_thre(self):
        return self.valid_dist_thre

    def get_outlier_dist_thre(self):
        return self.outlier_dist_thre

    def get_samples_toy_example(self, n_samples=10, n_outliers_ratio=0.4):  # For toy example
        if n_samples <= 0:
            n_samples = self.n_samples  # use default n_samples
        else:
            self.n_samples = n_samples  # update n_samples
        a, b, c = 1, -1, 0
        func_str = "Ground Truth : {}x + {}y + {} = 0".format(a, b, c)
        #print(func_str)
        np.random.seed(0)
        x = np.arange(n_samples)
        n_outliers = np.int(n_samples  * n_outliers_ratio)
        y = a*x + c  # 3x -y + 2 = 0  <==> y = 3x + 2
        x[:n_outliers] = 3 + 0.5 * np.random.normal(size=n_outliers)
        y[:n_outliers] = -3 + 10 * np.random.normal(size=n_outliers)
        x = x.reshape(-1, 1)  # nx1 data
        y = y.reshape(-1, 1)  # nx1 data
        for i in range(len(x)):
            self.update_samples(x[i], y[i])

    def line_function_from_ransac(self, model):
        ## y = Ax + B
        try:  # No specific line was estimated when all points are same
            x=np.asarray([0]); B = model.predict_y(x)  # y(0) = A*0 + B = B
            x=np.asarray([1]); A = model.predict_y(x) - B # y(1) - B = (A*1 + B) - B = A
        except:
            return 0, 0, 0
        A = A.squeeze().squeeze()
        B = B.squeeze().squeeze()
        #print("y = {}x + {}".format(A, B))
        ## format : y = Ax + B ==> ax +by + c = 0
        a = float(A)
        b = float(-1)
        c = float(B)
        #print("[vps] {}x + {}y + {} = 0".format(a, b, c))
        return a, b, c

    def shortest_distance(self, x1, y1, a, b, c):
        # https://mathbang.net/453
        d = abs((a * x1 + b * y1 + c)) / (np.sqrt(a * a + b * b)+1e-7)
        return d

    def update_samples(self, x, y):  # add new data to last position
        if self.n_sample_count < self.n_samples_max:
            self.n_sample_count += 1
        self.utm_xys[:-1] = self.utm_xys[1:]
        self.utm_xys[-1,:] = [x, y]

    def undo_update_samples(self): # remove last one
        if self.n_sample_count > self.n_samples + 1:  # larger than kernel size
            self.n_sample_count -= 1
            self.utm_xys[1:] = self.utm_xys[:-1]

    def get_samples(self, n_samples=0):
        if n_samples <= 0:
            n_samples = self.n_samples  # use default n_samples
        else:
            self.n_samples = n_samples  # update n_samples

        if n_samples > self.n_sample_count:  # Data is not sufficient
            n_samples = self.n_sample_count

        if n_samples == 0:
            return np.asarray([[0,0]])
        else:
            return self.utm_xys[-n_samples:]

    def check_valid(self, new_x, new_y):
        if new_x >0 and new_y > 0:
            self.update_samples(new_x, new_y)
        else:
            return False, None

        utm_xys = self.get_samples(self.n_samples)
        mean_xys = np.median(utm_xys, axis=0)
        if len(utm_xys) < self.n_samples_min:
            return False, mean_xys
        utm_distance = np.sqrt(np.sum((mean_xys - [new_x, new_y])**2))  # meter
        ## Check 2 : distance average point and current point in the small window
        if utm_distance > self.outlier_dist_thre:
            if self.continuous_outlier_count < self.continuous_outlier_count_max:
                self.continuous_outlier_count += 1
                self.undo_update_samples()
        else:
            self.continuous_outlier_count -= 1
            if self.continuous_outlier_count < self.continuous_outlier_count_min:
                self.continuous_outlier_count = self.continuous_outlier_count_min

            #print("[vps] Reject outlier ==========================> Reject outlier : {}".format(utm_distance))
        if utm_distance > self.valid_dist_thre:  # Check new point is near to mean of previous point.
            #print("[vps] Filter out ==========================> Filter out utm : {}".format(utm_distance))
            return False, mean_xys
        else:
            return True, mean_xys

    def check_valid_ori(self, new_x, new_y):
        self.update_samples(new_x, new_y)
        utm_xys = self.get_samples(self.n_samples)
        if len(utm_xys) < self.n_samples_min:
            return False
        try:
            ransac_model, inliers = ransac(utm_xys, LineModelND, min_samples=2, residual_threshold=0.5, max_trials=1000)
            a, b, c = self.line_function_from_ransac(ransac_model)
            d = self.shortest_distance(new_x, new_y, a, b, c)

            ## Check1 : distance between line and current point in the large window
            if d <= self.ransacline_distance_threshold:  # Check new point is near to ransac line
                inliers_count = np.sum(inliers)
                if self.n_mean_samples > inliers_count:
                    n_mean_samples = inliers_count
                else:
                    n_mean_samples = self.n_mean_samples
                mean_xys = np.median(utm_xys[inliers][(inliers_count-n_mean_samples):,:], axis=0)
                utm_distance = np.sqrt(np.sum((mean_xys - [new_x, new_y])**2))  # meter
                ## Check 2 : distance average point and current point in the small window
                if utm_distance < self.valid_dist_thre:  # Check new point is near to mean of previous point.
                    ## Check 3 : differential in the large window (noise to change to anohter road)
                    differential1 = np.sqrt(np.sum(np.sum(np.diff(utm_xys, axis=0), axis=0)**2))
                    #differential2 = np.sqrt(np.sum(np.sum(np.diff(np.diff(utm_xys, axis=0), axis=0), axis=0)**2))
                    differential = differential1
                    if differential < self.utm_differential_threshold:  # Suppress large disparity in the moving window.
                        print("[vps] Pass =========>d : {}, utm_distance : {},  differential : {}".format(d, utm_distance, differential), len(utm_xys))
                        return True
                    else:
                        print("[vps] Filter out =========================> differential : {}".format(differential))
                        print("[vps] Filter out =========================> differential : {}".format(differential))
                        print("[vps] Filter out =========================> differential : {}".format(differential))
                        print("[vps] Filter out =========================> differential : {}".format(differential))
                        print("[vps] Filter out =========================> differential : {}".format(differential))
                        print("[vps] Filter out =========================> differential : {}".format(differential))
                        return False
                else:
                    #print("[vps] Filter out ==========================> distance to utm : {}".format(utm_distance))
                    return False
            else:
                #print("[vps] Filter out ============================> distance to ransac line : {}".format(d))
                return False
        except:
            return False

if __name__ == "__main__":
    mVps_filter = vps_filter()
    if False:  # single value
        mVps_filter.get_samples_toy_example()
        isValidVps, filtered_xy = mVps_filter.check_valid(5, 30)
        print(isValidVps, filtered_xy)
    else:
        for i in range(20):
            x = np.random.rand(1)*10
            y = np.random.rand(1)*100
            isValidVps, filtered_xy = mVps_filter.check_valid(x, y)
            print(isValidVps, filtered_xy)

