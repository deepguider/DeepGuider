import cv2
import utm
import imutils
import copy
import folium
from ipdb import set_trace as bp
import yaml  # pip install pyyaml
import numpy as np

class DispMap:
    ''' DispMap class including utm coordinates of boundary.'''
    def __init__(self, cv_img, ltop=(0, 0), rbottom=(0, 0), incoord='latlon', coord_vflip=False, coord_hflip=False):
        '''
        ltop and rbottom are coordinates of GPS in lat/lon or utm.
        '''
        self.set_img(cv_img, set_refresh_img=True)
        self.coord_vflip = coord_vflip
        self.coord_hflip = coord_hflip
        ltop, rbottom = self.flip_coordinate(ltop, rbottom, coord_vflip, coord_hflip)
        self.ltop = ltop
        self.rbottom = rbottom
        self.incoord = incoord
        (self.h, self.w, _) = self.cv_img.shape
        self.x1, self.y1 = self.get_utm(ltop, incoord)
        self.x2, self.y2 = self.get_utm(rbottom, incoord)
        self.utm_ltop = (self.x1, self.y1)
        self.utm_rbottom = (self.x2, self.y2)
        self.set_utm_zone(52, "S")
        self.init_homography(incoord)

    def flip_coordinate(self, ltop, rbottom, coord_vflip=False, coord_hflip=False):
        ltop_x, ltop_y = ltop
        rbottom_x, rbottom_y = rbottom

        ## Normal coordinates increase from left-top to right-bottom.
        '''pts1 .... pts2
                ....
           pts3 .... pts4
        '''
        tmp_pts1 = [ltop_x, ltop_y]
        tmp_pts2 = [rbottom_x, ltop_y]
        tmp_pts3 = [ltop_x,    rbottom_y]
        tmp_pts4 = [rbottom_x, rbottom_y]
        if coord_vflip == True and coord_hflip==False:  # y-axis flip
            pts1 = tmp_pts3
            pts2 = tmp_pts4
            pts3 = tmp_pts1
            pts4 = tmp_pts2
        elif coord_vflip == False and coord_hflip==True:  # h-axis flip
            pts1 = tmp_pts2
            pts2 = tmp_pts1
            pts3 = tmp_pts4
            pts4 = tmp_pts3
        elif coord_vflip == True and coord_hflip==True:  # v-axis and h-axis flip
            pts1 = tmp_pts4
            pts2 = tmp_pts3
            pts3 = tmp_pts2
            pts4 = tmp_pts1
        else:  # normal
            pts1 = tmp_pts1
            pts2 = tmp_pts2
            pts3 = tmp_pts3
            pts4 = tmp_pts4
        ret_ltop, ret_rbottom = pts1, pts4
        return ret_ltop, ret_rbottom

    def get_4points(self, ltop, rbottom, coord_vflip=False, coord_hflip=False ):
        '''
        ltop_x, ltop_y = 0, 0
        rbottom_x, rbottom_y = self.w-1, self.h-1
        '''
        ltop, rbottom = self.flip_coordinate(ltop, rbottom, coord_vflip, coord_hflip)
        ltop_x, ltop_y = ltop
        rbottom_x, rbottom_y = rbottom
        ## Normal coordinates increase from left-top to right-bottom.
        '''pts1 .... pts2
                ....
           pts3 .... pts4
        '''
        pts1 = [ltop_x, ltop_y]
        pts2 = [rbottom_x, ltop_y]
        pts3 = [ltop_x,    rbottom_y]
        pts4 = [rbottom_x, rbottom_y]
        return np.array([pts1,pts2,pts3,pts4])

    def set_utm_zone(self, zone_num=52, zone_char='S'):
        self.utm_zone_num = zone_num
        self.utm_zone_char = zone_char

    def get_utm(self, xy=(0, 0), incoord='latlon'):
        if incoord.lower() == 'latlon':
            x, y, self.utm_zone_num, self.utm_zone_char = utm.from_latlon(xy[0], xy[1])
        else:  # 'utm'
            x, y = xy
        return x, y

    def init_homography(self, incoord='latlon'):
        ''' utm coord increase in up-direction(vflip) while img coord increase in down-direction(normal).'''
        pts_img = self.get_4points((0,0), (self.w-1, self.h-1))
        pts_utm = self.get_4points(self.utm_ltop, self.utm_rbottom, coord_vflip=False)

        self.H_utm2img, _ = cv2.findHomography(pts_utm, pts_img, cv2.RANSAC)
        self.H_img2utm, _ = cv2.findHomography(pts_img, pts_utm, cv2.RANSAC)

    def get_img_coord(self, xy=(0, 0), incoord='latlon'):
        x, y = self.get_utm(xy, incoord)  # convert to utm_x, utm_y
        new_coord = xy
        if type(self.H_utm2img) != type(None):
            new_coord = np.dot(self.H_utm2img, np.array([x,y,1])).astype(np.int)
        new_x, new_y = new_coord[0], new_coord[1]
        return new_x, new_y
    
    def get_map_coord(self, xy=(0, 0), outcoord='latlon'):  # convert image coordinate to map coordinate(utm)
        x, y = xy
        new_coord = xy
        if type(self.H_img2utm) != type(None):
            new_coord = np.dot(self.H_img2utm, np.array([x,y,1])).astype(np.int)
        new_x, new_y = new_coord[0], new_coord[1]
        if outcoord.lower() == 'latlon':
            new_x, new_y = utm.to_latlon(new_x, new_y, self.utm_zone_num, self.utm_zone_char)
        return new_x, new_y

    def get_img_coord_old(self, xy=(0, 0), incoord='latlon'):
        ''' utm_y increase toward up direction while image_y increase toward down direction.
        '''
        x, y = self.get_utm(xy, incoord)
        x = self.y_is_ax_b(self.x1, self.x2, self.w, x)
        y = self.y_is_ax_b(self.y1, self.y2, self.h, y, direction=1)
        return x, y

    def get_map_coord_old(self, xy=(0, 0), outcoord='latlon'):  # convert image coordinate to map coordinate(utm)
        ''' utm_y increase toward up direction while image_y increase toward down direction.
        '''
        (x, y) = xy
        x = self.inv_y_is_ax_b(self.x1, self.x2, self.w, x)
        y = self.inv_y_is_ax_b(self.y1, self.y2, self.h, y, direction=1)
        if outcoord.lower() == 'latlon':
            x, y = utm.to_latlon(x, y, self.utm_zone_num, self.utm_zone_char)
        return x, y

    def loopback_img_coord_conversion(self, xy=(0, 0), incoord='latlon'):  # For verification
        return self.get_img_coord(self.get_map_coord(xy, incoord), incoord)

    def loopback_utm_coord_conversion(self, xy=(0, 0), incoord='latlon'):  # For verification
        return self.get_map_coord(self.get_img_coord(xy, incoord), incoord)

    def y_is_ax_b(self, x1, x2, w, x, direction=0):
        a = w/(x2 - x1)  # a = 1279/(x2-x1)
        b = -1*a*x1
        y = a*x + b
        if direction is not 0:
            y = w - y
        return int(y)

    def inv_y_is_ax_b(self, x1, x2, w, y, direction=0):
        a = w/(x2 - x1)  # a = 1279/(x2-x1)
        b = -1*a*x1
        x = (y-b) / a
        if direction is not 0:  # To do : it this true? I didn't prove this code
            x = w - x
        return int(x)

    def draw_point_on_map(self, xy=(0, 0), incoord='latlon',
            radius=1, color=(255, 0, 0),  # BGR
            thickness=-1  #  -1 : fill, positive: thick
            ):
        # (584735.1354362543, 4477045.784565611, 17, 'T')  # utm.from_latlon(40.4397456, -80.0008864)
        # (585298.0976780445, 4476587.395387753, 17, 'T')  # utm.from_latlon(40.435559, -79.994311)
        if "img" in incoord:
            (x, y) = xy
        else:  ## "utm" or "latlon"
            x, y = self.get_img_coord(xy, incoord)
        x, y = int(x), int(y)
        cv2.circle(self.cv_img, (x, y), radius, color, thickness)
        return self.cv_img

    def draw_points_on_map(self, xys=[(0, 0),(0, 0),(0, 0)], incoord='latlon',
            radius=1, color=(255, 0, 0),  # BGR
            thickness=-1  #  -1 : fill, positive: thick
            ):  
        for xy in xys:
            ''' xy is a gps coordinate, utm[0:2] or latlon '''
            self.draw_point_on_map(xy, incoord, radius, color, thickness)

    def set_img(self, cv_img, set_refresh_img=False):
        self.cv_img = cv_img
        if set_refresh_img == True:
            self.cv_img_backup = copy.copy(cv_img)

    def get_img(self):
        return self.cv_img

    def get_image0(self):
        return self.cv_img_backup

    def get_image(self):
        return self.get_img()

    def refresh_img(self):
        self.cv_img = copy.copy(self.cv_img_backup)

def write_pose_to_txt(Xs, Ys):
    f = open("poses_latlon.py", 'w')
    data = "    pts = [\n"
    f.write(data)
    for xy in zip(Xs, Ys):
        x, y = mMap.get_map_coord(xy)
        data = "        ({}, {}),".format(x, y)
        data = "%d번째 줄입니다.\n" % i
        f.write(data)
    data = "    ]"
    f.write(data)
    f.close()

def get_latlon_from_line(line, sep=','):
    # line : 
    # 000000, 36.3798158583, 127.367339298
    try:
        line = line.split(sep)
        lat = float(line[1])
        lon = float(line[2])
    except:
        bp()
    return lat, lon

def get_latlon_from_txtfile(poseFile, sep=','):
    txt_lines = open(poseFile,'r').readlines()
    latlon = []
    for i, line in enumerate(txt_lines):
        # line : 
        # 000000, 36.3798158583, 127.367339298
        lat, lon = get_latlon_from_line(line, sep=sep)
        latlon.append((lat, lon))
    return latlon

def get_xy_from_line(line, sep=','):
    return get_latlon_from_line(line, sep)

def get_xyArray_from_txtfile(poseFile, sep=','):
    return get_latlon_from_txtfile(poseFile, sep)

def draw_pose_txtfile(poseFile, myMap, radius = 1, color = 'blue', fill = True, sep=','):
    txt_lines = open(poseFile,'r').readlines()
    # file content :
        # 000000, 36.3798158583, 127.367339298
        # 000000, 36.3798158583, 127.367339298
        # ...


    for i, line in enumerate(txt_lines):
        lat, lon = get_latlon_from_line(line, sep)

        folium.CircleMarker(
                location = [lat, lon],
                radius = radius,
                color = color,
                fill = fill
                ).add_to(myMap)
        if False:
            folium.Marker(
                    location = [lat, lon],
                    popup ="{}".format(line.split(sep)[0]),
                    icon = folium.Icon(color='red', icon='star')
                    ).add_to(myMap)
    return myMap


def get_empty_image(utm_data, max_win=1000):
    xmin, ymin = np.min(utm_data,axis=0)
    xmax, ymax = np.max(utm_data,axis=0)
    w = (xmax - xmin)
    h = (ymax - ymin)
    if w > h : 
        ratio = h/w 
        width = max_win
        height = int(ratio * width)
    else:
        ratio = w/h 
        height = max_win
        width = int(ratio * height)

    ## Generated empty image which includes all query utm points.
    empty_img = 64*np.ones((height, width, 3), np.uint8)

    ## Normal coordinates(latlon) increase from left-top to right-bottom. 
    ## But, utm coordinates increase from left-bottom to right-top. 
    '''pts1 .... pts2 
            .... 
       pts3 .... pts4 
    ''' 
    (ltop_x, ltop_y) = (xmin, ymin)
    (rbottom_x, rbottom_y) = (xmax, ymax)
    tmp_pts1 = [ltop_x, ltop_y] 
    tmp_pts2 = [rbottom_x, ltop_y] 
    tmp_pts3 = [ltop_x,    rbottom_y] 
    tmp_pts4 = [rbottom_x, rbottom_y] 
    # y-axis flip 
    pts1 = tmp_pts3 
    pts2 = tmp_pts4 
    pts3 = tmp_pts1 
    pts4 = tmp_pts2 
    utm_map_ltop, utm_map_rbottom = pts1, pts4 

    return empty_img, utm_map_ltop, utm_map_rbottom

def get_empty_image_old(utm_data, max_win=1000):
    xmin, ymin = np.min(utm_data,axis=0)
    xmax, ymax = np.max(utm_data,axis=0)
    utm_map_ltop = (xmin, ymin)
    utm_map_rbottom = (xmax, ymax)

    w = (xmax - xmin)
    h = (ymax - ymin)
    if w > h : 
        ratio = h/w 
        width = max_win
        height = int(ratio * width)
    else:
        ratio = w/h 
        height = max_win
        width = int(ratio * height)

    ## Generated empty image which includes all query utm points.
    empty_img = 64*np.ones((height, width, 3), np.uint8)
    return empty_img, utm_map_ltop, utm_map_rbottom

def get_split_utm_rect(utmDb, utmQ, utm_vertical_coord_flip=True, draw=False):
    incoord = 'utm'
    ## Generated empty image which includes all query utm points.
    utm_data = np.concatenate((utmQ, utmDb), axis=0)
    empty_img0, map_ltop, map_rbottom = get_empty_image(utm_data)
    empty_img = copy.copy(empty_img0)

    ## Draw Train/Test/Val rectangle
    mDraw = mouse_draw_shape(empty_img, map_ltop, map_rbottom, incoord, coord_vflip=utm_vertical_coord_flip)

    if draw == True:
        ## User needs to define each coordinate of regions for train/test/val, and needs to save it into yaml file
        mDraw.draw(utmQ, utmDb, incoord, radius=5)  # draw and save rectangle coordinates using mouse click events
        #map_img = mDraw.get_image()

    ## To do : get splited image list with splited utm list
    mDraw.classify_train_val_test(utmQ[0], incoord)

    return mDraw
