import folium
import utm
import os
import numpy as np
#import webbrowser
import time
#from selenium import webdriver
from ipdb import set_trace as bp
import argparse

def get_latlon_center_from_utm(lines):
    latList = []
    lonList = []
    for line in lines:
        lat, lon = get_latlon_from_utm(line)
        latList.append(lat)
        lonList.append(lon)
    latArray = np.array(latList)
    lonArray = np.array(lonList)
    return latArray.mean(), lonArray.mean()


def get_latlon_center_from_latlon(lines):
    latList = []
    lonList = []
    for line in lines:
        lat, lon = get_latlon_from_latlon(line)
        latList.append(lat)
        lonList.append(lon)
    latArray = np.array(latList)
    lonArray = np.array(lonList)
    try:
        lat_mean = latArray.mean()
        lon_mean = lonArray.mean()
    except:
        pass
    return lat_mean, lon_mean

def makedir(fdir):
    if not os.path.exists(fdir):
        if fdir != '':
            os.makedirs(fdir)

def get_latlon_from_utm(line):
    # line : 
    # 000000 354559.16244695638 4028082.80969046941 52 S
    line = line.split(' ')
    utm_x = float(line[1])
    utm_y = float(line[2])
    utm_zone = int(line[3])
    utm_compass = 'N'
    lat, lon = utm.to_latlon(utm_x, utm_y, utm_zone, utm_compass)
    return lat, lon


def get_latlon_from_latlon(line):
    # line : 
    # 000000 36.3798158583 127.367339298
    line = line.split(' ')
    lat = float(line[1])
    lon = float(line[2])
    return lat, lon


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Draw points on the Map')
    parser.add_argument('--coord', type=str, default='utm', help='Coordinate of pose', choices=['utm', 'latlon'])
    parser.add_argument('--ofname', type=str, default='myMap', help='Map file name')
    parser.add_argument('--ifname', type=str, default='./cropped_jpg/dbImg/poses_utm_daejeon.txt', help='pose file name')
    parser.add_argument('--zoom', type=int, default=13, help='Large number for more zoom in')
    parser.add_argument('--dispratio', type=float, default=0.01, help='0 :display no, 1: display all, 0.5: display 50% of all points')

    opt = parser.parse_args()

    mapFile = '{}.html'.format(opt.ofname)
    mapImg = '{}.png'.format(opt.ofname)
    mapZoom = opt.zoom
    poseFile = opt.ifname
    poseMode = opt.coord

    if os.path.exists(poseFile) == False:
        print("File not found : {}".format(poseFile))
        exit(0)
    txt_lines = open(poseFile,'r').readlines()
    # file content :
    # 000000 354559.16244695638 4028082.80969046941 52 S
    # 000001 354551.34206688614 4028083.00761361513 52 S
    # 000002 354535.71027641464 4028083.40333045507 52 S
    # ...

    # Get center position of all points 
    if poseMode == 'utm':
        lat, lon = get_latlon_center_from_utm(txt_lines)
    else: # latlon
        lat, lon = get_latlon_center_from_latlon(txt_lines)

    if np.isnan(lat) or np.isnan(lon):
        print("[draw_point_on_map.py] Invalid lat, lon file : {}".format(poseFile))
        exit(1)

    # Display center point
    myMap = folium.Map(location=[lat, lon], zoom_start=mapZoom) # larger number for zoom in
    # web = webbrowser.open_new(mapPath)
    #driver = webdriver.Chrome()
    #mapPath = 'file://{}'.format(os.path.join(os.getcwd(),mapFile)) #'file:///home/ccsmm/Naverlabs/GenerateDataset/myMap.html'
    lcnt = 0

    linecnt = len(txt_lines)
    skip = int(1.0/opt.dispratio)
    skip_cnt = 0

    # Display all points
    for line in txt_lines:
        if skip_cnt < skip:
            skip_cnt += 1
            continue
        else:
            skip_cnt = 0

        if poseMode == 'utm':
            lat, lon = get_latlon_from_utm(line)
        else: # latlon
            lat, lon = get_latlon_from_latlon(line)

        folium.CircleMarker(
                location = [lat, lon],
                radius = 1,
                color = 'blue',
                fill = True
                ).add_to(myMap)
        if False:
            folium.Marker(
                    location = [lat, lon],
                    popup ="{}".format(line.split(' ')[0]),
                    icon = folium.Icon(color='red', icon='star')
                    ).add_to(myMap)

    outdir=os.path.dirname(mapFile)
    makedir(outdir)
    myMap.save(mapFile)
    #driver.get(mapPath)
    print("Wait 5 seconds, map window is being opened...")
    # driver.refresh()
    time.sleep(0.1)
    #driver.save_screenshot(mapImg)
    #driver.close()
