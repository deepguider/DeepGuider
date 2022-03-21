#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ccsmm@etri.re.kr
# Copyright 2016 Massachusetts Institute of Technology
# Some functions are from https://github.com/AtsushiSakai/rosbag_to_csv   : message_to_csv, message_type_to_csv, bag_to_csv
# 

"""Extract images from a rosbag.
"""

import os
import argparse
import cv2
import rosbag
import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from ipdb import set_trace as bp
import numpy as np
import utm
import sys
import time
from datetime import datetime

resolution=['VGA', '720P']
image_resolution=resolution[1]

def makedir(fdir):
    import os
    if not os.path.exists(fdir):
        os.makedirs(fdir)

def rmfiles(path='./download_jpg/*.jpg'):
    import os,glob
    for f in glob.glob(path):
        os.remove(f)

def parse_bags_withSubDir(bag_dir, output_dir, uvc_topic, gps_topic, imu_topic, pose_latlon_file, pose_utm_file, pose_only, init_skip_meter=10, startIdx=0, unique_jpgname=False, sec_per_frame=0):
    total_count = startIdx
    if os.path.isfile(os.path.join(output_dir, pose_latlon_file)):
        os.remove(os.path.join(output_dir, pose_latlon_file))
    if os.path.isfile(os.path.join(output_dir, pose_utm_file)):
        os.remove(os.path.join(output_dir, pose_utm_file))

    for curr_path, dirs, files in os.walk(bag_dir):
        for name in files:
            if name.endswith((".bag".lower(), ".Bag")):
                bag_path = os.path.join(curr_path, name)
                if os.path.isfile(bag_path):
                    print "{} was found.".format(bag_path)

    print "============================================="
    for curr_path, dirs, files in os.walk(bag_dir):
        for name in files:
            if name.endswith((".bag".lower(), ".Bag")):
                bag_path = os.path.join(curr_path, name)
                if os.path.isfile(bag_path):
                    curr_count = parse_single_bag(bag_path, output_dir, uvc_topic, gps_topic, imu_topic,
                            pose_latlon_file, pose_utm_file, pose_only, init_skip_meter, total_count, unique_jpgname, sec_per_frame)
                    total_count += curr_count
                    print "{} : {} / {} ".format(bag_path, curr_count, total_count)

def message_to_csv(stream, msg, flatten=False):
    """
    stream: StringIO
    msg: message
    """
    try:
        for s in type(msg).__slots__:
            val = msg.__getattribute__(s)
            message_to_csv(stream, val, flatten)
    except:
        msg_str = str(msg)
        if msg_str.find(",") is not -1:
            if flatten:
                msg_str = msg_str.strip("(")
                msg_str = msg_str.strip(")")
                msg_str = msg_str.strip(" ")
            else:
                msg_str = "\"" + msg_str + "\""
        stream.write("," + msg_str)

def message_type_to_csv(stream, msg, parent_content_name=""):
    """
    stream: StringIO
    msg: message
    """
    try:
        for s in type(msg).__slots__:
            val = msg.__getattribute__(s)
            message_type_to_csv(stream, val, ".".join([parent_content_name,s]))
    except:
        stream.write("," + parent_content_name)

def bag_to_csv(fname, topic_names, output_dir='.', start_time=None, end_time=None):
    output_file_format="%t.csv"
    try:
        bag = rosbag.Bag(fname)
        streamdict= dict()
        stime = None
        etime = None
        #if opts.start_time:
        #    stime = rospy.Time(opts.start_time)
        #etime = None
        #if opts.end_time:
        #    etime = rospy.Time(opts.end_time)
    except Exception as e:
        print "failed to load bag file: %s', e"
        exit(1)
    try:
        for topic, msg, time in bag.read_messages(topics=topic_names,
                                                  start_time=stime,
                                                  end_time=etime):
            if streamdict.has_key(topic):
                stream = streamdict[topic]
            else:
                outpath = output_dir+fname[fname.rfind('/'):-4]+topic.replace('/','_')+'.csv'
                stream = open(outpath,'w')
                streamdict[topic] = stream
                # header
                if True:
                    stream.write("time")
                    message_type_to_csv(stream, msg)
                    stream.write('\n')

            stream.write(datetime.fromtimestamp(time.to_time()).strftime('%Y/%m/%d/%H:%M:%S.%f'))
            message_to_csv(stream, msg)
            stream.write('\n')
        [s.close for s in streamdict.values()]
    except Exception as e:
        print "fail: %s", e
    finally:
        bag.close()

def GetIntTime(t):
    #return int(int('{}'.format(t))/1e+9)
    return int(t.to_sec())

def GetTimeString(rospy_t):
    return time.strftime('%Y%m%d_%H:%M:%S', time.localtime(rospy_t.secs)) + ".{}".format(rospy_t.nsecs)

def GetTopicList(bag):
    return bag.get_type_and_topic_info().topics.keys()

def GetTopicByName(bag, string_topic):
    topic_list = bag.get_type_and_topic_info().topics.keys()
    for topic in topic_list:
        if string_topic.lower() in topic.lower():
            return topic
    return 'NotFound'

def GetTopic_UVC(bag, string_topic):
    topic_ret = GetTopicByName(bag, string_topic)
    if topic_ret == 'NotFound':
        topic_ret = GetTopicByName(bag, 'uvc')
    if topic_ret == 'NotFound':
        topic_ret = GetTopicByName(bag, 'camera')
    if topic_ret == 'NotFound':
        topic_ret = GetTopicByName(bag, 'image')
    if topic_ret == 'NotFound':
        topic_list = GetTopicList(bag)
        print "You need to choose a valid topic in ", topic_list, " for [Camera]"
    else:
        print "Detected ros topic of [Camera] : {}".format(topic_ret)
    return topic_ret


def GetTopic_GPS(bag, string_topic):
    topic_ret = GetTopicByName(bag, string_topic)
    if topic_ret == 'NotFound':
        topic_ret = GetTopicByName(bag, 'gps')
    if topic_ret == 'NotFound':
        topic_ret = GetTopicByName(bag, 'ascen')
    if topic_ret == 'NotFound':
        topic_ret = GetTopicByName(bag, 'novatel')
    if topic_ret == 'NotFound':
        topic_list = GetTopicList(bag)
        print("You need to choose a valid topic in ", topic_list, " for [GPS]")
    else:
        print "Detected ros topic of [GPS] : {}".format(topic_ret)
    return topic_ret

def GetTopic_IMU(bag, string_topic):
    topic_ret = GetTopicByName(bag, string_topic)
    if topic_ret == 'NotFound':
        topic_ret = GetTopicByName(bag, 'imu')
    if topic_ret == 'NotFound':
        topic_list = GetTopicList(bag)
        print("You need to choose a valid topic in ", topic_list, " for [IMU]")
    else:
        print "Detected ros topic of [IMU] : {}".format(topic_ret)
    return topic_ret


def sensors_to_csv(bag_file, output_dir, uvc_topic, gps_topic, imu_topic):
    bag = rosbag.Bag(bag_file, "r")

    ## Get correct topic_name
    gps_topic = GetTopic_GPS(bag, gps_topic)
    imu_topic = GetTopic_IMU(bag, imu_topic)

    makedir(output_dir)

    ## Write gps and imu data to csv file
    bag_to_csv(bag_file, gps_topic, output_dir)
    bag_to_csv(bag_file, imu_topic, output_dir)

    bag.close()


def parse_single_bag(bag_file, output_dir, uvc_topic, gps_topic, imu_topic, pose_latlon_file, pose_utm_file, pose_only, init_skip_meter=10,  startIdx=0, unique_jpgname=False, sec_per_frame=0):
    debug_write_tempory_posefile = False
    print "In  : %s.\nOut : %s" % (bag_file, output_dir)
    bag = rosbag.Bag(bag_file, "r")

    ## Get correct topic_name
    uvc_topic = GetTopic_UVC(bag, uvc_topic)
    gps_topic = GetTopic_GPS(bag, gps_topic)
    imu_topic = GetTopic_IMU(bag, imu_topic)
    bridge = CvBridge()

    makedir(output_dir)

    ## Save GPS topic and time stamp at temporary files to use them in synchronization between latlon.txt file and image file name.
    ## We compare iTime of utmArray and iTime of image to sync. time.
    utmLists = []
    latlonLists = []
    if debug_write_tempory_posefile == True:
        # Following two files are not used. For debugging.
        fpl = open('/tmp/pose_latlon_tmp.txt', 'w', buffering = 255)  # Save data at all time stamp
        fpu = open('/tmp/pose_utm_tmp.txt', 'w', buffering = 255)   # Save data at all time stamp
    init_sw = True
    normal_state = False
    distance = 0 
    for topic, msg, t in bag.read_messages(topics=[gps_topic]):
        iTime = t.to_sec()  # 1634172560.5195043

        if hasattr(msg,'lat'):
            lat = msg.lat
            lon = msg.lon
            alt = msg.hgt
        elif hasattr(msg,'latitude'):
            lat = msg.latitude
            lon = msg.longitude
            alt = msg.altitude

        if np.isnan(lat):
            lat = 36.38011018428967
        if np.isnan(lon):
            lon = 127.36808909085782
        if np.isnan(alt):
            alt = 0

        utm_x, utm_y, utm_region, utm_char = utm.from_latlon(lat, lon)

        if init_sw:
            utm_x_init, utm_y_init = utm_x, utm_y
            init_sw = False
        distance = np.linalg.norm([utm_x-utm_x_init, utm_y-utm_y_init])
        if normal_state:
            #print "(Moving) {} meters from 0".format(distance)
            is_moving = 1
        else:
            if distance >= init_skip_meter: # save after 10 meters from the beginning
                normal_state = True
                #print "(Moving) {} meters from 0".format(distance)
                is_moving = 1
            else:
                #print "(Parking) {} meters from 0".format(distance)
                is_moving = 0

        if debug_write_tempory_posefile == True:
            line = "{0:10.7f} {1:2.10f} {2:3.9f} {3}\n".format(iTime, lat, lon, alt)
            fpl.write(line)
            line = "{0:10.7f} {1:6.11f} {2:7.11f} {3:02d} {4:}\n".format(iTime, utm_x, utm_y, utm_region, utm_char)
            fpu.write(line)

        utmList = [iTime, float(utm_x), float(utm_y), int(utm_region), int(is_moving)]
        utmLists.append(utmList)

        latlonList = [iTime, float(lat), float(lon), float(alt)]
        latlonLists.append(latlonList)

    if debug_write_tempory_posefile == True:
        fpl.close()
        fpu.close()

    utmArray = np.array(utmLists)
    latlonArray = np.array(latlonLists)

    ## Save Image and its GPS location which are corresponded to time stamp
    fpl = open(os.path.join(output_dir, pose_latlon_file), 'a', buffering = 255) # appending write mode
    fpu = open(os.path.join(output_dir, pose_utm_file), 'a', buffering = 255) # appending write mode

    count = startIdx
    init_sw = True

    state_print_msg = 0
    prev_sec = 0
    for topic, msg, t in bag.read_messages(topics=[uvc_topic]):
        iTime = t.to_sec()
        idx = np.argmin(np.abs(utmArray[:,0] - iTime))
        #idxs = np.squeeze(np.where(utmArray[:,0] == iTime))
        #try:
        #    idx = np.argmin(utmArray[:,0] - iTime)
            #if idxs.size == 0:
            #    time_difference = min(utmArray[:,0] - iTime)
            #    if time_di
            #    continue
            #elif idxs.size == 1:
            #    idx = idxs
            #else:
            #    idx = idxs[0]
        #except:
            #bp()
        #    continue

        is_moving = int(utmArray[idx][4]) # image in normal state which exit from the beginning parking state
        if False and (is_moving == 0):
            if state_print_msg == 0:
                print "Skipping parking state : it saves images {} meters far from starting point".format(init_skip_meter)
                state_print_msg = 1
            continue
        if state_print_msg == 1:
                print "Saving images..."
                state_print_msg = 2

        curr_sec = t.to_sec()
        delta_sec = curr_sec - prev_sec
        if sec_per_frame > 0:  # 0 means that it saves all image as possible.
            if delta_sec < sec_per_frame:  # Too early
                continue

        if unique_jpgname == False: 
            fname = "{0:06d}.jpg".format(count)
        else:
            fname = "{0}_{1:06d}.jpg".format(os.path.basename(output_dir), count)

        line = "{0:06d} {1:6.11f} {2:7.11f} {3:02d} {4:}\n".format(count,
                utmArray[idx][1], utmArray[idx][2], int(utmArray[idx][3]), 'S') 
        # file content :
        # 000000 354559.16244695638 4028082.80969046941 52 S
        # 000001 354551.34206688614 4028083.00761361513 52 S
        fpu.write(line)

        line = "{0:06d} {1:2.10f} {2:3.9f} {3}\n".format(count,
                latlonArray[idx][1],latlonArray[idx][2], latlonArray[idx][3])
        fpl.write(line)

        if pose_only == False:
            #cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            cv_img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")  # 720, 1280, 3
            if image_resolution.lower() =='vga':
                cv_img = cv2.resize(cv_img, dsize=(640, 480), interpolation=cv2.INTER_AREA)
            cv2.imwrite(os.path.join(output_dir, fname), cv_img)
        count += 1
        sys.stdout.write("{} \r".format(fname))
        prev_sec = curr_sec  # 1634028485.9426162

    print(" ")
    fpl.close()
    fpu.close()
    bag.close()

    return count - startIdx


def main():
    """Extract a folder of images from a rosbag.
    """
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    parser.add_argument("--bag_file", type=str, default='2020-02-19-15-01-53.bag', help="Input ROS bag.")
    parser.add_argument("-s", "--start-time", dest="start_time", help="start time of bagfile", type=float)
    parser.add_argument("-e", "--end-time", dest="end_time", help="end time of bagfile", type=float)
    parser.add_argument("-n", "--no-header", dest="header", action="store_false", default=True, help="no header / flatten array value")

    parser.add_argument("--output_dir", type=str, default='extracted',  help="Output directory.")
    # You can know the name of topic using 'rosbag info 2020.bag' command
    #parser.add_argument("--uvc_topic", type=str, default='/uvc_image_raw/compressed',
    #parser.add_argument("--uvc_topic", type=str, default='/uvc_image_raw/compressed',
    parser.add_argument("--uvc_topic", type=str, default='/uvc_camera/image_raw/compressed', help="A part of string included in Image topic. You can know this string using : rosbag info some.bag")
    parser.add_argument("--gps_topic", type=str, default='/ascen_gps/fix', help="A part of string included in GPS topic. You can know this string using : rosbag info some.bag")
    parser.add_argument("--imu_topic", type=str, default='/imu/data', help="A part of string included in IMU topic. You can know this string using : rosbag info some.bag")
    parser.add_argument("--pose_latlon_file", type=str, default='poses_latlon_robot.txt')
    parser.add_argument("--pose_utm_file", type=str, default='poses_utm_robot.txt')
    parser.add_argument('--pose_only', action='store_true', help='It only extracts pose, otherwise extracts pose and images')
    parser.add_argument('--unique_jpgname', action='store_true', help='It adds rosbag filename in front of jpg name. For example 2020-02-19-15_000000.jpg instead of just 000000.jpg')
    parser.add_argument("--startIdx", type=int, default=0, help="Start index of filename.")
    parser.add_argument("--init_skip_meter", type=int, default=10, help="Skip parking state in which movement is in 10 meters.")
    parser.add_argument("--sec_per_frame", type=float, default=0.0, help="Save image every sec_per_frame seconds. 0.1 means 100ms, 1sec/0.1 (or 10) fps. 0 means all images.")

    opts = parser.parse_args()

    if os.path.isfile(opts.bag_file):
        sensors_to_csv(opts.bag_file, opts.output_dir, opts.uvc_topic, opts.gps_topic, opts.imu_topic)
        parse_single_bag(opts.bag_file, opts.output_dir, opts.uvc_topic, opts.gps_topic, opts.imu_topic,
                opts.pose_latlon_file, opts.pose_utm_file, opts.pose_only, opts.init_skip_meter, opts.startIdx, opts.unique_jpgname, opts.sec_per_frame)

    if os.path.isdir(opts.bag_file):
        parse_bags_withSubDir(opts.bag_file, opts.output_dir, opts.uvc_topic, opts.gps_topic, opts.imu_topic,
                opts.pose_latlon_file, opts.pose_utm_file, opts.pose_only, opts.init_skip_meter, opts.startIdx, opts.unique_jpgname, opts.sec_per_frame)


if __name__ == '__main__':
    main()
