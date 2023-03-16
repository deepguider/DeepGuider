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

image_resolution=['AsIs', 'VGA', '720P']

def __filename__():
    ''' _getframe(1) means parents call, (0) means this function '''
    return sys._getframe(1).f_code.co_filename

def __line__():
    ''' _getframe(1) means parents call, (0) means this function '''
    return sys._getframe(1).f_lineno

def __function__():
    ''' _getframe(1) means parents call, (0) means this function '''
    return sys._getframe(1).f_code.co_name

def makedir(fdir):
    import os
    if not os.path.exists(fdir):
        os.makedirs(fdir)

def rmfiles(path='./download_jpg/*.jpg'):
    import os,glob
    for f in glob.glob(path):
        os.remove(f)

#def parse_bags_withSubDir(bag_dir, output_dir, uvc_topic, gps_topic, exact_gps_topic, imu_topic, omni_topic, pose_latlon_file, pose_utm_file, pose_only, init_skip_meter=10, startIdx_uvc=0, startIdx_omni=0, unique_jpgname=False, sec_per_frame=0):
def parse_bags_withSubDir(opts):
    bag_dir, output_dir, uvc_topic, gps_topic, exact_gps_topic, imu_topic, omni_topic, pose_latlon_file, pose_utm_file, pose_only, init_skip_meter, startIdx_uvc, startIdx_omni, unique_jpgname, sec_per_frame = \
                        opts.bag_file, opts.output_dir, opts.uvc_topic, opts.gps_topic, opts.exact_gps_topic, opts.imu_topic, opts.omni_topic, \
                        opts.pose_latlon_file, opts.pose_utm_file, opts.pose_only, opts.init_skip_meter, opts.startIdx, opts.startIdx, opts.unique_jpgname, opts.sec_per_frame

    total_count_uvc = startIdx_uvc
    total_count_omni = startIdx_omni
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
                    #curr_count_uvc, curr_count_omni = parse_single_bag(bag_path, output_dir, uvc_topic, gps_topic, exact_gps_topic, imu_topic, omni_topic, pose_latlon_file, pose_utm_file, pose_only, init_skip_meter, total_count_uvc, total_count_omni, unique_jpgname, sec_per_frame)
                    sensors_to_csv(opts, bag_path)
                    if opts.csv_only == False:
                        parse_single_bag(opts)
                        curr_count_uvc, curr_count_omni = parse_single_bag(opts, bag_path)
                        total_count_uvc += curr_count_uvc
                        total_count_moni += curr_count_moni
                        print "{} [ UVC] : {} / {} ".format(bag_path, curr_count_uvc, total_count_uvc)
                        print "{} [OMNI] : {} / {} ".format(bag_path, curr_count_omni, total_count_omni)

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
        if string_topic.lower() == topic.lower():
            return topic
    for topic in topic_list:
        if string_topic.lower() in topic.lower():
            return topic
    return 'NotFound'

def GetTopic_OMNI(bag, string_topic, verbose=True):
    Title = "OMNI360Camera"
    topic_ret = GetTopicByName(bag, string_topic)
    if topic_ret == 'NotFound':
        topic_ret = GetTopicByName(bag, '/theta_driver_node/image_raw')  # default
    if topic_ret == 'NotFound':
        topic_ret = GetTopicByName(bag, 'theta360z1_raw')  # default
    if topic_ret == 'NotFound':
        topic_ret = GetTopicByName(bag, 'theta360z1_image')
    if topic_ret == 'NotFound':
        topic_ret = GetTopicByName(bag, 'theta360z1_compressed')
    if topic_ret == 'NotFound':
        if verbose == True:
            print "[X] Not Detected ros topic of [{}] : {} ==> [X], Check line {} in {}".format(Title, string_topic, __line__(), os.path.basename(__filename__()))
        return None
    else:
        if verbose == True:
            print "[O]     Detected ros topic of [{}] : {} ==> {}".format(Title, string_topic, topic_ret)
        return topic_ret

def GetTopic_UVC(bag, string_topic, verbose=True):
    Title = "UVC"
    topic_ret = GetTopicByName(bag, string_topic)
    if topic_ret == 'NotFound':
        topic_ret = GetTopicByName(bag, 'uvc')
    if topic_ret == 'NotFound':
        topic_ret = GetTopicByName(bag, 'camera')
    if topic_ret == 'NotFound':
        topic_ret = GetTopicByName(bag, 'image')
    if topic_ret == 'NotFound':
        if verbose == True:
            print "[X] Not Detected ros topic of [{}] : {} ==> [X], Check line {} in {}".format(Title, string_topic, __line__(), os.path.basename(__filename__()))
        return None
    else:
        if verbose == True:
            print "[O]     Detected ros topic of [{}] : {} ==> {}".format(Title, string_topic, topic_ret)
        return topic_ret

def GetTopic_Encoder(bag, string_l_topic, string_r_topic, exact_l_topic=False, exact_r_topic=False, verbose=True):
    ## left encoder topic
    Title = "dg_encoder(left)"
    string_topic = string_l_topic
    exact_topic = exact_l_topic
    topic_ret = GetTopicByName(bag, string_topic)
    if exact_topic == False:  # If you know exact topic name, then do not run this probing process.
        if topic_ret == 'NotFound':
            topic_ret = GetTopicByName(bag, '/dg_encoder/left_tick_data')
        if topic_ret == 'NotFound':
            topic_ret = GetTopicByName(bag, 'left_tick_data')
    if topic_ret == 'NotFound':
        if verbose == True:
            print "[X] Not Detected ros topic of [{}] : {} ==> [X], Check line {} in {}".format(Title, string_topic, __line__(), os.path.basename(__filename__()))
        topic_ret = None
    else:
        if verbose == True:
            print "[O]     Detected ros topic of [{}] : {} ==> {}".format(Title, string_topic, topic_ret)
    topic_l_ret = topic_ret

    ## right encoder topic
    Title = "dg_encoder(right)"
    string_topic = string_r_topic
    exact_topic = exact_r_topic
    topic_ret = GetTopicByName(bag, string_topic)
    if exact_topic == False:
        if topic_ret == 'NotFound':
            topic_ret = GetTopicByName(bag, '/dg_encoder/right_tick_data')
        if topic_ret == 'NotFound':
            topic_ret = GetTopicByName(bag, 'right_tick_data')
    if topic_ret == 'NotFound':
        if verbose == True:
            print "[X] Not Detected ros topic of [{}] : {} ==> [X], Check line {} in {}".format(Title, string_topic, __line__(), os.path.basename(__filename__()))
        topic_ret = None
    else:
        if verbose == True:
            print "[O]     Detected ros topic of [{}] : {} ==> {}".format(Title, string_topic, topic_ret)
    topic_r_ret = topic_ret

    return topic_l_ret, topic_r_ret

def GetTopic_GPS(bag, string_topic, exact_gps_topic=False, verbose=True):
    Title = "GPS"
    topic_ret = GetTopicByName(bag, string_topic)
    if exact_gps_topic == False:
        if topic_ret == 'NotFound':
            topic_ret = GetTopicByName(bag, '/ublox_gps/fix')
        if topic_ret == 'NotFound':
            topic_ret = GetTopicByName(bag, 'andro2linux_gps')
        if topic_ret == 'NotFound':
            topic_ret = GetTopicByName(bag, 'gps')
        if topic_ret == 'NotFound':
            topic_ret = GetTopicByName(bag, 'ascen')
        if topic_ret == 'NotFound':
            topic_ret = GetTopicByName(bag, 'novatel')
            
    if topic_ret == 'NotFound':
        if verbose == True:
            print "[X] Not Detected ros topic of [{}] : {} ==> [X], Check line {} in {}".format(Title, string_topic, __line__(), os.path.basename(__filename__()))
        return None
    else:
        if verbose == True:
            print "[O]     Detected ros topic of [{}] : {} ==> {}".format(Title, string_topic, topic_ret)
        return topic_ret

def GetTopic_IMU(bag, string_topic, verbose=True):
    Title = "IMU"
    topic_ret = GetTopicByName(bag, string_topic)
    if topic_ret == 'NotFound':
        topic_ret = GetTopicByName(bag, 'imu')
    if topic_ret == 'NotFound':
        if verbose == True:
            print "[X] Not Detected ros topic of [{}] : {} ==> [X], Check line {} in {}".format(Title, string_topic, __line__(), os.path.basename(__filename__()))
        return None
    else:
        if verbose == True:
            print "[O]     Detected ros topic of [{}] : {} ==> {}".format(Title, string_topic, topic_ret)
        return topic_ret

#def sensors_to_csv(bag_file, output_dir, uvc_topic, gps_topic, exact_gps_topic, imu_topic):
def sensors_to_csv(opts, bag_file=None):
    if bag_file is None:
        bag_file = opts.bag_file
    output_dir = opts.output_dir
    bag = rosbag.Bag(bag_file, "r")

    ## Get correct topic_name
    print("Searching correct sensors topic names ...")
    gps_topic = GetTopic_GPS(bag, opts.gps_topic, opts.exact_gps_topic, verbose=True)
    imu_topic = GetTopic_IMU(bag, opts.imu_topic, verbose=True)
    enc_left_topic, enc_right_topic = GetTopic_Encoder(bag, opts.enc_left_topic, opts.enc_right_topic, verbose=True)

    makedir(output_dir)

    ## Write gps and imu data to csv file
    print("Saving raw sensor data to .csv ...")
    if gps_topic is not None:
        bag_to_csv(bag_file, gps_topic, output_dir)
    if imu_topic is not None:
        bag_to_csv(bag_file, imu_topic, output_dir)
    if enc_left_topic is not None:
        bag_to_csv(bag_file, enc_left_topic, output_dir)
    if enc_right_topic is not None:
        bag_to_csv(bag_file, enc_right_topic, output_dir)

    bag.close()

def parse_and_save_gps_message(bag, gps_topic, pose_only, output_dir, pose_latlon_file, pose_utm_file, init_skip_meter):
    '''
    Save GPS topic and time stamp at temporary files to use them in synchronization between latlon.txt file and image file name.
    We compare iTime of utmArray and iTime of image to sync. time.
    '''
    debug_write_tempory_posefile = True
    utmLists = []
    latlonLists = []
    if debug_write_tempory_posefile == True:
        # Following two files are not used. For debugging.
        fnamel = os.path.join(output_dir, os.path.splitext(pose_latlon_file)[0] + "_raw" + os.path.splitext(pose_latlon_file)[1])
        fnameu = os.path.join(output_dir, os.path.splitext(pose_utm_file)[0] + "_raw" + os.path.splitext(pose_utm_file)[1])
        fpl = open(fnamel, 'w', buffering = 255)  # Save data at all time stamp
        fpu = open(fnameu, 'w', buffering = 255)  # Save data at all time stamp
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
        else:
            lat = 0.0
            lon = 0.0
            alt = 0.0

        if np.isnan(lat):
            lat = 0.0  # 36.38011018428967
        if np.isnan(lon):
            lon = 0.0  # 127.36808909085782
        if np.isnan(alt):
            alt = 0

        if lat <= 0 or lon <=0:
            continue

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

    return utmArray, latlonArray

def parse_and_save_camera_message(opts, bag, output_dir, pose_latlon_file, pose_utm_file, pose_only, init_skip_meter, camera_topic, utmArray, latlonArray, startIdx, unique_jpgname, save_resolution, compressed, sec_per_frame):
    ''' Save camera image and its GPS location which are corresponded to time stamp '''
    makedir(output_dir)

    fpl = open(os.path.join(output_dir, pose_latlon_file), 'a', buffering = 255) # appending write mode
    fpu = open(os.path.join(output_dir, pose_utm_file), 'a', buffering = 255) # appending write mode

    count = startIdx
    init_sw = True

    state_print_msg = 0
    prev_sec = 0
    bridge = CvBridge()
    if len(utmArray) == 0:
        print("No GPS synced to image. Perhaps you are using the pose_only option.")
        return count

    for topic, msg, t in bag.read_messages(topics=[camera_topic]):
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
            if compressed == True:  # Compressed image
                cv_img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")  # 720, 1280, 3
            else:  # Raw image
                cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            if save_resolution.lower() =='vga':
                cv_img = cv2.resize(cv_img, dsize=(640, 480), interpolation=cv2.INTER_AREA)
            if save_resolution.lower() =='720p':
                cv_img = cv2.resize(cv_img, dsize=(1280, 720), interpolation=cv2.INTER_AREA)
            if save_resolution.lower() =='asis':
                pass
            cv2.imwrite(os.path.join(output_dir, fname), cv_img)
        count += 1
        sys.stdout.write("{} \r".format(fname))
        prev_sec = curr_sec  # 1634028485.9426162

    print(" ")
    fpl.close()
    fpu.close()
    return count

#def parse_single_bag(bag_file, output_dir, uvc_topic, gps_topic, exact_gps_topic, imu_topic, omni_topic, pose_latlon_file, pose_utm_file, pose_only, init_skip_meter=10,  startIdx_uvc=0, startIdx_omni=0, unique_jpgname=False, sec_per_frame=0):
def parse_single_bag(opts, bag_file=None):
    if bag_file is None:
        bag_file = opts.bag_file
    output_dir, uvc_topic, gps_topic, exact_gps_topic, imu_topic, omni_topic, pose_latlon_file, pose_utm_file, pose_only, init_skip_meter,  startIdx_uvc, startIdx_omni, unique_jpgname, sec_per_frame = \
                          opts.output_dir, opts.uvc_topic, opts.gps_topic, opts.exact_gps_topic, opts.imu_topic, opts.omni_topic, \
                          opts.pose_latlon_file, opts.pose_utm_file, opts.pose_only, opts.init_skip_meter, opts.startIdx, opts.startIdx, opts.unique_jpgname, opts.sec_per_frame

    print "In  : %s.\nOut : %s" % (bag_file, output_dir)
    bag = rosbag.Bag(bag_file, "r")

    ## Get correct topic_name

    print "-----------------------------------------------------------------------"
    print "Searching topics from ", GetTopicList(bag)
    print "-----------------------------------------------------------------------"
    uvc_topic = GetTopic_UVC(bag, uvc_topic)
    gps_topic = GetTopic_GPS(bag, gps_topic, exact_gps_topic)
    imu_topic = GetTopic_IMU(bag, imu_topic)
    omni_topic = GetTopic_OMNI(bag, omni_topic)
    enc_left_topic, enc_right_topic = GetTopic_Encoder(bag, opts.enc_left_topic, opts.enc_right_topic, verbose=True)

    #if gps_topic is None or uvc_topic is None:  # If there are no gps and camera topic, then the rosbag file is useless.
    if gps_topic is None:  # If there are no gps and camera topic, then the rosbag file is useless.
        bag.close()
        return 0  # count = 0 

    utmArray, latlonArray = parse_and_save_gps_message(bag, gps_topic, pose_only, output_dir, pose_latlon_file, pose_utm_file, init_skip_meter)

    count_uvc = startIdx_uvc
    count_omni = startIdx_omni

    if uvc_topic is not None:
        output_image_dir = os.path.join(output_dir, "uvc_image")
        camera_topic, compressed, save_resolution, startIdx = uvc_topic, True, "VGA", startIdx_uvc
        count_uvc = parse_and_save_camera_message(opts, bag, output_image_dir, pose_latlon_file, pose_utm_file, pose_only, init_skip_meter, camera_topic, utmArray, latlonArray, startIdx, unique_jpgname, save_resolution, compressed, sec_per_frame)
            
    if omni_topic is not None:
        output_image_dir = os.path.join(output_dir, "omni_image")
        camera_topic, compressed, save_resolution, startIdx = omni_topic, False, "AsIs", startIdx_omni
        count_omni = parse_and_save_camera_message(opts, bag, output_image_dir, pose_latlon_file, pose_utm_file, pose_only, init_skip_meter, camera_topic, utmArray, latlonArray, startIdx, unique_jpgname, save_resolution, compressed, sec_per_frame)

    bag.close()
    return count_uvc - startIdx_uvc, count_omni - startIdx_omni


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
    parser.add_argument("--enc_left_topic", type=str, default='/dg_encoder/left_tick_data', help="")
    parser.add_argument("--enc_right_topic", type=str, default='/dg_encoder/right_tick_data', help="")
    parser.add_argument("--uvc_topic", type=str, default='/uvc_camera/image_raw/compressed', help="A part of string included in Image topic. You can know this string using : rosbag info some.bag")
    parser.add_argument("--gps_topic", type=str, default='/andro2linux_gps', help="A part of string included in GPS topic. You can know this string using : rosbag info some.bag")
    parser.add_argument('--exact_gps_topic', action='store_true', help='Only topic names for gps that match exactly are used.')
    parser.add_argument("--imu_topic", type=str, default='/imu/data', help="A part of string included in IMU topic. You can know this string using : rosbag info some.bag")
    parser.add_argument("--omni_topic", type=str, default='/theta360z1_raw', help="A part of string included in Omni camera topic. You can know this string using : rosbag info some.bag")
    parser.add_argument("--pose_latlon_file", type=str, default='poses_latlon_robot.txt')
    parser.add_argument("--pose_utm_file", type=str, default='poses_utm_robot.txt')
    parser.add_argument('--pose_only', action='store_true', help='It only extracts pose, otherwise extracts pose and images.')
    parser.add_argument('--csv_only', action='store_true', help='It only extracts sensor data (INU, GPS, Encoder) to csv.')
    parser.add_argument('--unique_jpgname', action='store_true', help='It adds rosbag filename in front of jpg name. For example 2020-02-19-15_000000.jpg instead of just 000000.jpg')
    parser.add_argument("--startIdx", type=int, default=0, help="Start index of filename.")
    parser.add_argument("--init_skip_meter", type=int, default=10, help="Skip parking state in which movement is in 10 meters.")
    parser.add_argument("--sec_per_frame", type=float, default=0.0, help="Save image every sec_per_frame seconds. 0.1 means 100ms, 1sec/0.1 (or 10) fps. 0 means all images.")
    opts = parser.parse_args()

    if os.path.isfile(opts.bag_file):
        #sensors_to_csv(opts.bag_file, opts.output_dir, opts.uvc_topic, opts.gps_topic, opts.exact_gps_topic, opts.imu_topic)
        sensors_to_csv(opts)
        #parse_single_bag(opts.bag_file, opts.output_dir, opts.uvc_topic, opts.gps_topic, opts.exact_gps_topic, opts.imu_topic, opts.omni_topic,
        #        opts.pose_latlon_file, opts.pose_utm_file, opts.pose_only, opts.init_skip_meter, opts.startIdx, opts.startIdx, opts.unique_jpgname, opts.sec_per_frame)
        if opts.csv_only == False:
            parse_single_bag(opts)

    if os.path.isdir(opts.bag_file):
        #parse_bags_withSubDir(opts.bag_file, opts.output_dir, opts.uvc_topic, opts.gps_topic, opts.exact_gps_topic, opts.imu_topic, opts.omni_topic,
        #        opts.pose_latlon_file, opts.pose_utm_file, opts.pose_only, opts.init_skip_meter, opts.startIdx, opts.startIdx, opts.unique_jpgname, opts.sec_per_frame)
        parse_bags_withSubDir(opts.bag_file, opts.output_dir, opts.uvc_topic, opts.gps_topic, opts.exact_gps_topic, opts.imu_topic, opts.omni_topic,
                opts.pose_latlon_file, opts.pose_utm_file, opts.pose_only, opts.init_skip_meter, opts.startIdx, opts.startIdx, opts.unique_jpgname, opts.sec_per_frame)


if __name__ == '__main__':
    main()
