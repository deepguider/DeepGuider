import numpy as np
import cv2
from ipdb import set_trace as bp

## Author :  ccsmm@etri.re.kr
## Current version cover site, topic_name.
## If you add an extra New Set like site and topic,
## then you need to add "val3 = self.get_single_of_NewSet(node_name)" at the end of self.read()

class load_cv2_yaml:
    def __init__(self, yaml_path="./dg_ros.yml"):
        self.yaml_path = yaml_path
        self.fs = None
        self.open()

    def open(self, re_load=False):
        if (self.fs is None) or (re_load == True):
            self.fs = cv2.FileStorage(self.yaml_path, cv2.FILE_STORAGE_READ)

    def _read(self, node_name_d0=None, node_name_d1=None, node_name_d2=None):
        '''
        val = _read("enable_vps")
        '''
        if node_name_d0 is not None:  # name in depth 0 (top)
            node = self.fs.getNode(node_name_d0)

        if node_name_d1 is not None:  # name in second depth
            node = node.getNode(node_name_d1)

        if node_name_d2 is not None:  # name in third depth
            node = node.getNode(node_name_d2)

        value = None
        if not node.empty():
            if node.isReal():
                value = node.real()
            elif node.isInt():
                value = int(node.real())
            elif node.isString():
                value = node.string()
            elif node.isSeq():
                value = []
                for i in range(node.size()):
                    node_seq = node.at(i)
                    if node_seq.isReal():
                        value_seq = node_seq.real()
                    elif node_seq.isInt():
                        value_seq = int(node_seq.real())
                    elif node_seq.isString():
                        value_seq = node_seq.string()
                    else:
                        print("Unexpected value format at {}".format(node_d0))
                        exit(0)
                    value.append(value_seq)

        return value

    def get_single_of_site(self, node_name="map_image_path", verbose=False):
        site = self.get_site()
        val = self._read(site, node_name)
        if verbose:
            print("{} : {}".format(node_name, val))
        return val

    def get_single_of_topic(self, node_name="topic_gps", verbose=False):
        topic = self.get_topic()
        val = self._read(topic, node_name)
        if verbose:
            print("{} : {}".format(node_name, val))
        return val

    def read(self, node_name="enable_vps", verbose=False):
        val0 = self._read(node_name)
        val1 = self.get_single_of_site(node_name)

        if val1 is not None:
            val = val1
        elif val0 is not None:
            val = val0
        else:
            print("[Intersection] Error during _reading {} from config file(.yml)".format(node_name))
            exit(0)

        if verbose:
            print("{} : {}".format(node_name, val))
        return val
    
    def get_list_at(self, index_name="site_index", lists_name="site_names", verbose=False):
        ## Site
        index = self._read(index_name)
        lists = self._read(lists_name)
        val = lists[index]
        if verbose:
            #print("{}/{} : {}".format(index, lists, val))  # for debugging
            print("{} : {}".format(index_name, val))  # for debugging
        return val

    def get_ip(self, verbose=False):
        index = "server_ip_index"
        lists = "server_ip_list"
        val = self.get_list_at(index, lists, verbose)
        return val
    
    def get_site(self, verbose=False):
        index = "site_index"
        lists = "site_names"
        val = self.get_list_at(index, lists, verbose)
        return val
    
    def get_topic(self, verbose=False):
        index = "topic_name_index"
        lists = "topic_names_set"
        val = self.get_list_at(index, lists, verbose)
        return val
    
    def get_robot_site(self, verbose=False):
        index = "robot_site_index"
        lists = "robot_site_set"
        val = self.get_list_at(index, lists, verbose)
        return val

    def close(self):
        self.fs.release()

def for_example(yml):
    ## For examples,
    ## For depth 0
    print("\n>>>> Example begin...")
    node_name="enable_360cam";val = yml.read(node_name); print("{} : {}".format(node_name, val))
    node_name="site_index";val = yml.read(node_name); print("{} : {}".format(node_name, val))
    node_name="site_names";val = yml.read(node_name); print("{} : {}".format(node_name, val))

    node_name="site_index";site_index = yml.read(node_name)
    node_name="site_names";site_names = yml.read(node_name)
    sites = site_names[site_index];print(sites)

    ## For depth 1
    ## You can read value under second level using following two options
    ## Option 1 : _read(sites, node_name)
    node_name="map_image_path";val = yml._read(sites, node_name); print("{} : {}".format(node_name, val))
    node_name="map_ref_point_latlon";val = yml._read(sites, node_name); print("{} : {}".format(node_name, val))
    ## Option 2 : read(node_name) # Use this for convenience
    node_name="map_image_path";val = yml.read(node_name); print("{}(final) : {}".format(node_name, val))
    node_name="map_ref_point_latlon";val = yml.read(node_name); print("{}(final) : {}".format(node_name, val))

    print("<<<< Example end...")

def for_vps(yml):
    verbose = True
    # yml.read(yml, "", True)  # True enable print

    print("\n>>>> Example for vps begin...")
    print("\n  ** Index **")
    yml.get_site(verbose)
    yml.get_topic(verbose)
    yml.get_robot_site(verbose)
    
    print("\n  ** Image Server **")
    yml.get_ip(verbose)
    yml.read("image_server_port", verbose)

    print("\n  ** ROS Topics **")
    yml.read("topic_360cam", verbose)
    yml.read("topic_360cam_crop", verbose)
    yml.read("topic_cam", verbose)
    yml.read("topic_gps", verbose)
    yml.read("topic_odo", verbose)

    print("\n  ** Map Info. **")
    yml.read("map_image_path", verbose)
    yml.read("map_data_path", verbose)

    print("\n  ** VPS Params. **")
    yml.read("enable_vps", verbose)
    yml.read("vps_load_dbfeat", verbose)
    yml.read("vps_save_dbfeat", verbose)
    yml.read("vps_use_custom_image_server", verbose)
    yml.read("vps_gps_accuracy", verbose)

    print("\n<<<< Example for vps end...")


def for_intersection(yml):
    verbose = True
    # yml.read(yml, "", True)  # True enable print

    yml.read("intersection_cam", verbose)

def get_param_for_intersection():
    yml = load_cv2_yaml("./dg_ros.yml")

    #for_example(yml)
    for_intersection(yml)

    yml.close()

if __name__ == "__main__":
    get_param_for_intersection()
