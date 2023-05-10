#ifndef __VPS_LOCALIZER__
#define __VPS_LOCALIZER__

#include "dg_core.hpp"
#include "vps/vps.hpp"
#include "dg_map_manager.hpp"

using namespace std;

namespace dg
{
    /**
    * @brief VPS Localizer
    */
    class VPSLocalizer : public VPS
    {
    public:
        bool initialize(SharedInterface* shared, std::string py_module_path = "./../src/vps", std::string server_ipaddr = "129.254.81.204", std::string server_port="10000", const int use_custom_image_server=0)
        {
            if (!VPS::initialize(py_module_path.c_str(), "vps", "vps")) return false;

            cv::AutoLock lock(m_localizer_mutex);
            m_shared = shared;
            m_server_ipaddr = server_ipaddr;
            m_server_port = server_port;
            m_use_custom_image_server = use_custom_image_server;
            return (m_shared != nullptr);
        }

        bool initialize_without_python(SharedInterface* shared, std::string server_ipaddr = "129.254.81.204", std::string server_port="10000")
        {
            cv::AutoLock lock(m_localizer_mutex);
            m_shared = shared;
            m_server_ipaddr = server_ipaddr;
            m_server_port = server_port;
            return (m_shared != nullptr);
        }

        bool apply(const cv::Mat image, const dg::Timestamp image_time, dg::Point2& streetview_xy, dg::Polar2& relative, double& streetview_confidence, double manual_gps_accuracy = 0.9)
        {
            // Top-1 match
            int N = 1;  // top-1
			dg::Timestamp odo_time = m_odo_time;
            if (m_shared == nullptr) return false;
            Pose2 pose = m_shared->getPose();
            LatLon ll = m_shared->toLatLon(pose);
            double pose_confidence = manual_gps_accuracy; // 0(vps search radius = 200m) ~ 1(search radius = 10m)
			/** In streetview image server, download_radius = int(10 + 190*(1-manual_gps_accuracy)) , 1:10m, 0.95:20m, 0.9:29m, 0.79:50, 0.0:200 meters **/
            if (!VPS::apply(image, N, ll.lat, ll.lon, pose_confidence, image_time, pose.x, pose.y, pose.theta)) return false;

            // matched streetview ID, confidence & image path(custom)
            cv::AutoLock lock(m_localizer_mutex);
            m_sv_id = 0;
            std::vector<VPSResult> vpss = get();
            if (vpss.empty()) return false;
            m_sv_id = vpss[0].id;
            if (m_sv_id == 0) return false;  // no valid matching between query and streetveiw due to lack of db images around query.
            streetview_confidence = vpss[0].confidence;
            //m_custom_dataset_abs_path = vpss[0].custom_dataset_abs_path;  // /home/dg/catkin_ws/devel/lib/dg_simple_ros/./data_vps/dataset/ImageRetrievalDB/custom_dataset_seoul_dbRobot_qRobot_220418/././rosImg/_2022-04-18-14-08-03/uvc_image/006276.jpg
            m_custom_dataset_abs_path = "./data_vps/netvlad_etri_datasets/dbImg/StreetView";  // The parents dir name has to be identical to root_dir = "./data_vps/netvlad_etri_datasets" in src/vps/config_seoul.py 

            // matched streetview xy
            if (m_use_custom_image_server)
            {
                double custom_lat = vpss[0].lat;
                double custom_lon = vpss[0].lon;
                streetview_xy = m_shared->toMetric(dg::LatLon(custom_lat, custom_lon));
				if (custom_lat <= 0 || custom_lon <=0)
				{
					return false;
				}
            }
            else
            {
                Map* map = m_shared->getMap();
                if (map == nullptr) return false;
                StreetView* sv = map->getView(m_sv_id);  // It has .x, .y
                if (sv == nullptr) return false;
                streetview_xy = *sv;            
            }

            // relative pose w.r.t. matched streetview image
            double rpose_tx = vpss[0].t_scaled_x;
            double rpose_ty = vpss[0].t_scaled_y;
            double rpose_tz = vpss[0].t_scaled_z;
            relative.lin = sqrt(rpose_tx*rpose_tx + rpose_ty*rpose_ty + rpose_tz*rpose_tz);
            relative.ang = vpss[0].pan;

            return true;
        }

        dg::ID getViewID()
        {
            cv::AutoLock lock(m_localizer_mutex);
            return m_sv_id;
        }

        inline bool file_exists(const std::string& name)
        {
            struct stat buffer;
            return (stat(name.c_str(), &buffer) == 0); 
        }

        cv::Mat getViewImage()
        {
            cv::Mat sv_image;
            dg::ID sv_id = getViewID();
            if (m_use_custom_image_server)
            {
                std::string fpath;

                fpath = cv::format("%s/%06ld_concat.jpg", m_custom_dataset_abs_path.c_str(), sv_id); // ex) 004648_concat.jpg, concatenated image horizontally
                if (file_exists(fpath))
			    {
			    	sv_image = cv::imread(fpath);
			    	printf("[vps] custom streetview db image [%s] was found.", fpath.c_str());
            		return sv_image;                
			    }

                fpath = cv::format("%s/%06ld.jpg", m_custom_dataset_abs_path.c_str(), sv_id); // ex) 004648.jpg, front
                if (file_exists(fpath))
			    {
			    	sv_image = cv::imread(fpath);
			    	printf("[vps] custom streetview db image [%s] was found.", fpath.c_str());
            		return sv_image;                
			    }

				for (size_t i = 0; i <= 2; i++)
				{
					for (size_t j = 0; j <= 11; j++)
					{
                		fpath = cv::format("%s/%06ld_pitch%d_yaw%02d.jpg", m_custom_dataset_abs_path.c_str(), sv_id, (int)i, (int)j); // ex) 004648_pitch0_yaw00.jpg, front
                		if (file_exists(fpath))
						{
							sv_image = cv::imread(fpath);
							printf("[vps] custom streetview db image [%s] was found.", fpath.c_str());
							break;
						}
					}
				}

            }
            else  // default
            {
                MapManager::getStreetViewImage(sv_id, sv_image, m_server_ipaddr, m_server_port, "f");
            }
            return sv_image;                
        }

		bool set_odometry(double odo_x, double odo_y, double heading, dg::Timestamp odo_time)
		{  // This feature is replaced by m_shared->getPose()
            cv::AutoLock lock(m_localizer_mutex);
			m_odo_pose = Pose2(odo_x, odo_y, heading);
			m_odo_time = odo_time;
			return true;
		}

    protected:
        SharedInterface* m_shared = nullptr;
        std::string m_server_ipaddr;
        std::string m_server_port;
        dg::ID m_sv_id = 0;
        int m_use_custom_image_server=0;
        std::string m_custom_dataset_abs_path;
        cv::Mutex m_localizer_mutex;
		Pose2 m_odo_pose;
		Timestamp m_odo_time;
		double m_odo_x = 0.0;
		double m_odo_y = 0.0;
		double m_heading = 0.0;
    };

} // End of 'dg'

#endif // End of '__VPS_LOCALIZER__'
