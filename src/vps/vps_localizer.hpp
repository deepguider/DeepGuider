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
        bool initialize(SharedInterface* shared, std::string py_module_path = "./../src/vps", std::string server_ipaddr = "129.254.81.204", std::string server_port="10000")
        {
            if (!VPS::initialize(py_module_path.c_str(), "vps", "vps")) return false;

            cv::AutoLock lock(m_localizer_mutex);
            m_shared = shared;
            m_server_ipaddr = server_ipaddr;
            m_server_port = server_port;
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

        bool apply(const cv::Mat image, const dg::Timestamp image_time, dg::Point2& streetview_xy, dg::Point2& custom_streetview_xy, dg::Polar2& relative, double& streetview_confidence, double manual_gps_accuracy, const int load_dbfeat, const int save_dbfeat)
        {
            int N = 1;  // top-1
            if (m_shared == nullptr) return false;
            Pose2 pose = m_shared->getPose();
            LatLon ll = m_shared->toLatLon(pose);
            // double pose_confidence = m_shared->getPoseConfidence(); // 0: vps search radius = 200m ~ 1: search radius = 10m
            double pose_confidence = manual_gps_accuracy; // 0(vps search radius = 200m) ~ 1(search radius = 10m)

			/** In streetview image server, download_radius = int(10 + 190*(1-manual_gps_accuracy)) , 1:10m, 0.95:20m, 0.9:29m, 0.79:50, 0.0:200 meters **/
            if (!VPS::apply(image, N, ll.lat, ll.lon, pose_confidence, image_time, m_server_ipaddr.c_str(), m_server_port.c_str(), load_dbfeat, save_dbfeat)) return false;

            std::vector<VPSResult> vpss = get();

            cv::AutoLock lock(m_localizer_mutex);
            m_sv_id = 0;
            m_sv_image = cv::Mat();
            if (vpss.empty()) return false;

            m_sv_id = vpss[0].id;
            if (m_sv_id == 0) return false;  // no valid matching between query and streetveiw due to lack of db images around query.

			m_custom_utm_x = vpss[0].utm_x;
			m_custom_utm_y = vpss[0].utm_y;

            m_rpose_pan = vpss[0].pan;
            m_rpose_tx = vpss[0].t_scaled_x;
            m_rpose_ty = vpss[0].t_scaled_y;
            m_rpose_tz = vpss[0].t_scaled_z;

            // To do : calculate relativepose from sv_id and (tx,ty,tz)
            relative = computeRelative(image, m_sv_id, m_sv_image);
            relative.lin = sqrt(m_rpose_tx*m_rpose_tx + m_rpose_ty*m_rpose_ty + m_rpose_tz*m_rpose_tz);
            relative.ang = m_rpose_pan;

            streetview_confidence = vpss[0].confidence;
			custom_streetview_xy = dg::Point2(m_custom_utm_x, m_custom_utm_y);

            Map* map = m_shared->getMap();
            if (map == nullptr) return false;
            StreetView* sv = map->getView(m_sv_id);
            if (sv == nullptr) return false;
            streetview_xy = *sv;            

            return true;
        }

        dg::ID getViewID()
        {
            cv::AutoLock lock(m_localizer_mutex);
            return m_sv_id;
        }

        cv::Mat getViewImage()
        {
            cv::AutoLock lock(m_localizer_mutex);
            return m_sv_image;
        }

        double m_custom_utm_x = 0;  // matched utm using custom roadview image
        double m_custom_utm_y = 0;
        double m_rpose_pan = 0;
        double m_rpose_tx = 0;
        double m_rpose_ty = 0;
        double m_rpose_tz = 0;

    protected:
        dg::Polar2 computeRelative(const cv::Mat image, ID sv_id, cv::Mat& sv_image)
        {
            dg::Polar2 relative = dg::Polar2(-1, CV_PI);
            if (MapManager::getStreetViewImage(sv_id, sv_image, m_server_ipaddr, m_server_port, "f") && !sv_image.empty())
            {
                // TODO: compute relative pose of matched streetview image w.r.t. camera image
            }

            return relative;
        }

        SharedInterface* m_shared = nullptr;
        std::string m_server_ipaddr;
        std::string m_server_port;
        dg::ID m_sv_id = 0;
        cv::Mat m_sv_image;
        cv::Mutex m_localizer_mutex;
    };

} // End of 'dg'

#endif // End of '__VPS_LOCALIZER__'
