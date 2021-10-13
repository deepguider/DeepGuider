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
        bool initialize(SharedInterface* shared, std::string server_ipaddr = "129.254.81.204", std::string py_module_path = "./../src/vps")
        {
            cv::AutoLock lock(m_mutex);
            m_shared = shared;
            m_server_ipaddr = server_ipaddr;
            if (!VPS::initialize("vps", py_module_path.c_str())) return false;
            return (m_shared != nullptr);
        }

        bool initialize_without_python(SharedInterface* shared, std::string server_ipaddr = "129.254.81.204")
        {
            cv::AutoLock lock(m_mutex);
            m_shared = shared;
            m_server_ipaddr = server_ipaddr;
            return (m_shared != nullptr);
        }

        bool apply(const cv::Mat image, const dg::Timestamp image_time, dg::Point2& streetview_xy, dg::Polar2& relative, double& streetview_confidence)
        {
            cv::AutoLock lock(m_mutex);
            printf("111111111111111111111111111111\n");
            if (m_shared == nullptr) return false;
           printf("2222222222222222222222222222222222\n");    
            // reset interval variables
            m_sv_id = 0;
            m_sv_image = cv::Mat();
           printf("33333333333333333333333\n");    
            // apply recognizer
            int N = 1;  // top-1
            Pose2 pose = m_shared->getPose();
           printf("4444444444444444444\n");                
            // double pose_confidence = m_shared->getPoseConfidence(); // 0: vps search radius = 230m ~ 1: search radius = 30m
            double pose_confidence = 1;
            LatLon ll = m_shared->toLatLon(pose);
            printf("55555555555555555\n");            
            if (!VPS::apply(image, N, ll.lat, ll.lon, pose_confidence, image_time, m_server_ipaddr.c_str())) return false;
            printf("6666666666666666\n");
            Map* map = m_shared->getMap();
            printf("7777777777777777777777\n");             
            if (map == nullptr) return false;
            printf("888888888888888888888888\n");             
            if (m_result.empty()) return false;
            printf("99999999999999999999999\n");             
            m_sv_id = m_result[0].id;
            printf("aaaaaaaaaaaaaaaaaaaaaaaaaa\n");                 
            if (m_sv_id == 0) return false;  // no valid matching between query and streetveiw due to lack of db images around query.
            printf("bbbbbbbbbbbbbbbbbbbbbbbbb\n");              
            StreetView* sv = map->getView(m_sv_id);
            if (sv == nullptr) return false;
            printf("ccccccccccccccccccccccccc\n");              
            streetview_xy = *sv;
            
            relative = computeRelative(image, m_sv_id, m_sv_image);
             printf("ddddddddddddddddddddddd\n");            
            streetview_confidence = m_result[0].confidence;
           
            return true;
        }

        dg::ID getViewID() { return m_sv_id; }

        cv::Mat getViewImage() { return m_sv_image; }

    protected:
        dg::Polar2 computeRelative(const cv::Mat image, ID sv_id, cv::Mat& sv_image)
        {
            dg::Polar2 relative = dg::Polar2(-1, CV_PI);
            if (MapManager::getStreetViewImage(sv_id, sv_image, "f", 10, "coex") && !sv_image.empty())
            {
                // TODO: compute relative pose of matched streetview image w.r.t. camera image
            }

            return relative;
        }

        SharedInterface* m_shared = nullptr;
        mutable cv::Mutex m_mutex;
        std::string m_server_ipaddr;
        dg::ID m_sv_id = 0;
        cv::Mat m_sv_image;
    };

} // End of 'dg'

#endif // End of '__VPS_LOCALIZER__'
