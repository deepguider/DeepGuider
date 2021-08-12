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
        bool initialize(SharedInterface* shared, std::string server_ipaddr, std::string py_module_path = "./../src/vps")
        {
            cv::AutoLock lock(m_mutex);
            m_shared = shared;
            m_server_ipaddr = server_ipaddr;
            if (!VPS::initialize("vps", py_module_path.c_str())) return false;
            return (m_shared != nullptr);
        }

        bool apply(const cv::Mat image, const dg::Timestamp image_time, dg::Point2& streetview_xy, dg::Polar2& relative, double& streetview_confidence, dg::ID& sv_id, cv::Mat& sv_image)
        {
            cv::AutoLock lock(m_mutex);
            if (m_shared == nullptr) return false;

            int N = 1;  // top-1
            Pose2 pose = m_shared->getPose();
            double pose_confidence = m_shared->getPoseConfidence(); // 0: vps search radius = 230m ~ 1: search radius = 30m
            LatLon ll = m_shared->toLatLon(pose);
            if (!VPS::apply(image, N, ll.lat, ll.lon, pose_confidence, image_time, m_server_ipaddr.c_str())) return false;
            if (m_result.empty()) return false;

            Map* map = m_shared->getMap();
            assert(map != nullptr);
            sv_id = m_result[0].id;
            StreetView* view = map->getView(sv_id);
            streetview_xy = *view;
            relative = computeRelative(image, sv_id, sv_image);
            streetview_confidence = m_result[0].confidence;
            return true;
        }

    protected:
        dg::Polar2 computeRelative(const cv::Mat image, ID sv_id, cv::Mat& sv_image)
        {
            dg::Polar2 relative = dg::Polar2(-1, CV_PI);

            if (MapManager::getStreetViewImage(sv_id, sv_image, "f") && !sv_image.empty())
            {
                // TODO: compute relative pose of matched streetview image w.r.t. camera image
            }

            return relative;
        }

        SharedInterface* m_shared = nullptr;
        mutable cv::Mutex m_mutex;
        std::string m_server_ipaddr;
    };

} // End of 'dg'

#endif // End of '__VPS_LOCALIZER__'
