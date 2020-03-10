#include "dg_core.hpp"
#include "dg_map_manager.hpp"
#include "dg_localizer.hpp"
#include "dg_road_recog.hpp"
#include "dg_poi_recog.hpp"
#include "dg_vps.hpp"
#include "dg_guidance.hpp"
#include "python_embedding.hpp"
#include <chrono>

#define VVS_NONASSERT
#include "vvs.h"

using namespace dg;
using namespace std;

class DeepGuiderSimple
{
public:
    DeepGuiderSimple() {}
    ~DeepGuiderSimple();

    bool initialize();
    int run();

protected:
    dg::RoadDirectionRecognizer m_roadTheta;
    dg::POIRecognizer m_poi;
    dg::VPS m_vps;
    dg::SimpleLocalizer m_localizer;
    dg::MapManager m_map_manager;
    dg::Guidance m_guider;
};


DeepGuiderSimple::~DeepGuiderSimple()
{
    //m_roadTheta.clear();
    //m_poi.clear();
    //m_vps.clear();

    //close_python_environment();
}


bool DeepGuiderSimple::initialize()
{
    printf("Initialize deepguider system...\n");

    // initialize python
    if (!init_python_environment("python3", "")) return false;
    printf("\tPython environment initialized!\n");
 
    // initialize map manager
    if (!m_map_manager.initialize()) return false;
    printf("\tMapManager initialized!\n");

    // initialize localizer
    //if (!m_localizer.initialize()) return false;
    printf("\tLocalizer initialized!\n");

    // initialize guidance
    //if (!m_guidance.initialize()) return false;
    printf("\tGuidance initialized!\n");

    // initialize roadTheta
    if (!m_roadTheta.initialize()) return false;
    printf("\tRoadTheta initialized!\n");

    // initialize POI
    //if (!m_poi.initialize()) return false;
    printf("\tPOI initialized!\n");

    // initialize VPS
    if (!m_vps.initialize()) return false;
    printf("\tVPS initialized!\n");

    return true;
}


std::vector<std::pair<double, dg::LatLon>> getExampleGPSData(const char* csv_file = "data/191115_ETRI_asen_fix.csv")
{
    cx::CSVReader csv;
    VVS_CHECK_TRUE(csv.open(csv_file));
    cx::CSVReader::Double2D csv_ext = csv.extDouble2D(1, { 2, 3, 7, 8 }); // Skip the header

    std::vector<std::pair<double, dg::LatLon>> data;
    for (auto row = csv_ext.begin(); row != csv_ext.end(); row++)
    {
        double timestamp = row->at(0) + 1e-9 * row->at(1);
        dg::LatLon ll(row->at(2), row->at(3));
        data.push_back(std::make_pair(timestamp, ll));
    }
    return data;
}


int DeepGuiderSimple::run()
{
    printf("Run deepguider system...\n");

    // load gps sensor data (ETRI dataset)
    const char* gps_file = "data/191115_ETRI_asen_fix.csv";
    const char* background_file = "data/NaverMap_ETRI(Satellite)_191127.png";
    auto gps_data = getExampleGPSData();
    VVS_CHECK_TRUE(!gps_data.empty());
    printf("\tSample gps data loaded!\n");

    // load image sensor data (ETRI dataset)
    const char* video_file = "data/191115_ETRI.avi";
    cv::VideoCapture video_data;
    VVS_CHECK_TRUE(video_data.open(video_file));
    double video_time_offset = gps_data.front().first - 0.5, video_time_scale = 1.75; // Calculated from 'bag' files
    double video_resize_scale = 0.4;
    cv::Point video_offset(32, 542);
    printf("\tSample image data loaded!\n");

    // start & goal position
    dg::LatLon gps_start = gps_data.front().second;
    dg::LatLon gps_goal = gps_data.back().second;
    printf("\tgps_start: lat=%lf, lon=%lf\n", gps_start.lat, gps_start.lon);
    printf("\tgps_goal: lat=%lf, lon=%lf\n", gps_goal.lat, gps_goal.lon);

    // generate path to the destination
    bool ok = m_map_manager.generatePath(gps_start.lat, gps_start.lon, gps_goal.lat, gps_goal.lon);
    if(!ok) return -1;
    dg::Path path = m_map_manager.getPath();
    dg::ID start_node = path.m_points.front();
    dg::ID dest_node = path.m_points.back();
    printf("\tPath generated! start=%zu, dest=%zu\n", start_node, dest_node);

    // download map along the path
    dg::Map map = m_map_manager.getMap(path);
    printf("\tMap downloaded from the server!\n");

    // check consistency between map and path
    int idx = 0;
    bool erase_edge_path = false;
    for(auto itr = path.m_points.begin(); itr!=path.m_points.end();)
    {
        dg::ID id = *itr;
        dg::Map::Node* node = map.findNode(id);
        if(node)
        {
            printf("\tpath[%d]: id=%zu, lat=%lf, lon=%lf, node_type=%d\n", idx, node->data.id, node->data.lat, node->data.lon, node->data.type);
            itr++;
        }
        else
        {
            dg::ID from = *prev(itr);
            dg::ID to = *next(itr);
            dg::Map::Edge* edge = map.findEdge(from, to);
            printf("\tpath[%d]: id=%zu, length=%lf, edge_type=%d\n", idx, id, edge->cost.length, edge->cost.type);

            if(erase_edge_path) itr = path.m_points.erase(itr);
            else itr++;
        }
        idx++;
    }

    // set map to localizer
    VVS_CHECK_TRUE(m_localizer.loadMap(map));

    // guidance: load files for guidance test (ask JSH)
    VVS_CHECK_TRUE(m_guider.setPathNMap(path, map));
    VVS_CHECK_TRUE(m_guider.initializeGuides());

    // guidance: Initial move (ask JSH)
    dg::Guidance::MoveStatus cur_status;
    std::vector<dg::Guidance::RobotGuide> cur_guide;
    cur_guide = m_guider.getInitGuide();

    // localizer: set initial pose of localizer
    dg::ID id_invalid = 0;
    Polar2 rel_pose_defualt(-1, CV_PI);     // default relative pose (invalid)
    double confidence_default = 1.0; 
    dg::Timestamp ts = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
    m_localizer.applyLocClue(start_node, rel_pose_defualt, ts, confidence_default);

    // initial pose of the system
    dg::TopometricPose pose_topo = m_localizer.getPoseTopometric();
    dg::Pose2 pose_metr = m_localizer.getPose();
    dg::LatLon pose_gps = m_localizer.getPoseGPS();
    double pose_confidence = m_localizer.getPoseConfidence();

    // run iteration
    
    int nitr = (int)gps_data.size();
    nitr = 2;
    int itr = 0;
    bool is_arrived = false;
    while (!is_arrived)
    {
        // sensor data
        cv::Mat image = cv::imread("road_sample.jpg");
        ts = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;

        // RoadTheta
        /*
        bool ok = m_roadTheta.apply(image, ts);
        if (ok)
        {
            double angle, confidence;
            m_roadTheta.get(angle, confidence);
            VVS_CHECK_TRUE(m_localizer.applyLocClue(id_invalid, Polar2(-1, angle), ts, confidence));
        }
        */

        // VPS
        int N = 3;  // top-3
        double gps_accuracy = 10;   // error boundary (unit: meter)
        ok = m_vps.apply(image, N, pose_gps.lat, pose_gps.lon, gps_accuracy, ts);
        if (ok)
        {
            std::vector<dg::ID> ids;
            std::vector<dg::Polar2> obs;
            std::vector<double> confs;

            std::vector<VPSResult> streetviews;
            m_vps.get(streetviews);
            printf("vps:\n");
            for (int k = 0; k < (int)streetviews.size(); k++)
            {
                ids.push_back(streetviews[k].id);
                obs.push_back(rel_pose_defualt);
                confs.push_back(streetviews[k].confidence);
                printf("\ttop%d: id=%zu, confidence=%lf, ts=%lf\n", k, streetviews[k].id, streetviews[k].confidence, ts);
            }
            VVS_CHECK_TRUE(m_localizer.applyLocClue(ids, obs, ts, confs));
        }

        // POI
        /*
        ok = m_poi.apply(image, ts);
        if (ok)
        {
            std::vector<dg::ID> ids;
            std::vector<Polar2> obs;
            std::vector<double> confs;

            std::vector<POIResult> pois;
            m_poi.get(pois);
            printf("poi:\n");
            for (int k = 0; k < (int)pois.size(); k++)
            {
                dg::ID poi_id = map_manager.get_poi(pois[k].label);
                ids.push_back(poi_id);
                obs.push_back(rel_pose_defualt);
                confs.push_back(pois[k].confidence);

                printf("\tpoi%d: x1=%d, y1=%d, x2=%d, y2=%d, label=%s, confidence=%lf, ts=%lf\n", k, pois[k].xmin, pois[k].ymin, pois[k].xmax, pois[k].ymax, pois[k].label.c_str(), pois[k].confidence, ts);
            }
            VVS_CHECK_TRUE(m_localizer.applyLocClue(ids, obs, ts, confs));
        }
        */

        // gps update
        if (itr < (int)gps_data.size())
        {
            dg::LatLon gps = gps_data[itr].second;
            printf("lat=%lf, lon=%lf, ts=%lf\n", gps.lat, gps.lon, ts);

            m_localizer.applyGPS(gps, ts);
        }

        // get updated pose & localization confidence
        pose_topo = m_localizer.getPoseTopometric();
        pose_metr = m_localizer.getPose();
        pose_gps = m_localizer.getPoseGPS();
        pose_confidence = m_localizer.getPoseConfidence();
        printf("localizer:\n");
        printf("\ttopo: node=%zu, edge=%d, dist=%lf, ts=%lf\n", pose_topo.node_id, pose_topo.edge_idx, pose_topo.dist, ts);
        printf("\tmetr: x=%lf, y=%lf, theta=%lf, ts=%lf\n", pose_metr.x, pose_metr.y, pose_metr.theta, ts);
        printf("\tconfidence: %lf\n", pose_confidence);

        // generate navigation guidance
        if (pose_confidence > 0.5)
        {
            // invoke normal guidance module
            cur_status = m_guider.applyPose(pose_topo);
            cur_guide = m_guider.getNormalGuide(cur_status);
        }
        else
        {
            // invoke exploration module
            std::vector<dg::Guidance::RobotGuide> getActiveExplorGuide();
        }

        // generate guidance (ask JSH)
        m_guider.printRobotGuide(cur_guide);

        // check arrival
        if (++itr >= nitr)
        {
            is_arrived = true;
        }

        // user break
        char key = cv::waitKey(1);
        if (key == 27) break;
    }

    return 0;
}


int main()
{
    DeepGuiderSimple deepguider;
    deepguider.initialize();
    deepguider.run();

    return 0;
}
