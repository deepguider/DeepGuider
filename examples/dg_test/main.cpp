#include "dg_core.hpp"
#include "dg_map_manager.hpp"
#include "dg_localizer.hpp"
#include "dg_road_recog.hpp"
#include "dg_poi_recog.hpp"
#include "dg_vps.hpp"
#include "dg_guidance.hpp"
#include "python_embedding.hpp"
#include <chrono>

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
    void generateSensorDataGPSFromPath(dg::Map& map, dg::Path& path, std::vector<dg::LatLon>& gps_data, int interval, double noise_level);

    dg::RoadDirectionRecognizer m_roadTheta;
    dg::POIRecognizer m_poi;
    dg::VPS m_vps;
    dg::SimpleLocalizer m_localizer;
    dg::MapManager m_map_manager;
    dg::Guidance m_guider;
};


DeepGuiderSimple::~DeepGuiderSimple()
{
    m_roadTheta.clear();
    m_poi.clear();
    m_vps.clear();

    close_python_environment();
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


void DeepGuiderSimple::generateSensorDataGPSFromPath(dg::Map& map, dg::Path& path, std::vector<dg::LatLon>& gps_data, int interval, double noise_level)
{
    if (path.countPoints() < 2)
    {
        printf("Error! Path length has to be at least 2\n");
        return;
    }

    gps_data.clear();

    // extract path node gps
    std::vector<dg::LatLon> path_gps;
    for (std::list<dg::ID>::iterator it = path.m_points.begin(); it != path.m_points.end(); it++)
    {
        dg::ID id = (*it);
        dg::Map::Node* node = map.findNode(id);
        double lat = node->data.lat;
        double lon = node->data.lon;

        printf("id = %zu, lat = %lf, lon = %lf\n", id, lat, lon);
        path_gps.push_back(dg::LatLon(lat, lon));
    }

    // extend gps data by interpolation
    gps_data.push_back(path_gps[0]);
    for (int i = 1; i < (int)path_gps.size(); i++)
    {
        double lat_prev = path_gps[i - 1].lat;
        double lon_prev = path_gps[i - 1].lon;
        double lat_cur = path_gps[i].lat;
        double lon_cur = path_gps[i].lon;
        for (int k = 1; k <= interval; k++)
        {
            double lat = lat_prev + k * (lat_cur - lat_prev) / interval;
            double lon = lon_prev + k * (lon_cur - lon_prev) / interval;
            gps_data.push_back(dg::LatLon(lat, lon));
        }
    }
}


int DeepGuiderSimple::run()
{
    printf("Run deepguider system...\n");

    // set start & destination node
    //dg::ID start_node = 0;
    //dg::ID dest_node = 10;

    // generate path to the destination
    //m_map_manager.generatePath(start_node, dest_node);

    // load pre-defined path for the test
    dg::Path path = m_map_manager.getPath("test_simple_path.json");
    dg::ID start_node = path.m_points.front();
    dg::ID dest_node = path.m_points.back();
    printf("\tSample Path generated! start=%zu, dest=%zu\n", start_node, dest_node);

    // load map along the path
    if (!m_map_manager.load(36.384063, 127.374733, 650.0))
    {
        printf("\tFailed to load sample map!\n");
    }
    printf("\tSample Map loaded!\n");

    // set map to localizer
    dg::Map map = m_map_manager.getMap();
    m_localizer.loadMap(map);

    // generate virtual gps sensor data along the path
    int interval = 10;
    double noise_level = 0;
    std::vector<dg::LatLon> gps_data;
    generateSensorDataGPSFromPath(map, path, gps_data, interval, noise_level);
    printf("\tSample gps data generated!\n");


    // guidance: load files for guidance test (ask JSH)
     m_guider.setPathNMap(path, map);
    m_guider.initializeGuides();
    printf("\n");

    // guidance: Initial move (ask JSH)
    m_guider.setInitRobotGuide();
    dg::Guidance::MoveStatus cur_status;
    std::vector<dg::Guidance::RobotGuide> cur_guide;

    // some default variables
    dg::ID id_invalid = 0;
    Polar2 rel_pose_defualt(-1, CV_PI);     // default relative pose (invalid)
    double confidence_default = 1.0; 

    // localizer: set initial pose of localizer
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
    bool is_arrive = false;
    while (!is_arrive)
    {
        // sensor data
        cv::Mat image = cv::imread("road_sample.jpg");
        ts = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;

        // RoadTheta
        bool ok = m_roadTheta.apply(image, ts);
        if (ok)
        {
            double angle, confidence;
            m_roadTheta.get(angle, confidence);
            m_localizer.applyLocClue(id_invalid, Polar2(-1, angle), ts, confidence);
        }

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
            m_localizer.applyLocClue(ids, obs, ts, confs);
        }

        // POI
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
                //dg::ID poi_id = map_manager.get_poi(pois[k].label);
                //ids.push_back(poi_id);
                //obs.push_back(rel_pose_defualt);
                //confs.push_back(pois[k].confidence);

                printf("\tpoi%d: x1=%d, y1=%d, x2=%d, y2=%d, label=%s, confidence=%lf, ts=%lf\n", k, pois[k].xmin, pois[k].ymin, pois[k].xmax, pois[k].ymax, pois[k].label.c_str(), pois[k].confidence, ts);
            }
            //m_localizer.applyLocClue(ids, obs, ts, confs);
        }

        // gps update
        if (itr < (int)gps_data.size())
        {
            dg::LatLon gps = gps_data[itr];
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

        // guidance (ask JSH)
        m_guider.printRobotGuide(cur_guide);

        // check arrival
        if (++itr >= nitr)
        {
            is_arrive = true;
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
