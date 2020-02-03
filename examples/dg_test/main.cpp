#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION

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

    dg::RoadDirectionRecognizer m_recognizer_roaddir;
    dg::VPS m_recognizer_vps;
    dg::POIRecognizer m_recognizer_poi;
    dg::SimpleLocalizer m_localizer;
    dg::MapManager m_map_manager;
    dg::Guidance m_guider;
};


DeepGuiderSimple::~DeepGuiderSimple()
{
    m_recognizer_roaddir.clear();
    m_recognizer_vps.clear();
    m_recognizer_poi.clear();

    close_python_environment();
}


bool DeepGuiderSimple::initialize()
{
    printf("Initialize deepguider system...\n");

    // initialize python
    if (!init_python_environment()) return false;
    printf("\tPython environment initialized!\n");

    // initialize map manager
    //if (!m_map_manager.initialize()) return false;
    printf("\tMapManager initialized!\n");

    // initialize localizer
    //if (!m_localizer.initialize()) return false;
    printf("\tLocalizer initialized!\n");

    // initialize guidance
    //if (!m_guidance.initialize()) return false;
    printf("\tGuidance initialized!\n");

    // initialize recognizers
    if (!m_recognizer_roaddir.initialize()) return false;
    printf("\tRoadDirectionRecognizer initialized!\n");
    //if (!m_recognizer_poi.initialize()) return false;
    //printf("\tPOI initialized!\n");
    //if (!m_recognizer_vps.initialize()) return false;
    //printf("\tVPS initialized!\n");

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

        printf("id = %llu, lat = %lf, lon = %lf\n", id, lat, lon);
        path_gps.push_back(dg::LatLon(lat, lon));
    }

    // exdend gps data by interpolation
    gps_data.push_back(path_gps[0]);
    for (int i = 1; i < (int)path_gps.size(); i++)
    {
        double lat_prev = path_gps[i-1].lat;
        double lon_prev = path_gps[i-1].lon;
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
    dg::Path path = m_map_manager.getPath("test_simple_Path.json");
    dg::ID start_node = path.m_points.front();
    dg::ID dest_node = path.m_points.back();
    printf("\tSample Path generated! start=%llu, dest=%llu\n", start_node, dest_node);

    // load map along the path
    if (!m_map_manager.load(36.384063, 127.374733, 650.0))
    {
        printf("\tFailed to load sample map!\n");
    }
    printf("\tSample Map loaded!\n");

    // set map to localizer
    dg::Map map = m_map_manager.getMap();
    m_localizer.loadMap(map);

    // generate virtual gps sensor data from the path
    int interval = 10;
    double noise_level = 0;    
    std::vector<dg::LatLon> gps_data;
    generateSensorDataGPSFromPath(map, path, gps_data, interval, noise_level);
    printf("\tSample gps data generated!\n");

    //load files for guidance test (ask JSH)
    m_guider.loadPathFiles("Path_ETRIFrontgateToParisBaguette.txt", m_guider.m_path);
    std::vector<dg::TopometricPose> Loc;
    m_guider.loadLocFiles("Loc_ETRIFrontgateToParisBaguette.txt", Loc);
    m_guider.generateGuide();	//generate guide

    //Initial move (ask JSH)
    dg::Guidance::Guide initG(m_guider.m_guide[0]);
    dg::Guidance::Action InitA(dg::Guidance::GO_FORWARD, 0);
    std::vector<dg::Guidance::InstantGuide> curGuide;
    curGuide.push_back(dg::Guidance::InstantGuide(initG, InitA));
    dg::TopometricPose curPose;
    dg::Guidance::Status curStatus;

    // set initial pose
    dg::Timestamp t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
    m_localizer.applyLocClue(start_node, Polar2(-1, -1), t, 1);

    dg::Pose2 pose_metr = m_localizer.getPose();
    dg::TopometricPose pose_topo = m_localizer.getPoseTopometric();
    double confidence = m_localizer.getPoseConfidence();

    // run iteration
    int nitr = (int)gps_data.size();
    //nitr = 2;
    int itr = 0;
    bool is_arrive = false;
    printf("\n");
    while (!is_arrive)
    {
        // sensor data
        cv::Mat image = cv::imread("road_sample.jpg");
        t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;

        // run recognizers & update localizer
        bool ok = m_recognizer_roaddir.apply(image, t);
        if (ok)
        {
            double angle, prob;
            //m_recognizer_roaddir.get(angle, prob);
            //m_localizer.applyLocClue(pose_topo.node_id, Polar2(-1, angle), t, prob);
        }

        // VPS
        int N = 3;  // top-3
        double gps_accuracy = 10;   // meter error boundary
        ok = m_recognizer_vps.apply(image, N, pose_metr.x, pose_metr.y, gps_accuracy, t);
        if (ok)
        {
            std::vector<VPSResult> streetviews;
            m_recognizer_vps.get(streetviews);
            printf("vps:\n");
            for (int k = 0; k < streetviews.size(); k++)
            {
                printf("\ttop%d: id=%ld, confidence=%lf, t=%lf\n", k, streetviews[k].id, streetviews[k].confidence, t);
            }
            //m_localizer.applyGPS(dg::LatLon(lat, lon), t, confidence);
        }

        // POI


        // gps update
        if (itr < (int)gps_data.size())
        {
            dg::LatLon gps = gps_data[itr];
            //m_localizer.applyPosition()
            //m_localizer.applyPosition(gps, t);
            printf("lat=%lf, lon=%lf, time=%lf\n", gps.lat, gps.lon, t);
        }

        // get updated pose & localization confidence
        pose_metr = m_localizer.getPose();
        pose_topo = m_localizer.getPoseTopometric();
        confidence = m_localizer.getPoseConfidence();
        printf("x=%lf, y=%lf, theta=%lf, time=%lf\n", pose_metr.x, pose_metr.y, pose_metr.theta, t);

        // generate navigation guidance
        if (confidence > 0.5)
        {
            // invoke normal guidance module
        }
        else
        {
            // invoke exploration module
        }

        // generate guidance (ask JSH)
        curPose = Loc[itr];
        curStatus = m_guider.checkStatus(curPose);
        curGuide = m_guider.provideNormalGuide(curGuide, curStatus);

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


int main(int argc, char* argv[])
{
    DeepGuiderSimple deepguider;
    bool ok = deepguider.initialize();
    return deepguider.run();
}
