#include "dg_core.hpp"
#include "dg_map_manager.hpp"
#include "dg_localizer.hpp"
#include "dg_road_recog.hpp"
#include "dg_poi_recog.hpp"
#include "dg_vps.hpp"
#include "dg_guidance.hpp"
#include "python_embedding.hpp"
#include <chrono>

#define VVS_NO_ASSERT
#include "vvs.h"

using namespace dg;
using namespace std;


class DeepGuiderSimple
{
public:
    DeepGuiderSimple() {}
    ~DeepGuiderSimple();

    bool initialize();
    int run(const char* gps_file = "data/191115_ETRI_asen_fix.csv", const char* video_file = "data/191115_ETRI.avi", const char* background_file = "data/NaverMap_ETRI(Satellite)_191127.png");

protected:
    dg::RoadDirectionRecognizer m_roadTheta;
    dg::POIRecognizer m_poi;
    dg::VPS m_vps;
    dg::SimpleLocalizer m_localizer;
    dg::MapManager m_map_manager;
    dg::Guidance m_guider;

    // enable/disable submodules
    bool enable_roadtheta = false;
    bool enable_vps = false;
    bool enable_poi = false;
    bool recording = false;
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
    //if (!m_guider.initialize()) return false;
    printf("\tGuidance initialized!\n");

    // initialize roadTheta
    if (enable_roadtheta && !m_roadTheta.initialize()) return false;
    if (enable_roadtheta) printf("\tRoadTheta initialized!\n");

    // initialize VPS
    if (enable_vps && !m_vps.initialize()) return false;
    if (enable_vps) printf("\tVPS initialized!\n");

    // initialize POI
    if (enable_poi && !m_poi.initialize()) return false;
    if (enable_poi) printf("\tPOI initialized!\n");

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


int DeepGuiderSimple::run(const char* gps_file /*= "data/191115_ETRI_asen_fix.csv"*/, const char* video_file /*= "data/191115_ETRI.avi"*/, const char* background_file /*= "data/NaverMap_ETRI(Satellite)_191127.png"*/)
{
    cx::VideoWriter video;
    if (recording)
    {
        time_t start_t;
        time(&start_t);
        tm _tm = *localtime(&start_t);
        char szfilename[255];
        strftime(szfilename, 255, "dg_test_%y%m%d_%H%M%S.avi", &_tm);
        std::string filename = szfilename;
        video.open(filename, 30);
    }

    printf("Run deepguider system...\n");

    // load gps sensor data (ETRI dataset)
    auto gps_data = getExampleGPSData(gps_file);
    VVS_CHECK_TRUE(!gps_data.empty());
    printf("\tSample gps data loaded!\n");

    // load image sensor data (ETRI dataset)
    cv::VideoCapture video_data;
    VVS_CHECK_TRUE(video_data.open(video_file));
    double video_time_offset = gps_data.front().first - 0.5, video_time_scale = 1.75; // Calculated from 'bag' files
    double video_resize_scale = 0.4;
    cv::Point video_offset(32, 542);
    double video_time = video_time_scale * video_data.get(cv::VideoCaptureProperties::CAP_PROP_POS_MSEC) / 1000 + video_time_offset;
    printf("\tSample video data loaded!\n");

    // start & goal position
    dg::LatLon gps_start = gps_data.front().second;
    dg::LatLon gps_dest = gps_data.back().second;
    printf("\tgps_start: lat=%lf, lon=%lf\n", gps_start.lat, gps_start.lon);
    printf("\tgps_dest: lat=%lf, lon=%lf\n", gps_dest.lat, gps_dest.lon);

    // generate path to the destination
    dg::Path path = m_map_manager.getPath(gps_start.lat, gps_start.lon, gps_dest.lat, gps_dest.lon);
    dg::ID nid_start = path.pts.front().node->id;
    dg::ID nid_dest = path.pts.back().node->id;
    printf("\tPath generated! start=%zu, dest=%zu\n", nid_start, nid_dest);

    // fix lat,lon error in the download map (temporal fix -> it should be corrected at the map server)
    dg::Map& map = m_map_manager.getMap();
    for (auto itr = map.nodes.begin(); itr != map.nodes.end(); itr++)
    {
        if (itr->lat > itr->lon) { double l = itr->lat; itr->lat = itr->lon; itr->lon = l; }
    }

    // check consistency between map and path
    for (int idx = 0; idx < (int)path.pts.size(); idx++)
    {
        if (path.pts[idx].node)
        {
            dg::ID node_id = path.pts[idx].node->id;
            dg::Node* node = map.findNode(node_id);
            printf("\tpath[%d]: node_id=%zu, id=%zu, lat=%lf, lon=%lf, type=%d\n", idx, node->id, node_id, node->lat, node->lon, node->type);
        }
        if (path.pts[idx].edge)
        {
            dg::ID edge_id = path.pts[idx].edge->id;
            dg::ID from = path.pts[idx].edge->node1->id;
            dg::ID to = path.pts[idx].edge->node2->id;
            dg::Edge* edge = map.findEdge(from, to);
            printf("\tpath[%d]: edge_id=%zu, id=%zu, length=%lf, type=%d\n", idx, edge->id, edge_id, edge->length, edge->type);
        }
    }

    // set map to localizer
    dg::LatLon ref_node(36.383837659737, 127.367880828442);
    VVS_CHECK_TRUE(m_localizer.setReference(ref_node));
    VVS_CHECK_TRUE(m_localizer.loadMap(map));

    // prepare GUI map
    dg::SimpleRoadPainter painter;
    VVS_CHECK_TRUE(painter.setParamValue("pixel_per_meter", 1.045));
    VVS_CHECK_TRUE(painter.setParamValue("canvas_margin", 0));
    VVS_CHECK_TRUE(painter.setParamValue("canvas_offset", { 344, 293 }));
    VVS_CHECK_TRUE(painter.setParamValue("grid_step", 100));
    VVS_CHECK_TRUE(painter.setParamValue("grid_unit_pos", { 120, 10 }));
    VVS_CHECK_TRUE(painter.setParamValue("node_radius", 4));
    VVS_CHECK_TRUE(painter.setParamValue("node_font_scale", 0));
    VVS_CHECK_TRUE(painter.setParamValue("node_color", { 255, 50, 255 }));
    VVS_CHECK_TRUE(painter.setParamValue("edge_color", { 200, 100, 100 }));
    VVS_CHECK_TRUE(painter.setParamValue("edge_thickness", 2));

    cv::Mat map_image = cv::imread(background_file);
    VVS_CHECK_TRUE(!map_image.empty());
    dg::RoadMap road_map = m_localizer.getMap();
    VVS_CHECK_TRUE(painter.drawMap(map_image, road_map));
    dg::CanvasInfo map_info = painter.getCanvasInfo(road_map, map_image.size());

    dg::DirectedGraph<dg::Point2ID, double>::Node* node_prev = nullptr;
    for (int idx = 0; idx < (int)path.pts.size(); idx++)
    {
        dg::ID node_id = path.pts[idx].node->id;
        dg::DirectedGraph<dg::Point2ID, double>::Node* node = road_map.findNode(node_id);
        if (node){
            if (node_prev) painter.drawEdge(map_image, map_info, node_prev->data, node->data, 0, cv::Vec3b(200, 0, 0), 2);
            if (node_prev) painter.drawNode(map_image, map_info, node_prev->data, 5, 0, cv::Vec3b(50, 0, 255));
            painter.drawNode(map_image, map_info, node->data, 5, 0, cv::Vec3b(50, 0, 255));
            node_prev = node;
        }
    }
    printf("\tGUI map initialized!\n");
    cv::namedWindow("deep_guider");

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
    m_localizer.applyLocClue(nid_start, rel_pose_defualt, gps_data[0].first, confidence_default);

    // initial pose of the system
    dg::TopometricPose pose_topo = m_localizer.getPoseTopometric();
    dg::Pose2 pose_metric = m_localizer.getPose();
    dg::LatLon pose_gps = m_localizer.getPoseGPS();
    double pose_confidence = m_localizer.getPoseConfidence();

    // run iteration
    int maxItr = (int)gps_data.size();
    //maxItr = 10;
    int itr = 0;
    bool is_arrived = false;
    while (!is_arrived && itr<maxItr)
    {
        dg::Timestamp t1 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;

        // gps update
        const dg::Timestamp gps_time = gps_data[itr].first;
        const dg::LatLon gps_datum = gps_data[itr].second;
        printf("[GPS] lat=%lf, lon=%lf, ts=%lf\n", gps_datum.lat, gps_datum.lon, gps_time);
        VVS_CHECK_TRUE(m_localizer.applyGPS(gps_datum, gps_time));

        // video image
        cv::Mat video_image;
        while (video_time <= gps_time)
        {
            video_data >> video_image;
            if (video_image.empty()) break;
            video_time = video_time_scale * video_data.get(cv::VideoCaptureProperties::CAP_PROP_POS_MSEC) / 1000 + video_time_offset;
        }

        // RoadTheta
        if (enable_roadtheta && !video_image.empty() && m_roadTheta.apply(video_image, video_time))
        {
            double angle, confidence;
            m_roadTheta.get(angle, confidence);
            VVS_CHECK_TRUE(m_localizer.applyLocClue(id_invalid, Polar2(-1, angle), video_time, confidence));
        }

        // VPS
        int N = 3;  // top-3
        double gps_accuracy = 1;   // 0: search radius = 230m ~ 1: search radius = 30m
        if (enable_vps && !video_image.empty() && m_vps.apply(video_image, N, pose_gps.lat, pose_gps.lon, gps_accuracy, video_time))
        {
            std::vector<dg::ID> ids;
            std::vector<dg::Polar2> obs;
            std::vector<double> confs;
            std::vector<VPSResult> streetviews;
            m_vps.get(streetviews);
            printf("[VPS]\n");
            for (int k = 0; k < (int)streetviews.size(); k++)
            {
                ids.push_back(streetviews[k].id);
                obs.push_back(rel_pose_defualt);
                confs.push_back(streetviews[k].confidence);
                printf("\ttop%d: id=%zu, confidence=%lf, ts=%lf\n", k, streetviews[k].id, streetviews[k].confidence, video_time);
            }
            VVS_CHECK_TRUE(m_localizer.applyLocClue(ids, obs, video_time, confs));
        }

        // POI
        if(enable_poi && !video_image.empty() && m_poi.apply(video_image, video_time))
        {
            std::vector<dg::ID> ids;
            std::vector<Polar2> obs;
            std::vector<double> confs;
            std::vector<POIResult> pois;
            m_poi.get(pois);
            printf("[POI]\n");
            for (int k = 0; k < (int)pois.size(); k++)
            {
                //dg::ID poi_id = m_map_manager.get_poi(pois[k].label);
                dg::ID poi_id = 0;
                ids.push_back(poi_id);
                obs.push_back(rel_pose_defualt);
                confs.push_back(pois[k].confidence);
                printf("\tpoi%d: x1=%d, y1=%d, x2=%d, y2=%d, label=%s, confidence=%lf, ts=%lf\n", k, pois[k].xmin, pois[k].ymin, pois[k].xmax, pois[k].ymax, pois[k].label.c_str(), pois[k].confidence, video_time);
            }
            VVS_CHECK_TRUE(m_localizer.applyLocClue(ids, obs, video_time, confs));
        }

        // get updated pose & localization confidence
        pose_topo = m_localizer.getPoseTopometric();
        pose_metric = m_localizer.getPose();
        pose_gps = m_localizer.getPoseGPS();
        pose_confidence = m_localizer.getPoseConfidence();
        printf("[Localizer]\n");
        printf("\ttopo: node=%zu, edge=%d, dist=%lf, ts=%lf\n", pose_topo.node_id, pose_topo.edge_idx, pose_topo.dist, gps_time);
        printf("\tmetr: x=%lf, y=%lf, theta=%lf, ts=%lf\n", pose_metric.x, pose_metric.y, pose_metric.theta, gps_time);
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
        // TODO

        //------------------------------ GUI display ---------------------------------------------------------
        // draw gps history on the map
        dg::Point2 gps_pt = m_localizer.toMetric(gps_datum);
        painter.drawNode(map_image, map_info, dg::Point2ID(0, gps_pt), 1, 0, cv::Vec3b(0, 255, 0));

        // draw video image as subwindow on the map
        cv::Mat image = map_image.clone();
        if (!video_image.empty())
        {
            cv::resize(video_image, video_image, cv::Size(), video_resize_scale, video_resize_scale);
            cv::Rect rect(video_offset, video_offset + cv::Point(video_image.cols, video_image.rows));
            if (rect.br().x < image.cols && rect.br().y < image.rows) image(rect) = video_image * 1;
        }

        // draw robot on the map
        dg::Pose2 pose_m = m_localizer.toTopmetric2Metric(pose_topo);
        painter.drawNode(image, map_info, dg::Point2ID(0, pose_m.x, pose_m.y), 10, 0, cx::COLOR_YELLOW);
        painter.drawNode(image, map_info, dg::Point2ID(0, pose_m.x, pose_m.y), 8, 0, cx::COLOR_BLUE);

        // draw status message (localization)
        cv::String info_topo = cv::format("Node: %zu, Edge: %d, D: %.3f (Lat: %.6f, Lon: %.6f)", pose_topo.node_id, pose_topo.edge_idx, pose_topo.dist, pose_gps.lat, pose_gps.lon);
        cv::putText(image, info_topo, cv::Point(5, 30), cv::FONT_HERSHEY_PLAIN, 1.9, cv::Scalar(0, 200, 0), 2);

        // recording
        if (recording) video << image;

        cv::imshow("deep_guider", image);
        int key = cv::waitKey(1);
        if (key == cx::KEY_SPACE) key = cv::waitKey(0);
        if (key == cx::KEY_ESC) break;

        dg::Timestamp t2 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
        printf("Iteration: %d (it took %lf seconds)\n", itr, t2 - t1);

        // update iteration
        itr++;
    }

    printf("End deepguider system...\n");

    return 0;
}


int main()
{
    DeepGuiderSimple deepguider;
    deepguider.initialize();
    deepguider.run();

    return 0;
}
