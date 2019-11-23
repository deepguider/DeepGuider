#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION

#include "dg_core.hpp"
#include "dg_map_manager.hpp"
#include "dg_localizer.hpp"
#include "dg_road_recog.hpp"
//#include "dg_poi_recog.hpp"
#include "dg_vps.hpp"
#include <chrono>

#define PY_SSIZE_T_CLEAN
#include <Python.h>

class DeepGuiderSimple
{
public:
    DeepGuiderSimple() {}
    ~DeepGuiderSimple() { close_python_environment(); }

    bool initialize();
    int run();

protected:
    bool init_python_environment();
    void close_python_environment();
    bool m_python_initialized = false;

    dg::RoadDirectionRecognizer m_recognizer_roaddir;
    dg::VPS m_recognizer_vps;
    dg::SimpleMetricLocalizer m_localizer;
    dg::MapManager m_map_manager;
};

using namespace dg;
using namespace std;


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

    // initialize recognizers
    if (!m_recognizer_roaddir.initialize()) return false;
    printf("\tRoadDirectionRecognizer initialized!\n");

    if (!m_recognizer_vps.initialize()) return false;
    printf("\tVPS initialized!\n");

    return true;
}


void DeepGuiderSimple::close_python_environment()
{
    if (!m_python_initialized) return;

    // clear recognizers memory
    m_recognizer_roaddir.clear();
    m_recognizer_vps.clear();

    // Close the Python Interpreter
    Py_FinalizeEx();
}


bool DeepGuiderSimple::init_python_environment()
{
    close_python_environment();

    wchar_t* program = Py_DecodeLocale("dg_test", NULL);
    if (program == NULL){
        fprintf(stderr, "Fatal error: cannot decode dg_test\n");
        return false;
    }
    Py_SetProgramName(program);
    Py_Initialize();
    m_python_initialized = true;
    return true;
}


int DeepGuiderSimple::run()
{
    printf("Run deepguider system...\n");

    // set start & destination node
    //dg::ID start_node = 0;
    //dg::ID dest_node = 10;

    // generate path to the destination
    //m_map_manager.generatePath(start_node, dest_node);

    // get the generated path
    dg::Path path = m_map_manager.getPath("test_simple_Path.json");
    dg::ID start_node = path.m_points.front();
    dg::ID dest_node = path.m_points.back();
    printf("\tSample Path generated!\n");

    // load map & get map
    if (!m_map_manager.load(36.384063, 127.374733, 650.0))
    {
        printf("\tSample Map failed to load!\n");
    }
    printf("\tSample Map loaded!\n");

    // set map to localizer
    dg::Map map = m_map_manager.getMap();
    m_localizer.loadMap(map);

    // set initial pose
    dg::Timestamp t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
    m_localizer.applyLocClue(start_node, Polar2(-1, -1), t, 1);

    dg::Pose2 pose_metr = m_localizer.getPose();
    dg::TopometricPose pose_topo = m_localizer.getPoseTopometric();
    double confidence = m_localizer.getPoseConfidence();

    // run iteration
    int nitr = 2;
    int i = 0;
    bool is_arrive = false;
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
            m_recognizer_roaddir.get(angle, prob);
            m_localizer.applyLocClue(pose_topo.node_id, Polar2(-1, angle), t, prob);
        }

        ok = m_recognizer_vps.apply(image, t, pose_metr.y, pose_metr.x, 10.0);
        if (ok)
        {
            double lat, lon, prob;
            m_recognizer_vps.get(lat, lon, prob);
            m_localizer.applyPosition(dg::LonLat(lon, lat), t, prob);
        }

        // get updated pose & localization confidence
        pose_metr = m_localizer.getPose();
        pose_topo = m_localizer.getPoseTopometric();
        confidence = m_localizer.getPoseConfidence();

        // generate navigation guidance
        if (confidence > 0.5)
        {
            // invoke normal guidance module
        }
        else
        {
            // invoke exploration module
        }

        // check arrival
        if (++i >= nitr)
        {
            is_arrive = true;
        }
    }

    return 0;
}


int main(int argc, char* argv[])
{
    DeepGuiderSimple deepguider;
    bool ok = deepguider.initialize();
    return deepguider.run();
}
