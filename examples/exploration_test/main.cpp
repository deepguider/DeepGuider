#include "dg_core.hpp"
#include "dg_exploration.hpp"
#include "dg_utils.hpp"
#include <chrono>

using namespace dg;
using namespace std;


void test_image_run(ActiveNavigation& active_nav, GuidanceManager::Guidance guidance, const char* image_file = "obs_sample.png", int nItr = 5)
{
    printf("#### Test Image Run ####################\n");
    cv::Mat image = cv::imread(image_file);
    VVS_CHECK_TRUE(!image.empty());
    for (int i = 1; i <= nItr; i++)
    {
        dg::Timestamp t1 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
        VVS_CHECK_TRUE(active_nav.apply(image, guidance, t1));

        std::vector<ExplorationGuidance> actions;
        GuidanceManager::GuideStatus status;
        active_nav.get(actions, status);
        dg::Timestamp t2 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
        printf("iteration: %d (it took %lf seconds)\n", i, t2 - t1);
        for (int k = 0; k < actions.size(); k++)
        {
            printf("\t action %d: [%lf, %lf, %lf]\n", k, actions[k].theta1, actions[k].d, actions[k].theta2);
        }

    }
}


int main()
{
    // Initialize the Python interpreter
    init_python_environment("python3", "");

    // Initialize Python module
    ActiveNavigation active_nav;
    Timestamp t1 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
    if (!active_nav.initialize())
    {
        return -1;
    }
    Timestamp t2 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
    printf("Initialization: it took %lf seconds\n", t2 - t1);

    // Run the Python module
    // test_image_run(active_nav, guidance, false);

    // Clear the Python module
    active_nav.clear();

    // Close the Python Interpreter
    close_python_environment();

    printf("done..\n");

    return 0;
}
