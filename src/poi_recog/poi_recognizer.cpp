#include "poi_recognizer.hpp"
#include "../opencx.hpp"

using namespace dg;
using namespace std;
using namespace cv;

int main(){
    POIRecognizer poi_recog;

    cv::Mat image = cv::imread("./data/test/input/test_starbucks.png")
    Timestamp t = 10;
    if (poi_recog.apply(image, t) < 0){
        return -1;
    }

    double ang, p;
    poi_recog.get(ang, p);
    printf("angle = %lf, prob = %lf\n", ang, p);

    return 0;
}
