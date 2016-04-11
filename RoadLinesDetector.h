#ifndef DRIVERASSISTANT_ROADLINESDETECTOR_H
#define DRIVERASSISTANT_ROADLINESDETECTOR_H

#define DEFAULT_ANGLE       0
#define ANGLE_MIN           20
#define ANGLE_MAX           75
#define MAX_ANGLE_DIFF      10
#define MAX_LAST_COUNTER    5

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class RoadLinesDetector {

    Vec4i last_left_line, last_right_line;
    double last_left_angle, last_right_angle;
    int last_left_counter, last_right_counter;

    void filter_lines(vector<Vec4i> lines, vector<Vec4i> &good_lines, int left_offset, int width);

    void extrapolate_lines(vector<Vec4i> &lines, int top_offset, int height);

    Rect get_roi(int width, int height);

public:
    RoadLinesDetector() : last_left_angle(DEFAULT_ANGLE),
                          last_right_angle(DEFAULT_ANGLE),
                          last_left_counter(0),
                          last_right_counter(0) { }

    void init();

    void detect(Mat gray_image, vector<Vec4i> &output);
};

#endif
