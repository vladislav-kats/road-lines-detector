#include "RoadLinesDetector.h"

void RoadLinesDetector::init() {
    last_left_counter = 0;
    last_right_counter = 0;
    last_left_angle = DEFAULT_ANGLE;
    last_right_angle = DEFAULT_ANGLE;
}

void RoadLinesDetector::detect(Mat gray_image, vector<Vec4i> &output) {
    int width = gray_image.size().width;
    int height = gray_image.size().height;

    Rect roi = get_roi(width, height);
    Mat gray = gray_image(roi);

    Mat temp, edges;
    double otsu_threshold = threshold(gray, temp, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
    Canny(gray, edges, otsu_threshold * 0.5, otsu_threshold);

    vector<Vec4i> lines;
    HoughLinesP(edges, lines, 1, CV_PI / 180, 50,
                height / 10,    // min line length
                height / 4      // max line gap
    );
    filter_lines(lines, output, roi.x, width);
    extrapolate_lines(output, roi.y, height);
}

void RoadLinesDetector::filter_lines(vector<Vec4i> lines, vector<Vec4i> &good_lines, int left_offset, int width) {
    printf("All detected lines count: %d", lines.size());

    int i_left = -1, i_right = -1,
            min_left = numeric_limits<int>::max(),
            min_right = numeric_limits<int>::max();
    double left_angle = 0, right_angle = 0;

    for (size_t i = 0; i < lines.size(); i++) {
        lines[i][0] += left_offset;
        lines[i][2] += left_offset;

        int dx = lines[i][2] - lines[i][0], dy = lines[i][3] - lines[i][1];
        double angle = atan2f(dy, dx) * 180 / CV_PI;

        // skip horizontal and vertical lines
        if (abs(angle) <= ANGLE_MIN || abs(angle) >= ANGLE_MAX) {
            continue;
        }

        // 2 closest lines to the center
        int bottom_x = lines[i][1] > lines[i][3] ? lines[i][0] : lines[i][2];
        if (lines[i][0] + lines[i][2] < width) {
            // left
            if (angle > -ANGLE_MAX && angle < -ANGLE_MIN
                && (last_left_angle == DEFAULT_ANGLE || abs(angle - last_left_angle) < MAX_ANGLE_DIFF || last_left_counter > MAX_LAST_COUNTER)
                && (width / 2 - bottom_x < min_left)) {
                printf("Left angle: %f", angle);
                min_left = width / 2 - bottom_x;
                i_left = i;
                left_angle = angle;
            }
        } else {
            // right
            if (angle > ANGLE_MIN && angle < ANGLE_MAX
                && (last_right_angle == DEFAULT_ANGLE || abs(angle - last_right_angle) < MAX_ANGLE_DIFF || last_right_counter > MAX_LAST_COUNTER)
                && (bottom_x - width / 2 < min_right)) {
                printf("Right angle: %f", angle);
                min_right = bottom_x - width / 2;
                i_right = i;
                right_angle = angle;
            }
        }
    }

    if (i_left != -1) {
        last_left_line = lines[i_left];
        last_left_angle = left_angle;
        last_left_counter = 0;
    } else {
        printf("Using last left line; last_left_counter = %d", last_left_counter);
        last_left_counter++;
    }

    if (i_right != -1) {
        last_right_line = lines[i_right];
        last_right_angle = right_angle;
        last_right_counter = 0;
    } else {
        printf("Using last right line; last_right_counter = %d", last_right_counter);
        last_right_counter++;
    }

    good_lines.push_back(last_left_line);
    good_lines.push_back(last_right_line);
}

void RoadLinesDetector::extrapolate_lines(vector<Vec4i> &lines, int top_offset, int height) {
    for (size_t i = 0; i < lines.size(); i++) {
        Vec4i &line = lines[i];  // [0] - x1, [1]- y1, [2] - x2, [3] - y2
        double k = (line[1] - line[3]) / (double) (line[0] - line[2]);
        double b = line[1] - k * line[0] + top_offset;

        int bottom_x = line[1] < line[3] ? 0 : 2;
        line[bottom_x] = cvRound((top_offset - b) / k);
        line[bottom_x == 0 ? 1 : 3] = top_offset;
        line[bottom_x == 0 ? 2 : 0] = cvRound((height - b) / k);
        line[bottom_x == 0 ? 3 : 1] = height;
    }
}

Rect RoadLinesDetector::get_roi(int width, int height) {
    int y_offset = height * 3 / 5;

    if (last_left_angle == DEFAULT_ANGLE || last_right_angle == DEFAULT_ANGLE) {
        return Rect(0, y_offset, width, height - y_offset);
    }

    vector<Vec4i> lines;
    Vec4i left(last_left_line), right(last_right_line);
    lines.push_back(left);
    lines.push_back(right);
    extrapolate_lines(lines, y_offset, height);

    int x_error = width / 8;
    int left_threshold = max(min((lines[0][1] < lines[0][3] ? lines[0][0] : lines[0][2]) - x_error, width), 0);
    int right_threshold = min(max((lines[1][1] < lines[1][3] ? lines[1][0] : lines[1][2]) + x_error, 0), width);

    if (right_threshold < left_threshold) {
        left_threshold = 0;
        right_threshold = width;
    }

    if (last_left_counter >= MAX_LAST_COUNTER || last_right_counter >= MAX_LAST_COUNTER) {
        int left_err = (int) ((double) last_left_counter / MAX_LAST_COUNTER * x_error);
        int right_err = (int) ((double) last_right_counter / MAX_LAST_COUNTER * x_error);
        // gradually increase ROI to left/right
        return Rect(last_left_counter >= MAX_LAST_COUNTER ? max(left_threshold - left_err, 0) : left_threshold,
                    y_offset,
                    (last_right_counter >= MAX_LAST_COUNTER ? min(right_threshold + right_err, width) : right_threshold) - left_threshold,
                    height - y_offset);
    }

    return Rect(left_threshold, y_offset, right_threshold - left_threshold, height - y_offset);
}
