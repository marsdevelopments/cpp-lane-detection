#include "lane-detection.h"

#include <cmath>

#include <opencv2/video/tracking.hpp>

#include "constants.h"

// #include <opencv2/line_descriptor/descriptor.hpp>

using namespace cv;

constexpr size_t average_size = 5;

bool long_missing_flag = false;
size_t missing_frames_counter = 0;

std::array<std::pair<int, int>, average_size> left_average = {{{0, 0},
                                                               {0, 0},
                                                               {0, 0},
                                                               {0, 0},
                                                               {0, 0}}};

std::array<std::pair<int, int>, average_size> right_average = {{{0, 0},
                                                                {0, 0},
                                                                {0, 0},
                                                                {0, 0},
                                                                {0, 0}}};

void clear_averages()
{
    for (size_t i = 0; i < average_size; ++i)
    {
        left_average[i].first = 0;
        left_average[i].second = 0;

        right_average[i].first = 0;
        right_average[i].second = 0;
    }
}

void add_average(std::array<std::pair<int, int>, average_size> &source, int x1, int x2)
{
    for (size_t i = source.size() - 1, end = 1; i >= end; --i)
    {
        source[i] = source[i - 1];
    }

    source[0] = {x1, x2};
}

std::pair<int, int> get_average(const std::array<std::pair<int, int>, average_size> &source)
{
    std::pair<int, int> average;

    int divider = 0;

    for (auto cord : source)
    {
        if (cord.first == 0)
            continue;

        average.first += cord.first;
        average.second += cord.second;
        ++divider;
    }

    if (divider == 0)
        return std::pair<int, int>(0, 0);

    average.first /= divider;
    average.second /= divider;

    return average;
}

void display_selected_lines(const Mat image, const std::vector<Vec4i> lines)
{
    cv::Mat window(image);

    for (auto line : lines)
    {
        cv::line(window, Point(line[0], line[1]), Point(line[2], line[3]), Scalar(0, 0, 255), 3, LINE_AA);
    }

    imshow("selected", window);
    waitKey(0);
}

int get_line_score(const Mat image, const Vec4i &line)
{
    const double x1 = line[0];
    const double y1 = line[1];
    const double x2 = line[2];
    const double y2 = line[3];

    // x1, y1, x2, y2 and img given as input
    double len = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)) + 0.0001;
    double dx = (x2 - x1) / len; // Division by zero safe
    double dy = (y2 - y1) / len;

    int sum = 0;
    for (int i = 0; i < len; ++i)
    {
        int x = static_cast<int>(x1 + (dx * i + 0.5)); // 0.5 for rounding when truncating
        int y = static_cast<int>(y1 + (dy * i + 0.5));
        sum += image.at<uint8_t>(y, x) > 200; // White value higher than 200
    }

    return sum;
}

Vec4i get_best_line(cv::Mat image, const std::vector<Vec4i> &lines)
{
    int best_sum = -1;
    Vec4i best_line;

    for (const Vec4i line : lines)
    {
        int line_score = get_line_score(image, line);

        if (line_score < best_sum)
            continue;

        best_sum = line_score;
        best_line = line;
    }

    return best_line;
}


Mat draw_points(Mat source, Vec4i pointsX, int y1, int y2)
{
    const int leftX1 = pointsX[0];
    const int leftX2 = pointsX[1];
    const int rightX1 = pointsX[2];
    const int rightX2 = pointsX[3];

    Mat mask = Mat::zeros(source.size(), source.type());

    // Draw lines and fill poly made up of the four points described above if both bools are true
    Mat dst; // Holds blended image

    line(source, Point(rightX1, y1), Point(rightX2, y2), Scalar(255, 0, 0), 7);

    line(source, Point(leftX1, y1), Point(leftX2, y2), Scalar(255, 0, 0), 7);

    Point pts[4] = {
        Point(leftX1, y1),  // Starting point left lane
        Point(leftX2, y2),  // Ending point left lane
        Point(rightX2, y2), // Ending point right lane
        Point(rightX1, y1)  // Starting point right lane
    };

    fillConvexPoly(mask, pts, 4, Scalar(235, 229, 52)); // Color is light blue

    // Blend the mask and source image together
    addWeighted(source, 1, mask, 0.3, 0.0, dst);

    // Return blended image
    return dst;
}

Mat draw_from_average(Mat source, bool drawLeftLane, bool drawRightLane, int y1, int y2)
{
    const std::pair<int, int> left = get_average(left_average);
    const std::pair<int, int> right = get_average(right_average);

    Vec4i pointsX(left.first, left.second, right.first, right.second);

    if (drawRightLane && drawLeftLane)
    {
        missing_frames_counter = 0;

        if (long_missing_flag)
        {
            long_missing_flag = false;
            return source;
        }
    }
    else if (!drawRightLane && !drawLeftLane)
    {
        ++missing_frames_counter;
    }

    if (missing_frames_counter >= average_size)
    {
        long_missing_flag = true;
        clear_averages();
        return source;
    }

    return draw_points(source, pointsX, y1, y2);
}

Mat drawLanes(Mat source, std::vector<Vec4i> lines)
{
    const int y1 = source.rows;                                                       // Y coordinate of starting point of both the left and right lane
    const int y2 = static_cast<int>(source.rows * (1 - cst::kTrapezoidHeight * 0.9)); // Y coordinate of ending point of both the left and right lane

    // Stop if there are no lines, just return original image without lines
    if (lines.size() == 0)
    {
        return draw_from_average(source, false, false, y1, y2);
        // return source;
    }

    // lines.resize(50);

    const int imgCenter = source.cols / 2;

    // Set drawing lanes to true
    bool drawRightLane = true;
    bool drawLeftLane = true;

    // Find lines with a slope higher than the slope threshold
    std::vector<double> slopes;
    std::vector<Vec4i> goodLines;

    for (Vec4i line : lines)
    {
        const double x1 = line[0];
        const double y1 = line[1];
        const double x2 = line[2];
        const double y2 = line[3];

        // Calculate slope
        double slope;

        if (x2 - x1 == 0) // Avoid division by zero
            slope = 999;  // Basically infinte slope
        else
            slope = (y2 - y1) / (x2 - x1);

        if (abs(slope) <= cst::kSlopeThreshold)
            continue;

        slopes.push_back(slope);
        goodLines.push_back(line);
    }

    // display_selected_lines(source, goodLines);

    /* Split the good lines into two categories: right and left
    The right lines have a positive slope and the left lines have a negative slope */
    std::vector<Vec4i> rightLines;
    std::vector<Vec4i> leftLines;

    for (int i = 0; i < slopes.size(); i++)
    {
        const Vec4i line = goodLines[i];

        if (slopes[i] >= 0 /* && line[0] >= imgCenter && line[2] >= imgCenter */)
            rightLines.push_back(line);

        if (slopes[i] < 0 /* && line[0] < imgCenter && line[2] < imgCenter */)
            leftLines.push_back(line);
    }

    /* Now that we've isolated the right lane lines from the left lane lines,
    it is time to form two lane lines out of all the lines we've detected.
    A line is defined as 2 points: a starting point and an ending point.
    So up to this point the right and left lane basically consist of multiple hough lines.
    Our goal at this step is to use linear regression to find the two best fitting lines:
    one through the points at the left side to form the left lane
    and one through the points at the right side to form the right lane */

    // We start with the right side points
    std::vector<int> rightLinesX;
    std::vector<int> rightLinesY;
    double rightB1, rightB0; // Slope and intercept

    for (int i = 0; i < rightLines.size(); i++)
    {
        rightLinesX.push_back(rightLines[i][0]); // X of starting point of line
        rightLinesX.push_back(rightLines[i][2]); // X of ending point of line
        rightLinesY.push_back(rightLines[i][1]); // Y of starting point of line
        rightLinesY.push_back(rightLines[i][3]); // Y of ending point of line
    }

    if (rightLinesX.size() > 0)
    {
        std::vector<double> coefRight = estimate_coefficients<int, double>(rightLinesX, rightLinesY); // y = b1x + b0
        rightB1 = coefRight[0];
        rightB0 = coefRight[1];
    }
    else
    {
        rightB1 = 1;
        rightB0 = 1;
        drawRightLane = false;
    }

    // Now the points at the left side
    std::vector<int> leftLinesX;
    std::vector<int> leftLinesY;
    double leftB1, leftB0; // Slope and intercept

    for (int i = 0; i < leftLines.size(); i++)
    {
        leftLinesX.push_back(leftLines[i][0]); // X of starting point of line
        leftLinesX.push_back(leftLines[i][2]); // X of ending point of line
        leftLinesY.push_back(leftLines[i][1]); // Y of starting point of line
        leftLinesY.push_back(leftLines[i][3]); // Y of ending point of line
    }

    if (leftLinesX.size() > 0)
    {
        std::vector<double> coefLeft = estimate_coefficients<int, double>(leftLinesX, leftLinesY); // y = b1x + b0
        leftB1 = coefLeft[0];
        leftB0 = coefLeft[1];
    }
    else
    {
        leftB1 = 1;
        leftB0 = 1;
        drawLeftLane = false;
    }

    /* Now we need to find the two points for the right and left lane:
    starting points and ending points */

    /* 0.5 = kTrapezoidHeight (see RegionOfInterest), we set the y coordinate of the ending point
    below the trapezoid height (0.4) to draw shorter lanes. I think that looks nicer. */

    // y = b1x + b0 --> x = (y - b0) / b1
    int rightX1 = static_cast<int>((y1 - rightB0) / rightB1); // X coordinate of starting point of right lane
    int rightX2 = static_cast<int>((y2 - rightB0) / rightB1); // X coordinate of ending point of right lane

    int leftX1 = static_cast<int>((y1 - leftB0) / leftB1); // X coordinate of starting point of left lane
    int leftX2 = static_cast<int>((y2 - leftB0) / leftB1); // X coordinate of ending point of left lane

    if (rightX1 < 0 || rightX2 < 0)
        drawRightLane = false;

    if (leftX1 < 0 || leftX2 < 0)
        drawLeftLane = false;

    /* If the ending point of the right lane is on the left side of the left lane (or vice versa),
    return source image without drawings, because this should not be happening in real life. */

    if (drawLeftLane && drawRightLane && (rightX2 < leftX2 || leftX2 > rightX2))
    {
        drawLeftLane = false;
        drawRightLane = false;
    }

    if (drawLeftLane && drawRightLane && (rightX1 < leftX1 || leftX1 > rightX1))
    {
        drawLeftLane = false;
        drawRightLane = false;
    }

    if (drawLeftLane)
        add_average(left_average, leftX1, leftX2);

    if (drawRightLane)
        add_average(right_average, rightX1, rightX2);

    return draw_from_average(source, drawLeftLane, drawRightLane, y1, y2);
}

std::vector<Vec4i> houghLines(Mat frame, Mat result, bool drawHough)
{
    double rho = 3; // Distance resolution in pixels of the Hough grid
    // double theta = 1 * M_PI / 180; // Angular resolution in radians of the Hough grid
    double theta = 3.14 / 180; // Angular resolution in radians of the Hough grid
    int thresh = 20;           // Minimum number of votes (intersections in Hough grid cell)
    double minLineLength = 35; // Minimum number of pixels making up a line
    double maxGapLength = 100; // Maximum gap in pixels between connectable line segments

    std::vector<Vec4i> linesP; // Will hold the results of the detection
    HoughLinesP(frame, linesP, rho, theta, thresh, minLineLength, maxGapLength);

    if (drawHough)
    {
        for (size_t i = 0; i < linesP.size(); i++)
        {
            Vec4i l = linesP[i];
            line(result, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 3, LINE_AA);
        }
        imshow("Hough Lines", result);
        // waitKey(0);
    }

    return linesP;
}

bool isDayTime(Mat source)
{
    /* I've noticed that, in general, daytime images/videos require different color
    filters than nighttime images/videos. For example, in darker light it is better
    to add a gray color filter in addition to the white and yellow one */

    Scalar s = mean(source); // Mean pixel values

    /* I chose these cut off values by looking at the mean pixel values of multiple
    daytime and nighttime images */
    if (s[0] < 30 || s[1] < 33 && s[2] < 30)
    {
        return false;
    }

    return true;
}