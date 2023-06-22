#include "lane_detection.hpp"

#include <vector>
#include <numeric>

#include "constants.hpp"

LaneDetection::LaneDetection()
{
}

cv::Mat LaneDetection::find_lines(const cv::Mat &frame, const cv::Mat& edited_frame)
{
    frame_ = frame;

    houghLines(edited_frame, true);
    drawLanes();

    return frame_;
}

void LaneDetection::clear_averages()
{
    for (size_t i = 0; i < average_size; ++i)
    {
        left_average[i].first = 0;
        left_average[i].second = 0;

        right_average[i].first = 0;
        right_average[i].second = 0;
    }
}

void LaneDetection::add_average(std::array<std::pair<int, int>, average_size> &source, int x1, int x2)
{
    for (size_t i = source.size() - 1, end = 1; i >= end; --i)
    {
        source[i] = source[i - 1];
    }

    source[0] = {x1, x2};
}

std::pair<int, int> LaneDetection::get_average(const std::array<std::pair<int, int>, average_size> &source)
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

void LaneDetection::draw_points(cv::Vec4i pointsX, int y1, int y2)
{
    const int leftX1 = pointsX[0];
    const int leftX2 = pointsX[1];
    const int rightX1 = pointsX[2];
    const int rightX2 = pointsX[3];

    cv::Mat mask = cv::Mat::zeros(frame_.size(), frame_.type());

    // Draw lines and fill poly made up of the four points described above if both bools are true
    cv::Mat dst; // Holds blended image

    cv::line(frame_, cv::Point(rightX1, y1), cv::Point(rightX2, y2), cv::Scalar(255, 0, 0), 7);

    cv::line(frame_, cv::Point(leftX1, y1), cv::Point(leftX2, y2), cv::Scalar(255, 0, 0), 7);

    cv::Point pts[4] = {
        cv::Point(leftX1, y1),  // Starting point left lane
        cv::Point(leftX2, y2),  // Ending point left lane
        cv::Point(rightX2, y2), // Ending point right lane
        cv::Point(rightX1, y1)  // Starting point right lane
    };

    cv::fillConvexPoly(mask, pts, 4, cv::Scalar(235, 229, 52)); // Color is light blue

    // Blend the mask and source image together
    addWeighted(frame_, 1, mask, 0.3, 0.0, dst);
}

void LaneDetection::draw_from_average(bool drawLeftLane, bool drawRightLane, int y1, int y2)
{
    const std::pair<int, int> left = get_average(left_average);
    const std::pair<int, int> right = get_average(right_average);

    cv::Vec4i pointsX(left.first, left.second, right.first, right.second);

    if (drawRightLane && drawLeftLane)
    {
        missing_frames_counter = 0;

        if (long_missing_flag)
        {
            long_missing_flag = false;
            return;
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
        return;
    }

    draw_points(pointsX, y1, y2);
}

void LaneDetection::drawLanes()
{
    const int y1 = frame_.rows;                                                       // Y coordinate of starting point of both the left and right lane
    const int y2 = static_cast<int>(frame_.rows * (1 - cst::kTrapezoidHeight * 0.9)); // Y coordinate of ending point of both the left and right lane

    // Stop if there are no lines, just return original image without lines
    if (hough_lines_.size() == 0)
    {
        return draw_from_average(false, false, y1, y2);
    }

    // lines.resize(50);

    const int imgCenter = frame_.cols / 2;

    // Set drawing lanes to true
    bool drawRightLane = true;
    bool drawLeftLane = true;

    // Find lines with a slope higher than the slope threshold
    std::vector<double> slopes;
    std::vector<cv::Vec4i> goodLines;

    for (cv::Vec4i line : hough_lines_)
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
    std::vector<cv::Vec4i> rightLines;
    std::vector<cv::Vec4i> leftLines;

    for (int i = 0; i < slopes.size(); i++)
    {
        const cv::Vec4i line = goodLines[i];

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

    return draw_points(cv::Vec4i(leftX1, leftX2, rightX1, rightX2), y1, y2);

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

    draw_from_average(drawLeftLane, drawRightLane, y1, y2);
}

void LaneDetection::houghLines(const cv::Mat &edited_frame, bool drawHough)
{
    double rho = 3; // Distance resolution in pixels of the Hough grid
    // double theta = 1 * M_PI / 180; // Angular resolution in radians of the Hough grid
    double theta = 3.14 / 180; // Angular resolution in radians of the Hough grid
    int thresh = 20;           // Minimum number of votes (intersections in Hough grid cell)
    double minLineLength = 35; // Minimum number of pixels making up a line
    double maxGapLength = 100; // Maximum gap in pixels between connectable line segments

    hough_lines_.clear();
    cv::Mat hough_mat = edited_frame.clone();

    cv::HoughLinesP(edited_frame.clone(), hough_lines_, rho, theta, thresh, minLineLength, maxGapLength);

    if (drawHough)
    {
        for (size_t i = 0; i < hough_lines_.size(); i++)
        {
            cv::Vec4i l = hough_lines_[i];
            line(hough_mat, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
        }
        imshow("Hough Lines", hough_mat);
        // waitKey(0);
    }

    // return linesP;
}

template <typename T, typename X>
X LaneDetection::multiplyAndSum(std::vector<T> A, std::vector<T> B)
{
    X sum;
    std::vector<T> temp;
    for (int i = 0; i < A.size(); i++)
    {
        temp.push_back(A[i] * B[i]);
    }
    sum = std::accumulate(temp.begin(), temp.end(), 0);

    return sum;
}

template <typename T, typename X>
std::vector<X> LaneDetection::estimate_coefficients(std::vector<T> A, std::vector<T> B)
{
    // Sample size
    size_t N = A.size();

    // Calculate mean of X and Y
    X meanA = std::accumulate(A.begin(), A.end(), 0.0) / A.size();
    X meanB = std::accumulate(B.begin(), B.end(), 0.0) / B.size();

    // Calculating cross-deviation and deviation about x
    X SSxy = multiplyAndSum<T, T>(A, B) - (N * meanA * meanB);
    X SSxx = multiplyAndSum<T, T>(A, A) - (N * meanA * meanA);

    // Calculating regression coefficients
    X slopeB1 = SSxy / SSxx;
    X interceptB0 = meanB - (slopeB1 * meanA);

    // Return vector, insert slope first and then intercept
    std::vector<X> coef;
    coef.push_back(slopeB1);
    coef.push_back(interceptB0);
    return coef;
}
