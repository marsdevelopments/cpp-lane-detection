#include "RandomLineBuilder.hpp"

int RandomLineBuilder::get_line_score(const cv::Mat &processed_frame, const int x1, const int y1, const int x2, const int y2)
{
    // line length
    const double length = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)) + 0.0001;
    double dx = (x2 - x1) / length; // Division by zero safe
    double dy = (y2 - y1) / length;

    int score = 0;

    for (int i = 0, end = static_cast<int>(length); i < end; ++i)
    {
        int x = static_cast<int>(x1 + (dx * i /*  + 0.5 */)); 
        int y = static_cast<int>(y1 + (dy * i /*  + 0.5 */));

        // get pixel value at (row, column)
        score += processed_frame.at<uint8_t>(y, x);
    }

    return score;
}