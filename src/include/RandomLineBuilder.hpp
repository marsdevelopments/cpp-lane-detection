#pragma once

#include <numeric>

#include <opencv2/opencv.hpp>

#include "constants.hpp"

class RandomLineBuilder
{
public:
    template <int S>
    static std::array<std::pair<int, int>, S> build_lines(const int top_min, const int top_max, const int bottom_min, const int bottom_max)
    {
        std::random_device r;
        std::default_random_engine generator{r()};
        std::uniform_int_distribution<int> top_distribution(top_min, top_max);
        std::uniform_int_distribution<int> bottom_distribution(bottom_min, bottom_max);

        std::array<std::pair<int, int>, cst::lines_array_size> output;

        for (int i = 0, end = cst::lines_array_size; i < end; ++i)
        {
            std::pair<int, int> line;
            line.first = top_distribution(generator);
            line.second = bottom_distribution(generator);

            output.at(i) = line;
        }

        return output;
    }

    static std::pair<int, int> select_best_line(const std::array<std::pair<int, int>, cst::lines_array_size> &lines, const cv::Mat &processed_frame)
    {
        edited_frame_ = processed_frame;

        int best_line_score = -1;
        size_t best_line_index = SIZE_MAX;

        for (size_t i = 0, end = lines.size(); i < end; ++i)
        {
            const std::pair<int, int> line = lines.at(i);

            const int line_score = get_line_score(line.first, cst::y_top, line.second, cst::y_bottom);

            if (line_score < best_line_score)
                continue;

            best_line_score = line_score;
            best_line_index = i;
        }

        if (best_line_index == SIZE_MAX)
            return std::pair<int, int>(-1, -1);

        return lines.at(best_line_index);
    }

    static int get_line_score(const int x1, const int y1, const int x2, const int y2)
    {
        const double length = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)) + 0.0001;
        double dx = (x2 - x1) / length; // Division by zero safe
        double dy = (y2 - y1) / length;

        int score = 0;

        for (int i = 0, end = static_cast<int>(length); i < end; ++i)
        {
            int x = static_cast<int>(x1 + (dx * i /*  + 0.5 */)); // 0.5 for rounding when truncating
            int y = static_cast<int>(y1 + (dy * i /*  + 0.5 */));

            score += edited_frame_.at<uint8_t>(y, x); // White value higher than 200
        }

        return score;
    }

private:
    static cv::Mat edited_frame_;
};