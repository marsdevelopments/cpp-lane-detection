#pragma once

#include <numeric>
#include <random>

#include <opencv2/opencv.hpp>

#include "constants.hpp"

class RandomLineBuilder
{
public:
  template<int S>
  static std::array<std::pair<int, int>, S> build_lines(const int top_min,
                                                        const int top_max,
                                                        const int bottom_min,
                                                        const int bottom_max)
  {
    std::random_device r;
    std::default_random_engine generator{ r() };
    std::uniform_int_distribution<int> top_distribution(
      (top_min >= 0 ? top_min : 0),
      (top_max <= cst::kVideoWidth ? top_max : cst::kVideoWidth));
    std::uniform_int_distribution<int> bottom_distribution(
      (bottom_min >= 0 ? bottom_min : 0),
      (bottom_max <= cst::kVideoWidth ? bottom_max : cst::kVideoWidth));

    std::array<std::pair<int, int>, S> output;

    for (int i = 0, end = S; i < end; ++i) {
      std::pair<int, int> line;
      line.first = top_distribution(generator);
      line.second = bottom_distribution(generator);

      output.at(i) = line;
    }

    return output;
  }

  template<int S>
  static std::pair<int, int> select_best_line(
    const cv::Mat& processed_frame,
    const std::array<std::pair<int, int>, S>& lines)
  {
    int best_line_score = -1;
    size_t best_line_index = SIZE_MAX;

    for (size_t i = 0, end = lines.size(); i < end; ++i) {
      const std::pair<int, int> line = lines.at(i);

      const int line_score = get_line_score(
        processed_frame, line.first, cst::y_top, line.second, cst::y_bottom);

      if (line_score < best_line_score)
        continue;

      best_line_score = line_score;
      best_line_index = i;
    }

    if (best_line_index == SIZE_MAX || best_line_score == 0)
      return std::pair<int, int>(-1, -1);

    return lines.at(best_line_index);
  }

  static int get_line_score(const cv::Mat& processed_frame,
                            const int x1,
                            const int y1,
                            const int x2,
                            const int y2);
};