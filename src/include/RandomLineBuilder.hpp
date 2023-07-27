#pragma once

#include <numeric>
#include <random>

#include <opencv2/opencv.hpp>

#include "constants.hpp"

class RandomLineBuilder
{
public:
  /**
   * @brief build a set with size S, containing pairs of top and bottom X
   * coordinates
   *
   * @param S size of the returned array
   * @param top_min minimum possible top coordinate
   * @param top_max maximum possible top coordinate
   * @param bottom_min minimum possible bottom coordinate
   * @param bottom_max maximum possible bottom coordinate
   *
   * @return array of size S with pair of X coordinates
   */
  template<int S>
  static std::array<std::pair<int, int>, S> build_lines(const int top_min,
                                                        const int top_max,
                                                        const int bottom_min,
                                                        const int bottom_max)
  {
    std::random_device r;
    std::default_random_engine generator{ r() };

    std::uniform_int_distribution<int> top_distribution(
      (top_min < 0 ? 0 : top_min),
      (top_max > cst::kVideoWidth ? cst::kVideoWidth : top_max));

    std::uniform_int_distribution<int> bottom_distribution(
      (bottom_min < 0 ? 0 : bottom_min),
      (bottom_max > cst::kVideoWidth ? cst::kVideoWidth : bottom_max));

    std::array<std::pair<int, int>, S> output;

    for (int i = 0, end = S; i < end; ++i) {
      std::pair<int, int> line;
      line.first = top_distribution(generator);
      line.second = bottom_distribution(generator);

      output.at(i) = line;
    }

    return output;
  }

  /**
   * @brief select a line which traverses the most white pixels
   *
   * @param processed_frame frame in which lines will be tested,
   * should be a processed, black & white one
   * @param lines array of pairs with X coordinates to select from
   *
   * @return top and bottom X coordinates of the best line or -1, -1 if not
   * found
   */
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
        processed_frame, line.first, cst::kTopY, line.second, cst::kBottomY);

      if (line_score < best_line_score)
        continue;

      best_line_score = line_score;
      best_line_index = i;
    }

    if (best_line_index == SIZE_MAX || best_line_score == 0)
      return std::pair<int, int>(-1, -1);

    return lines.at(best_line_index);
  }

  /**
   * @brief Calculates the ammount of white pixels a given line traverses
   *
   * @param processed_frame frame in which lines will be tested, should be a
   * black & white one
   * @param x1 top X coordinate
   * @param y1 top Y coordinate
   * @param x2 bottom X coordinate
   * @param y2 bottom Y coordinate
   *
   * @return ammount of white pixels given line traverses
   */
  static int get_line_score(const cv::Mat& processed_frame,
                            const int x1,
                            const int y1,
                            const int x2,
                            const int y2);
};