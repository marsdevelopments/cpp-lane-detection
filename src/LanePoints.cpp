#include "LanePoints.hpp"

LanePoints::LanePoints() {}

LanePoints::LanePoints(const int& left_top_,
                       const int& left_bottom_,
                       const int& right_top_,
                       const int& right_bottom_)
  : LanePoints(left_top_, left_bottom_, right_top_, right_bottom_, false, false)
{
}

LanePoints::LanePoints(const int& left_top_,
                       const int& left_bottom_,
                       const int& right_top_,
                       const int& right_bottom_,
                       const bool& left_found_,
                       const bool& right_found_)
  : left_top(left_top_)
  , left_bottom(left_bottom_)
  , right_top(right_top_)
  , right_bottom(right_bottom_)
  , left_found(left_found_)
  , right_found(right_found_)
{
}