#include "LanePoints.hpp"

LanePoints::LanePoints() {}

LanePoints::LanePoints(const int &left_top_, const int &left_bottom_, const int &right_top_, const int &right_bottom_)
    : LanePoints(left_top_, left_bottom_, right_top_, right_bottom_, false, false)
{
}

LanePoints::LanePoints(const int &left_top_, const int &left_bottom_, const int &right_top_, const int &right_bottom_, const bool &draw_left_, const bool &draw_right_)
    : left_top(left_top_), left_bottom(left_bottom_), right_top(right_top_), right_bottom(right_bottom_), draw_left(draw_left_), draw_right(draw_right_)
{
}