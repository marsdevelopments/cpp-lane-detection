#include "LanePoints.hpp"

LanePoints::LanePoints() {}

LanePoints::LanePoints(int left_top_, int left_bottom_, int right_top_, int right_bottom_)
    : left_top(left_top_), left_bottom(left_bottom_), right_top(right_top_), right_bottom(right_bottom_)
{
}