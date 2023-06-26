#pragma once

struct LanePoints
{
    int left_top = -1;
    int left_bottom = -1;
    int right_top = -1;
    int right_bottom = -1;

    LanePoints();

    LanePoints(int left_top_, int left_bottom_, int right_top_, int right_bottom_);
};