#pragma once

struct LanePoints
{
    int left_top = -1;
    int left_bottom = -1;
    int right_top = -1;
    int right_bottom = -1;

    bool draw_left = false;
    bool draw_right = false;

    LanePoints();

    LanePoints(const int &left_top_, const int &left_bottom_, const int &right_top_, const int &right_bottom_);

    LanePoints(const int &left_top_, const int &left_bottom_, const int &right_top_, const int &right_bottom_, const bool &draw_left_, const bool &draw_right_);
};