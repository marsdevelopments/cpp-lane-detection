#pragma once

struct LanePoints
{
    int left_top = -1;
    int left_bottom = -1;
    int right_top = -1;
    int right_bottom = -1;

    bool left_found = false;
    bool right_found = false;

    LanePoints();

    LanePoints(const int &left_top_, const int &left_bottom_, const int &right_top_, const int &right_bottom_);

    LanePoints(const int &left_top_, const int &left_bottom_, const int &right_top_, const int &right_bottom_, const bool &left_found_, const bool &right_found_);
};