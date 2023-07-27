#pragma once

// holds coordinates of a lane
struct LanePoints
{
    // storing only X coordinates because Y coordinates are constant 
    // and are equal to Y coordinates of the ROI

    // left line top X coordinate
    int left_top = -1;
    // left line bottom X coordinate
    int left_bottom = -1;
    // right line top X coordinate
    int right_top = -1;
    // right line bottom X coordianate
    int right_bottom = -1;

    // flags to indicate whether lane lines where found and are valid
    bool left_found = false;
    bool right_found = false;

    LanePoints();

    LanePoints(const int &left_top_, const int &left_bottom_, const int &right_top_, const int &right_bottom_);

    LanePoints(const int &left_top_, const int &left_bottom_, const int &right_top_, const int &right_bottom_, const bool &left_found_, const bool &right_found_);
};