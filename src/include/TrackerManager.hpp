#pragma once

#include <string>

class TrackerManager
{
public:
    TrackerManager(const std::string video_path);

    void start_tracking();

private:
    const std::string video_path_;
};