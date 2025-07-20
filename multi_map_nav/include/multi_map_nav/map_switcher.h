#pragma once
#include <ros/ros.h>
#include <ros/package.h>
#include <cstdlib> 
#include <iostream>
#include <string>
#include <fstream>

class MapSwitcher {
public:
    MapSwitcher();  
    void switchToMap(const std::string& map_name);  

private:
    ros::NodeHandle nh_;  
    std::string map_folder_;  
};
