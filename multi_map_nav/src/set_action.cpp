#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <multi_map_nav/NavigateToGoalAction.h>
#include <iostream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "navigation_client");

    if (argc != 4)
    {
        std::cerr << "Usage: navigation_client <map_name> <target_x> <target_y>" << std::endl;
        return 1;
    }

    std::string map_name = argv[1];
    double target_x = std::stod(argv[2]);
    double target_y = std::stod(argv[3]);

    actionlib::SimpleActionClient<multi_map_nav::NavigateToGoalAction> client("navigate_to_goal", true);

    ROS_INFO("Waiting for action Server to start...");
    client.waitForServer();
    ROS_INFO("Action server started.");

    multi_map_nav::NavigateToGoalGoal goal;
    goal.target_map = map_name;
    goal.target_x = target_x;
    goal.target_y = target_y;

    client.sendGoal(goal);
    client.waitForResult();


    if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        auto result = client.getResult();
        std::cout << "Result: success = " << std::boolalpha << result->success
                  << ", message = " << result->message << std::endl;
    }
    else
    {
        std::cout << "Action failed. State: " << client.getState().toString() << std::endl;
    }

    return 0;
}
