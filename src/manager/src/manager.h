#ifndef MANAGER_H
#define MANAGER_H

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <actionlib_msgs/GoalStatus.h>
#include <nav_msgs/GetMap.h>
#include <std_msgs/String.h>
#include "manager_msgs/Plan.h"
#include "manager_msgs/Status.h"
#include "thread.h"
#include "plan.h"

class Manager : public Thread
{
public:
    Manager(int argc, char **argv);
    virtual ~Manager();

    bool init();

    void TopicSubPub();
    void run() override;

private:
    Plan *plan_ = nullptr;


    /* plan */
    std::string plan_topic;
    ros::Subscriber sub_plan_;
    void planCallback(const manager_msgs::Plan::ConstPtr &msg);
    void addPlan(const manager_msgs::Plan::ConstPtr &plan);
    void delPlan();
    void runPlan();
    PlanType_t last_type_;
    int last_id;
    bool isBusy;

    /* cmd_plan */
    std::string cmd_plan_topic;
    ros::Subscriber sub_cmd_plan_;
    void cmdPlanCallback(const manager_msgs::Status::ConstPtr &msg);

    /* back */
    std::string back_topic;
    ros::Publisher pub_back_;
    std_msgs::String msg_back;
    std::stringstream stream_;
    char msg_buf_[256];

    /* move_base/cancel */
    std::string cancel_plan_topic;
    ros::Publisher pub_cancel_plan;
    actionlib_msgs::GoalID goalId;

    /* move_base_simple/goal */
    std::string goal_topic;
    ros::Publisher pub_goal_;
    ros::Timer set_goal_timer;
    void set_goal_timer_callback(const ros::TimerEvent &e);

    /* move_base/result */
    std::string goal_result_topic;
    ros::Subscriber sub_goal_result_;
    move_base_msgs::MoveBaseActionResult action_result;
    void actionResult_call(const move_base_msgs::MoveBaseActionResult::ConstPtr &msg);

    /* map */
    std::string map_topic;
    std::string mapname_;
    ros::Subscriber sub_map_;
    bool isSave_map_;

    void mapCallback(const nav_msgs::OccupancyGridConstPtr &map);

    int frame_rate_ = 40;
    int thread_num_ = 6;

    int argc_;
    char** argv_;

    ros::NodeHandle *nh;
    manager_msgs::Status status_;
};

#endif