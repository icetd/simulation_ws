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
#include "can.h"
#include "Detector.h"

#define OIL_NEEDLE_NUM  6
 
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
    
    
    Can *can_;
    void initCan();
    void canCallback(struct can_frame rx_frame);

    Detector *detector_;
    int32_t x_, y_, z_;
    bool isArucoEfficient_;
    void arucoCallback(int32_t x, int32_t y, int32_t z);
    void updateAruco(int32_t x, int32_t y, int32_t z);
 
    int32_t oil_needle_capacity_[OIL_NEEDLE_NUM + 1];
    int32_t alignment_tank_id[OIL_NEEDLE_NUM + 1];
    int32_t alignment_offset_x[OIL_NEEDLE_NUM + 1];
    int32_t alignment_offset_y[OIL_NEEDLE_NUM + 1];

    bool isAligMentDone_;                   // 对接完成标志
    bool oil_Init_done_[OIL_NEEDLE_NUM + 1];    // 取油初始化完成
    bool oil_Wash_done_[OIL_NEEDLE_NUM + 1];    // 洗针管完成
    bool oil_Take_done_[OIL_NEEDLE_NUM + 1];    // 取油完成
    float oil_value_[OIL_NEEDLE_NUM + 1]; //针管容量
    bool isCancelAlignMentDone_;            // 取消对接完成
    bool isGetOilDone_[OIL_NEEDLE_NUM + 1];     // 取油任务完成
    
    void getOilTask(int id);


    void setAlignMentTankId(int32_t tank_id);
    void setAlignMentOffsetY(int32_t offset_y);    
    void setAlignMentOffsetX(int32_t offset_x);
    void startAlignMent(int id);
    
    void setOilVaule(uint16_t id, uint8_t value);
    void startTakeOil(uint16_t id);

    void startCancelAlignMent(int id);
};

#endif
