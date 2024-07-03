#ifndef PLAN_H
#define PLAN_H

#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseActionResult.h>

typedef enum
{
    STOP = 0,
    PAUSE,
    MOVE,
    TASK,
} PlanType_t;

typedef enum
{
    NOTING = 0,
    GET_OIL1 = 1,
    GET_OIL2 = 2,
    GET_OIL3 = 3,
    GET_OIL4 = 4,
    GET_OIL5 = 5,
    GET_OIL6 = 6
} ActionType_t;

const char PLAN_TYPE[4][20] = {"STOP", "PAUSE", "MOVE", "TASK"};

class Plan
{
public:
    using PlanGoalMap = std::map<int, geometry_msgs::PoseStamped>;
    using PlanTypeMap = std::map<int, PlanType_t>;
    using ActionTypeMap = std::map<int, ActionType_t>;

    Plan();
    ~Plan();

    void init();

    void add(int id, geometry_msgs::PoseStamped &poseStamped, PlanType_t type, ActionType_t task);
    void del(int id);

    void setCurrentId(int id) {
        currentId_ = id;
    }

    int getCurrentId() {
        return currentId_;
    }

    int getmaxId() {
        return maxPlanNum;
    }

    geometry_msgs::PoseStamped getCurrentGoal();
    PlanType_t getCurrentType();
    ActionType_t getCurrentAction();

private:
    PlanGoalMap planGoalMap_;
    PlanTypeMap planTypeMap_;
    ActionTypeMap actionTypeMap_;

    
    int currentId_;
    int maxPlanNum;
};

#endif