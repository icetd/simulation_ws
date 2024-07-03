#include "plan.h"

Plan::Plan()
{
}

Plan::~Plan()
{
}

void Plan::init()
{
    planGoalMap_.clear();
    planTypeMap_.clear();
    maxPlanNum = planGoalMap_.size();
}

void Plan::add(int id , geometry_msgs::PoseStamped &poseStamped, PlanType_t type, ActionType_t actionType)
{
    planGoalMap_.insert(std::make_pair(id, poseStamped));
    planTypeMap_.insert(std::make_pair(id, type));
    actionTypeMap_.insert(std::make_pair(id, actionType));
    maxPlanNum = planGoalMap_.size();
}

void Plan::del(int id)
{
    planGoalMap_.erase(id);
    planTypeMap_.erase(id);
    maxPlanNum = planGoalMap_.size();
}

geometry_msgs::PoseStamped Plan::getCurrentGoal()
{
    auto iter = planGoalMap_.find(currentId_);
    return iter->second;
}

PlanType_t Plan::getCurrentType()
{   
    auto iter = planTypeMap_.find(currentId_);
    return iter->second;
}


ActionType_t Plan::getCurrentAction()
{
    auto iter = actionTypeMap_.find(currentId_);
    return iter->second;
}
