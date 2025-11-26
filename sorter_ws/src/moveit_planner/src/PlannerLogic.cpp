#include "moveit_planner/PlannerLogic.hpp"


using namespace moveit::hybrid_planning;


bool PlannerLogic::initialize(const std::shared_ptr<HybridPlanningManager> &hybrid_planning_manager) {
    this->hybrid_planning_manager = hybrid_planning_manager;
    return true;
}


ReactionResult PlannerLogic::react(const HybridPlanningEvent &event) {
    
}


ReactionResult PlannerLogic::react(const std::string &event) {

}