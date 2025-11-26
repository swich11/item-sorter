#include <moveit/hybrid_planning_manager/planner_logic_interface.h>



namespace moveit::hybrid_planning 
{


class PlannerLogic : public PlannerLogicInterface {
public:
    PlannerLogic() = default;
    ~PlannerLogic() = default;

    // The plugin needs a shared pointer to the hybrid planning manager to access its member functions like planGlobalTrajectory()
    bool initialize(const std::shared_ptr<HybridPlanningManager>& hybrid_planning_manager) override;

    // This function can be used to implement reaction to some default Hybrid Planning events
    ReactionResult react(const HybridPlanningEvent& event) override;

    // Here are reactions to custom events encoded as string implemented
    ReactionResult react(const std::string& event) override;

private:
    std::shared_ptr<HybridPlanningManager> hybrid_planning_manager;
};
};