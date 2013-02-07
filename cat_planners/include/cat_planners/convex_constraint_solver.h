#ifndef CAT_PLANNERS_CONVEX_CONSTRAINT_SOLVER_H
#define CAT_PLANNERS_CONVEX_CONSTRAINT_SOLVER_H

#include <cat_planners/CVXConfig.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/MotionPlanResponse.h>
#include <moveit_msgs/MotionPlanDetailedResponse.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_state/conversions.h>
#include <ros/ros.h>

namespace cat_planners
{

class ConvexConstraintSolver : public planning_interface::Planner
{
  public:
    ConvexConstraintSolver();
    virtual ~ConvexConstraintSolver();

    /*********************************************************/
    /// Subclass may implement methods below
    virtual void init(const kinematic_model::KinematicModelConstPtr& model) {}

    /// Get a short string that identifies the planning interface
    virtual std::string getDescription(void) const { return "Adam's potential field planner/solver."; }

    /// Get the names of the known planning algorithms (values that can be filled as planner_id in the planning request)
    virtual void getPlanningAlgorithms(std::vector<std::string> &algs) const { algs.clear(); }

    /**********************************************************/
    /// Subclass must implement methods below
    virtual bool solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                       const planning_interface::MotionPlanRequest &req,
                       planning_interface::MotionPlanResponse &res) const;


    virtual bool solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
               const planning_interface::MotionPlanRequest &req,
               planning_interface::MotionPlanDetailedResponse &res) const
    {
      planning_interface::MotionPlanResponse res2;
      if (solve(planning_scene, req, res2))
      {
        //res.trajectory_start = res2.trajectory_start;
        res.trajectory_.push_back(res2.trajectory_);
        res.description_.push_back("Constrained motion.");
        res.processing_time_.push_back(res2.planning_time_);
        return true;
      }
      return false;
    }

    /// Determine whether this plugin instance is able to represent this planning request
    virtual bool canServiceRequest(const moveit_msgs::MotionPlanRequest &req) const
    {
      ROS_WARN("This planner is allowing all motion planning requests, and may not be doing the right thing...");
      return true;
    }

    /// Request termination, if a solve() function is currently computing plans
    virtual void terminate(void) const {
    }

protected:

    class DynamicReconfigureImpl;
    DynamicReconfigureImpl *reconfigure_impl_;
    cat_planners::CVXConfig config_;

};


} // cat_planners

#endif // CAT_PLANNERS_CONVEX_CONSTRAINT_SOLVER_H
