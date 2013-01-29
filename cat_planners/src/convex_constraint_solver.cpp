#include <cat_planners/convex_constraint_solver.h>
#include <pluginlib/class_list_macros.h>
#include <moveit/kinematic_model/kinematic_model.h>
#include <moveit/kinematic_state/kinematic_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <cat_planners/util.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <moveit/collision_detection/collision_octomap_filter.h>
#include <dynamic_reconfigure/server.h>

// this implementation uses the problem-specific QP solver generated by CVXGEN
//#include "cvxgen_quad/solver.h"
#include "cvxgen_constraints/solver.h"
//CVX_Quad cvx;
CVX_Constraints cvx;

#define ROSCONSOLE_MIN_SEVERITY ROSCONSOLE_SEVERITY_DEBUG

// - - - - - - - - - - - - - - - - - - - - - - -  - - - - - - - - - - - - - - - - - - - - - -
namespace cat_planners {

class ConvexConstraintSolver::DynamicReconfigureImpl
{
public:

  DynamicReconfigureImpl(ConvexConstraintSolver *owner) : owner_(owner),
    dynamic_reconfigure_server_(ros::NodeHandle("~/cvx_solver"))
  {
    dynamic_reconfigure_server_.setCallback(boost::bind(&DynamicReconfigureImpl::dynamicReconfigureCallback, this, _1, _2));
  }

private:

  void dynamicReconfigureCallback(cat_planners::CVXConfig &config, uint32_t level)
  {
    cat_planners::CVXConfig old_config = owner_->config_ ;
    owner_->config_ = config;
  }

  ConvexConstraintSolver *owner_;
  dynamic_reconfigure::Server<cat_planners::CVXConfig> dynamic_reconfigure_server_;
};

} // namespace cat_planners

// - - - - - - - - - - - - - - - - - - - - - - -  - - - - - - - - - - - - - - - - - - - - - -


namespace cat_planners{

ConvexConstraintSolver::ConvexConstraintSolver()
{
  reconfigure_impl_ = new DynamicReconfigureImpl(this);
}

ConvexConstraintSolver::~ConvexConstraintSolver()
{
  if(reconfigure_impl_)
    delete reconfigure_impl_;
}

  // TODO can we pick the epsilon size more intelligently for each joint?

// This is "global" storage for the last collision state of the joint state group. It is
// a *TERRIBLE HACK* but I don't know how else to re-use old collision info...
boost::shared_ptr<collision_detection::CollisionResult> last_collision_result;


bool ConvexConstraintSolver::solve(const planning_scene::PlanningSceneConstPtr& a_planning_scene,
                   const moveit_msgs::MotionPlanRequest &req,
                   moveit_msgs::MotionPlanResponse &res) const
{
  // The raw algorithm laid out in Chan et. al. is as follows:
  //  1. Compute unconstrained motion to go from proxy to goal.
  //  2. Get the current contact set by moving the proxy toward the goal by some epsilon and get all collision points.
  //  3. Compute constrained motion (convex solver).
  //  4. Compute collisions along the constrained motion path.
  //  5. Set proxy to stop at the first new contact along path.


  // In our framework this will look more like this:
  // Outside ths planner:
  //  1. Set cartesian goal as a "constraint".

  // Inside the planner:
  //  1. Get the "current" contact set from the last collision result
  //     (or, take an unconstrained step to get the contact set, but then reset the proxy)
  //  2. Compute constrained motion (convex solver).
  //  3. Subdivide motion subject to some sort of minimum feature size.
  //  4. Move proxy in steps, checking for colliding state along the way. Optionally use interval bisection to refine.

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -/

// ====================================================================================================================
// ================================== Begin by extracting state and model information =================================
// ====================================================================================================================

  ros::Time planning_start_time = ros::Time::now();
  ros::Time planning_time_limit = planning_start_time + req.allowed_planning_time;
  ROS_DEBUG_NAMED("cvx_solver", "Entering ConvexConstraintSolver!");

  const int MAX_PROXY_STATES = config_.max_proxy_states;
  if(MAX_PROXY_STATES < 1)
  {
    ROS_ERROR("Can't have fewer than 1 proxy state, aborting!");
    res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    return false;
  }

  std::string planning_frame = ros::names::resolve("/", a_planning_scene->getPlanningFrame());

  // We "getCurrentState" just to populate the structure with auxiliary info, then copy in the transform info from the planning request.
  // Reserve space for upcoming proxy states...
  std::vector<kinematic_state::KinematicStatePtr> proxy_states;
  proxy_states.reserve(MAX_PROXY_STATES);
  int state_count = 1;
  proxy_states.push_back( kinematic_state::KinematicStatePtr( new kinematic_state::KinematicState(a_planning_scene->getCurrentState())));
  kinematic_state::robotStateToKinematicState(*(a_planning_scene->getTransforms()), req.start_state, *proxy_states.front());

  std::string group_name = req.group_name;

  // Get the main group (for planning)
  const boost::shared_ptr<const srdf::Model>& srdf = a_planning_scene->getKinematicModel()->getSRDF();
  const kinematic_model::JointModelGroup* jmg = a_planning_scene->getKinematicModel()->getJointModelGroup(group_name);

  // Check if there are subgroups...
  const std::vector<std::string>& subgroup_names = jmg->getSubgroupNames();
  if(subgroup_names.size() > 0)
  {
    ROS_WARN("Subgroups are not supported yet! (Group [%s] has %zd subgroups.) Aborting... ",
             group_name.c_str(), subgroup_names.size());
    res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
    return false;
  }


  std::string ee_group_name, ee_control_frame;
  int ee_index = -1;
  const std::vector<srdf::Model::EndEffector>&eef = srdf->getEndEffectors();
  for(size_t i = 0; i < eef.size(); i++)
  {
    if (jmg->hasLinkModel(eef[i].parent_link_) || jmg->getName() == eef[i].parent_group_)
    {
      ee_index = i;
      break;
    }
  }
  if(ee_index == -1)
  {
    ROS_WARN("Did not find any end-effectors attached to group [%s]! Aborting...", group_name.c_str());
    res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
    return false;
  }

  // we found an end-effector for this selected group
  ee_group_name = eef[ee_index].component_group_;
  ee_control_frame = eef[ee_index].parent_link_;

  ROS_DEBUG_NAMED("cvx_solver", "Solving for group [%s], end-effector [%s], control frame [%s].",
                  group_name.c_str(), ee_group_name.c_str(), ee_control_frame.c_str());

  // This can be computed once since it's only a model.
  const kinematic_model::JointModelGroup* ee_jmg = a_planning_scene->getKinematicModel()->getJointModelGroup(ee_group_name);

// ====================================================================================================================
// ============== Construct vectors for joint information so that we can easily work with them later ==================
// ====================================================================================================================

  ROS_DEBUG_NAMED("cvx_solver", "Vectorizing joint info.");  // about 80-120 us

  const std::map<std::string, unsigned int>& joint_index_map = jmg->getJointVariablesIndexMap();

  int num_joints = 7; // TODO definitely a magic number here...
  std::vector<double> limits_min(num_joints), limits_max(num_joints);
  std::vector<std::string> joint_names(num_joints);

  std::map<std::string, unsigned int>::const_iterator jim_it;
  for(jim_it = joint_index_map.begin(); jim_it != joint_index_map.end(); ++jim_it)
  {
    const std::string& joint_name = jim_it->first;
    unsigned int joint_index = jim_it->second;

    moveit_msgs::JointLimits limit = jmg->getJointModel(joint_name)->getVariableLimits()[0];
    if(limit.has_position_limits)
    {
      limits_min[joint_index] = limit.min_position;
      limits_max[joint_index] = limit.max_position;
    }
    else
    {
      // TODO this is... sort of a hack. Can we add scalars to the constraints that allow us to "turn them off"?
      limits_min[joint_index] = -1E3;
      limits_max[joint_index] =  1E3;
    }
    joint_names[joint_index] = joint_name;
    //ROS_DEBUG_NAMED("cvx_solver_math", "Joint [%d] [%s] has min %.2f, value %.2f, max %.2f",
    //                joint_index, joint_name.c_str(), limits_min[joint_index], joint_vector[joint_index], limits_max[joint_index]);
  }

  // ====================================================================================================================
  // ============================================ Extract goal "constraints" ============================================
  // ====================================================================================================================

    ROS_DEBUG_NAMED("cvx_solver", "Extracting goal constraints."); // about 140-340 us
    const moveit_msgs::Constraints &c = req.goal_constraints[0];

    // ==================================
    // First do lots of error checking...
    // ==================================
    if(c.position_constraints.size() != 1 || c.orientation_constraints.size() != 1)
    {
      ROS_ERROR("Currently require exactly one position and orientation constraint. (Have %zd and %zd.) Aborting...",
                c.position_constraints.size(), c.orientation_constraints.size());
      res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
      return false;
    }
    moveit_msgs::PositionConstraint    pc = c.position_constraints[0];
    moveit_msgs::OrientationConstraint oc = c.orientation_constraints[0];
    pc.header.frame_id = ros::names::resolve("/", pc.header.frame_id);
    oc.header.frame_id = ros::names::resolve("/", oc.header.frame_id);

    if(pc.link_name != oc.link_name)
    {
      ROS_ERROR("Position [%s] and orientation [%s] goals are not for the same link. Aborting...",
                pc.link_name.c_str(), oc.link_name.c_str());
      res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
      return false;
    }
    if(pc.constraint_region.primitive_poses.size() != 1)
    {
      if(pc.constraint_region.primitive_poses.size() == 0)
        ROS_ERROR("No primitive_pose specified for position constraint region. Aborting...");
      if(pc.constraint_region.primitive_poses.size() > 1)
        ROS_ERROR("Should only specificy one primitice_pose for goal region. Aborting...");
      res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
      return false;
    }

    if(pc.header.frame_id != planning_frame)
    {
      ROS_WARN("The position goal header [%s] and planning_frame [%s] don't match, have to abort!",
               pc.header.frame_id.c_str(), planning_frame.c_str() );
      res.error_code.val = moveit_msgs::MoveItErrorCodes::FRAME_TRANSFORM_FAILURE;
      return false;
    }
    if(oc.header.frame_id != planning_frame)
    {
      ROS_WARN("The orientation goal header [%s] and planning_frame [%s] don't match, have to abort!",
               oc.header.frame_id.c_str(), planning_frame.c_str() );
      res.error_code.val = moveit_msgs::MoveItErrorCodes::FRAME_TRANSFORM_FAILURE;
      return false;
    }

    // ==================================
    // Now actually extract goal pose
    // ==================================
    Eigen::Vector3d goal_point;
    Eigen::Quaterniond goal_quaternion_e;
    { // scoped so we don't pollute function scope with these message temps
      const geometry_msgs::Point& msg_goal_point = pc.constraint_region.primitive_poses[0].position;
      const geometry_msgs::Quaternion& msg_goal_orientation = oc.orientation;
      goal_point = Eigen::Vector3d(msg_goal_point.x, msg_goal_point.y, msg_goal_point.z);
      goal_quaternion_e = Eigen::Quaterniond(msg_goal_orientation.w, msg_goal_orientation.x, msg_goal_orientation.y, msg_goal_orientation.z);
    }


  // **********************************************************************************************************************
  // **********************************************************************************************************************
  // Loop here
  // **********************************************************************************************************************

  // create a re-useable collision request query
  collision_detection::CollisionRequest collision_request;
  collision_request.max_contacts = config_.max_contacts;
  collision_request.contacts = true;
  collision_request.distance = false;
  collision_request.verbose = false;
  collision_request.group_name = group_name;

  // Check how much time is left
  ros::Duration time_remaining = planning_time_limit - ros::Time::now();

  while(time_remaining.toSec() > config_.minimum_reserve_time
        && state_count < MAX_PROXY_STATES)
  {
    const kinematic_state::JointStateGroup* jsg = proxy_states.back()->getJointStateGroup(group_name);
    std::vector<double> joint_vector(num_joints);
    for(size_t i = 0; i < joint_names.size(); ++i)
    {
      joint_vector[i] = proxy_states.back()->getJointState(joint_names[i])->getVariableValues()[0];
    }

  // ====================================================================================================================
  // ======== Extract all contact points and normals from previous collision state, get associated Jacobians ============
  // ====================================================================================================================

    // TODO: we can avoid storing a global collision state if we instead use the planner to take a small step with no constraints to get a useable "goal state".
    // How much time does that take?

    ROS_DEBUG_NAMED("cvx_solver", "Extracting current contact constraints."); // about 60-200 us

    std::vector<Eigen::MatrixXd> contact_jacobians;
    std::vector<Eigen::Vector3d> contact_normals;

    if(!last_collision_result)
      last_collision_result.reset(new collision_detection::CollisionResult());

    for( collision_detection::CollisionResult::ContactMap::const_iterator it = last_collision_result->contacts.begin(); it != last_collision_result->contacts.end(); ++it)
    {
      std::string contact1 = it->first.first;
      std::string contact2 = it->first.second;
      std::string group_contact;
      if(      jmg->hasLinkModel(contact1) || ee_jmg->hasLinkModel(contact1) ) group_contact = contact1;
      else if( jmg->hasLinkModel(contact2) || ee_jmg->hasLinkModel(contact2) ) group_contact = contact2;
      else
      {
        //ROS_WARN("Contact isn't on group [%s], skipping...", req.motion_plan_request.group_name.c_str());
        continue;
      }

      const std::vector<collision_detection::Contact>& vec = it->second;

      for(size_t contact_index = 0; contact_index < vec.size(); contact_index++)
      {
        Eigen::Vector3d point =   vec[contact_index].pos;
        Eigen::Vector3d normal =  vec[contact_index].normal;
        double depth = vec[contact_index].depth;

        // Contact point needs to be expressed with respect to the link; normals should stay in the common frame
        kinematic_state::LinkState *link_state = proxy_states.back()->getLinkState(group_contact);
        Eigen::Affine3d link_T_world = link_state->getGlobalCollisionBodyTransform().inverse();
        point = link_T_world*point;

        Eigen::MatrixXd jacobian;
        if(jsg->getJacobian(group_contact, point, jacobian))
        {
          contact_jacobians.push_back(jacobian);
          if(!config_.refine_normals && contact1.find("octomap") != std::string::npos || contact2.find("octomap") != std::string::npos)
            normal = -1.0*normal;
          contact_normals.push_back(normal);
        }
        ROS_DEBUG_NAMED("cvx_solver_contacts", "Contact between [%s] and [%s] p = [%.3f, %.3f %.3f], n = [%.2f %.2f %.2f]",
                 contact1.c_str(), contact2.c_str(),
                 point(0), point(1), point(2),
                 normal(0), normal(1), normal(2));
      }
    }

  // ====================================================================================================================
  // ============================================= Compute cartesian error ==============================================
  // ====================================================================================================================

    ROS_DEBUG_NAMED("cvx_solver", "Computing cartesian error."); // about 140-340 us

    // Do some transforms...
    Eigen::Affine3d planning_T_link = proxy_states.back()->getLinkState(pc.link_name)->getGlobalLinkTransform();
    Eigen::Vector3d ee_point_in_ee_frame = Eigen::Vector3d(pc.target_point_offset.x, pc.target_point_offset.y, pc.target_point_offset.z);
    Eigen::Vector3d ee_point_in_planning_frame = planning_T_link*ee_point_in_ee_frame;

    // TODO need to make sure these are expressed in the same frame.
    Eigen::Vector3d x_error = goal_point - ee_point_in_planning_frame;
    Eigen::Vector3d delta_x = x_error;
    double x_error_mag = x_error.norm();
    if(x_error_mag > config_.max_linear_error)
      delta_x = x_error/x_error_mag*config_.max_linear_error;
    ROS_DEBUG_NAMED("cvx_solver_math", "Delta position in planning_frame = [%.3f, %.3f, %.3f]", delta_x[0], delta_x[1], delta_x[2]);


    // ===================================
    // = = = = Rotations are gross = = = =
    // ===================================
    Eigen::Quaterniond link_quaternion_e = Eigen::Quaterniond(planning_T_link.rotation());
    tf::Quaternion link_quaternion_tf, goal_quaternion_tf;
    tf::quaternionEigenToTF( link_quaternion_e, link_quaternion_tf );
    tf::quaternionEigenToTF( goal_quaternion_e, goal_quaternion_tf );

    tf::Quaternion delta_quaternion = link_quaternion_tf.inverse()*goal_quaternion_tf;
    double rotation_angle = delta_quaternion.getAngle();
    double clipped_rotation_fraction = std::min<double>(1.0, config_.max_angle_error/fabs(rotation_angle));
    if(clipped_rotation_fraction < 0) ROS_ERROR("Clipped rotation fraction < 0, look into this!");

    tf::Matrix3x3 clipped_delta_matrix;
    tf::Quaternion clipped_goal_quaternion = link_quaternion_tf.slerp(goal_quaternion_tf, clipped_rotation_fraction);
    clipped_delta_matrix.setRotation( link_quaternion_tf.inverse()*clipped_goal_quaternion );


    tf::Vector3 delta_euler;
    clipped_delta_matrix.getRPY(delta_euler[0], delta_euler[1], delta_euler[2]);
    //ROS_DEBUG_NAMED("cvx_solver", "Delta euler in link frame = [%.3f, %.3f, %.3f]", delta_euler[0], delta_euler[1], delta_euler[2]);
    tf::Matrix3x3 link_matrix(link_quaternion_tf);
    delta_euler = link_matrix * delta_euler;
    ROS_DEBUG_NAMED("cvx_solver_math", "Delta euler in planning_frame = [%.3f, %.3f, %.3f]", delta_euler[0], delta_euler[1], delta_euler[2]);


    // ===================================
    // ==== Get end-effector Jacobian ====
    // ===================================
    if(ee_control_frame != pc.link_name)
      ROS_WARN("ee_control_frame [%s] and position_goal link [%s] aren't the same, this could be bad!", ee_control_frame.c_str(), pc.link_name.c_str());


    ROS_DEBUG_NAMED("cvx_solver_math", "Getting end-effector Jacobian for local point %.3f, %.3f, %.3f on link [%s]",
             ee_point_in_ee_frame(0), ee_point_in_ee_frame(1), ee_point_in_ee_frame(2), ee_control_frame.c_str());
    Eigen::MatrixXd ee_jacobian;
    if(!jsg->getJacobian(ee_control_frame , ee_point_in_ee_frame , ee_jacobian))
    {
      ROS_ERROR("Unable to get end-effector Jacobian! Can't plan, exiting...");
      res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_LINK_NAME;
      return false;
    }

    //ROS_DEBUG_STREAM_NAMED("cvx_solver_math", "End-effector jacobian in planning frame is: \n" << ee_jacobian);

  // ====================================================================================================================
  // =================================== Pack into solver data structure, run solver ====================================
  // ====================================================================================================================

    ROS_DEBUG_NAMED("cvx_solver", "Packing data into the cvx solver."); // about 70-90 us

    // CVX Settings
    cvx.set_defaults();
    cvx.setup_indexing();
    cvx.settings.verbose = 0;

    // ===================================
    // ======== Load problem data ========
    // ===================================
    unsigned int N = num_joints; // number of joints in the chain

    // End-effector Jacobian
    for(unsigned int row = 0; row < 3; row++ )
    {
      for(unsigned int col = 0; col < N; col++ )
      {
        // CVX matrices are COLUMN-MAJOR!!!
        cvx.params.J_v[col*3 + row] = ee_jacobian(row,   col);
        cvx.params.J_w[col*3 + row] = ee_jacobian(row+3, col);
      }
    }

    // Weights for pieces of the objective function
    cvx.params.weight_x[0] = config_.xdot_error_weight; // was 1.0.    Translational error
    cvx.params.weight_w[0] = config_.wdot_error_weight; // was 0.1.    Angular error (error in radians is numerically much larger than error in meters)
    cvx.params.weight_q[0] = config_.delta_q_weight;    // was 0.001.  Only want to barely encourage values to stay small...

    // Constraints from contact set
    unsigned int MAX_CONSTRAINTS = 25;
    unsigned int constraint_count = std::min<size_t>(MAX_CONSTRAINTS, contact_normals.size());
    for(unsigned int constraint = 0; constraint < MAX_CONSTRAINTS; constraint++)
    {
      if(constraint < constraint_count)
      {
        // CVX matrices are COLUMN-MAJOR!!!
        for (int j = 0; j < 3*7; j++) {
          cvx.params.J_c[constraint][j] = contact_jacobians[constraint](j%3, j/3);
        }
        for (int j = 0; j < 3; j++) {
          cvx.params.normal[constraint][j] = contact_normals[constraint][j];
        }
      }
      else{
        //printf("setting to zero\n");
        for (int j = 0; j < 3*7; j++) {
          cvx.params.J_c[constraint][j] = 0;
        }
        for (int j = 0; j < 3; j++) {
          cvx.params.normal[constraint][j] = 0;
        }
      }
    }

    // Joint states and limits
    for(unsigned int index = 0; index < N; index++ )
    {
      cvx.params.q[index] = joint_vector[index];
      cvx.params.q_min[index] = limits_min[index];
      cvx.params.q_max[index] = limits_max[index];
    }

    // Goal velocity
    for(unsigned int index = 0; index < 3; index++ )
    {
      cvx.params.x_d[index] = delta_x(index);
      cvx.params.w_d[index] = delta_euler[index];
    }

    // ===================================
    // ====== Solve at high speed! =======
    // ===================================
    ROS_DEBUG_NAMED("cvx_solver", "Running solver!"); // about 260-300 us
    long num_iters = 0;
    num_iters = cvx.solve();
    if(!cvx.work.converged)
    {
      ROS_WARN("solving failed to converge in %ld iterations.", num_iters);
      res.error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
      return false;
    }

    // ===================================
    // ====== Unpack solver result =======
    // ===================================
    ROS_DEBUG_NAMED("cvx_solver", "Found solution in %ld iterations. Unpacking result...", num_iters); // about 120-170 us

    //Eigen::VectorXd joint_deltas(N);
    std::map<std::string, double> goal_update;
    double largest_change = 0.0;
    for(size_t joint_index = 0; joint_index < joint_names.size(); joint_index++)
    {
      if( fabs(cvx.vars.q_d[joint_index]) > largest_change )
        largest_change = fabs(cvx.vars.q_d[joint_index]);
      goal_update[joint_names[joint_index]] = cvx.vars.q_d[joint_index] + joint_vector[joint_index];
      ROS_DEBUG_NAMED("cvx_solver_math", "Updated joint [%zd] [%s]: %.3f + %.3f",
                      joint_index,
                      joint_names[joint_index].c_str(),
                      joint_vector[joint_index],
                      cvx.vars.q_d[joint_index]);
    }
    if(largest_change < config_.proxy_joint_tolerance)
    {
      ROS_DEBUG_NAMED("cvx_solver", "Largest joint change [%.3f] is smaller than proxy threshold [%.3f]. Exiting...",
                largest_change, config_.proxy_joint_tolerance);
      res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
      break;
    }

    kinematic_state::KinematicStatePtr next_state(new kinematic_state::KinematicState(*proxy_states.back()));
    next_state->setStateValues(goal_update);
    next_state->updateLinkTransforms();

    //  // Compute and print some debug stuff
    //  if(false)
    //  {
    //    Eigen::VectorXd cartesian_deltas = ee_jacobian*joint_deltas;
    //    ROS_DEBUG_NAMED("cvx_solver_math", "Desired velocity: translate [%.3f, %.3f, %.3f]  euler [%.2f, %.2f, %.2f]",
    //                    delta_x(0), delta_x(1), delta_x(2),
    //                    delta_euler[0], delta_euler[1], delta_euler[2]);
    //    ROS_DEBUG_NAMED("cvx_solver_math", "Computed velocity:    translate [%.3f, %.3f, %.3f]  euler [%.2f, %.2f, %.2f]",
    //                    cartesian_deltas(0), cartesian_deltas(1), cartesian_deltas(2),
    //                    cartesian_deltas(3), cartesian_deltas(4), cartesian_deltas(5));
    //  }


  // ====================================================================================================================
  // =========================================== Check for new collisions ===============================================
  // ====================================================================================================================

    ROS_DEBUG_NAMED("cvx_solver", "Checking for collisions in new state.");

    ros::Time collision_start = ros::Time::now();
    last_collision_result->clear();
    a_planning_scene->checkCollision(collision_request, *last_collision_result, *next_state);
    ROS_DEBUG_NAMED("cvx_solver", "Collision check took %.3f ms", (ros::Time::now() - collision_start).toSec() * 1000.0 );
    // Refine normals on the last collision result, if applicable!
    if(last_collision_result->collision && config_.refine_normals)
    {
      ros::Time collision_start = ros::Time::now();
      const collision_detection::CollisionWorld::ObjectConstPtr& octomap_object =
          a_planning_scene->getCollisionWorld()->getObject(planning_scene::PlanningScene::OCTOMAP_NS);
      int modified = collision_detection::refineContactNormals(octomap_object, *last_collision_result,
                                                               config_.bbx_search_size,
                                                               config_.min_angle_change, false,
                                                               0.5, 1.5); // *** TODO *** make these paramters ***
      ROS_DEBUG_NAMED("cvx_solver", "Adjusted %d of %zd contact normals in %.3f ms",
                      modified, last_collision_result->contact_count, (ros::Time::now() - collision_start).toSec() * 1000.0 );
    }
    if(last_collision_result->collision)
    {
      if(config_.velocity_constraint_only)
      {
        ROS_DEBUG_NAMED("cvx_solver", "Not saving this point because we are enforcing the position-constraint.");
      }
      else
      {
        ROS_DEBUG_NAMED("cvx_solver", "In collision, but saving this point because we are using velocity contraint only.");
        proxy_states.push_back(next_state);
        state_count++;
      }
      break;
    }
    else
    {
      proxy_states.push_back(next_state);
      state_count++;
    }

    // Check how much time is left
    time_remaining = planning_time_limit - ros::Time::now();
    ROS_DEBUG_NAMED("cvx_solver", "Have %d proxy states, have %.3f ms of %.3f ms remaining. ",
                    state_count, time_remaining.toSec()*1000.0, req.allowed_planning_time.toSec()*1000.0);
  }


// ====================================================================================================================
// ============================= Create a trajectory from the proxy states =================================
// ====================================================================================================================

  if(state_count < 2)
  {
    ROS_DEBUG_NAMED("cvx_solver", "Only have %d proxy states, not returning a trajectory.", state_count);
    return false;
  }

  std::vector<kinematic_state::KinematicStatePtr> filtered_proxies;
  filtered_proxies.reserve(proxy_states.size());
  for(size_t i = 0; i < proxy_states.size() - 1; ++i)
  {
    if( 0 == (i % config_.state_skipping) )
      filtered_proxies.push_back(proxy_states[i]);
  }
  // Always add the last state
  filtered_proxies.push_back(proxy_states.back());

  ROS_DEBUG_NAMED("cvx_solver", "Creating trajectory with %zd points.", filtered_proxies.size());
  trajectory_msgs::JointTrajectory& traj = res.trajectory.joint_trajectory;
  kinematicStateVectorToJointTrajectory(filtered_proxies, group_name, traj);
  traj.header.frame_id = planning_frame;
  res.trajectory_start = req.start_state;
  res.group_name = group_name;
  res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

  ROS_DEBUG_NAMED("cvx_solver_result", "CVX trajectory has %zd points.", traj.points.size());
  ROS_DEBUG_NAMED("cvx_solver", "CVX planning done in %.3f ms.", (ros::Time::now() - planning_start_time).toSec() * 1000.0);
  return true;
}

} // namespace cat_planners

PLUGINLIB_EXPORT_CLASS( cat_planners::ConvexConstraintSolver, planning_interface::Planner);

