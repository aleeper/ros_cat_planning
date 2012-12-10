#include <cat_planners/convex_constraint_solver.h>
#include <pluginlib/class_list_macros.h>
#include <moveit/kinematic_model/kinematic_model.h>
//#include <planning_models/kinematic_model/joint_model_group-inc.h>
#include <moveit/kinematic_state/kinematic_state.h>
#include <moveit/planning_scene/planning_scene.h>
//#include <planning_models/kinematic_state/joint_state_group-inc.h>
#include <cat_planners/util.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <moveit/collision_detection/collision_octomap_filter.h>

// this implementation uses the problem-specific QP solver generated by CVXGEN
//#include "cvxgen_quad/solver.h"
#include "cvxgen_constraints/solver.h"
//CVX_Quad cvx;
CVX_Constraints cvx;


namespace cat_planners{

  // TODO can we pick the epsilon size more intelligently for each joint?

// This is "global" storage for the last collision state of the joint state group. It is
// a *TERRIBLE HACK* but I don't know how else to re-use old collision info...
collision_detection::CollisionResult last_collision_result;




bool ConvexConstraintSolver::solve(const planning_scene::PlanningSceneConstPtr& a_planning_scene,
                   const moveit_msgs::GetMotionPlan::Request &req,
                   moveit_msgs::GetMotionPlan::Response &res) const
{
  // Need to add in joint limit constraints
  // Get the position constraints from the collision detector

  // The raw algorithm laid out in Chan et. al. is as follows:
  //  1. Compute unconstrained motion to go from proxy to goal.
  //  2. Get the current contact set by moving the proxy toward the goal by some epsilon and get all collision points.
  //  3. Compute constrained motion (convex solver).
  //  4. Compute collisions along the constrained motion path.
  //  5. Set proxy to stop at the first new contact along path.


  // In our framework this will look more like this:
  // Outside ths planner:
  //  1. Compute error between cartesian end effector and cartesian goal.
  //  2. Cap pose error (based on some heuristic?) because we are about to make a linear approximation.
  //  3. Use Jinverse to compute the joint deltas for the pose error (or perhaps we should frame this as a velocity problem).
  //  4. Add the joint deltas to the start state to get a goal state.

  // Inside the planner:
  //  1. Get the "current" contact set by moving the proxy toward the goal by some epsilon and get all collision points.
  //  2. Compute constrained motion (convex solver).
  //  3. Subdivide motion subject to some sort of minimum feature size.
  //  4. Move proxy in steps, checking for colliding state along the way. Optionally use interval bisection to refine.

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -/

  std::string group_name = req.motion_plan_request.group_name;
  std::string ee_group_name;
  std::string ee_control_frame;
  if(group_name == "right_arm"){ ee_group_name = "r_end_effector"; ee_control_frame = "r_wrist_roll_link"; }
  if(group_name == "left_arm") { ee_group_name = "l_end_effector"; ee_control_frame = "l_wrist_roll_link"; }


  ROS_DEBUG("Solving for group [%s], end-effector [%s], control frame [%s] using ConvexConstraintSolver!", group_name.c_str(), ee_group_name.c_str(), ee_control_frame.c_str());


  // We "getCurrentState" just to populate the structure with auxiliary info, then copy in the transform info from the planning request.
  kinematic_state::KinematicState start_state = a_planning_scene->getCurrentState();
  kinematic_state::robotStateToKinematicState(*(a_planning_scene->getTransforms()), req.motion_plan_request.start_state, start_state);

  // constrained_goal_state is the optimization output before interval stepping, proxy_state will be used as we step around.
  kinematic_state::KinematicState proxy_state = start_state;
  kinematic_state::KinematicState constrained_goal_state = start_state;

  std::string planning_frame = ros::names::resolve("/", a_planning_scene->getPlanningFrame());


    const kinematic_state::JointStateGroup * jsg = proxy_state.getJointStateGroup(group_name);
    const kinematic_model::JointModelGroup * jmg = a_planning_scene->getKinematicModel()->getJointModelGroup(group_name);
//    std::string end_effector_group_name = jmg->getAttachedEndEffectorGroupName();
//    const std::vector<std::string>& subgroups = jmg->getSubgroupNames();

//    ROS_INFO("End-effector group name is [%s].", end_effector_group_name.c_str());
//    for(size_t i = 0; i< subgroups.size(); i++)
//      ROS_INFO("Subgroup [%zd] is [%s].", i, subgroups[i].c_str());
    const kinematic_model::JointModelGroup * ee_jmg = a_planning_scene->getKinematicModel()->getJointModelGroup(ee_group_name);


//    const std::vector<const kinematic_model::KinematicModel::JointModel*>& joint_models = jmg->getJointModels();
//    std::map<std::string, unsigned int> joint_index_map;
//    for(size_t i = 0; i < joint_models.size(); i++)
//    {
//      //ROS_INFO("Group [%s] has joint %d: [%s]", req.motion_plan_request.group_name.c_str(), i, joint_models[i]->getName().c_str());
//      joint_index_map[joint_models[i]->getName()] = i;
//    }

    const std::map<std::string, unsigned int>& joint_index_map = jmg->getJointVariablesIndexMap();

    std::vector<double> limits_min, limits_max, joint_vector;
    std::vector<std::string> joint_names;

    limits_min.resize(7);
    limits_max.resize(7);
    joint_vector.resize(7);
    joint_names.resize(7);


    {
      const moveit_msgs::Constraints &c = req.motion_plan_request.goal_constraints[1];
      for (std::size_t i = 0 ; i < c.joint_constraints.size() ; ++i)
      {
        std::string joint_name = c.joint_constraints[i].joint_name;
        if(joint_index_map.find(joint_name) == joint_index_map.end())
        {
          ROS_WARN("Didin't find [%s] in the joint map, ignoring...", joint_name.c_str());
          continue;
        }
        unsigned int joint_index = joint_index_map.find(joint_name)->second;

        std::vector<moveit_msgs::JointLimits> limits = a_planning_scene->getKinematicModel()->getJointModel(joint_name)->getVariableLimits();
        moveit_msgs::JointLimits limit = limits[0];
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
        joint_vector[joint_index] = start_state.getJointState(joint_name)->getVariableValues()[0];
        joint_names[joint_index] = joint_name;
        //ROS_INFO("Joint [%d] [%s] has min %.2f, value %.2f, max %.2f", joint_index, joint_name.c_str(), limits_min[joint_index], joint_vector[joint_index], limits_max[joint_index]);
      }
    }

// ======== Extract all contact points and normals from previous collision state, get associated Jacobians ========

    // TODO: we can avoid storing a global collision state if we instead use the planner to take a small step with no constraints to get a useable "goal state".

    std::vector<Eigen::MatrixXd> contact_jacobians;
    std::vector<Eigen::Vector3d> contact_normals;

    for( collision_detection::CollisionResult::ContactMap::const_iterator it = last_collision_result.contacts.begin(); it != last_collision_result.contacts.end(); ++it)
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
        Eigen::Vector3d point =     vec[contact_index].pos;
        Eigen::Vector3d normal =  vec[contact_index].normal;
        double depth = vec[contact_index].depth;
//        ROS_INFO("Contact between [%s] and [%s] point: %.2f %.2f %.2f normal: %.2f %.2f %.2f depth: %.3f",
//                 contact1.c_str(), contact2.c_str(),
//                 point(0), point(1), point(2),
//                 normal(0), normal(1), normal(2),
//                 depth);
        // Contact point needs to be expressed with respect to the link; normals should stay in the common frame
        kinematic_state::LinkState *link_state = start_state.getLinkState(group_contact);
        Eigen::Affine3d link_T_world = link_state->getGlobalCollisionBodyTransform().inverse();
        point = link_T_world*point;

        Eigen::MatrixXd jacobian;
        if(jsg->getJacobian(group_contact, point, jacobian))
        {
          contact_jacobians.push_back(jacobian);
//          if(contact1.find("octomap") != std::string::npos || contact2.find("octomap") != std::string::npos)
//            normal = -1.0*normal;
          contact_normals.push_back(normal);
        }
      }
    }


// ======== Extract goal "constraints" ========

  const moveit_msgs::Constraints &c = req.motion_plan_request.goal_constraints[0];

  // Position and Orientation
  if(c.position_constraints.size() != 1 || c.orientation_constraints.size() != 1)
  {
    ROS_ERROR("Currently require exactly one position and orientation constraint. Aborting...");
    return false;
  }
  moveit_msgs::PositionConstraint    pc = c.position_constraints[0];
  moveit_msgs::OrientationConstraint oc = c.orientation_constraints[0];
  pc.header.frame_id = ros::names::resolve("/", pc.header.frame_id);
  oc.header.frame_id = ros::names::resolve("/", oc.header.frame_id);

  if(pc.link_name != oc.link_name)
  {
    ROS_ERROR("Currently can't support position and orientation goals that are not for the same link. Aborting...");
    return false;
  }
  if(pc.constraint_region.primitive_poses.size() == 0)
  {
    ROS_ERROR("Need to specify a single primitive_pose for position constraint region. Aborting...");
    return false;
  }
  if(pc.constraint_region.primitive_poses.size() != 1)
  {
    ROS_ERROR("Need exactly one 'pose' for the end-effector goal region. Aborting...");
  }


  if(pc.header.frame_id != planning_frame)
    ROS_WARN("The position goal header [%s] and planning_frame [%s] don't match, things are probably all wrong!",
             pc.header.frame_id.c_str(), planning_frame.c_str() );
  if(oc.header.frame_id != planning_frame)
    ROS_WARN("The orientation goal header [%s] and planning_frame [%s] don't match, things are probably all wrong!",
             oc.header.frame_id.c_str(), planning_frame.c_str() );

  Eigen::Vector3d goal_point;
  Eigen::Quaterniond goal_quaternion_e;
  { // scoped so we don't pollute function scope with these message temps
    geometry_msgs::Point msg_goal_point = pc.constraint_region.primitive_poses[0].position;
    geometry_msgs::Quaternion msg_goal_orientation = oc.orientation;
    goal_point = Eigen::Vector3d(msg_goal_point.x, msg_goal_point.y, msg_goal_point.z);
    goal_quaternion_e = Eigen::Quaterniond(msg_goal_orientation.w, msg_goal_orientation.x, msg_goal_orientation.y, msg_goal_orientation.z);
  }


  //ROS_INFO("Computing position and orientation error...");
  kinematic_state::LinkState *link_state = start_state.getLinkState(pc.link_name);
  Eigen::Affine3d planning_T_link = link_state->getGlobalLinkTransform();

  Eigen::Vector3d ee_point_in_ee_frame = Eigen::Vector3d(pc.target_point_offset.x, pc.target_point_offset.y, pc.target_point_offset.z);
  Eigen::Vector3d ee_point_in_planning_frame = planning_T_link*ee_point_in_ee_frame;

  // TODO need to make sure these are expressed in the same frame.
  Eigen::Vector3d x_error = goal_point - ee_point_in_planning_frame;
  Eigen::Vector3d delta_x = x_error;
  double x_error_mag = x_error.norm();
  double LINEAR_CLIP = 0.02;
  if(x_error_mag > LINEAR_CLIP)
    delta_x = x_error/x_error_mag*LINEAR_CLIP;


  // = = = = Rotations are gross = = = =
  Eigen::Quaterniond link_quaternion_e = Eigen::Quaterniond(planning_T_link.rotation());
  tf::Quaternion link_quaternion_tf, goal_quaternion_tf;
  tf::quaternionEigenToTF( link_quaternion_e, link_quaternion_tf );
  tf::quaternionEigenToTF( goal_quaternion_e, goal_quaternion_tf );

  tf::Quaternion delta_quaternion = link_quaternion_tf.inverse()*goal_quaternion_tf;
  double rotation_angle = delta_quaternion.getAngle();
  //tf::Vector3 rotation_axis = delta_quaternion.getAxis();
  //ROS_INFO("Delta is [%.3f] radians about [%.3f, %.3f, %.3f]", rotation_angle, rotation_axis.x(), rotation_axis.y(), rotation_axis.z());
  double ANGLE_CLIP = 0.2;
  double clipped_rotation_fraction = std::min<double>(1.0, ANGLE_CLIP/fabs(rotation_angle));
  if(clipped_rotation_fraction < 0) ROS_ERROR("Clipped rotation fraction < 0, look into this!");

  tf::Matrix3x3 clipped_delta_matrix;
  tf::Quaternion clipped_goal_quaternion = link_quaternion_tf.slerp(goal_quaternion_tf, clipped_rotation_fraction);
  clipped_delta_matrix.setRotation( link_quaternion_tf.inverse()*clipped_goal_quaternion );


  tf::Vector3 delta_euler;
  clipped_delta_matrix.getRPY(delta_euler[0], delta_euler[1], delta_euler[2]);
  //ROS_INFO("Delta euler in link frame = [%.3f, %.3f, %.3f]", delta_euler[0], delta_euler[1], delta_euler[2]);
  tf::Matrix3x3 link_matrix(link_quaternion_tf);
  delta_euler = link_matrix * delta_euler;
  //ROS_INFO("Delta euler in planning_frame = [%.3f, %.3f, %.3f]", delta_euler[0], delta_euler[1], delta_euler[2]);


  // Get end-effector Jacobian
//  std::string ee_control_frame = "r_wrist_roll_link";
//  if(false && jmg->isChain())
//    ee_control_frame = a_planning_scene->getKinematicModel()->getJointModelGroup(jmg->getAttachedEndEffectorGroupName())->getEndEffectorParentGroup().second;
//  else
//    ROS_WARN("Using r_wrist_roll_link as HARD_CODED value.");
  if(ee_control_frame != pc.link_name)
    ROS_WARN("ee_control_frame [%s] and position_goal link [%s] aren't the same, this could be bad!", ee_control_frame.c_str(), pc.link_name.c_str());


//  ROS_INFO("Getting end-effector Jacobian for local point %.3f, %.3f, %.3f on link [%s]",
//           ee_point_in_ee_frame(0),
//           ee_point_in_ee_frame(1),
//           ee_point_in_ee_frame(2),
//           ee_control_frame.c_str());
  Eigen::MatrixXd ee_jacobian;
  if(!jsg->getJacobian(ee_control_frame , ee_point_in_ee_frame , ee_jacobian))
  {
    ROS_ERROR("Unable to get end-effector Jacobian! Can't plan, exiting...");
    return false;
  }

  //ROS_INFO_STREAM("End-effector jacobian in planning frame is: \n" << ee_jacobian);


// ======== Pack into solver data structure, run solver. ========

  //ROS_INFO("Packing data into the cvx solver...");
//  Vars vars;
//  Params params;
//  Workspace work;
//  Settings settings;

  // CVX Settings
  cvx.set_defaults();
  cvx.setup_indexing();
  cvx.settings.verbose = 0;

  // - - - - - - - load all problem instance data - - - - - - - //

  unsigned int N = 7; // number of joints in the chain

  // end-effector Jacobian
  // TODO magic numbers (though I suppose the CVX solver is already hard-coded)
  for(unsigned int row = 0; row < 3; row++ )
  {
    for(unsigned int col = 0; col < N; col++ )
    {
      // CVX matrices are COLUMN-MAJOR!!!
      cvx.params.J_v[col*3 + row] = ee_jacobian(row,   col);
      cvx.params.J_w[col*3 + row] = ee_jacobian(row+3, col);
    }
  }

  // Set weights for objective terms
  cvx.params.weight_x[0] = 1.0; // translational error
  cvx.params.weight_w[0] = 0.1; // angular error (error in radians is numerically much larger than error in meters)
  cvx.params.weight_q[0] = 0.001; // only want to barely encourage values to stay small...

  // set up constraints from contact set
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

  for(unsigned int index = 0; index < N; index++ )
  {
    cvx.params.q[index] = joint_vector[index];
    cvx.params.q_min[index] = limits_min[index];
    cvx.params.q_max[index] = limits_max[index];
  }

  // TODO should these be clipped down at all? :)
  for(unsigned int index = 0; index < 3; index++ )
  {
    cvx.params.x_d[index] = delta_x(index);  // TODO magic minus sign!!
    cvx.params.w_d[index] = delta_euler[index];
  }



  // - - - - - - - Solve our problem at high speed! - - - - - - - //
  long num_iters = 0;
  num_iters = cvx.solve();
  if(!cvx.work.converged)
  {
    ROS_WARN("solving failed to converge in %ld iterations.", num_iters);
    return false;
  }

// ======== Unpack solver result into constrained goal state. ========



  ROS_DEBUG("Finished solving in %ld iterations, unpacking data...", num_iters);
  Eigen::VectorXd joint_deltas(N);
  std::map<std::string, double> goal_update;
  for(size_t joint_index = 0; joint_index < joint_names.size(); joint_index++)
  {
    std::string joint_name = joint_names[joint_index];
    joint_deltas(joint_index) = cvx.vars.q_d[joint_index];
    goal_update[joint_name] = cvx.vars.q_d[joint_index] + start_state.getJointState(joint_name)->getVariableValues()[0];
    //ROS_INFO("Updated joint [%zd] [%s]: %.3f + %.3f", joint_index, joint_name.c_str(), start_state.getJointState(joint_name)->getVariableValues()[0], cvx.vars.q_d[joint_index]);
  }
  Eigen::VectorXd cartesian_deltas = ee_jacobian*joint_deltas;

  //ROS_INFO("Raw Error: translate [%.3f, %.3f, %.3f]",
  //         x_error(0), x_error(1), x_error(2));
  ROS_DEBUG("ClipError: translate [%.3f, %.3f, %.3f]  euler [%.2f, %.2f, %.2f]",
           delta_x(0), delta_x(1), delta_x(2),
           delta_euler[0], delta_euler[1], delta_euler[2]);
  ROS_DEBUG("Output:    translate [%.3f, %.3f, %.3f]  euler [%.2f, %.2f, %.2f]",
           cartesian_deltas(0), cartesian_deltas(1), cartesian_deltas(2),
           cartesian_deltas(3), cartesian_deltas(4), cartesian_deltas(5));


  //goal_update[vars.qdd_c[index] + ]
  constrained_goal_state.setStateValues(goal_update);


// ======== Step toward constrained goal, checking for new collisions along the way ========

  //double proxy_goal_tolerance = 0.1;
  // TODO magic number
  double interpolation_step = 0.333;
  double interpolation_progress = interpolation_step;
  collision_detection::CollisionResult collision_result;
  while(interpolation_progress <= 1.0)
  {
    //ROS_INFO("Doing interpolation, with progress %.2f ", interpolation_progress);

    // TODO this interpolation scheme might not allow the arm to slide along contacts very well...
    kinematic_state::KinematicStatePtr point;
    point.reset(new kinematic_state::KinematicState(constrained_goal_state));
    start_state.interpolate(constrained_goal_state, interpolation_progress, *point);

    // get contact set
    collision_detection::CollisionRequest collision_request;
    // TODO magic number
    collision_request.max_contacts = 20;
    collision_request.contacts = true;
    collision_request.distance = false;
    collision_request.verbose = false;

    collision_result.clear();
    a_planning_scene->checkCollision(collision_request, collision_result, *point);

    if(collision_result.collision)
    {
      const collision_detection::CollisionWorld::ObjectConstPtr& octomap_object = a_planning_scene->getCollisionWorld()->getObject(planning_scene::PlanningScene::OCTOMAP_NS);
      collision_detection::refineContactNormals(octomap_object, collision_result, false);
      // TODO this is a hack way of passing this information.
      if(req.motion_plan_request.num_planning_attempts > 1)
      {
        ROS_INFO("Saving the proxy point in collision because we are just doing velocity constraint.");
        proxy_state = *point;
      }
      break;
    }
    else
    {
      proxy_state = *point;
    }
    // TODO Use proxy_goal_tolerance to exit if we are close enough!
    interpolation_progress += interpolation_step;
  }
  // Store the last collision result!
  last_collision_result = collision_result;

// ======== Convert proxy state to a "trajectory" ========
  //ROS_INFO("Converting proxy to a trajectory, and returning...");

  moveit_msgs::RobotTrajectory rt;
  trajectory_msgs::JointTrajectory traj;
  trajectory_msgs::JointTrajectoryPoint pt;
  sensor_msgs::JointState js;

  kinematic_state::kinematicStateToJointState(proxy_state, js);

  // getJointNames
  for(size_t i = 0 ; i < js.name.size(); i++)
  {
    std::string name = js.name[i];
    if( !jmg->hasJointModel(name) ) continue;

    traj.joint_names.push_back(js.name[i]);
    pt.positions.push_back(js.position[i]);
    if(js.velocity.size()) pt.velocities.push_back(js.velocity[i]);
  }
  pt.time_from_start = ros::Duration(req.motion_plan_request.allowed_planning_time*1.0);
  traj.points.push_back(pt);

  traj.header.stamp = ros::Time::now();
  traj.header.frame_id = "odom_combined";

  res.trajectory.joint_trajectory = traj;

  return true;
}

} // namespace cat_planners

PLUGINLIB_DECLARE_CLASS(cat_planners, ConvexConstraintSolver,
                        cat_planners::ConvexConstraintSolver,
                        planning_interface::Planner);

