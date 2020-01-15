#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cstdlib>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

namespace rvt = rviz_visual_tools;
static const std::string PLANNING_GROUP = "moveGroup";
static const int OFFSET_VARIANCE = 20;

geometry_msgs::Pose target_pose;
double hT_x, hT_y, hT_z;                 // old position x, y
double target_x, target_y, target_z;
bool isPlanning;
bool isReadyToPlan;

/*
moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

// Raw pointers are frequently used to refer to the planning group for improved performance.
const robot_state::JointModelGroup* joint_model_group ;

moveit_visual_tools::MoveItVisualTools visual_tools("world");
Eigen::Isometry3d text_pose;
*/

class MoveGroupControl {
  public:
    moveit::planning_interface::MoveGroupInterface move_group;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const robot_state::JointModelGroup* joint_model_group ;

    moveit_visual_tools::MoveItVisualTools visual_tools;
    Eigen::Isometry3d text_pose;

    MoveGroupControl():
    move_group(PLANNING_GROUP), visual_tools("world")
    {
      setup();
    }
    void setup()
    {
      joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
      visual_tools.deleteAllMarkers();
      visual_tools.loadRemoteControl();
      text_pose = Eigen::Isometry3d::Identity();
      text_pose.translation().z() = 1.25;
      visual_tools.publishText(text_pose, "MoveGroupInterface", rvt::WHITE, rvt::XLARGE);

      visual_tools.trigger();

      ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());

      // We can also print the name of the end-effector link for this group.
      ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

      // We can get a list of all the groups in the robot:
      ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
      std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
                std::ostream_iterator<std::string>(std::cout, ", "));
    }
   
    void displayGoalPose(double _target_x, double _target_y, double _target_z)
    {
      const double SAFE_D = 0.5;
      visual_tools.deleteAllMarkers();
      target_pose = move_group.getCurrentPose().pose;
      target_pose.position.x -= _target_x /1000;   // from mm to m
      target_pose.position.y -=  _target_y /1000;   // from mm to m
      target_pose.position.z += _target_z / 1000 - SAFE_D;
      visual_tools.publishAxisLabeled(target_pose, "targetPose");
      visual_tools.trigger();
    }

    void planToPose(geometry_msgs::Pose targetPose)
    {
      move_group.setPoseTarget(targetPose);

      moveit::planning_interface::MoveGroupInterface::Plan my_plan;

      bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      ROS_INFO_NAMED("Visualizing plan (pose goal) %s", success ? "SUCCESS" : "FAILED");

      /* Sleep to give Rviz time to visualize the plan. */
      if(success) 
      {
        sleep(3.0);
        move_group.execute(my_plan);
        sleep(5.0);
      }
      isPlanning = false;
    }
};

/*
void moveGroupInterfaceSetup()
{
  move_group = moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
  visual_tools = moveit_visual_tools::MoveItVisualTools("world");

  joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();
  text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.25;
  visual_tools.publishText(text_pose, "MoveGroupInterface", rvt::WHITE, rvt::XLARGE);

  visual_tools.trigger();

  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));
}
*/

MoveGroupControl* p_moveGroupCtrl;

/*

void displayGoalPose(MoveGroupControl *mMGC, double _target_x, double _target_y, double _target_z)
{
  const double SAFE_D = 0.1;
  mMGC->visual_tools.deleteAllMarkers();
  target_pose = mMGC->move_group.getCurrentPose().pose;
  target_pose.position.x -= _target_x /1000;   // from mm to m
  target_pose.position.y -=  _target_y /1000;   // from mm to m
  target_pose.position.z += _target_z / 1000 - SAFE_D;
  mMGC->visual_tools.publishAxisLabeled(target_pose, "targetPose");
  mMGC->visual_tools.trigger();
}

void planToPose(MoveGroupControl *mMGC, geometry_msgs::Pose targetPose)
{
  mMGC->move_group.setPoseTarget(targetPose);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (mMGC->move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("Visualizing plan (pose goal) %s", success ? "SUCCESS" : "FAILED");

  if(success) 
  {
    sleep(3.0);
    mMGC->move_group.execute(my_plan);
    sleep(5.0);
  }
  isPlanning = false;
}
*/

#pragma region ========== helpers ===============
class BadConversion : public std::runtime_error {
public:
  BadConversion(std::string const& s)
    : std::runtime_error(s)
    { }
};

double convertToDouble(std::string const& s, bool failIfLeftoverChars = true)
{
  std::istringstream i(s);
  double x;
  char c;
  if (!(i >> x) || (failIfLeftoverChars && i.get(c)))
    throw BadConversion("convertToDouble(\"" + s + "\")");
  return x;
}
#pragma endregion ========== helpers ===============

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
  std::string s = msg->data.c_str();
  std::vector<std::string> vs;
  boost::split(vs, s, [](char c){return c == ',';});
  std::stringstream ss1,ss2,ss3;
  ss1<<vs[0];
  ss2<<vs[1];
  ss3<<vs[2];

  target_x = convertToDouble(ss1.str().c_str());
  target_y = convertToDouble(ss2.str().c_str());
  target_z = convertToDouble(ss3.str().c_str());

  if(abs(target_x - hT_x) > OFFSET_VARIANCE || abs(target_y - hT_y) > OFFSET_VARIANCE|| abs(target_z - hT_z) > OFFSET_VARIANCE)
  {
    hT_x = target_x;
    hT_y = target_y;
    hT_z = target_z;

    ROS_INFO("X: [%f], Y: [%f], Z: [%f]", target_x, target_y, target_z);
    p_moveGroupCtrl->displayGoalPose(target_x, target_y, target_z);

    isReadyToPlan = true;
  }
  else
  {
    isReadyToPlan = false;
  }
  
}

int main(int argc, char** argv)
{
  target_x = 0; target_y = 0;
  hT_x = 0; hT_y = 0;
  isPlanning = false;
  isReadyToPlan = false;
  // ============ subscriber =============== 
  ros::init(argc, argv, "nOffset_subscriber");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("tCenter_offset", 1000, chatterCallback);
  ros::Rate loop_rate(10); // update every 1/x s

  // === Move group =====
  ros::init(argc, argv, "staubli_move_group_interface");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  //MoveGroupControl mgc;
  //p_visual_tools = &mgc.visual_tools;

  p_moveGroupCtrl = new MoveGroupControl();

  ROS_INFO("Waiting to detect a red circle", target_x, target_y);
  //p_moveGroupCtrl->visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to shutdown");
  
  while (ros::ok())
  {
    //if(isPlanning)
    {
      //ROS_INFO("Planning in progress...");
    }
    //else
    {
      if(isReadyToPlan)
      {
        p_moveGroupCtrl->visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to make a plan");

        p_moveGroupCtrl->planToPose(target_pose);
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
