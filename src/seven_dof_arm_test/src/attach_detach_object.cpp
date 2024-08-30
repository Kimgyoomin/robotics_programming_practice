#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <Pose.h>
#include <geometry_msgs/Pose.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "seven_dof_arm_planner");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Initialize group and scene interface
  moveit::planning_interface::MoveGroupInterface group("arm");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::PlanningSceneInterface current_scene;
  // Wait For scene initialization
  sleep(2.0);

  // moveit::planning_interface::PlanningSceneInterface 객체 생성 후
  // moveit_msgs::AttachedCollisionObject 인스턴스 초기화
  // 로봇 몸체의 특정 링크에 연결될 씬 객체에 대한 정보 입력
  moveit_msgs::CollisionObject grasping_object;
  grasping_object.id = "grasping_object";

  // 연결 정보 입력
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.03;
  primitive.dimensions[1] = 0.03;
  primitive.dimensions[2] = 0.08;

  // geometry 정보 입력
  geometry_msgs::Pose pose;
  pose.orientation.w = -1.0;
  pose.position.x = 0.03;
  pose.position.y = 0.0;
  pose.position.z = 0.65;

  grasping_object.primitives.push_back(primitive);
  grasping_object.primitive_poses.push_back(pose);
  grasping_object.operation = grasping_object.ADD;
  grasping_object.header.frame_id = "base_link";
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(grasping_object);

  current_scene.addCollisionObjects(collision_objects);

  sleep(4.0);

  // Attach object to robot
  ROS_INFO("Attaching object grasping_object to robot's body");
  moveit_msgs::AttachedCollisionObject attached_object;
  attached_object.link_name = "grasping_frame";
  attached_object.object = grasping_object;
  current_scene.applyAttachedCollisionObject(attached_object);
  sleep(4.0);

  ROS_INFO("Detaching object grasping_object to robot's body");
  grasping_object.operation = grasping_object.REMOVE;
  attached_object.link_name = "grasping_frame";
  attached_object.object = grasping_object;
  current_scene.applyAttachedCollisionObject(attached_object);

  sleep(1.0);

  ros::shutdown();

  // 객체가 로봇에 성공적으로 부착되면 링크는 녹색에서 보라색으로 바뀌고 로봇 모션과 함께 이동
  // 로봇 본체에서 객체를 분리하려면 다음 코드와 

}