#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64.h>
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "seven_dof_arm_planner");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);

  spinner.start();

  moveit::planning_interface::MoveGroupInterface group("arm");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  string line;

  // scene initialization을 기다림
  sleep(2.0);

  // object into the scene
  moveit::planning_interface::PlanningSceneInterface current_scene;
  geometry_msgs::Pose pose;

  // Add grasping obj to the scene
  // 층돌 객체를 만드는 것
  // 충돌 객체 생성
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.03;
  primitive.dimensions[1] = 0.03;
  primitive.dimensions[2] = 0.08;

  // grasping_object를 만들기
  moveit_msgs::CollisionObject grasping_object;
  grasping_object.id = "grasping_object";
  pose.orientation.w = 1.0;
  pose.position.x = 0.41;
  pose.position.y = 0.0;
  pose.position.z = 0.35;

  grasping_object.primitives.push_back(primitive);
  grasping_object.primitive_poses.push_back(pose);
  grasping_object.operation = grasping_object.ADD;
  grasping_object.header.frame_id = "base_link";

  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.3;
  primitive.dimensions[1] = 0.5;
  primitive.dimensions[2] = 0.32;

  // grasping_talbe 만들기
  moveit_msgs::CollisionObject grasping_table;
  grasping_table.id = "grasping_table";
  pose.position.x = 0.46;
  pose.position.y = 0.0;
  pose.position.z = 0.15;
  grasping_table.primitives.push_back(primitive);
  grasping_table.primitive_poses.push_back(pose);
  grasping_table.operation = grasping_object.ADD;
  grasping_table.header.frame_id = "base_link";

  // 충돌 객체(collision_object)를 플래닝 씬에 추가
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(grasping_object);
  collision_objects.push_back(grasping_table);

  // 플래닝 씬이 적절하게 구성되었기에 다음과 같이 엔드 이펙터를 물체 가까이 가져와
  // 픽업을 위해 작업 공간에서 정의된 하나의 위치로 로봇의 모션을 요청 가능
  current_scene.addCollisionObjects(collision_objects);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  const robot_state::JointModelGroup *joint_model_group =
  group.getCurrentState()->getJointModelGroup("arm");

  // approaching
  geometry_msgs::Pose target_pose;
  target_pose.orientation.x = 0;
  target_pose.orientation.y = 0;
  target_pose.orientation.z = 0;
  target_pose.orientation.w = 1;
  target_pose.position.x = 0.28;
  target_pose.position.y = 0.0;
  target_pose.position.z = 0.35;
  group.setPoseTarget(target_pose);
  group.move();
  sleep(2.0);

  cout << "First motion done!" << endl;

  // Grasping
  target_pose.position.x = 0.34;
  target_pose.position.y = 0.0;
  target_pose.position.z = 0.35;
  group.setPoseTarget(target_pose);
  group.move();

  cout << "attaching!" << endl;

  // 잡는 데에 성공 후 로봇의 엔드 이펙터에 물체를 연결해 작업 공간의
  // 다른 위치에 배치할 수 있다.
  moveit_msgs::AttachedCollisionObject att_coll_object;
  att_coll_object.object.id = "grasping_object";
  att_coll_object.link_name = "gripper_finger_link1";
  att_coll_object.object.operation = moveit_msgs::CollisionObject::ADD;
  planning_scene_interface.applyAttachedCollisionObject(att_coll_object);

  sleep(2.0);

  cout << "second motion! " << endl;
  // move far away from the grasping position
  target_pose.position.x = 0.34;
  target_pose.position.y = 0.0;
  target_pose.position.z = 0.4;
  group.setPoseTarget(target_pose);
  group.move();
  sleep(2.0);

  cout << "Picking motion ! " << endl;

  // Picking
  target_pose.orientation.x = -1;
  target_pose.orientation.y = 0;
  target_pose.orientation.z = 0;
  target_pose.orientation.w = 0;
  target_pose.position.x = 0.34;
  target_pose.position.y = -0.1;
  target_pose.position.z = 0.4;
  group.setPoseTarget(target_pose);
  group.move();

  cout << "Picked !" << endl;

  target_pose.position.x = 0.34;
  target_pose.position.y = -0.1;
  target_pose.position.z = 0.35;
  group.setPoseTarget(target_pose);
  group.move();

  // 로봇의 그리퍼에서 물체를 제거
  att_coll_object.object.operation = moveit_msgs::CollisionObject::REMOVE;
  att_coll_object.link_name = "gripper_finger_link1";
  att_coll_object.object.id = "grasping_object";
  planning_scene_interface.applyAttachedCollisionObject(att_coll_object);

  target_pose.position.x = 0.32;
  target_pose.position.y = -0.1;
  target_pose.position.z = 0.35;
  group.setPoseTarget(target_pose);
  group.move();

  ros::shutdown();


}