#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "remove_collision_object");
  ros::NodeHandle nh;

  ros::AsyncSpinner spin(1);
  spin.start();

  // 객체 생성
  moveit::planning_interface::PlanningSceneInterface current_scene;

  sleep(5.0);

  // 충돌 객체 ID를 포함하는 문자열의 벡터를 만들기, 충돌 객체 ID : seven_dof_arm_cylinder
  // 문자열 푸시 후 removeCollisionObjects호출 후 플래닝 씬에서 충돌 객체 제거
  std::vector<std::string> object_ids;
  object_ids.push_back("seven_dof_arm_cylinder");
  current_scene.removeCollisionObjects(object_ids);

  ros::shutdown();

  return 0;
}