#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <unistd.h>

// 현재 쿼터니안으로 나타나는 coordinate이기에 
// cartesian에서 quaternion으로 변환하는 것을 넣을 것
#include <tf/transform_datatypes.h> //  Quaternion 변환을 위함
int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "add_collision_object");
  ros::NodeHandle nh;

  ros::AsyncSpinner spin(1);
  spin.start();

  // moveit::planning_interface::PlanningSceneInterface 객체 생성
  // Moveit 플래닝씬에 접근, 어떤 작업도 생성 가능
  // PlanningSceneInterface 객체 인스턴스 생성을 기다리기 위해 5초 sleep
  moveit::planning_interface::PlanningSceneInterface current_scene;
  sleep(5.0);

  // moveit_msgs::CollisionObject 충돌 객체 메시지 생성
  // 현재 플래닝 중인 씬으로 전송
  // 객체의 이름은 객체의 id
  moveit_msgs::CollisionObject cylinder;
  cylinder.id = "seven_dof_arm_cylinder";

  // 충돌 객체 메시지를 만든 후에는, 기본모양의 종류, 속성을 정의하는
  // shape_msgs::SolidPrimitive 타입의 메시지 생성
  shape_msgs::SolidPrimitive primitive;

  primitive.type = primitive.CYLINDER;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.6;
  primitive.dimensions[1] = 0.2;
  primitive.dimensions[2] = 0.2;

  // 충돌 객체 Pose 정의
  // shape 메시지 생성 후 이 객체의 포즈 정의를 위해 geometry_msgs::Pose 메시지 생성
  // 플래닝 씬을 보면서 수정해야하는 부분
  geometry_msgs::Pose pose;
  // 바로 Quaternion으로 지정하기에는 바로 알 수 있는 값이 아니여서 익숙한
  // Euler Angle을 사용하기로 함
  // pose.orientation.w = 1.0;
  // pose.orientation.x = 0.0;
  // pose.orientation.y = -0.4;
  // pose.orientation.z = 0.4;

  // 원하는 RPY값 설정
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;

  // RPY를 쿼터니언으로 변환
  tf::Quaternion q;
  q.setRPY(roll, pitch, yaw);

  // 변환된 쿼터니언을 pose.orientation에서 설정
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();

  // 위의 orientation은 회전과 관련한 위치
  // position을 통해 실린더의 직교좌표계의 위치를 지정
  pose.position.x = 1.0;
  pose.position.y = 0.0;
  pose.position.z = 10.0;

  // 기본객체, 포즈를 실린더 충돌 객체에 추가
  // 플래닝 씬을 추가
  cylinder.primitives.push_back(primitive);
  cylinder.primitive_poses.push_back(pose);
  cylinder.operation = cylinder.ADD;
  cylinder.header.frame_id = "base_link";

  // 벡터에 충돌 객체 삽입
  // moveit_msgs::CollisionObject타입의 collision_objects라는 벡터 생성
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(cylinder);

  // 충돌 객체의 벡터를 현재 플래닝 씬에 추가
  // PlanningSceneInterface 클래스 내, addCollisionObjects()는 
  // 플래닝에 객체를 추가하는 데 사용
  current_scene.addCollisionObjects(collision_objects);
  sleep(2.0);

  ros::shutdown();

  return 0;


}