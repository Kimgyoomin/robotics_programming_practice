// ROS 노드에서 충돌 정보를 얻으려면 다음과 같은 코딩이 필요하다
// 자가 충돌 및 환경 충돌을 확인하고 어떤 링크가 충돌했는지 알 수 있다.
#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/kinematic_constraints/utils.h>
#include <eigen_conversions/eigen_msg.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arm_kinematics");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // 로봇의 기구 모델을 플래닝 씬에 로드한다.
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);


  // 로봇의 현재 상태에서 자가 충돌 테스트를 위해 다음 두 가지 인스턴스를 만들 수 있다.
  // collision_detection::CollisionRequest, collision_detection::CollisionResult
  // 이 객체 생성 후 MoveIt! 충돌 확인 함수인 planning_scene.checkSelfCollision()에 전달
  // 이는 충돌 결과를 Colllision_result 객체에 제공
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  planning_scene.checkSelfCollision(collision_request, collision_result);
  ROS_INFO_STREAM("1. Self collision Test: "<< (collision_result.collision ? "in" : "not in")
                  << "self collision");
  
  // Change the current state of the Robot
  robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();
  current_state.setToRandomPositions();
  collision_result.clear();

  // 충돌 결과를 Collision_result 객체에 제공한다. 다음처럼 세부 정보를 출력할 수 있다.
  planning_scene.checkCollision(collision_request, collision_result);
  ROS_INFO_STREAM("2. Self collision Test(Change the state): " << (collision_result.collision ? "in" : "not in"));

  // 특정 그룹에서 충돌을 테스트 하기 위해선, 다음과 같이 group_name을 언급
  collision_request.group_name = "arm";
  current_state.setToRandomPositions();
  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request, collision_result);

  ROS_INFO_STREAM("3. Self collision Test(In a group): " << (collision_result.collision ? "in" : "not in"));

  // Getting contact Info
  // manually set the arm to a position where we know
  // internal (self) collisions happen. Note that this state is now
  // actually outside the joint limits, which we can also check for directly
  std::vector<double> joint_values;
  const robot_model::JointModelGroup* joint_model_group =
    current_state.getJointModelGroup("arm");
  current_state.copyJointGroupPositions(joint_model_group, joint_values);
  joint_values[0] = 1.57; // hard-coded since we know collisions will happen here
  current_state.setJointGroupPositions(joint_model_group, joint_values);
  ROS_INFO_STREAM("4. collision points"
                  << (current_state.satisfiesBounds(joint_model_group) ? "valid" : "not valid"));

  collision_request.contacts = true;
  collision_request.max_contacts = 1000;

  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request, collision_result);

  ROS_INFO_STREAM("5. Self collision Test: " << (collision_result.collision ? "in" : "not in")
                  << "self collision");

  collision_detection::CollisionResult::ContactMap::const_iterator it;
  for(it = collision_result.contacts.begin();
      it != collision_result.contacts.end();
      ++it)
  {
    ROS_INFO("6. Contact between: %s and %s",
              it->first.first.c_str(),
              it->first.second.c_str());
  }  
  
  // 전체 충돌 검사를 수행하려면 planning_scene.checkCollision()이라는 함수 사용
  // 이 함수에서 현재 로봇 상태와 ACM 매트릭스를 언급해야함

  collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
  robot_state::RobotState copied_state = planning_scene.getCurrentState();

  collision_detection::CollisionResult::ContactMap::const_iterator it2;
  for(it2 = collision_result.contacts.begin();
      it2 != collision_result.contacts.end();
      ++it2)
  {
    acm.setEntry(it2->first.first, it2->first.second, true);
  }
  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request, collision_result, copied_state, acm);

  ROS_INFO_STREAM("6. Self collision Test after modified ACM: "<< (collision_result.collision ? "in" : "not in")
                  << "self collision");

  ros::shutdown();
  return 0;

}

