/***********************************************************************
Copyright 2019 Wuhan PS-Micro Technology Co., Itd.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
***********************************************************************/

#include "probot_grasping/grasping_demo.h"

GraspingDemo::GraspingDemo(ros::NodeHandle n_, float pregrasp_x, float pregrasp_y, float pregrasp_z, float length, float breadth,int color) :
    it_(n_), 
    armgroup("manipulator"), 
    grippergroup("gripper"), 
    vMng_(length, breadth)
{
  this->nh_ = n_;

 //获取base_link和camera_link之间的关系，也就是手眼标定的结果
  try
  {
    this->tf_camera_to_robot.waitForTransform("/base_link", "/camera_link", ros::Time(0), ros::Duration(50.0));
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("[adventure_tf]: (wait) %s", ex.what());
    ros::Duration(1.0).sleep();
  }

//如果查询得到的话，就将结果保存到camera_to_robot_，保存x,y,z和四元数一共7个值
  try
  {
    this->tf_camera_to_robot.lookupTransform("/base_link", "/camera_link", ros::Time(0), (this->camera_to_robot_));
  }

  catch (tf::TransformException &ex)
  {
    ROS_ERROR("[adventure_tf]: (lookup) %s", ex.what());
  }

  grasp_running = false;
  
  this->pregrasp_x = pregrasp_x;
  this->pregrasp_y = pregrasp_y;
  this->pregrasp_z = pregrasp_z;


  //让机械臂运动到初始的位置
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::WallDuration(3.0).sleep();
  //调用该函数控制机械臂运动到设定的位置
  ROS_INFO_STREAM("Getting into the Grasping Position....");
  attainPosition(pregrasp_x, pregrasp_y, pregrasp_z);

  // Subscribe to input video feed and publish object location
   //订阅图像话题，一旦收到图像信息，就会进入到callback当中(依据需要识别的颜色进行划分)
  if(color == 1) {
    image_sub_ = it_.subscribe("/probot_anno/camera/image_raw", 1, &GraspingDemo::imageCb_Green, this);
  }
  else if (color == 2)
  {
    image_sub_ = it_.subscribe("/probot_anno/camera/image_raw", 1, &GraspingDemo::imageCb_Blue, this);
  }
  else
  {
    image_sub_ = it_.subscribe("/probot_anno/camera/image_raw", 1, &GraspingDemo::imageCb_Red, this);
  }
}

void GraspingDemo::imageCb_Green(const sensor_msgs::ImageConstPtr &msg)
{
  if (!grasp_running)
  {
    ROS_INFO_STREAM("Processing the Image to locate the Object...");
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    //调用vision_manager中的函数获取目标的位置，位置坐标是以摄像头中心点的位置作为0坐标
    float obj_x, obj_y;
    vMng_.get2DLocation_Green(cv_ptr->image, obj_x, obj_y);

    // Temporary Debugging
    std::cout<< " X-Co-ordinate in Camera Frame :" << obj_x << std::endl;
    std::cout<< " Y-Co-ordinate in Camera Frame :" << obj_y << std::endl;

    //通过坐标变换，将二维坐标变换为相机坐标系下的三维坐标，在本程序中与URDF建模有关系
    obj_camera_frame.setZ(-obj_y);
    obj_camera_frame.setY(-obj_x);
    obj_camera_frame.setX(0.45);

    //将相机坐标系下的位置转化为base_link坐标系下的坐标
    obj_robot_frame = camera_to_robot_ * obj_camera_frame;
    grasp_running = true;

    // Temporary Debugging
    std::cout<< " X-Co-ordinate in Robot Frame :" << obj_robot_frame.getX() << std::endl;
    std::cout<< " Y-Co-ordinate in Robot Frame :" << obj_robot_frame.getY() << std::endl;
    std::cout<< " Z-Co-ordinate in Robot Frame :" << obj_robot_frame.getZ() << std::endl;
  }
}

void GraspingDemo::imageCb_Blue(const sensor_msgs::ImageConstPtr &msg)
{
  if (!grasp_running)
  {
    ROS_INFO_STREAM("Processing the Image to locate the Object...");
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // ROS_INFO("Image Message Received");
    float obj_x, obj_y;
    vMng_.get2DLocation_Blue(cv_ptr->image, obj_x, obj_y);

    // Temporary Debugging
    std::cout<< " X-Co-ordinate in Camera Frame :" << obj_x << std::endl;
    std::cout<< " Y-Co-ordinate in Camera Frame :" << obj_y << std::endl;

    obj_camera_frame.setZ(-obj_y);
    obj_camera_frame.setY(-obj_x);
    obj_camera_frame.setX(0.45);

    obj_robot_frame = camera_to_robot_ * obj_camera_frame;
    grasp_running = true;

    // Temporary Debugging
    std::cout<< " X-Co-ordinate in Robot Frame :" << obj_robot_frame.getX() << std::endl;
    std::cout<< " Y-Co-ordinate in Robot Frame :" << obj_robot_frame.getY() << std::endl;
    std::cout<< " Z-Co-ordinate in Robot Frame :" << obj_robot_frame.getZ() << std::endl;
  }
}

void GraspingDemo::imageCb_Red(const sensor_msgs::ImageConstPtr &msg)
{
  if (!grasp_running)
  {
    ROS_INFO_STREAM("Processing the Image to locate the Object...");
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // ROS_INFO("Image Message Received");
    float obj_x, obj_y;
    vMng_.get2DLocation_Red(cv_ptr->image, obj_x, obj_y);

    // Temporary Debugging
    std::cout<< " X-Co-ordinate in Camera Frame :" << obj_x << std::endl;
    std::cout<< " Y-Co-ordinate in Camera Frame :" << obj_y << std::endl;

    obj_camera_frame.setZ(-obj_y);
    obj_camera_frame.setY(-obj_x);
    obj_camera_frame.setX(0.45);

    obj_robot_frame = camera_to_robot_ * obj_camera_frame;
    grasp_running = true;

    // Temporary Debugging
    std::cout<< " X-Co-ordinate in Robot Frame :" << obj_robot_frame.getX() << std::endl;
    std::cout<< " Y-Co-ordinate in Robot Frame :" << obj_robot_frame.getY() << std::endl;
    std::cout<< " Z-Co-ordinate in Robot Frame :" << obj_robot_frame.getZ() << std::endl;
  }
}

void GraspingDemo::attainPosition(float x, float y, float z)
{
  // ROS_INFO("The attain position function called");

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link_2");
  visual_tools.deleteAllMarkers();

  // For getting the pose
  geometry_msgs::PoseStamped currPose = armgroup.getCurrentPose();

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation = currPose.pose.orientation;

  // Starting Postion before picking
  target_pose1.position.x = x;
  target_pose1.position.y = y;
  target_pose1.position.z = z;
  armgroup.setPoseTarget(target_pose1);

  /* Uncomment Following section to visualize in rviz */
  // We can print the name of the reference frame for this robot.
  // ROS_INFO("Reference frame: %s", armgroup.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  // ROS_INFO("Reference frame: %s", armgroup.getEndEffectorLink().c_str());

  // ROS_INFO("Group names: %s",  armgroup.getName().c_str());

  /*ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

   const robot_state::JointModelGroup *joint_model_group =
  armgroup.getCurrentState()->getJointModelGroup("arm");

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  // visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("next step");*/

  armgroup.move();
}

void GraspingDemo::attainObject( int color)
{
  // ROS_INFO("The attain Object function called");
  attainPosition(obj_robot_frame.getX(), obj_robot_frame.getY(), obj_robot_frame.getZ() + 0.05);

  // Open Gripper
  ros::WallDuration(1.0).sleep();
  grippergroup.setNamedTarget("open");
  grippergroup.move();

  // Slide down the Object
  geometry_msgs::PoseStamped currPose = armgroup.getCurrentPose();
  geometry_msgs::Pose target_pose1;
  if(color == 1 || color == 0){
     target_pose1.orientation = currPose.pose.orientation;
     target_pose1.position = currPose.pose.position;
  }
else
 {
    //  target_pose1.orientation.x = currPose.pose.orientation.y;
    //  target_pose1.orientation.y = currPose.pose.orientation.x;
    //  target_pose1.orientation.z = currPose.pose.orientation.w;
    //  target_pose1.orientation.w = currPose.pose.orientation.z;
     target_pose1.orientation = currPose.pose.orientation;
     target_pose1.position = currPose.pose.position;
 }

  target_pose1.position.z = obj_robot_frame.getZ() - 0.05;
  armgroup.setPoseTarget(target_pose1);
  armgroup.move();
}

void GraspingDemo::grasp()
{
  // ROS_INFO("The Grasping function called");

  ros::WallDuration(1.0).sleep();
  grippergroup.setNamedTarget("close");
  grippergroup.move();
}

void GraspingDemo::lift(int color)
{
  // ROS_INFO("The lift function called");

  // For getting the pose
  geometry_msgs::PoseStamped currPose = armgroup.getCurrentPose();

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation = currPose.pose.orientation;
  target_pose1.position = currPose.pose.position;

  // Starting Postion after picking
  //target_pose1.position.z = target_pose1.position.z + 0.06;


  if(color == 1){
    if(target_pose1.position.y < 0.06){
      target_pose1.position.y = target_pose1.position.y + 0.06;
    }
    else
    {
      target_pose1.position.y = target_pose1.position.y - 0.06;
    }
  }
  else if (color == 2)
  {
    // if(target_pose1.position.x > 0.3 && target_pose1.position.x < 0.4){
    //   target_pose1.position.x = target_pose1.position.x + 0.06;
    // }
    // else
    // {
    //   target_pose1.position.x = target_pose1.position.x - 0.06;
    // }
        if(target_pose1.position.y < 0.06){
      target_pose1.position.y = target_pose1.position.y + 0.06;
    }
    else
    {
      target_pose1.position.y = target_pose1.position.y - 0.06;
    }
  }
  else
  {
      if(target_pose1.position.y > -0.06){
      target_pose1.position.y = target_pose1.position.y - 0.06;
    }
    else
    {
      target_pose1.position.y = target_pose1.position.y + 0.06;
    }
  }

  armgroup.setPoseTarget(target_pose1);
  armgroup.move();

  // Open Gripper
  ros::WallDuration(1.0).sleep();
  grippergroup.setNamedTarget("open");
  grippergroup.move();

  target_pose1.position.z = target_pose1.position.z + 0.1;
  armgroup.setPoseTarget(target_pose1);
  armgroup.move();
}

void GraspingDemo::goHome()
{
  geometry_msgs::PoseStamped currPose = armgroup.getCurrentPose();

  // Go to Home Position
  attainPosition(pregrasp_x, pregrasp_y, pregrasp_z);
  attainPosition(homePose.pose.position.x, homePose.pose.position.y, homePose.pose.position.z);
  
  // Close Gripper
  grippergroup.setNamedTarget("close");
  grippergroup.move();
}

void GraspingDemo::initiateGrasping(int color)
{
  //开启新的线程
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::WallDuration(3.0).sleep();

  //获取当前的位置
  homePose = armgroup.getCurrentPose();
  
  //调用attainObject()函数使机械臂靠近目标
  ROS_INFO_STREAM("Approaching the Object....");
  attainObject(color);

  //夹取物体
  ROS_INFO_STREAM("Attempting to Grasp the Object now..");
  grasp();

  //夹住物体做一个小范围移动
  ROS_INFO_STREAM("Lifting the Object....");
  lift(color);

  //机械臂返回到初始状态
  ROS_INFO_STREAM("Going back to home position....");
  goHome();

  grasp_running = false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_grasping");
  float length, breadth, pregrasp_x, pregrasp_y, pregrasp_z;
  ros::NodeHandle n;

  if (!n.getParam("probot_grasping/table_length", length))
    length = 0.3;
  if (!n.getParam("probot_grasping/table_breadth", breadth))
    breadth = 0.3;
  if (!n.getParam("probot_grasping/pregrasp_x", pregrasp_x))
    pregrasp_x = 0;
  if (!n.getParam("probot_grasping/pregrasp_y", pregrasp_y))
    pregrasp_y = 0;
  if (!n.getParam("probot_grasping/pregrasp_z", pregrasp_z))
    pregrasp_z = 0.28;

  GraspingDemo Grasp_Green(n, pregrasp_x, pregrasp_y, pregrasp_z, length, breadth,1);
  GraspingDemo Grasp_Red(n, pregrasp_x, pregrasp_y, pregrasp_z, length, breadth,0);
  GraspingDemo Grasp_Blue(n, pregrasp_x, pregrasp_y, pregrasp_z, length, breadth,2);
  ROS_INFO_STREAM("Waiting for three seconds..");

  ros::WallDuration(3.0).sleep();
  while (ros::ok())
  {
    // Process image callback
    ros::spinOnce();

    Grasp_Blue.initiateGrasping(2);

    Grasp_Green.initiateGrasping(1);

    Grasp_Red.initiateGrasping(0);
  }
  return 0;
}
