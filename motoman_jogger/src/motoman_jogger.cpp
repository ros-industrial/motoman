#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <motoman_jogger/Deltas.h>
#include <boost/thread/thread.hpp>
#include <motoman_jogger/set_velocity.h>

sensor_msgs::JointState current_joints;
std::map<std::string, int> joint_name_map;
robot_state::RobotStatePtr kinematic_state;
trajectory_msgs::JointTrajectory dummy_traj;
trajectory_msgs::JointTrajectoryPoint point;
const robot_state::JointModelGroup* joint_model_group;
ros::Time start_time, time_last, last_callback;
ros::Publisher streaming_pub;
motoman_jogger::Deltas deltas;
double jogging_velocity;
double max_translation_acc, translation_acc;
double max_rotation_acc, rotation_acc;

void joint_state_cb(sensor_msgs::JointState msg)
{
  current_joints = msg;
}

Eigen::MatrixXd invert(Eigen::MatrixXd J)
{
  return(J.transpose()*(J*J.transpose()).inverse());
}

void cartesian_delta_cb(motoman_jogger::Deltas new_deltas)
{
  last_callback = ros::Time::now();
  //Check valid deltas:
  if(new_deltas.deltas.size() != 6)
  {
    ROS_ERROR("Cartesian jogging deltas must be of size 6. Ignoring message.");
    return;
  }
  deltas = new_deltas; 
}

bool set_vel_cb(motoman_jogger::set_velocity::Request &req, motoman_jogger::set_velocity::Response &res)
{
  if(req.speed < 0 || req.speed > 1.0)
  {
    ROS_WARN("Invalid jogging speed requested. Value must be on interval [0,1].");
    res.result = false;
  }
  else
  {
    jogging_velocity = req.speed;
    translation_acc = req.speed*max_translation_acc;
    rotation_acc = req.speed*max_rotation_acc;
    res.result = true;
  }
  return res.result;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motoman_jogger");
  ros::NodeHandle n;
  
  streaming_pub = n.advertise<trajectory_msgs::JointTrajectory>("joint_command", 10);
  ros::Subscriber cart_delta_sub = n.subscribe<motoman_jogger::Deltas>("cartesian_jogging_deltas", 1, cartesian_delta_cb);
  
  //Set up joint subscriber 
  ros::Subscriber joint_sub = n.subscribe("joint_states", 1, joint_state_cb);
  moveit::planning_interface::MoveGroup arm(argv[1]);
  
  //Set up server to set velocity
  ros::ServiceServer set_vel_server = n.advertiseService("set_jogging_velocity", set_vel_cb);
  
  double dt = 0.1;
  double move_timeout = 2.0;
  jogging_velocity = 1.0;
  
  double cart_translation_vel = .75;
  max_translation_acc = .015;
  translation_acc = max_translation_acc;
  double k_translation = 0.1;
  double b_translation = 0.95;
  
  double cart_rotation_vel = 1.5;
  max_rotation_acc = 0.05;
  rotation_acc = max_rotation_acc;
  double k_rotation = 0.1;
  double b_rotation = 0.95;
  
  double jacobian_condition = 1.0;

  Eigen::VectorXd d_X(6), d_theta(7), cart_acc(6), cart_vel(6);
  Eigen::VectorXd vel_err(6), err_dot(6);
  Eigen::MatrixXd J(6,7);
  Eigen::Vector3d ref_point(0.0, 0.0, 0.0);
  
  //initialize Cartesian delta vector;
  cart_acc.setZero();
  cart_vel.setZero();
  d_X.setZero();
  

  //Get set up for kinematics:
  robot_model_loader::RobotModelLoader model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = model_loader.getModel();
  ROS_INFO("Model Frame: %s", kinematic_model->getModelFrame().c_str());
  
  kinematic_state = boost::shared_ptr<robot_state::RobotState>(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  joint_model_group = kinematic_model->getJointModelGroup("sia10");
  
  
  const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();
  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states");
 
  //Initialize jogging start.
  dummy_traj.joint_names = current_joints.name;  
  for(unsigned int joint=0; joint<7; joint++)
  {
    point.positions.push_back(current_joints.position.at(joint));
    point.velocities.push_back(0);
  }
  point.time_from_start = ros::Duration(0.0);
  dummy_traj.points.push_back(point);
  streaming_pub.publish(dummy_traj);
     
  start_time = ros::Time::now();
  time_last = ros::Time::now();
  deltas.deltas.resize(6,0);
  
  while(ros::ok)
  {
    if((ros::Time::now() - last_callback).toSec() > .1)
    {
      for(unsigned int i=0; i<6; i++)
      {
        deltas.deltas.at(i) = 0.0;
      }
    }  

    //Time since last point:
    //ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states");
    dt = (ros::Time::now() - time_last).toSec();
    time_last = ros::Time::now();
    kinematic_state->setVariableValues(current_joints); 
    
    //Compute command velocity
    Eigen::VectorXd vel_command(6);
    double sum_squares = 0;
    for(unsigned int i=0; i<3; i++)
    {
      sum_squares = sum_squares + deltas.deltas.at(i)*deltas.deltas.at(i);
    }
    double delta_mag = pow(sum_squares, 0.5);
    for(unsigned int i=0; i<3; i++)
    {
      if(delta_mag > 0.0)
        vel_command[i] = deltas.deltas.at(i)/delta_mag*jogging_velocity*cart_translation_vel;
      else
        vel_command[i] = 0.0;
    }
    
    sum_squares = 0;
    for(unsigned int i=3; i<6; i++)
    {
      sum_squares = sum_squares + deltas.deltas.at(i)*deltas.deltas.at(i);
    }
    delta_mag = pow(sum_squares, 0.5);
    for(unsigned int i=3; i<6; i++)
    {
      if(delta_mag > 0.0)
        vel_command[i] = deltas.deltas.at(i)/delta_mag*jogging_velocity*cart_rotation_vel;
      else
        vel_command[i] = 0.0;
    }
    //std::cout << "vel_command norm: " << vel_command.norm() << std::endl;
    //Compute velocity error.
    err_dot = 1/dt*((vel_command - cart_vel)-vel_err);
    vel_err = vel_command - cart_vel;
   
    double err_mag = vel_err.norm();
    //std::cout << "vel_error norm: " << vel_err.squaredNorm() << std::endl;
    
    //Compute acceleration
    if(err_mag <= 0.015)
    {
      cart_vel = vel_command;
      cart_acc.setZero();
    }
    else
    {
      for(unsigned int i=0; i<3; i++)
      {
        cart_acc[i] = vel_err.normalized()[i]*translation_acc/dt;
      }
      for(unsigned int i=3; i<6; i++)
      {
        cart_acc[i] = vel_err.normalized()[i]*rotation_acc/dt;
      }
      cart_vel = (cart_vel + dt*cart_acc);
    }
   
    if(cart_vel.norm() == 0)
      cart_vel.setZero(); 
    
    d_X = cart_vel*dt; 
    //std::cout << "accel:\n" << cart_acc << std::endl;
    //std::cout << "err_dot:\n" << err_dot << std::endl;
    //std::cout << "vel:\n" << cart_vel << std::endl; 
    //std::cout <<" velocity norm: " << cart_vel.norm() <<std::endl;
    //std::cout << "velocity:\n" << cart_vel << std::endl;
    //std::cout << "d_X:\n" << d_X << std::endl;
    //std::cout << "vel_err.norm: " << vel_err.norm() << std::endl;
    
    //Get the Jacobian
    kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),ref_point, J);
    d_theta = invert(J)*d_X;
    
    //check condition number:
    jacobian_condition = J.norm()*invert(J).norm();
    //std::cout << "Jacobian condition: " << jacobian_condition << std::endl;
    if(jacobian_condition > 39)
    {
      ROS_ERROR("Cannot do Cartesian Jogging due to ill-conditioned Jacobian");
      return 0;
    }
    
    //std::cout << "d_theta:\n" << d_theta << std::endl;
    for(unsigned int j=0; j<7; j++)
    {
      point.positions.at(j) = current_joints.position.at(j) + d_theta[j];
      point.velocities.at(j) = d_theta[j]/dt;
    }
    
    point.time_from_start = ros::Time::now() - start_time;
    dummy_traj.points.at(0) = point;
    streaming_pub.publish(dummy_traj);
    ros::Duration(0.01).sleep();
    
    ros::spinOnce();
  }
  ros::shutdown();
  return 1;
}
