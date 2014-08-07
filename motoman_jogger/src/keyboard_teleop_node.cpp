#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <motoman_jogger/Deltas.h>
#include <curses.h>

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "keyboard_teleop");
  ros::NodeHandle n;
  
  ros::Publisher delta_pub = n.advertise<motoman_jogger::Deltas>("cartesian_jogging_deltas", 1);
  motoman_jogger::Deltas deltas;
  ROS_INFO("Keyboard teleop online.");
  
  //system("stty raw");
  
  while(ros::ok)
  {
    //Clear the deltas.
    deltas.deltas.clear();
    deltas.deltas.resize(6,0);
    initscr();
    //nodelay(stdscr, TRUE);
    noecho();
    cbreak();
    
    char command = getch();
    //ros::Duration(0.1).sleep();
    //std::cout << command << std::endl;
    switch(command) 
    {
      case '1':
        deltas.deltas.at(0) = 1.0;
        break;
      case '2':
        deltas.deltas.at(1) = 1.0;
        break; 
      case '3':
        deltas.deltas.at(2) = 1.0;
        break; 
      case '4':
        deltas.deltas.at(3) = 1.0;
        break; 
      case '5':
        deltas.deltas.at(4) = 1.0;
        break; 
      case '6':
        deltas.deltas.at(5) = 1.0;
        break;
      case 'q':
        deltas.deltas.at(0) = -1.0;
        break;
      case 'w':
        deltas.deltas.at(1) = -1.0;
        break; 
      case 'e':
        deltas.deltas.at(2) = -1.0;
        break; 
      case 'r':
        deltas.deltas.at(3) = -1.0;
        break; 
      case 't':
        deltas.deltas.at(4) = -1.0;
        break; 
      case 'y':
        deltas.deltas.at(5) = -1.0;
        break;
      case ERR:
        break;
      default:
        break;  
    }
    delta_pub.publish(deltas);
    ros::spinOnce();
    //system("stty cooked");
  } 
  
  return(0);  
}
