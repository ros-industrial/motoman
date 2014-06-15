#ifndef ROBOT_GROUP_H
#define ROBOT_GROUP_H

#include <vector>
#include <string>

class RobotGroup{

public:
    RobotGroup() {};

    std::vector<std::string> get_joint_names()
    {
      return this->joint_names_;
    }

    int get_group_id()
    {
      return this->GroupID_;
    }

    void set_group_id(int gid)
    {
        this->GroupID_ = gid;
    }


    void set_joint_names(std::vector<std::string> jnames)
    {
        this->joint_names_ = jnames;
    }

protected:

    std::vector<std::string> joint_names_;
    int GroupID_;

};

#endif // ROBOT_GROUP_H
