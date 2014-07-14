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

    std::string get_name()
    {
        return this->name_;
    }

    std::string get_ns()
    {
        return this->ns_;
    }

    int get_group_id()
    {
      return this->GroupID_;
    }

    void set_name(std::string name)
    {
        this->name_ = name;
    }

   void set_ns(std::string ns)
    {
        this->ns_ = ns;
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
    std::string name_;
    std::string ns_;

};

#endif // ROBOT_GROUP_H
