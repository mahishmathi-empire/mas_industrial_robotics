/**
 * @brief A class that represents the world model of the atwork arena including
 * the robot and the objects.
 */

#ifndef WORLD_MODEL_HPP
#define WORLD_MODEL_HPP

#include <ctime>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>

// workstation type enum
enum WorkstationType
{
  NORMAL,
  SHELF,
  CAVITY,
  ROTATING_TABLE
};

// workstation data structure
struct Workstation
{
  std::string name;
  WorkstationType type;
  double height;
  std::vector<int> object_ids;
};

// object data structure
struct AtworkObject
{
  std::string name;
  int id;
  geometry_msgs::msg::PoseStamped pose;
  std::string workstation_name;
  std::time_t added_time;
};

// map from string to workstation type
static std::map<std::string, WorkstationType> workstation_type_map = {
  { "normal", WorkstationType::NORMAL },
  { "shelf", WorkstationType::SHELF },
  { "cavity", WorkstationType::CAVITY },
  { "rotating_table", WorkstationType::ROTATING_TABLE }
};

class WorldModel
{
public:
  WorldModel();

  virtual ~WorldModel();

private:
  // ============================ Members ============================
  std::map<std::string, Workstation> workstations_;
  std::map<int, AtworkObject> atwork_objects_;

public:
  // ============================ Methods ============================

  /**
   * @brief Adds a workstation to the world model.
   *
   * @param name Name of the workstation.
   * @param type Type of the workstation.
   * @param height Height of the workstation.
   */
  void addWorkstation(
    const std::string &name, 
    const std::string &type, 
    const double &height);

  /**
   * @brief Removes a workstation from the world model.
   * 
   * @param name Name of the workstation.
   */
  void removeWorkstation(
    const std::string &name);

  /**
   * @brief adds an object to a workstation
   * 
   * @param workstation_name name of the workstation
   * @param object_name name of the object
   * @param object_id id of the object
   * @param object_pose pose of the object
  */
  void addObjectToWorkstation(
    const std::string &workstation_name,
    const std::string &object_name,
    const int &object_id,
    const geometry_msgs::msg::PoseStamped &object_pose);

  /**
   * @brief removes an object from a workstation
   * 
   * @param workstation_name name of the workstation
   * @param object_id id of the object
  */
  void removeObjectFromWorkstation(
    const std::string &workstation_name,
    const int &object_id);
  
  /**
   * @brief gets IDs of all objects on a workstation
   * 
   * @param workstation_name name of the workstation
  */
  std::vector<int> getWorkstationObjectIds(
    const std::string &workstation_name);

  /**
   * @brief gets all objects on a workstation
   * 
   * @param workstation_name name of the workstation
  */
  std::vector<std::string> getWorkstationObjects(
    const std::string &workstation_name);

  /**
   * @brief gets all workstation objects
  */
  std::vector<Workstation> getAllWorkstations();

  /**
   * @brief gets the height of a workstation
   * 
   * @param workstation_name name of the workstation
  */
  int getWorkstationHeight(
    const std::string &workstation_name);
};

#endif // WORLD_MODEL_HPP