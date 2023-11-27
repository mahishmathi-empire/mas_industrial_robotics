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

#include "mir_interfaces/msg/object_list.hpp"
#include "mir_interfaces/msg/object.hpp"
#include "mir_interfaces/msg/Workstation.hpp"

class WorldModel
{
public:
  WorldModel();

  virtual ~WorldModel();

private:
  // ============================ Members ============================
  std::map<std::string, Workstation> workstations_;

public:
  typedef std::vector<mir_interfaces::msg::Object> ObjectVector;
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
   * @param object_list list of objects including workstation_name, pose, id and name
  */
  void addObjectToWorkstation(
    const mir_interfaces::msg::ObjectList::SharedPtr object_list);

  /**
   * @brief removes an object from a workstation
   * 
   * @param workstation_name name of the workstation
   * @param object_name name of the object
  */
  void removeObjectFromWorkstation(
    const std::string &workstation_name,
    const std::string &object_name);

  /**
   * @brief gets all objects on a workstation
   * 
   * @param workstation_name name of the workstation
  */
  std::vector<mir_interfaces::msg::WorldModelObject> getWorkstationObjects(
    const std::string &workstation_name);

  /**
   * @brief gets all workstations
  */
  std::vector<mir_interfaces::msg::Workstation> getAllWorkstations();

  /**
   * @brief gets the height of a workstation
   * 
   * @param workstation_name name of the workstation
  */
  int getWorkstationHeight(
    const std::string &workstation_name);

  /**
   * @brief removes duplicate objects from two lists based on their distance
   * if distance is smaller than 0.02m, the object from the secondary list is removed
   * 
   * @param ojectlist_primary primary list of objects
   * @param ojectlist_secondary secondary list of objects
   */
  ObjectVector WorldModel::filterDuplicatesByDistance(
  const ObjectVector &objectlist_primary,
  const ObjectVector &objectlist_secondary);
};

#endif // WORLD_MODEL_HPP