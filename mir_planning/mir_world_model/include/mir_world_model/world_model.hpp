/**
 * @brief A class that represents the world model of the atwork arena including
 * the robot and the objects.
 */

#ifndef WORLD_MODEL_HPP
#define WORLD_MODEL_HPP

#include <ctime>
#include <map>
#include <string>
#include <vector>
#include <iostream>

#include <geometry_msgs/msg/pose_stamped.hpp>

// object data structure
struct AtworkObject
{
  std::string name;
  geometry_msgs::msg::PoseStamped pose;
  std::time_t added_time;
};

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
  std::vector<AtworkObject> atwork_objects;
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
  
public:
  // ============================ Methods ============================

  // -------------------------  Workstations ----------------------------
  /**
   * @brief Adds a workstation to the world model
   *
   * @param name - name of the workstation
   * @param type - type of the workstation, as defined by the WorkstationType
   * enum
   * @param height - height of the workstation
   */
  void addWorkstation(std::string name, WorkstationType type, double height);

  /**
   * @brief Adds a workstation to the world model
   *
   * @param name - name of the workstation
   * @param type - type of the workstation, as string
   * enum
   * @param height - height of the workstation
   */
  void addWorkstation(std::string name, std::string type, double height);

  /**
   * @brief Removes a workstation from the world model
   *
   * @param name - name of the workstation
   */
  void removeWorkstation(std::string name);

  /**
   * @brief Returns a workstation from the world model
   *
   * @param name - name of the workstation
   * @param workstation - reference to the workstation object
   */
  void getWorkstation(std::string name, Workstation& workstation);

  /**
   * @brief Returns all workstations from the world model
   *
   * @param workstations - reference to the vector of workstations
   */
  void getWorkstations(std::vector<Workstation>& workstations);

  /**
   * @brief Returns the type of a workstation
   *
   * @param name - name of the workstation
   * @return WorkstationType - type of the workstation
   */
  WorkstationType getWorkstationType(std::string name);

  /**
   * @brief Returns the height of a workstation
   *
   * @param name - name of the workstation
   * @return double - height of the workstation
   */
  double getWorkstationHeight(std::string name);

  // -------------------------  Atwork objects ----------------------------
  /**
   * @brief Adds an object to a workstation with unique id
   *
   * @param workstation_name - name of the workstation
   * @param object_name - name of the object
   * @param object_pose - pose of the object
   */
  void addAtworkObjectToWorkstation(
    std::string workstation_name,
    std::string object_name,
    geometry_msgs::msg::PoseStamped object_pose);

  /**
   * @brief Removes an object from a workstation
   *
   * @param workstation_name - name of the workstation
   * @param object_name - name of the object
   */
  void removeAtworkObjectFromWorkstation(std::string workstation_name,
                                         std::string object_name);

  /**
   * @brief Returns all objects from a workstation
   *
   * @param workstation_name - name of the workstation
   * @param atwork_objects - reference to the vector of objects
   */
  void getWorkstationObjects(std::string workstation_name,
                             std::vector<AtworkObject>& atwork_objects);

  /**
   * @brief Returns an object from a workstation by name
   *
   * @param workstation_name - name of the workstation
   * @param object_name - name of the object
   * @param atwork_object - reference to the object
   */
  void getWorkstationObject(std::string workstation_name,
                            std::string object_name,
                            AtworkObject& atwork_object);

  /**
   * @brief Returns an object from a workstation by id
   *
   * @param workstation_name - name of the workstation
   * @param object_id - id of the object
   * @param atwork_object - reference to the object
   */
  void getWorkstationObject(std::string workstation_name,
                            int object_id,
                            AtworkObject& atwork_object);

  /**
   * @brief Returns the pose of an object
   *
   * @param object_name - name of the object
   * @param object_pose - reference to the pose of the object
   */
  void getAtworkObjectPose(std::string object_name,
                           geometry_msgs::msg::PoseStamped& object_pose);

  /**
   * @brief Returns the added time of an object
   *
   * @param object_name - name of the object
   * @param added_time - reference to the added time of the object
   */
  void getAtworkObjectAddedTime(std::string object_name,
                                std::time_t& added_time);

  /**
   * @brief Returns the workstation an object is on
   *
   * @param object_name - name of the object
   * @param workstation_name - reference to the name of the workstation
   */
  void getObjectWorkstation(std::string object_name,
                            std::string& workstation_name);
};

#endif // WORLD_MODEL_HPP