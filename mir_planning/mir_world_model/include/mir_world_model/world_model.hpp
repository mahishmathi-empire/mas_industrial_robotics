/**
 * @brief A class that represents the world model of the atwork arena including the robot and the objects.
 * 
 * WorldModel class description
 *  ============================ Objects ============================
 *  - Workstations object - Can be N workstations in the arena - should be dynamic
 *    - Workstation type - can be of type: Normal, Shelf, Cavity, Rotating table
 *    - Workstation name - string: WS01-WSN, RT01-RTN, CV01-CVN, SH01-SHN
 *    - Workstation height - double: 0.0 - 1.0 (meters)
 *    - Objects on the workstation - vector of objects
 *  - Atwork object - Can be N objects in the arena - should be dynamic
 *    - Object name - string
 *    - Object pose - geometry_msgs::Pose
 *    - Object added time - ros::Time
 *  ============================ Methods ============================
 * - addWorkstation - adds a workstation to the world model
 * - removeWorkstation - removes a workstation from the world model
 * - addAtworkObjectToWorkstation - adds an object to a workstation
 * - removeAtworkObjectFromWorkstation - removes an object from a workstation
 * - getWorkstation - returns a workstation from the world model
 * - getWorkstations - returns all workstations from the world model
 * - getWorkstationObjects - returns all objects from a workstation
 * - getWorkstationObject - returns an object from a workstation
 * - getWorkstationType - returns the type of a workstation
 * - getWorkstationHeight - returns the height of a workstation
 * - getAtworkObjectPose - returns the pose of an object
 * - getAtworkObjectAddedTime - returns the added time of an object
 * - getObjectWorkstation - returns the workstation an object is on
*/

#ifndef WORLD_MODEL_HPP
#define WORLD_MODEL_HPP

#include <string>
#include <vector>
#include <map>
#include <ctime>

#include <geometry_msgs/msg/pose_stamped.hpp>


class WorldModel
{
public:
  
  WorldModel();
  
  virtual ~WorldModel();

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

  // ============================ Methods ============================
  // -------------------------  Workstations ----------------------------
  /**
   * @brief Adds a workstation to the world model
   * 
   * @param name - name of the workstation
   * @param type - type of the workstation, as defined by the WorkstationType enum
   * @param height - height of the workstation
   */
  void addWorkstation(std::string name, WorkstationType type, double height);

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
  void addAtworkObjectToWorkstation(std::string workstation_name, std::string object_name, geometry_msgs::msg::PoseStamped object_pose);

  /**
   * @brief Removes an object from a workstation
   * 
   * @param workstation_name - name of the workstation
   * @param object_name - name of the object
   */
  void removeAtworkObjectFromWorkstation(std::string workstation_name, std::string object_name);

  /**
   * @brief Returns all objects from a workstation
   * 
   * @param workstation_name - name of the workstation
   * @param atwork_objects - reference to the vector of objects
   */
  void getWorkstationObjects(std::string workstation_name, std::vector<AtworkObject>& atwork_objects);

  /**
   * @brief Returns an object from a workstation by name
   * 
   * @param workstation_name - name of the workstation
   * @param object_name - name of the object
   * @param atwork_object - reference to the object
   */
  void getWorkstationObject(std::string workstation_name, std::string object_name, AtworkObject& atwork_object);

  /**
   * @brief Returns an object from a workstation by id
   * 
   * @param workstation_name - name of the workstation
   * @param object_id - id of the object
   * @param atwork_object - reference to the object
   */
  void getWorkstationObject(std::string workstation_name, int object_id, AtworkObject& atwork_object);

  /**
   * @brief Returns the pose of an object
   * 
   * @param object_name - name of the object
   * @param object_pose - reference to the pose of the object
   */
  void getAtworkObjectPose(std::string object_name, geometry_msgs::msg::PoseStamped& object_pose);

  /**
   * @brief Returns the added time of an object
   * 
   * @param object_name - name of the object
   * @param added_time - reference to the added time of the object
   */
  void getAtworkObjectAddedTime(std::string object_name, std::time_t& added_time);

  /**
   * @brief Returns the workstation an object is on
   * 
   * @param object_name - name of the object
   * @param workstation_name - reference to the name of the workstation
   */
  void getObjectWorkstation(std::string object_name, std::string& workstation_name);

};


#endif // WORLD_MODEL_HPP