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
 * - addAtworkObject - adds an object to the world model
 * - removeAtworkObject - removes an object from the world model
 * - addAtworkObjectToWorkstation - adds an object to a workstation
 * - removeAtworkObjectFromWorkstation - removes an object from a workstation
 * - getWorkstation - returns a workstation from the world model
 * - getAtworkObject - returns an object from the world model
 * - getWorkstations - returns all workstations from the world model
 * - getAtworkObjects - returns all objects from the world model
 * - getWorkstationObjects - returns all objects from a workstation
 * - getWorkstationObject - returns an object from a workstation
 * - getWorkstationType - returns the type of a workstation
 * - getWorkstationHeight - returns the height of a workstation
 * - getWorkstationName - returns the name of a workstation
 * - getAtworkObjectName - returns the name of an object
 * - getAtworkObjectPose - returns the pose of an object
 * - getAtworkObjectAddedTime - returns the added time of an object
 * - getObjectWorkstation - returns the workstation an object is on
*/

