/**
 * @brief A class that represents the world model of the atwork arena including
 * the robot and the objects.
 *
 */

#include <mir_world_model/world_model.hpp>

WorldModel::WorldModel()
{
  std::cout << "WorldModel created" << std::endl;
}

WorldModel::~WorldModel() {}

// ============================ Methods ============================
void 
WorldModel::addWorkstation(
  std::string name, 
  std::string type, 
  double height)
{
  Workstation workstation;
  workstation.name = name;
  workstation.type = workstation_type_map[type];
  workstation.height = height;
  workstations_[name] = workstation; // add workstation to map
}

void 
WorldModel::removeWorkstation(
  std::string name)
{
  workstations_.erase(name);
}

void 
WorldModel::addObjectToWorkstation(
  std::string workstation_name,
  std::string object_name,
  int object_id,
  geometry_msgs::msg::PoseStamped object_pose)
{
  AtworkObject atwork_object;
  atwork_object.name = object_name;
  atwork_object.id = object_id;
  atwork_object.pose = object_pose;
  atwork_object.workstation_name = workstation_name;
  atwork_object.added_time = std::time(nullptr); // TODO: check if this is in required format
  workstations_[workstation_name].object_ids.push_back(object_id);
  atwork_objects_[object_id] = atwork_object;
}

void 
WorldModel::removeObjectFromWorkstation(
  std::string workstation_name,
  int object_id)
{ 
  // remove object from workstation
  std::vector<int>::iterator it = std::find(
    workstations_[workstation_name].object_ids.begin(),
    workstations_[workstation_name].object_ids.end(),
    object_id);
  if (it != workstations_[workstation_name].object_ids.end())
  {
    workstations_[workstation_name].object_ids.erase(it);
  }
}

std::vector<int> 
WorldModel::getWorkstationObjectIds(
  std::string workstation_name)
{
  return workstations_[workstation_name].object_ids;
}

std::vector<std::string> 
WorldModel::getWorkstationObjects(
  std::string workstation_name)
{
  std::vector<std::string> objects;
  for (int id : workstations_[workstation_name].object_ids)
  {
    objects.push_back(atwork_objects_[id].name);
  }
  return objects;
}

std::vector<Workstation> 
WorldModel::getAllWorkstations()
{
  std::vector<Workstation> workstations;
  for (auto const& workstation : workstations_)
  {
    workstations.push_back(workstation.second);
  }
  return workstations;
}

int 
WorldModel::getWorkstationHeight(
  std::string workstation_name)
{
  return workstations_[workstation_name].height;
}

