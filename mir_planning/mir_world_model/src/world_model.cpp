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
  const std::string &name, 
  const std::string &type, 
  const double &height)
{
  Workstation workstation;
  workstation.name = name;
  workstation.type = workstation_type_map[type];
  workstation.height = height;
  workstations_[name] = workstation; // add workstation to map
}

void 
WorldModel::removeWorkstation(
  const std::string &name)
{
  workstations_.erase(name);
}

void 
WorldModel::addObjectToWorkstation(
  const mir_interfaces::msg::ObjectList::SharedPtr object_list)
{
  ObjectVector objects_from_rgb;
  ObjectVector objects_from_pcl;

  std::string &workstation_name = object_list->workstation_name;
  ObjectVector workstation_objects = getWorkstationObjects(workstation_name); ;

  for (auto object : object_list->objects)
  {
    // add object to respective source list
    if(object.database_id > 99)
    {
      objects_from_rgb.push_back(object);
    }
    else
    {
      objects_from_pcl.push_back(object);
    }
  }

  if (objects_from_rgb.size() > 0 && objects_from_pcl.size() > 0)
  {
    ObjectVector objects_combined = 
      filterDuplicatesByDistance(&objects_from_rgb, &objects_from_pcl); //this wil also remove the duplicates 
  }
  else if (objects_from_rgb.size() > 0)
  {
    ObjectVector &objects_combined = objects_from_rgb;
  }
  else if (objects_from_pcl.size() > 0)
  {
    ObjectVector &objects_combined = objects_from_pcl;
  }

  // find duplicates between workstation_objects and objects_combined
  ObjectVector objects_final = 
    filterDuplicatesByDistance(&objects_combined, &workstation_objects);

  workstations_[workstation_name].objects = objects_final;

}

void 
WorldModel::removeObjectFromWorkstation(
  const std::string &workstation_name,
  const int &object_id)
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


ObjectVector
WorldModel::getWorkstationObjects(
  const std::string &workstation_name)
{
  return workstations_[workstation_name].objects;
}

std::vector<Workstation> 
WorldModel::getAllWorkstations()
{
  std::vector<Workstation> workstations;
  for (auto const& workstation : workstations_)
  {
    workstations.push_back(workstation);
  }
  return workstations;
}

int 
WorldModel::getWorkstationHeight(
  const std::string &workstation_name)
{
  return workstations_[workstation_name].height;
}

ObjectVector 
WorldModel::filterDuplicatesByDistance(
  const ObjectVector &objectlist_primary,
  const ObjectVector &objectlist_secondary)
{
  ObjectVector objectlist_combined;
  for (auto const& object_primary : objectlist_secondary)
  {
    bool duplicate = false;
    for (auto const& object_secondary : objectlist_primary)
    {
      float distance = sqrt(
        pow(object_primary.pose.position.x - object_secondary.pose.position.x, 2) +
        pow(object_primary.pose.position.y - object_secondary.pose.position.y, 2));

      if (distance < 0.02)
      {
        duplicate = true;
        break;
      }

    }
    if (!duplicate)
    {
      objectlist_combined.push_back(object_secondary);
    }
  }
  // return the objectlist_combined + objectlist_primary
  objectlist_combined.insert(
    objectlist_combined.end(), 
    objectlist_primary.begin(), 
    objectlist_primary.end());

}