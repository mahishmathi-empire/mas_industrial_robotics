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
// -------------------------  Workstations ----------------------------
void
WorldModel::addWorkstation(std::string name, std::string type, double height)
{
  WorkstationType workstation_type = workstation_type_map[type];

  Workstation workstation;
  workstation.name = name;
  workstation.type = workstation_type;
  workstation.height = height;
  workstations_.insert(std::pair<std::string, Workstation>(name, workstation));
}

void
WorldModel::getWorkstation(std::string name, Workstation& workstation)
{
  workstation = workstations_[name];
}

void
WorldModel::getWorkstations(std::vector<Workstation>& workstations)
{
  for (auto& workstation : workstations_) {
    workstations.push_back(workstation.second);
  }
}

void
WorldModel::addAtworkObjectToWorkstation(
  std::string workstation_name,
  std::string object_name,
  int object_id,
  geometry_msgs::msg::PoseStamped object_pose)
{
  // create object
  AtworkObject object;
  object.name = object_name;
  object.id = object_id;
  object.pose = object_pose;
  object.added_time = std::time(nullptr);
  object.workstation_name = workstation_name;

  // add object to list of objects
  atwork_objects_.insert(
    std::pair<std::string, AtworkObject>(std::to_string(object_id), object));

  // link object to workstation
  workstations_[workstation_name].object_ids.push_back(object_id);
}

void
WorldModel::getWorkstationObjects(std::string workstation_name,
                                  std::vector<AtworkObject>& objects)
{
  for (auto& object_id : workstations_[workstation_name].object_ids) {
    objects.push_back(atwork_objects_[std::to_string(object_id)]);
  }
}

void
WorldModel::getWorkstationObject(std::string workstation_name,
                                 std::string object_name,
                                 AtworkObject& object)
{
  for (auto& object_id : workstations_[workstation_name].object_ids) {
    if (atwork_objects_[std::to_string(object_id)].name == object_name) {
      object = atwork_objects_[std::to_string(object_id)];
      return;
    }
  }
}

void
WorldModel::getObjectById(int object_id, AtworkObject& atwork_object)
{
  atwork_object = atwork_objects_[std::to_string(object_id)];
}