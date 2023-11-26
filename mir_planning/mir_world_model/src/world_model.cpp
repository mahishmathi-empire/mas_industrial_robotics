/**
 * @brief A class that represents the world model of the atwork arena including the robot and the objects.
 * 
 */

#include <mir_world_model/world_model.hpp>

WorldModel::WorldModel()
{
  std::cout << "WorldModel created" << std::endl;
}

WorldModel::~WorldModel()
{
}

// ============================ Methods ============================
// -------------------------  Workstations ----------------------------
void WorldModel::addWorkstation(std::string name, std::string type, double height)
{
  WorkstationType workstation_type = workstation_type_map[type];

  Workstation workstation;
  workstation.name = name;
  workstation.type = workstation_type;
  workstation.height = height;
  workstations_.insert(std::pair<std::string, Workstation>(name, workstation));
}

void WorldModel::getWorkstation(std::string name, Workstation& workstation)
{
  workstation = workstations_[name];
}

void WorldModel::getWorkstations(std::vector<Workstation>& workstations)
{
  for (auto& workstation : workstations_) {
    workstations.push_back(workstation.second);
  }
}