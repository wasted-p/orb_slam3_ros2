#include "hexapod_sim_plugins/ResetListenerSystem.hpp"
#include <gz/plugin/Register.hh>
#include <iostream>

namespace reset_listener_system {

ResetListenerSystem::ResetListenerSystem() {
  std::cout
      << "✅ [ResetListenerSystem] Plugin successfully loaded and constructed!"
      << std::endl;
  std::cout << "🔄 [ResetListenerSystem] Ready to listen for reset events..."
            << std::endl;
}

void ResetListenerSystem::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm, gz::sim::EventManager &_eventMgr) {
  std::cout << "⚙️ [ResetListenerSystem] Configured, will block default reset "
               "behavior."
            << std::endl;
}

void ResetListenerSystem::Reset(const gz::sim::UpdateInfo &_info,
                                gz::sim::EntityComponentManager &_ecm) {
  std::cout << "\n🚨 [ResetListenerSystem] RESET EVENT DETECTED!" << std::endl;
  std::cout << "⏰ [ResetListenerSystem] Reset triggered at simulation time: "
            << _info.simTime.count() / 1e9 << " seconds" << std::endl;
  std::cout << "🚫 [ResetListenerSystem] Blocking default reset behavior - no "
               "action taken."
            << std::endl;
}

ResetListenerSystem::~ResetListenerSystem() {
  std::cout << "🛑 [ResetListenerSystem] Plugin destroyed - cleaning up..."
            << std::endl;
}

} // namespace reset_listener_system

GZ_ADD_PLUGIN(reset_listener_system::ResetListenerSystem, gz::sim::System,
              reset_listener_system::ResetListenerSystem::ISystemConfigure,
              reset_listener_system::ResetListenerSystem::ISystemReset)
