// In ResetListenerSystem.hh
#include <gz/math/Pose3.hh>
#include <gz/sim/System.hh>

namespace reset_listener_system {
class ResetListenerSystem
    : public gz::sim::System,
      public gz::sim::ISystemConfigure, // Add this interface
      public gz::sim::ISystemReset {
public:
  ResetListenerSystem();

public:
  ~ResetListenerSystem();

  // Add Configure method
public:
  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_eventMgr) override;

public:
  void Reset(const gz::sim::UpdateInfo &_info,
             gz::sim::EntityComponentManager &_ecm) override;

  // Add storage for initial states
private:
};
} // namespace reset_listener_system
