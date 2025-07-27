#include <ignition/gazebo/System.hh>
#include <radarays_gazebo_plugins/radarays_embree_gzplugin.hpp>

namespace ignition::gazebo
{
  class RegisterRadaRaysEmbreePlugin : public ignition::gazebo::System,
                                       public ISystemConfigure
  {
  public:
    /////////////////////////////////////////////
    /// \brief Destructor
    virtual ~RegisterRadaRaysEmbreePlugin() = default;

    /////////////////////////////////////////////
    /// \brief Called when the plugin is loaded
    void Configure(const ignition::gazebo::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   ignition::gazebo::EntityComponentManager &_ecm,
                   ignition::gazebo::EventManager &)
    {
        RCLCPP_INFO(rclcpp::get_logger("RegisterRadaRaysEmbreePlugin"),
                    "[RegisterRadaRaysEmbreePlugin] Loading RadaRaysEmbree Plugin...");
        sensors::RegisterRadaRaysEmbree();
    }
  };

  // Register this plugin with Ignition Gazebo
  IGNITION_ADD_PLUGIN(RegisterRadaRaysEmbreePlugin,
                      ignition::gazebo::System,
                      RegisterRadaRaysEmbreePlugin::ISystemConfigure)
} // namespace ignition::gazebo
