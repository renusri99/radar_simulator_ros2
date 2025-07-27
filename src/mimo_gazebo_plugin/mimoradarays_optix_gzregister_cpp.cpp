#include <ignition/gazebo/System.hh>
#include "mimo_radar_optix_plugin.h"

namespace ignition
{
namespace gazebo
{

class RegisterMIMORadarOptixPlugin : public SystemPlugin
{
public:
    /////////////////////////////////////////////
    /// \brief Destructor
    virtual ~RegisterMIMORadarOptixPlugin() {}

    /////////////////////////////////////////////
    /// \brief Called after the plugin has been constructed.
    public: void Load() override
    {
        // Ensure the MIMO Radar Plugin is registered
        RegisterMIMORadarOptix();
    }

    /////////////////////////////////////////////
    /// \brief Called once after Load
    private: void Init() override
    {
    }
};

// Register this plugin with Ignition Gazebo
IGNITION_ADD_PLUGIN(RegisterMIMORadarOptixPlugin, ignition::gazebo::SystemPlugin)

} // namespace gazebo
} // namespace ignition
