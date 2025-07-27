#ifndef IGNITION_GAZEBO_RADARAYS_EMBREE_PLUGIN_H
#define IGNITION_GAZEBO_RADARAYS_EMBREE_PLUGIN_H

#include <rclcpp/rclcpp.hpp>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/EntityComponentManager.hh>

#include <rmagine/math/types.h>
#include <rmagine/types/Memory.hpp>
#include <rmagine/types/sensor_models.h>
#include <rmagine/simulation/SphereSimulatorEmbree.hpp>
#include <rmagine/simulation/OnDnSimulatorEmbree.hpp>
#include <rmagine/noise/Noise.hpp>

#include <mutex>
#include <shared_mutex>
#include <memory>
#include <unordered_map>

#include <radarays_ros/Radar.hpp>
#include <radarays_ros/RadarMaterials.h>
#include <radarays_ros/RadarParams.h>
#include <radarays_ros/radar_types.h>

namespace rm = rmagine;

namespace ignition::gazebo
{

class RadaRaysEmbree : public ignition::gazebo::System,
                       public ISystemConfigure,
                       public ISystemUpdate
{
public:
    RadaRaysEmbree();
    virtual ~RadaRaysEmbree();

    void Configure(const ignition::gazebo::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   ignition::gazebo::EntityComponentManager &_ecm,
                   ignition::gazebo::EventManager &eventMgr) override;

    void Update(const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;

protected:
    void loadParamsSDF();
    void loadParams();
    void searchForMaterials();
    void simulate(rmagine::Transform Tsm);

    // Radar Model Updates
    void updateRadarModel();

    // ROS 2 Node
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr param_sub_;

    std::unordered_map<unsigned int, rm::OnDnSimulatorEmbreePtr> m_sims;

    // Radar Parameters
    radarays_ros::RadarParams m_params;
    rm::SphericalModel m_radar_model;
    radarays_ros::RadarParams m_cfg;

    // Material Properties
    int m_material_id_air = 0;
    std::vector<int> m_object_materials;
    bool m_materials_searched = false;

    float m_wave_energy_threshold = 0.001;
    std::vector<radarays_ros::DirectedWave> m_waves_start;
    bool m_resample = true;
    float m_max_signal = 120.0;

    // Entity Management
    ignition::gazebo::Entity sensorEntity;
    ignition::gazebo::EntityComponentManager *entityComponentManager_;
};

} // namespace ignition::gazebo

#endif // IGNITION_GAZEBO_RADARAYS_EMBREE_PLUGIN_H
