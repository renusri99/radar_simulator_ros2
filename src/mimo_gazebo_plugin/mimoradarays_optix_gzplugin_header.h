#ifndef GAZEBO_RADARAYS_MIMO_OPTIX_PLUGIN_H
#define GAZEBO_RADARAYS_MIMO_OPTIX_PLUGIN_H

#include <ros/ros.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/RaySensor.hh>

#include <rmagine/math/types.h>
#include <rmagine/types/Memory.hpp>
#include <rmagine/types/sensor_models.h>
#include <rmagine/simulation/SphereSimulatorOptix.hpp>
#include <rmagine/simulation/OnDnSimulatorOptix.hpp>

#include <rmagine/noise/Noise.hpp>

#include <mutex>
#include <shared_mutex>
#include <memory>
#include <optional>
#include <unordered_map>

#include <rmagine_gazebo_plugins/rmagine_optix_spherical_gzplugin.h>

#include <radarays_ros/MIMORadar.hpp>
#include <radarays_ros/RadarMaterials.h>
#include <radarays_ros/RadarModelConfig.h>
#include <radarays_ros/RadarParams.h>
#include <radarays_ros/radar_types.h>

#include <dynamic_reconfigure/server.h>

namespace rm = rmagine;

namespace gazebo
{

namespace sensors
{
class RadaRaysMIMOOptix : public RmagineOptixSpherical
{
public:
    using Base = RmagineOptixSpherical;

    using ResT = rm::Bundle<
                rm::Hits<rm::VRAM_CUDA>,
                rm::Ranges<rm::VRAM_CUDA>,
                rm::Normals<rm::VRAM_CUDA>,
                rm::ObjectIds<rm::VRAM_CUDA>
            >;

    RadaRaysMIMOOptix();
    virtual ~RadaRaysMIMOOptix();

    virtual void Init() override;
    virtual void Load(const std::string& world_name) override;

    // simulation buffer
    cv::Mat m_polar_image;

protected:
    virtual bool UpdateImpl(const bool _force) override;

    void uploadBuffers();
    void initWorkMemory();
    void loadParamsSDF();
    void loadParams();
    void searchForMaterials();
    void simulate(rmagine::Transform Tsm);

    // For MIMO Radar
    void updateDynCfg(radarays_ros::RadarModelConfig &config, uint32_t level);

    // ROS NODE
    std::shared_ptr<ros::NodeHandle> m_nh_p;

    // DynRec
    std::shared_ptr<dynamic_reconfigure::Server<radarays_ros::RadarModelConfig> > m_dyn_rec_server;

    // Params
    radarays_ros::RadarParams m_params;
    rm::SphericalModel m_radar_model;
    radarays_ros::RadarModelConfig m_cfg;

    // MIMO Radar Specific Attributes
    int m_num_antennas_tx = 4; // Default MIMO TX antennas
    int m_num_antennas_rx = 4; // Default MIMO RX antennas
    float m_antenna_spacing = 0.5; // Default spacing in wavelengths

    // materials
    int m_material_id_air = 0;
    std::vector<int> m_object_materials;
    bool m_materials_searched = false;

    // simulator
    rm::OnDnSimulatorOptixPtr m_sim;

    // global GPU buffers
    rm::Memory<int, rm::VRAM_CUDA> m_object_materials_gpu;
    rm::Memory<radarays_ros::RadarMaterial, rm::VRAM_CUDA> m_materials_gpu;

    // preallocated memory for faster mem handling
    std::vector<rm::OnDnModel_<rm::VRAM_CUDA> > m_waves_gpu;
    std::vector<rm::Memory<radarays_ros::DirectedWaveAttributes, rm::VRAM_CUDA> > m_wave_attributes_gpu;
    std::vector<rm::Memory<radarays_ros::Signal, rm::VRAM_CUDA> > m_signals;
    std::vector<rm::Memory<uint8_t, rm::VRAM_CUDA> > m_signal_mask;
    std::vector<ResT> m_results;

    float m_wave_energy_threshold = 0.001;
    std::vector<radarays_ros::DirectedWave> m_waves_start;
    bool m_resample = true;
    float m_max_signal = 120.0;
};

using RadaRaysMIMOOptixPtr = std::shared_ptr<RadaRaysMIMOOptix>;

// will by generated in cpp
void RegisterRadaRaysMIMOOptix();

} // namespace sensors

} // namespace gazebo

#endif // GAZEBO_RADARAYS_MIMO_OPTIX_PLUGIN_H