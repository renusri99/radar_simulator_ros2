#include <radarays_gazebo_plugins/radarays_embree_gzplugin.h>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/RaySensor.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <iostream>
#include <boost/algorithm/string/replace.hpp>

#include <rmagine/noise/GaussianNoise.hpp>
#include <rmagine/noise/UniformDustNoise.hpp>
#include <rmagine/noise/RelGaussianNoise.hpp>

#include <radarays_ros/RadarCPU.hpp>

#include <boost/bind.hpp>

#include <radarays_ros/ros_helper.h>
#include <radarays_ros/radar_algorithms.h>
#include <radarays_ros/image_algorithms.h>
#include <cv_bridge/cv_bridge.h>
#include <omp.h>

#include <rmagine/util/StopWatch.hpp>
#include <rmagine/util/prints.h>

using namespace radarays_ros;
namespace rm = rmagine;

namespace ignition::gazebo
{

static rm::Transform to_rm(const ignition::math::Pose3d& pose)
{
    rmagine::Transform T;
    T.R.x = pose.Rot().X();
    T.R.y = pose.Rot().Y();
    T.R.z = pose.Rot().Z();
    T.R.w = pose.Rot().W();
    T.t.x = pose.Pos().X();
    T.t.y = pose.Pos().Y();
    T.t.z = pose.Pos().Z();
    return T;
}

RadaRaysEmbree::RadaRaysEmbree()
{
    node_ = std::make_shared<rclcpp::Node>("mimo_radar_node");
    RCLCPP_INFO(node_->get_logger(), "[RadaRaysEmbree] Constructed.");
}

RadaRaysEmbree::~RadaRaysEmbree()
{
    RCLCPP_INFO(node_->get_logger(), "[RadaRaysEmbree] Destroyed.");
}

void RadaRaysEmbree::loadParamsSDF()
{
    // Placeholder for loading parameters from SDF in Ignition Gazebo
}
void RadaRaysEmbree::loadParams()
{
    RCLCPP_INFO(node_->get_logger(), "[RadaRaysEmbree] Loading Parameters...");

    m_params = default_params();
    m_material_id_air = 0;
    m_wave_energy_threshold = 0.001;
    m_resample = true;

    // MIMO Radar Model Update (Multiple Tx/Rx Antennas)
    m_radar_model.tx_count = 4;  // Number of transmit antennas
    m_radar_model.rx_count = 4;  // Number of receive antennas
    m_radar_model.theta.size = 1024; // Horizontal resolution
    m_radar_model.phi.size = 128; // Vertical resolution
    m_radar_model.theta.inc = -(2 * M_PI) / 400;
    m_radar_model.theta.min = 0.0;
    m_radar_model.phi.inc = 1.0;
    m_radar_model.phi.min = 0.0;

    // 2 default materials: air, wall stone
    m_params.materials.data.resize(2);
    
    // Air Material
    radarays_ros::RadarMaterial material_air;
    material_air.velocity = 0.3; // m/ns
    material_air.ambient  = 1.0;
    material_air.diffuse  = 0.0;
    material_air.specular = 1.0;
    m_params.materials.data[0] = material_air;

    // Wall Stone Material
    radarays_ros::RadarMaterial material_wall_stone;
    material_wall_stone.velocity = 0.0;
    material_wall_stone.ambient  = 1.0;
    material_wall_stone.diffuse  = 0.0;
    material_wall_stone.specular = 3000.0;
    m_params.materials.data[1] = material_wall_stone;

    // Default Object Material Assignments
    size_t n_default_objects = 100;
    m_object_materials.resize(n_default_objects, 1); // Assign all to wall stone

    // Initialize MIMO radar output image matrix
    m_polar_image = cv::Mat_<unsigned char>(0, m_radar_model.theta.size);

    // Load additional parameters from SDF in Ignition Gazebo
    loadParamsSDF();

    // TODO: Instead of ROS parameter server, explore Ignition Gazebo XML-based param handling
    RCLCPP_INFO(node_->get_logger(), "[RadaRaysEmbree] Parameters Loaded.");
}
void RadaRaysEmbree::searchForMaterials()
{
    RCLCPP_INFO(node_->get_logger(), "[RadaRaysEmbree] Searching for Materials...");

    m_object_materials.clear();
    m_params.materials.data.resize(1);

    // Default Air Material
    radarays_ros::RadarMaterial material_air;
    material_air.velocity = 0.3;
    material_air.ambient = 1.0;
    material_air.diffuse = 0.0;
    material_air.specular = 1.0;
    m_params.materials.data[0] = material_air;

    // Access world and entities in Ignition Gazebo
    auto entities = this->entityComponentManager_.EntitiesWithComponents<components::Model>();
    
    for (auto entity : entities)
    {
        auto model_comp = entityComponentManager_.Component<components::Name>(entity);
        if (!model_comp) continue;
        
        std::string model_name = model_comp->Data();

        auto links = entityComponentManager_.EntitiesWithComponents<components::Link>();
        
        for (auto link : links)
        {
            auto link_comp = entityComponentManager_.Component<components::Name>(link);
            if (!link_comp) continue;
            
            std::string link_name = link_comp->Data();

            auto visuals = entityComponentManager_.EntitiesWithComponents<components::Visual>();
            
            for (auto visual : visuals)
            {
                auto vis_comp = entityComponentManager_.Component<components::Name>(visual);
                if (!vis_comp) continue;
                
                std::string vis_name = vis_comp->Data();

                // Get radar material properties if present
                radarays_ros::RadarMaterial material;
                bool has_custom_material = false;

                auto material_comp = entityComponentManager_.Component<components::Material>(visual);
                if (material_comp)
                {
                    has_custom_material = true;
                    material.velocity = material_comp->Data().Get<float>("velocity", 0.3);
                    material.ambient = material_comp->Data().Get<float>("ambient", 1.0);
                    material.diffuse = material_comp->Data().Get<float>("diffuse", 0.0);
                    material.specular = material_comp->Data().Get<float>("specular", 1.0);
                    
                    RCLCPP_INFO(node_->get_logger(), "Found custom radar material for %s/%s/%s",
                                model_name.c_str(), link_name.c_str(), vis_name.c_str());
                }
                else
                {
                    // Default material (wall stone)
                    RCLCPP_WARN(node_->get_logger(), "-- No radar material found. Assigning default.");
                    material.velocity = 0.0;
                    material.ambient = 1.0;
                    material.diffuse = 0.0;
                    material.specular = 3000.0;
                }

                // Construct entity name
                std::string entity_name = model_name + "::" + link_name + "::" + vis_name;

                // Lock shared access to Embree map
                if (m_map_mutex)
                    m_map_mutex->lock_shared();

                int obj_id = -1;
                if (m_map)
                {
                    RCLCPP_INFO(node_->get_logger(), "Searching for: %s in Embree map...", entity_name.c_str());

                    for (auto &elem : m_map->scene->geometries())
                    {
                        if (elem.second->name == entity_name)
                        {
                            RCLCPP_INFO(node_->get_logger(), "Found object ID: %d", elem.first);
                            obj_id = elem.first;
                            break;
                        }
                    }
                }
                else
                {
                    RCLCPP_ERROR(node_->get_logger(), "ERROR: No Embree map found.");
                    if (m_map_mutex)
                        m_map_mutex->unlock_shared();
                    continue;
                }

                // Unlock after reading Embree map
                if (m_map_mutex)
                    m_map_mutex->unlock_shared();

                if (obj_id >= 0)
                {
                    RCLCPP_INFO(node_->get_logger(), "Assigning radar material to object ID: %d", obj_id);

                    m_params.materials.data.push_back(material);
                    size_t mat_id = m_params.materials.data.size() - 1;

                    // Ensure object material list has space
                    if (m_object_materials.size() <= obj_id)
                        m_object_materials.resize(obj_id + 1, 0); // Default material ID 0 (air)

                    // Assign material to object
                    m_object_materials[obj_id] = mat_id;
                }
            }
        }
    }

    RCLCPP_INFO(node_->get_logger(), "[RadaRaysEmbree] Material search completed.");
}
void RadaRaysEmbree::Init()
{
    RCLCPP_INFO(node_->get_logger(), "[RadaRaysEmbree] Initializing...");

    loadParams();

    // Set up ROS 2 parameter handling instead of dynamic_reconfigure
    node_->declare_parameter("radar_model.beam_width", m_params.model.beam_width);
    node_->declare_parameter("radar_model.n_samples", m_params.model.n_samples);
    node_->declare_parameter("radar_model.n_reflections", m_params.model.n_reflections);

    auto param_callback = [this](const rclcpp::Parameter &param) {
        if (param.get_name() == "radar_model.beam_width" ||
            param.get_name() == "radar_model.n_samples" ||
            param.get_name() == "radar_model.n_reflections")
        {
            updateRadarModel();
        }
    };

    param_sub_ = node_->add_on_set_parameters_callback(param_callback);

    if (!m_map)
    {
        m_waiting_for_map = true;
    }

    RCLCPP_INFO(node_->get_logger(), "[RadaRaysEmbree] Initialization Complete.");
}

void RadaRaysEmbree::Load(const std::string& world_name)
{
    RCLCPP_INFO(node_->get_logger(), "[RadaRaysEmbree] Loaded into world: %s", world_name.c_str());
}
void RadaRaysEmbree::simulate(rm::Transform Tsm)
{
    if (m_polar_image.rows != m_cfg.n_cells)
    {
        RCLCPP_INFO(node_->get_logger(), "[RadaRaysEmbree] Resizing radar image to %d cells.", m_cfg.n_cells);
        m_polar_image.resize(m_cfg.n_cells);
    }

    m_polar_image.setTo(cv::Scalar(0));

    std::vector<float> denoising_weights;
    int denoising_mode = 0;

    if (m_cfg.signal_denoising > 0)
    {
        if (m_cfg.signal_denoising == 1)
        {
            denoising_mode = m_cfg.signal_denoising_triangular_mode * m_cfg.signal_denoising_triangular_width;
            denoising_weights = make_denoiser_triangular(m_cfg.signal_denoising_triangular_width, denoising_mode);
        }
        else if (m_cfg.signal_denoising == 2)
        {
            denoising_mode = m_cfg.signal_denoising_gaussian_mode * m_cfg.signal_denoising_gaussian_width;
            denoising_weights = make_denoiser_gaussian(m_cfg.signal_denoising_gaussian_width, denoising_mode);
        }
        else if (m_cfg.signal_denoising == 3)
        {
            denoising_mode = m_cfg.signal_denoising_mb_mode * m_cfg.signal_denoising_mb_width;
            denoising_weights = make_denoiser_maxwell_boltzmann(m_cfg.signal_denoising_mb_width, denoising_mode);
        }

        if (!denoising_weights.empty())
        {
            double denoising_mode_val = denoising_weights[denoising_mode];
            for (auto &weight : denoising_weights)
                weight /= denoising_mode_val;
        }
    }

    // Initialize waves for MIMO Radar
    std::vector<DirectedWave> waves_start(m_radar_model.tx_count * m_radar_model.rx_count);

    for (auto &wave : waves_start)
    {
        wave.energy = 1.0;
        wave.polarization = 0.5;
        wave.frequency = 76.5; // GHz
        wave.velocity = 0.3;   // m/ns (speed of wave in air)
        wave.material_id = 0;  // air
        wave.time = 0.0;       // ns
        wave.ray.orig = {0.0, 0.0, 0.0};
        wave.ray.dir = {1.0, 0.0, 0.0};
    }

    int n_cells = m_polar_image.rows;
    int n_angles = m_polar_image.cols;

    if (m_resample)
    {
        m_waves_start = sample_cone_local(
            waves_start,
            m_params.model.beam_width,
            m_params.model.n_samples,
            m_cfg.beam_sample_dist,
            m_cfg.beam_sample_dist_normal_p_in_cone);
        m_resample = false;
    }

    rm::StopWatch sw_radar_sim;
    sw_radar_sim();

    #pragma omp parallel for
    for (size_t angle_id = 0; angle_id < n_angles; angle_id++)
    {
        int tid = omp_get_thread_num();
        auto sims_it = m_sims.find(tid);

        if (sims_it == m_sims.end())
        {
            m_sims[tid] = std::make_shared<rm::OnDnSimulatorEmbree>(m_map);
            sims_it = m_sims.find(tid);
            sims_it->second->setTsb(rm::Transform::Identity());

            #pragma omp critical
            RCLCPP_INFO(node_->get_logger(), "Created new simulator for thread %d.", tid);
        }
        auto sim = sims_it->second;

        if (!sim)
        {
            RCLCPP_ERROR(node_->get_logger(), "ERROR: Simulator instance is empty.");
            continue;
        }

        std::vector<DirectedWave> waves = m_waves_start;

        rm::OnDnModel model = make_model(waves);
        sim->setModel(model);

        rm::Transform Tas;
        Tas.R = rm::EulerAngles{0.0, 0.0, m_radar_model.getTheta(angle_id)};
        Tas.t = m_radar_model.getOrigin(0, angle_id);

        rm::Transform Tam = Tsm * Tas;

        rm::Memory<rm::Transform> Tams(1);
        Tams[0] = Tam;

        std::vector<Signal> signals;

        for (size_t pass_id = 0; pass_id < m_params.model.n_reflections; pass_id++)
        {
            using ResT = rm::Bundle<
                rm::Hits<rm::RAM>,
                rm::Ranges<rm::RAM>,
                rm::Normals<rm::RAM>,
                rm::ObjectIds<rm::RAM>>;

            ResT results;

            results.hits.resize(model.size());
            results.ranges.resize(model.size());
            results.normals.resize(model.size());
            results.object_ids.resize(model.size());

            sim->simulate(Tams, results);

            std::vector<DirectedWave> waves_new;

            for (size_t i = 0; i < waves.size(); i++)
            {
                DirectedWave wave = waves[i];
                const float wave_range = results.ranges[i];
                const rmagine::Vector surface_normal = results.normals[i].normalize();
                const unsigned int obj_id = results.object_ids[i];

                if (obj_id > 10000)
                {
                    continue;
                }

                const DirectedWave incidence = wave.move(wave_range);

                DirectedWave reflection = incidence;
                DirectedWave refraction = incidence;

                if (incidence.material_id == m_material_id_air)
                {
                    refraction.material_id = m_object_materials[obj_id];
                }
                else
                {
                    refraction.material_id = m_material_id_air;
                }

                float v_refraction = 1.0;

                if (incidence.material_id != refraction.material_id)
                {
                    v_refraction = m_params.materials.data[refraction.material_id].velocity;
                }
                else
                {
                    v_refraction = incidence.velocity;
                }

                auto res = fresnel(surface_normal, incidence, v_refraction);

                reflection.ray.dir = res.first.ray.dir;
                reflection.energy = res.first.energy;

                if (reflection.energy > m_wave_energy_threshold)
                {
                    waves_new.push_back(reflection);

                    if (reflection.material_id == m_material_id_air)
                    {
                        auto material = m_params.materials.data[refraction.material_id];

                        double incidence_angle = get_incidence_angle(surface_normal, incidence);
                        double return_energy_path = back_reflection_shader(
                            incidence_angle,
                            reflection.energy,
                            material.ambient,
                            material.diffuse,
                            material.specular);

                        float time_back = incidence.time * 2.0;
                        signals.push_back({time_back, return_energy_path});
                    }
                }

                refraction.ray.dir = res.second.ray.dir;
                refraction.energy = res.second.energy;

                if (refraction.energy > m_wave_energy_threshold)
                {
                    waves_new.push_back(refraction);
                }
            }

            waves = waves_new;

            if (pass_id < m_params.model.n_reflections - 1)
            {
                model = make_model(waves);
                sim->setModel(model);
            }
        }
    }

    double el_radar_sim = sw_radar_sim();
    RCLCPP_INFO(node_->get_logger(), "Radar simulation completed in %.6f seconds.", el_radar_sim);
}
bool RadaRaysEmbree::UpdateImpl(const bool _force)
{
    if (!m_map)
    {
        if (!m_waiting_for_map)
        {
            RCLCPP_WARN(node_->get_logger(), "[RadaRaysEmbree] Waiting for RmagineEmbreeMap...");
            m_waiting_for_map = true;
        }
        return false;
    }
    else
    {
        if (m_waiting_for_map)
        {
            RCLCPP_INFO(node_->get_logger(), "[RadaRaysEmbree] Map received!");
            m_waiting_for_map = false;
        }

        if (!m_materials_searched)
        {
            RCLCPP_INFO(node_->get_logger(), "[RadaRaysEmbree] Searching for materials...");
            searchForMaterials();
            m_materials_searched = true;
        }
    }

    auto pose = this->entityComponentManager_.Component<ignition::gazebo::components::Pose>(this->sensorEntity);
    if (!pose)
    {
        RCLCPP_WARN(node_->get_logger(), "[RadaRaysEmbree] No Pose Component Found!");
        return false;
    }

    rm::Transform Tbm = to_rm(pose->Data());
    rm::Transform Tsm = Tbm * m_Tsb;

    if (m_map_mutex)
    {
        m_map_mutex->lock_shared();
    }

    simulate(Tsm);

    if (m_map_mutex)
    {
        m_map_mutex->unlock_shared();
    }

    return true;
}

void RadaRaysEmbree::updateRadarModel()
{
    RCLCPP_INFO(node_->get_logger(), "[RadaRaysEmbree] Updating Radar Model...");

    double beam_width, n_samples, n_reflections;
    node_->get_parameter("radar_model.beam_width", beam_width);
    node_->get_parameter("radar_model.n_samples", n_samples);
    node_->get_parameter("radar_model.n_reflections", n_reflections);

    if (beam_width != m_params.model.beam_width || n_samples != m_params.model.n_samples || n_reflections != m_params.model.n_reflections)
    {
        m_resample = true;
    }

    // Update radar model
    m_params.model.beam_width = beam_width * M_PI / 180.0;
    m_params.model.n_samples = n_samples;
    m_params.model.n_reflections = n_reflections;

    RCLCPP_INFO(node_->get_logger(), "[RadaRaysEmbree] Radar Model Updated.");
}

IGNITION_ADD_PLUGIN(RadaRaysEmbree, ignition::gazebo::System,
                    RadaRaysEmbree::ISystemConfigure,
                    RadaRaysEmbree::ISystemUpdate)


