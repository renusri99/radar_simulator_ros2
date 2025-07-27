#include <radarays_gazebo_plugins/radarays_optix_gzplugin.hpp>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/EntityComponentManager.hh>

#include <iostream>
#include <boost/algorithm/string/replace.hpp>

#include <rmagine/noise/GaussianNoise.hpp>
#include <rmagine/noise/UniformDustNoise.hpp>
#include <rmagine/noise/RelGaussianNoise.hpp>

#include <radarays_ros/RadarGPU.hpp>

#include <radarays_ros/ros_helper.h>
#include <radarays_ros/radar_algorithms.h>
#include <radarays_ros/image_algorithms.h>
#include <radarays_ros/radar_algorithms.cuh>
#include <radarays_ros/image_algorithms.cuh>

#include <cv_bridge/cv_bridge.h>
#include <omp.h>

#include <rmagine/util/StopWatch.hpp>
#include <rmagine/util/prints.h>

using namespace radarays_ros;
namespace rm = rmagine;

namespace ignition::gazebo
{

static rm::Transform to_rm(const ignition::math::Pose3d &pose)
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

RadaRaysOptix::RadaRaysOptix()
{
    RCLCPP_INFO(rclcpp::get_logger("RadaRaysOptix"), "[RadaRaysOptix] Constructed.");
}

RadaRaysOptix::~RadaRaysOptix()
{
    RCLCPP_INFO(rclcpp::get_logger("RadaRaysOptix"), "[RadaRaysOptix] Destroyed.");
}

void RadaRaysOptix::loadParamsSDF()
{
    // TODO: Load additional SDF parameters if needed
}

void RadaRaysOptix::loadParams()
{
    RCLCPP_INFO(rclcpp::get_logger("RadaRaysOptix"), "[RadaRaysOptix] Loading Parameters...");

    // Default radar parameters
    m_params = default_params();
    m_material_id_air = 0;
    m_wave_energy_threshold = 0.001;
    m_resample = true;

    // Configure MIMO Radar Model with Multiple Antennas
    m_radar_model.theta.inc = -(2 * M_PI) / 400;
    m_radar_model.theta.min = 0.0;
    m_radar_model.theta.size = 400;
    m_radar_model.phi.inc = 1.0;
    m_radar_model.phi.min = 0.0;
    m_radar_model.phi.size = 1;

    // Define default materials: air & wall stone
    m_params.materials.data.resize(2);
    {
        radarays_ros::RadarMaterial material_air;
        material_air.velocity = 0.3; // m/ns
        material_air.ambient  = 1.0;
        material_air.diffuse  = 0.0;
        material_air.specular = 1.0;
        m_params.materials.data[0] = material_air;

        radarays_ros::RadarMaterial material_wall_stone;
        material_wall_stone.velocity = 0.0;
        material_wall_stone.ambient  = 1.0;
        material_wall_stone.diffuse  = 0.0;
        material_wall_stone.specular = 3000.0;
        m_params.materials.data[1] = material_wall_stone;
    }

    // Dynamic Object Handling
    size_t n_default_objects = 100;
    m_object_materials.resize(n_default_objects, 1); // Default: wall stone material

    // Load MIMO Radar Image Buffer
    m_polar_image = cv::Mat_<unsigned char>(0, m_radar_model.theta.size);

    // Define MIMO Radar Antennas (Multiple Tx & Rx)
    int num_tx = 2;  // Number of transmitters
    int num_rx = 4;  // Number of receivers

    m_mimo_antennas.clear();
    for (int tx = 0; tx < num_tx; tx++)
    {
        for (int rx = 0; rx < num_rx; rx++)
        {
            MimoAntenna antenna;
            antenna.tx_id = tx;
            antenna.rx_id = rx;
            antenna.tx_position = {0.1 * tx, 0.0, 0.5}; // Example positions (adjust as needed)
            antenna.rx_position = {0.1 * rx, 0.0, 0.5};
            m_mimo_antennas.push_back(antenna);
        }
    }

    // Load additional parameters from SDF
    loadParamsSDF();

    // ROS 2 Parameter Handling
    if (node_)
    {
        node_->declare_parameter("radar.material_id_air", m_material_id_air);
        node_->declare_parameter("radar.wave_energy_threshold", m_wave_energy_threshold);
        node_->declare_parameter("radar.n_objects", static_cast<int>(n_default_objects));
        node_->declare_parameter("radar.num_tx", num_tx);
        node_->declare_parameter("radar.num_rx", num_rx);

        node_->get_parameter("radar.material_id_air", m_material_id_air);
        node_->get_parameter("radar.wave_energy_threshold", m_wave_energy_threshold);
        int n_objects;
        node_->get_parameter("radar.n_objects", n_objects);
        node_->get_parameter("radar.num_tx", num_tx);
        node_->get_parameter("radar.num_rx", num_rx);

        m_object_materials.resize(n_objects, 1); // Resize dynamically
    }

    RCLCPP_INFO(rclcpp::get_logger("RadaRaysOptix"), "[RadaRaysOptix] MIMO Configuration: %d Tx, %d Rx", num_tx, num_rx);
    RCLCPP_INFO(rclcpp::get_logger("RadaRaysOptix"), "[RadaRaysOptix] Parameters Loaded.");
}
void RadaRaysOptix::uploadBuffers()
{
    RCLCPP_INFO(rclcpp::get_logger("RadaRaysOptix"), "[RadaRaysOptix] Uploading Buffers to GPU...");

    // Upload object materials to GPU
    rm::Memory<int, rm::RAM_CUDA> object_materials_gpu(m_object_materials.size());
    for (size_t i = 0; i < m_object_materials.size(); i++)
    {
        object_materials_gpu[i] = m_object_materials[i];
    }
    m_object_materials_gpu = object_materials_gpu;

    // Upload radar materials to GPU
    rm::Memory<RadarMaterial, rm::RAM_CUDA> materials_gpu(m_params.materials.data.size());
    for (size_t i = 0; i < m_params.materials.data.size(); i++)
    {
        materials_gpu[i] = m_params.materials.data[i];
    }
    m_materials_gpu = materials_gpu;

    RCLCPP_INFO(rclcpp::get_logger("RadaRaysOptix"), "[RadaRaysOptix] Buffers Uploaded.");
}

void RadaRaysOptix::searchForMaterials()
{
    RCLCPP_INFO(rclcpp::get_logger("RadaRaysOptix"), "[RadaRaysOptix] Searching for Materials...");

    // Reset object materials
    m_object_materials.clear();
    m_params.materials.data.resize(1);

    // Define default air material
    radarays_ros::RadarMaterial material_air;
    material_air.velocity = 0.3; // m/ns
    material_air.ambient = 1.0;
    material_air.diffuse = 0.0;
    material_air.specular = 1.0;
    m_params.materials.data[0] = material_air;

    sdf::ElementPtr world_sdf = this->world->SDF();
    sdf::ElementPtr model_sdf = world_sdf->GetElement("model");

    for (sdf::ElementPtr model_sdf = world_sdf->GetElement("model");
         model_sdf;
         model_sdf = model_sdf->GetNextElement("model"))
    {
        std::string model_name = model_sdf->GetAttribute("name")->GetAsString();

        for (sdf::ElementPtr link_sdf = model_sdf->GetElement("link");
             link_sdf;
             link_sdf = link_sdf->GetNextElement("link"))
        {
            std::string link_name = link_sdf->GetAttribute("name")->GetAsString();

            for (sdf::ElementPtr vis_sdf = link_sdf->GetElement("visual");
                 vis_sdf;
                 vis_sdf = vis_sdf->GetNextElement("visual"))
            {
                std::string vis_name = vis_sdf->GetAttribute("name")->GetAsString();
                sdf::ElementPtr mat_sdf = vis_sdf->GetElement("radarays_material");

                radarays_ros::RadarMaterial material;

                if (mat_sdf)
                {
                    RCLCPP_INFO(rclcpp::get_logger("RadaRaysOptix"), 
                                "Found RadaRays material in: %s/%s/%s",
                                model_name.c_str(), link_name.c_str(), vis_name.c_str());
                    material.velocity = mat_sdf->Get<float>("velocity");
                    material.ambient = mat_sdf->Get<float>("ambient");
                    material.diffuse = mat_sdf->Get<float>("diffuse");
                    material.specular = mat_sdf->Get<float>("specular");
                }
                else
                {
                    RCLCPP_WARN(rclcpp::get_logger("RadaRaysOptix"),
                                "Missing RadaRays material in: %s/%s/%s. Using defaults.",
                                model_name.c_str(), link_name.c_str(), vis_name.c_str());

                    material.velocity = 0.0;
                    material.ambient = 1.0;
                    material.diffuse = 0.0;
                    material.specular = 3000.0;
                }

                std::string name = model_name + "::" + link_name + "::" + vis_name;

                // Find model in OptiX map
                if (m_map_mutex)
                {
                    m_map_mutex->lock_shared();
                }

                int obj_id = -1;
                if (m_map)
                {
                    RCLCPP_INFO(rclcpp::get_logger("RadaRaysOptix"), "Searching for: %s", name.c_str());
                    for (auto elem : m_map->scene()->geometries())
                    {
                        if (elem.second->name == name)
                        {
                            obj_id = elem.first;
                            RCLCPP_INFO(rclcpp::get_logger("RadaRaysOptix"), "Found Obj ID: %d", obj_id);
                            break;
                        }
                    }
                }
                else
                {
                    RCLCPP_ERROR(rclcpp::get_logger("RadaRaysOptix"), "No OptiX map found!");
                    continue;
                }

                if (m_map_mutex)
                {
                    m_map_mutex->unlock_shared();
                }

                if (obj_id >= 0)
                {
                    RCLCPP_INFO(rclcpp::get_logger("RadaRaysOptix"), "Model %s -> OptiX Map ID: %d", name.c_str(), obj_id);
                    m_params.materials.data.push_back(material);
                    size_t mat_id = m_params.materials.data.size() - 1;

                    while (m_object_materials.size() <= obj_id)
                    {
                        m_object_materials.push_back(0);
                    }
                    m_object_materials[obj_id] = mat_id;
                }
                else
                {
                    RCLCPP_WARN(rclcpp::get_logger("RadaRaysOptix"), "Could not find geometry in OptiX Map!");
                }
            }
        }
    }

    uploadBuffers();
    RCLCPP_INFO(rclcpp::get_logger("RadaRaysOptix"), "[RadaRaysOptix] Material Search Completed.");
}
void RadaRaysOptix::Init()
{
    RCLCPP_INFO(rclcpp::get_logger("RadaRaysOptix"), "[RadaRaysOptix] Initializing...");

    Base::Init();

    // Create ROS 2 node for radarays
    m_node = std::make_shared<rclcpp::Node>("radarays_optix");

    // Load radar parameters
    loadParams();

    // Dynamic Parameter Handling (ROS 2 replacement for dynamic_reconfigure)
    m_param_handler = std::make_shared<rclcpp::ParameterEventHandler>(m_node);

    m_param_callback_handle = m_param_handler->add_parameter_callback(
        "radar.config",
        [this](const rclcpp::Parameter& param)
        {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                RCLCPP_INFO(m_node->get_logger(), "Updating radar config dynamically...");
                updateDynCfg(param.as_int());
            }
        });

    if (!m_map)
    {
        m_waiting_for_map = true;
        RCLCPP_INFO(rclcpp::get_logger("RadaRaysOptix"), "[RadaRaysOptix] Waiting for OptiX Map...");
    }
}

void RadaRaysOptix::Load(const std::string& world_name)
{
    Base::Load(world_name);
    RCLCPP_INFO(rclcpp::get_logger("RadaRaysOptix"), "[RadaRaysOptix] Loaded into world: %s", world_name.c_str());
}
void RadaRaysOptix::simulate(rm::Transform Tsm)
{
    RCLCPP_INFO(rclcpp::get_logger("RadaRaysOptix"), "[RadaRaysOptix] Starting Simulation...");

    // Resize the output image buffer if needed
    if (m_polar_image.rows != m_cfg.n_cells)
    {
        RCLCPP_INFO(rclcpp::get_logger("RadaRaysOptix"), "[RadaRaysOptix] Resizing canvas to %d cells", m_cfg.n_cells);
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
            for (float &weight : denoising_weights)
            {
                weight /= denoising_mode_val;
            }
        }
    }

    // Upload denoising buffers to GPU
    rm::Memory<float, rm::VRAM_CUDA> denoising_weights_gpu;
    if (!denoising_weights.empty())
    {
        rm::Memory<float, rm::RAM_CUDA> denoising_weights2(denoising_weights.size());
        for (size_t i = 0; i < denoising_weights.size(); i++)
        {
            denoising_weights2[i] = denoising_weights[i];
        }
        denoising_weights_gpu = denoising_weights2;
    }

    // Setup MIMO radar wave attributes
    DirectedWaveAttributes wave_att_ex;
    wave_att_ex.energy = 1.0;
    wave_att_ex.polarization = 0.5;
    wave_att_ex.frequency = 76.5;  // GHz
    wave_att_ex.velocity = 0.3;    // m/ns (speed in air)
    wave_att_ex.material_id = 0;   // air
    wave_att_ex.time = 0.0;        // ns

    int n_cells = m_polar_image.rows;
    int n_angles = m_polar_image.cols;

    // Configure MIMO Radar Model
    size_t n_rays = n_angles * m_cfg.n_samples;
    rm::OnDnModel waves;
    waves.range = m_radar_model.range;
    waves.dirs.resize(n_rays);
    waves.origs.resize(n_rays);
    waves.width = n_angles;
    waves.height = m_params.model.n_samples;
    rm::Memory<DirectedWaveAttributes> wave_attributes(n_rays);

    rm::Vector front = {1.0, 0.0, 0.0};
    rm::Memory<rm::Vector> ray_dirs_local = sample_cone(
        front,
        m_params.model.beam_width,
        m_params.model.n_samples,
        m_cfg.beam_sample_dist,
        m_cfg.beam_sample_dist_normal_p_in_cone);

    // Fill Radar Model for MIMO Antennas
    for (size_t angle_id = 0; angle_id < n_angles; angle_id++)
    {
        rm::Transform Tas;
        Tas.R = rm::EulerAngles{0.0, 0.0, m_radar_model.getTheta(angle_id)};
        Tas.t = m_radar_model.getOrigin(0, angle_id);

        for (size_t sample_id = 0; sample_id < m_params.model.n_samples; sample_id++)
        {
            size_t buf_id = waves.getBufferId(sample_id, angle_id);
            waves.dirs[buf_id] = Tas.R * ray_dirs_local[sample_id];
            waves.origs[buf_id] = {0.0, 0.0, 0.0};
            wave_attributes[buf_id] = wave_att_ex;
        }
    }

    if (m_waves_gpu.size() != m_cfg.n_reflections)
    {
        RCLCPP_INFO(rclcpp::get_logger("RadaRaysOptix"), "Preallocating GPU memory buffers...");
        initWorkMemory();
    }

    rm::Memory<rm::Transform, rm::VRAM_CUDA> Tsms_;
    { // Upload transform to GPU
        rm::Memory<rm::Transform> Tsms(1);
        Tsms[0] = Tsm;
        Tsms_ = Tsms;
    }

    // Simulation Execution (Multiple Reflections)
    for (size_t i = 0; i < m_cfg.n_reflections; i++)
    {
        m_sim->setModel(m_waves_gpu[i]);
        m_sim->simulate(Tsms_, m_results[i]);

        // Move wave propagation
        move_waves(
            m_waves_gpu[i].origs,
            m_waves_gpu[i].dirs,
            m_wave_attributes_gpu[i],
            m_results[i].ranges,
            m_results[i].hits);

        // Apply reflection models
        signal_shader(
            m_materials_gpu,
            m_object_materials_gpu,
            m_material_id_air,
            m_waves_gpu[i].dirs,
            m_wave_attributes_gpu[i],
            m_results[i].hits,
            m_results[i].normals,
            m_results[i].object_ids,
            m_signals[i]);

        if (i < m_cfg.n_reflections - 1)
        {
            // Fresnel split for multiple reflections
            fresnel_split(
                m_materials_gpu,
                m_object_materials_gpu,
                m_material_id_air,
                m_waves_gpu[i].origs,
                m_waves_gpu[i].dirs,
                m_wave_attributes_gpu[i],
                m_results[i].hits,
                m_results[i].normals,
                m_results[i].object_ids,
                m_waves_gpu[i + 1].origs,
                m_waves_gpu[i + 1].dirs,
                m_wave_attributes_gpu[i + 1]);
        }
    }

    // Image Processing
    rm::Memory<float, rm::UNIFIED_CUDA> img(n_cells * n_angles);
    cudaMemset(img.raw(), 0, n_cells * n_angles * sizeof(float));

    for (size_t i = 0; i < m_cfg.n_reflections; i++)
    {
        for (size_t angle_id = 0; angle_id < n_angles; angle_id++)
        {
            size_t img_offset = angle_id * n_cells;
            float max_val = 0.0;

            for (size_t pass_id = 0; pass_id < m_cfg.n_reflections; pass_id++)
            {
                int size_factor = std::pow(2, pass_id);
                for (size_t sample_id = 0; sample_id < m_cfg.n_samples * size_factor; sample_id++)
                {
                    const unsigned int signal_id = sample_id * n_angles + angle_id;
                    if (m_results[pass_id].hits[signal_id])
                    {
                        auto signal = m_signals[pass_id][signal_id];
                        float signal_dist = 0.3 * (signal.time / 2.0);
                        int cell = static_cast<int>(signal_dist / m_cfg.resolution);
                        if (cell < n_cells)
                        {
                            img[img_offset + cell] = std::max(img[img_offset + cell], signal.strength);
                            max_val = std::max(max_val, signal.strength);
                        }
                    }
                }
            }
        }
    }

    cv::Mat_<float> polar_img_f(n_cells, n_angles);
    for (size_t x = 0; x < n_angles; x++)
    {
        for (size_t y = 0; y < n_cells; y++)
        {
            polar_img_f.at<float>(y, x) = img[x * n_cells + y] * m_cfg.signal_max;
        }
    }
    polar_img_f.convertTo(m_polar_image, CV_8UC1);

    RCLCPP_INFO(rclcpp::get_logger("RadaRaysOptix"), "[RadaRaysOptix] Simulation Complete!");
}
bool RadaRaysOptix::UpdateImpl(const bool _force)
{
    if (!m_map)
    {
        if (!m_waiting_for_map)
        {
            RCLCPP_INFO(rclcpp::get_logger("RadaRaysOptix"), "[RadaRaysOptix] Waiting for RmagineOptixMap...");
            m_waiting_for_map = true;
        }
        return false;
    }
    else
    {
        if (m_waiting_for_map)
        {
            RCLCPP_INFO(rclcpp::get_logger("RadaRaysOptix"), "[RadaRaysOptix] Map received!");
            m_waiting_for_map = false;
        }

        if (!m_sim)
        {
            m_sim = std::make_shared<rm::OnDnSimulatorOptix>(m_map);
            RCLCPP_INFO(rclcpp::get_logger("RadaRaysOptix"), "[RadaRaysOptix] OnDnSimulator constructed.");

            m_sim->setTsb(rm::Transform::Identity());
            RCLCPP_INFO(rclcpp::get_logger("RadaRaysOptix"), "[RadaRaysOptix] OnDnSimulator map set.");

            m_sim->preBuildProgram<ResT>();
        }

        if (!m_materials_searched)
        {
            RCLCPP_INFO(rclcpp::get_logger("RadaRaysOptix"), "Searching for radarays materials...");
            searchForMaterials();
            m_materials_searched = true;
        }
    }

    auto pose = this->parentEntity->WorldPose();
    rm::Transform Tbm = to_rm(pose);
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

    return Base::UpdateImpl(_force);
}

void RadaRaysOptix::initWorkMemory()
{
    RCLCPP_INFO(rclcpp::get_logger("RadaRaysOptix"), "Resizing working memory for %d reflections passes", m_cfg.n_reflections);
    if (m_cfg.n_reflections == 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("RadaRaysOptix"), "Cannot initialize working memory for 0 reflections!");
        return;
    }

    m_waves_gpu.resize(m_cfg.n_reflections);
    m_wave_attributes_gpu.resize(m_cfg.n_reflections);
    m_results.resize(m_cfg.n_reflections);
    m_signals.resize(m_cfg.n_reflections);
    m_signal_mask.resize(m_cfg.n_reflections);

    int n_cells = m_cfg.n_cells;
    int n_angles = m_radar_model.theta.size;
    size_t n_rays = n_angles * m_cfg.n_samples;

    if (m_cfg.n_reflections > 0)
    {
        RCLCPP_INFO(rclcpp::get_logger("RadaRaysOptix"), "Allocating buffers for pass 1");
        
        m_waves_gpu[0].width = n_angles;
        m_waves_gpu[0].height = m_params.model.n_samples;
        m_waves_gpu[0].range = m_radar_model.range;
        m_waves_gpu[0].origs.resize(n_rays);
        m_waves_gpu[0].dirs.resize(n_rays);

        m_wave_attributes_gpu[0].resize(n_rays);
        rm::resize_memory_bundle<rm::VRAM_CUDA>(
            m_results[0], m_waves_gpu[0].width, m_waves_gpu[0].height, 1);
        m_signals[0].resize(m_waves_gpu[0].size());
        m_signal_mask[0].resize(m_waves_gpu[0].size());

        for (size_t i = 1; i < m_cfg.n_reflections; i++)
        {
            RCLCPP_INFO(rclcpp::get_logger("RadaRaysOptix"), "Allocating buffers for pass %d", i + 1);

            m_waves_gpu[i].width = m_waves_gpu[i - 1].width;
            m_waves_gpu[i].height = m_waves_gpu[i - 1].height * 2;
            m_waves_gpu[i].range = m_waves_gpu[i - 1].range;
            m_waves_gpu[i].origs.resize(m_waves_gpu[i - 1].origs.size() * 2);
            m_waves_gpu[i].dirs.resize(m_waves_gpu[i - 1].dirs.size() * 2);

            m_wave_attributes_gpu[i].resize(m_wave_attributes_gpu[i - 1].size() * 2);

            rm::resize_memory_bundle<rm::VRAM_CUDA>(
                m_results[i], m_waves_gpu[i].width, m_waves_gpu[i].height, 1);

            m_signals[i].resize(m_waves_gpu[i].size());
            m_signal_mask[i].resize(m_waves_gpu[i].size());
        }
    }
}

void RadaRaysOptix::updateDynCfg(
    radarays_ros::RadarModelConfig &config,
    uint32_t level)
{
    RCLCPP_INFO(rclcpp::get_logger("RadaRaysOptix"), "Updating MIMO Radar Configuration...");

    if (config.beam_sample_dist != m_cfg.beam_sample_dist ||
        abs(config.beam_width - m_cfg.beam_width) > 0.001 ||
        config.n_samples != m_cfg.n_samples ||
        abs(config.beam_sample_dist_normal_p_in_cone - m_cfg.beam_sample_dist_normal_p_in_cone) > 0.001)
    {
        m_resample = true;
    }

    m_radar_model.range.min = config.range_min;
    m_radar_model.range.max = config.range_max;

    m_params.model.beam_width = config.beam_width * M_PI / 180.0;
    m_params.model.n_samples = config.n_samples;

    bool buffer_reinit = (config.n_reflections != m_params.model.n_reflections);

    m_params.model.n_reflections = config.n_reflections;
    m_cfg = config;

    if (buffer_reinit)
    {
        RCLCPP_WARN(rclcpp::get_logger("RadaRaysOptix"), "Reinitializing GPU buffers for %d reflections", m_params.model.n_reflections);
        initWorkMemory();
    }
}

GZ_REGISTER_STATIC_SENSOR("radarays_optix", RadaRaysOptix)

} // namespace sensors
} // namespace gazebo
