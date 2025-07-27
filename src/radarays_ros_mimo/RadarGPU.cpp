#include "radarays_ros/RadarGPU.hpp"

#include <cv_bridge/cv_bridge.h>
#include <omp.h>
#include <random>
#include <complex>
#include <algorithm>

#include <radarays_ros/radar_algorithms.h>
#include <radarays_ros/image_algorithms.cuh>
#include <radarays_ros/radar_algorithms.cuh>
#include <rmagine/util/StopWatch.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace rm = rmagine;
using namespace std::complex_literals;

namespace radarays_ros
{

RadarGPU::RadarGPU(
    std::shared_ptr<rclcpp::Node> node,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::shared_ptr<tf2_ros::TransformListener> tf_listener,
    std::string map_frame,
    std::string sensor_frame,
    rm::OptixMapPtr map)
: Radar(node, tf_buffer, tf_listener, map_frame, sensor_frame),
  m_map(map),
  m_wavelength(0.0039f) // 76.5GHz wavelength
{
    // Load parameters with ROS 2 style
    m_cfg.num_tx = node->declare_parameter<int>("num_tx", 3);
    m_cfg.num_rx = node->declare_parameter<int>("num_rx", 4);
    m_cfg.n_cells = node->declare_parameter<int>("n_cells", 512);
    m_cfg.resolution = node->declare_parameter<double>("resolution", 0.05);
    m_cfg.signal_max = node->declare_parameter<double>("signal_max", 255.0);
    m_cfg.energy_max = node->declare_parameter<double>("energy_max", 1.0);
    m_cfg.code_length = node->declare_parameter<int>("code_length", 8);
    
    // Material and reflection parameters
    m_cfg.n_reflections = node->declare_parameter<int>("n_reflections", 3);
    m_cfg.include_motion = node->declare_parameter<bool>("include_motion", false);
    m_cfg.record_multi_reflection = node->declare_parameter<bool>("record_multi_reflection", true);
    m_cfg.record_multi_path = node->declare_parameter<bool>("record_multi_path", false);
    m_cfg.multipath_threshold = node->declare_parameter<double>("multipath_threshold", 0.7);
    
    // Denoising parameters
    m_cfg.signal_denoising = node->declare_parameter<int>("signal_denoising", 1);
    m_cfg.signal_denoising_triangular_width = node->declare_parameter<int>("signal_denoising_triangular_width", 5);
    m_cfg.signal_denoising_triangular_mode = node->declare_parameter<int>("signal_denoising_triangular_mode", 2);
    
    // Noise parameters
    m_cfg.ambient_noise = node->declare_parameter<int>("ambient_noise", 2);
    m_cfg.ambient_noise_at_signal_0 = node->declare_parameter<double>("ambient_noise_at_signal_0", 0.1);
    m_cfg.ambient_noise_at_signal_1 = node->declare_parameter<double>("ambient_noise_at_signal_1", 0.01);
    m_cfg.ambient_noise_energy_max = node->declare_parameter<double>("ambient_noise_energy_max", 0.2);
    m_cfg.ambient_noise_energy_min = node->declare_parameter<double>("ambient_noise_energy_min", 0.01);
    m_cfg.ambient_noise_energy_loss = node->declare_parameter<double>("ambient_noise_energy_loss", 0.1);

    // Initialize MIMO array
    initializeMIMOArray();
    generateOrthogonalCodes();
}

void RadarGPU::initializeMIMOArray()
{
    const float spacing = m_wavelength / 2.0f;
    m_antennas.clear();

    // Create uniform linear array
    for(int tx = 0; tx < m_cfg.num_tx; ++tx) {
        for(int rx = 0; rx < m_cfg.num_rx; ++rx) {
            AntennaConfig antenna;
            antenna.position = Eigen::Vector3f(
                0.0f, 
                (tx * m_cfg.num_rx + rx) * spacing, 
                0.0f);
            antenna.is_transmitter = (rx == 0); // First RX in each group is TX
            m_antennas.push_back(antenna);
        }
    }

    m_virtual_array_size = m_cfg.num_tx * m_cfg.num_rx;
}

void RadarGPU::generateOrthogonalCodes()
{
    // Generate Hadamard codes for orthogonal transmission
    m_orthogonal_codes.resize(m_cfg.num_tx, std::vector<std::complex<float>>(m_cfg.code_length));
    
    // Basic Hadamard matrix generation
    int n = m_cfg.code_length;
    cv::Mat hadamard = cv::Mat::ones(1, 1, CV_32FC1);
    
    for(int k = 1; k < n; k *= 2) {
        cv::Mat tmp = cv::Mat::zeros(2*k, 2*k, CV_32FC1);
        hadamard.copyTo(tmp(cv::Rect(0, 0, k, k));
        hadamard.copyTo(tmp(cv::Rect(k, k, k, k));
        hadamard.copyTo(tmp(cv::Rect(0, k, k, k)));
        -hadamard.copyTo(tmp(cv::Rect(k, 0, k, k)));
        hadamard = tmp;
    }
    
    // Assign to transmitters
    for(int tx = 0; tx < m_cfg.num_tx; ++tx) {
        for(int i = 0; i < m_cfg.code_length; ++i) {
            m_orthogonal_codes[tx][i] = hadamard.at<float>(tx % hadamard.rows, i);
        }
    }
}

sensor_msgs::msg::Image::SharedPtr RadarGPU::simulate(rclcpp::Time stamp)
{
    // Update transform
    if(!updateTsm(stamp)) {
        RCLCPP_ERROR(m_node->get_logger(), "Transform update failed");
        return nullptr;
    }

    rm::StopWatch sw;
    sw();

    // Initialize output image (range x virtual_array)
    cv::Mat polar_image = cv::Mat::zeros(m_cfg.n_cells, m_virtual_array_size, CV_32FC1);

    // Generate all TX waveforms
    std::vector<std::vector<std::complex<float>>> tx_signals(m_cfg.num_tx);
    #pragma omp parallel for
    for(int tx = 0; tx < m_cfg.num_tx; tx++) {
        tx_signals[tx] = generateWaveform(tx);
    }

    // Prepare GPU memory for materials
    rm::Memory<int, rm::RAM_CUDA> object_materials2(m_object_materials.size());
    for(size_t i=0; i<m_object_materials.size(); i++) {
        object_materials2[i] = m_object_materials[i];
    }
    rm::Memory<int, rm::VRAM_CUDA> object_materials_gpu = object_materials2;

    rm::Memory<RadarMaterial, rm::RAM_CUDA> materials2(m_params.materials.data.size());
    for(size_t i=0; i<m_params.materials.data.size(); i++) {
        materials2[i] = m_params.materials.data[i];
    }
    rm::Memory<RadarMaterial, rm::VRAM_CUDA> materials_gpu = materials2;

    // Simulate all TX-RX paths
    #pragma omp parallel for collapse(2)
    for(int tx = 0; tx < m_cfg.num_tx; tx++) {
        for(int rx = 0; rx < m_cfg.num_rx; rx++) {
            int virt_idx = tx * m_cfg.num_rx + rx;
            
            // Configure base wave
            DirectedWaveAttributes wave_att_ex;
            wave_att_ex.energy = 1.0f;
            wave_att_ex.polarization = 0.5f;
            wave_att_ex.frequency = 76.5f; // GHz
            wave_att_ex.velocity = 0.3f;   // m/ns
            wave_att_ex.material_id = m_material_id_air;
            wave_att_ex.time = 0.0f;
            
            // Set antenna positions
            const auto& tx_ant = m_antennas[tx * m_cfg.num_rx];
            const auto& rx_ant = m_antennas[tx * m_cfg.num_rx + rx];
            
            rm::OnDnModel waves;
            waves.range = m_radar_model.range;
            waves.width = 1; // Single angle for this pair
            waves.height = m_params.model.n_samples;
            waves.dirs.resize(m_params.model.n_samples);
            waves.origs.resize(m_params.model.n_samples);
            rm::Memory<DirectedWaveAttributes> wave_attributes(m_params.model.n_samples);

            rm::Vector front = {1.0, 0.0, 0.0};
            rm::Memory<rm::Vector> ray_dirs_local = sample_cone(
                front,
                m_params.model.beam_width,
                m_params.model.n_samples,
                m_cfg.beam_sample_dist,
                m_cfg.beam_sample_dist_normal_p_in_cone
            );

            // Create transform from TX to RX
            rm::Vector dir = {
                rx_ant.position.x() - tx_ant.position.x(),
                rx_ant.position.y() - tx_ant.position.y(),
                rx_ant.position.z() - tx_ant.position.z()
            };
            dir.normalizeInplace();
            
            rm::Transform Tas;
            Tas.R = rm::Quaternion::fromTwoVectors(front, dir);
            Tas.t = {tx_ant.position.x(), tx_ant.position.y(), tx_ant.position.z()};

            for(size_t sample_id = 0; sample_id < m_params.model.n_samples; sample_id++) {
                waves.dirs[sample_id] = Tas.R * ray_dirs_local[sample_id];
                waves.origs[sample_id] = {0.0, 0.0, 0.0};
                wave_attributes[sample_id] = wave_att_ex;
            }

            // Transfer to GPU
            rm::OnDnModel_<rm::VRAM_CUDA> waves_gpu;
            waves_gpu.width = waves.width;
            waves_gpu.height = waves.height;
            waves_gpu.range = waves.range;
            waves_gpu.origs = waves.origs;
            waves_gpu.dirs = waves.dirs;

            rm::Memory<DirectedWaveAttributes, rm::VRAM_CUDA> wave_attributes_gpu = wave_attributes;
            
            // Create simulator
            auto sim = std::make_shared<rm::OnDnSimulatorOptix>(m_map);
            sim->setTsb(rm::Transform::Identity());
            sim->preBuildProgram<rm::Bundle<rm::Hits<>, rm::Ranges<>, rm::Normals<>, rm::ObjectIds<>>>();
            sim->setModel(waves_gpu);

            rm::Transform Tsm = Tsm_last;
            rm::Memory<rm::Transform> Tsms(1);
            Tsms[0] = Tsm;

            // Simulate reflections
            using ResT = rm::Bundle<rm::Hits<>, rm::Ranges<>, rm::Normals<>, rm::ObjectIds<>>;
            ResT results;
            rm::resize_memory_bundle<rm::VRAM_CUDA>(results, waves_gpu.width, waves_gpu.height, 1);
            
            sim->simulate(Tsms, results);
            cudaDeviceSynchronize();

            // Process results
            move_waves(
                waves_gpu.origs,
                waves_gpu.dirs,
                wave_attributes_gpu,
                results.ranges, 
                results.hits);
            
            rm::Memory<Signal, rm::VRAM_CUDA> signals(waves_gpu.size());
            signal_shader(
                materials_gpu,
                object_materials_gpu,
                m_material_id_air,
                waves_gpu.dirs,
                wave_attributes_gpu,
                results.hits,
                results.normals,
                results.object_ids,
                signals
            );

            // Transfer signals back to CPU
            rm::Memory<Signal> signals_cpu = signals;
            rm::Memory<uint8_t> hits_cpu = results.hits;

            // Create range profile for this antenna pair
            cv::Mat slice = cv::Mat::zeros(m_cfg.n_cells, 1, CV_32FC1);
            
            // Convert signals to range bins
            for(size_t sample_id = 0; sample_id < m_params.model.n_samples; sample_id++) {
                if(hits_cpu[sample_id]) {
                    auto signal = signals_cpu[sample_id];
                    float signal_dist = signal.time * wave_att_ex.velocity / 2.0f;
                    int cell = static_cast<int>(signal_dist / m_cfg.resolution);
                    
                    if(cell >= 0 && cell < m_cfg.n_cells) {
                        // Apply phase information to complex signal
                        std::complex<float> sig_val = 
                            tx_signals[tx][cell] * 
                            std::polar(signal.strength, signal.phase_shift);
                        
                        slice.at<float>(cell) += std::abs(sig_val);
                    }
                }
            }

            // Store in polar image
            slice.copyTo(polar_image.col(virt_idx));
        }
    }

    // Process signals
    applyMIMOSeparation(polar_image);
    applyBeamforming(polar_image);

    // Normalize and convert to 8-bit
    double max_val;
    cv::minMaxLoc(polar_image, nullptr, &max_val);
    polar_image *= m_cfg.signal_max / max_val;
    
    cv::Mat output_image;
    polar_image.convertTo(output_image, CV_8UC1);
    
    auto msg = cv_bridge::CvImage(
        std_msgs::msg::Header(), 
        "mono8", 
        output_image).toImageMsg();
    
    msg->header.stamp = stamp;
    msg->header.frame_id = m_sensor_frame;

    RCLCPP_DEBUG(m_node->get_logger(), "GPU Simulation took %.3f ms", sw()*1000.0);

    return msg;
}

} // namespace radarays_ros