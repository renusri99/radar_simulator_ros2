#include "radarays_ros/RadarCPU.hpp"

#include <cv_bridge/cv_bridge.h>
#include <omp.h>
#include <random>
#include <complex>
#include <algorithm>

#include <radarays_ros/radar_algorithms.h>
#include <radarays_ros/image_algorithms.h>
#include <rmagine/simulation/OnDnSimulatorEmbree.hpp>
#include <rmagine/types/Memory.hpp>
#include <rmagine/types/Bundle.hpp>
#include <rmagine/util/fresnel.hpp>
#include <rmagine/util/StopWatch.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace rm = rmagine;
using namespace std::complex_literals;

namespace radarays_ros
{

RadarCPU::RadarCPU(
    std::shared_ptr<rclcpp::Node> node,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::shared_ptr<tf2_ros::TransformListener> tf_listener,
    std::string map_frame,
    std::string sensor_frame,
    rm::EmbreeMapPtr map)
: Radar(node, tf_buffer, tf_listener, map_frame, sensor_frame),
  m_map(map),
  m_wavelength(0.0039f) // 76.5GHz wavelength
{
    // Initialize simulator
    m_simulator = std::make_shared<rm::OnDnSimulatorEmbree>(m_map);
    m_simulator->setTsb(rm::Transform::Identity());

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
    
    // Initialize denoising weights
    updateDenoisingWeights();
}

void RadarCPU::updateDenoisingWeights()
{
    m_denoising_weights.clear();
    m_denoising_mode = 0;

    if(m_cfg.signal_denoising > 0) {
        if(m_cfg.signal_denoising == 1) {
            // Triangular denoising
            m_denoising_mode = m_cfg.signal_denoising_triangular_mode * 
                              m_cfg.signal_denoising_triangular_width;
            m_denoising_weights = make_denoiser_triangular(
                m_cfg.signal_denoising_triangular_width,
                m_denoising_mode
            );
        } else if(m_cfg.signal_denoising == 2) {
            // Gaussian denoising
            m_denoising_weights = make_denoiser_gaussian(
                m_cfg.signal_denoising_gaussian_width,
                m_cfg.signal_denoising_gaussian_mode
            );
        } else if(m_cfg.signal_denoising == 3) {
            // Maxwell-Boltzmann denoising
            m_denoising_weights = make_denoiser_maxwell_boltzmann(
                m_cfg.signal_denoising_mb_width,
                m_cfg.signal_denoising_mb_mode
            );
        }

        // Normalize weights
        if(!m_denoising_weights.empty()) {
            float mode_val = m_denoising_weights[m_denoising_mode];
            for(auto& weight : m_denoising_weights) {
                weight /= mode_val;
            }
        }
    }
}

void RadarCPU::initializeMIMOArray()
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

void RadarCPU::generateOrthogonalCodes()
{
    // Generate Hadamard codes for orthogonal transmission
    m_orthogonal_codes.resize(m_cfg.num_tx, std::vector<std::complex<float>>(m_cfg.code_length));
    
    // Basic Hadamard matrix generation
    int n = m_cfg.code_length;
    cv::Mat hadamard = cv::Mat::ones(1, 1, CV_32FC1);
    
    for(int k = 1; k < n; k *= 2) {
        cv::Mat tmp = cv::Mat::zeros(2*k, 2*k, CV_32FC1);
        hadamard.copyTo(tmp(cv::Rect(0, 0, k, k)));
        hadamard.copyTo(tmp(cv::Rect(k, k, k, k)));
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

std::vector<std::complex<float>> RadarCPU::generateWaveform(int tx_id)
{
    std::vector<std::complex<float>> waveform(m_cfg.n_cells, 0.0f);
    
    // Spread the orthogonal code across all range bins
    for(int i = 0; i < m_cfg.n_cells; ++i) {
        int code_idx = i % m_cfg.code_length;
        waveform[i] = m_orthogonal_codes[tx_id][code_idx];
    }
    
    return waveform;
}

std::vector<Signal> RadarCPU::simulateReflections(
    const DirectedWave& wave,
    const rm::OnDnSimulatorEmbreePtr& sim)
{
    std::vector<Signal> signals;
    std::vector<DirectedWave> waves = { wave };

    for(size_t pass_id = 0; pass_id < m_cfg.n_reflections; ++pass_id) {
        rm::OnDnModel model = make_model(waves);
        sim->setModel(model);

        rm::Memory<rm::Transform> T(1);
        T[0] = rm::Transform::Identity();

        using ResT = rm::Bundle<rm::Hits<>, rm::Ranges<>, rm::Normals<>, rm::ObjectIds<>>;
        ResT results;
        results.hits.resize(model.size());
        results.ranges.resize(model.size());
        results.normals.resize(model.size());
        results.object_ids.resize(model.size());

        sim->simulate(T, results);

        std::vector<DirectedWave> next_waves;

        for(size_t i = 0; i < waves.size(); ++i) {
            const float range = results.ranges[i];
            if(range <= 0.0f || range > 1000.0f) continue;

            const rm::Vector normal = results.normals[i].normalize();
            const unsigned obj_id = results.object_ids[i];
            if(obj_id > 10000) continue;

            DirectedWave incidence = waves[i].move(range);
            DirectedWave reflection = incidence;
            DirectedWave refraction = incidence;

            // Handle material transition
            refraction.material_id = (incidence.material_id == m_material_id_air) 
                ? m_object_materials[obj_id] 
                : m_material_id_air;

            const float v_refract = m_params.materials.data[refraction.material_id].velocity;
            auto fresnel_result = fresnel(normal, incidence, v_refract);

            // Process reflection
            reflection.ray.dir = fresnel_result.first.ray.dir;
            reflection.energy = fresnel_result.first.energy;
            
            if(reflection.energy > m_wave_energy_threshold) {
                // Calculate total path length and phase shift
                const float total_path = incidence.time * wave.velocity;
                const float phase_shift = 2 * M_PI * total_path / m_wavelength;
                
                // Create signal for direct path
                if(pass_id == 0 || m_cfg.record_multi_reflection) {
                    signals.push_back({
                        total_path * 2.0f, // Round trip time
                        reflection.energy,
                        phase_shift
                    });
                }

                // Create signals for multipath
                if(m_cfg.record_multi_path && pass_id > 0) {
                    rm::Vector dir_sensor_to_hit = reflection.ray.orig;
                    const double distance_to_sensor = dir_sensor_to_hit.l2norm();
                    dir_sensor_to_hit.normalizeInplace();
                    
                    const double sensor_view_scalar = wave.ray.dir.dot(dir_sensor_to_hit);
                    if(sensor_view_scalar > m_cfg.multipath_threshold) {
                        const double time_to_sensor = distance_to_sensor / reflection.velocity;
                        const double angle = angle_between(-reflection.ray.dir, dir_sensor_to_hit);
                        
                        auto material = m_params.materials.data[refraction.material_id];
                        double multipath_energy = back_reflection_shader(
                            angle,
                            reflection.energy,
                            material.ambient,
                            material.diffuse,
                            material.specular
                        );
                        
                        signals.push_back({
                            incidence.time + time_to_sensor,
                            multipath_energy,
                            phase_shift
                        });
                    }
                }

                next_waves.push_back(reflection);
            }

            // Process refraction
            refraction.ray.dir = fresnel_result.second.ray.dir;
            refraction.energy = fresnel_result.second.energy;
            
            if(refraction.energy > m_wave_energy_threshold) {
                refraction.moveInplace(0.001f);
                next_waves.push_back(refraction);
            }
        }

        waves = next_waves;
    }

    return signals;
}

void RadarCPU::applyAmbientNoise(cv::Mat& slice, int angle_id)
{
    if(m_cfg.ambient_noise <= 0) return;

    std::random_device rand_dev;
    std::mt19937 gen(rand_dev());
    std::uniform_real_distribution<float> dist_uni(0.0, 1.0);

    // Find max signal for normalization
    double max_val = 0.0;
    cv::minMaxLoc(slice, nullptr, &max_val);

    // Apply noise
    for(int r = 0; r < slice.rows; ++r) {
        float signal = slice.at<float>(r);
        float p;

        if(m_cfg.ambient_noise == 1) { // Uniform noise
            p = dist_uni(gen);
        } else { // Perlin noise (default)
            double scale = 0.05;
            double scale2 = 0.2;
            double random_begin = dist_uni(gen) * 1000.0;
            
            double p_perlin1 = perlin_noise(
                random_begin + r * scale, 
                angle_id * scale);
            
            double p_perlin2 = perlin_noise(
                random_begin + r * scale2, 
                angle_id * scale2);

            p = 0.9 * p_perlin1 + 0.1 * p_perlin2;
        }

        // Noise amplitude varies with signal strength
        float signal_norm = 1.0f - (signal / max_val);
        float signal_weight = std::pow(signal_norm, 4.0f);
        
        float noise_amp = (signal_weight * m_cfg.ambient_noise_at_signal_0 + 
                         (1.0f - signal_weight) * m_cfg.ambient_noise_at_signal_1) * max_val;

        // Distance-dependent noise
        float x = (r + 0.5f) * m_cfg.resolution;
        float distance_atten = (m_cfg.ambient_noise_energy_max - m_cfg.ambient_noise_energy_min) * 
                             std::exp(-m_cfg.ambient_noise_energy_loss * x) + 
                             m_cfg.ambient_noise_energy_min;

        float y_noise = noise_amp * p * distance_atten;
        slice.at<float>(r) = signal + std::abs(y_noise);
    }
}

void RadarCPU::applyDenoising(cv::Mat& slice)
{
    if(m_denoising_weights.empty()) return;

    cv::Mat denoised = cv::Mat::zeros(slice.size(), CV_32FC1);
    
    for(int r = 0; r < slice.rows; ++r) {
        for(size_t vid = 0; vid < m_denoising_weights.size(); ++vid) {
            int glob_id = vid + r - m_denoising_mode;
            if(glob_id >= 0 && glob_id < slice.rows) {
                denoised.at<float>(glob_id) += 
                    slice.at<float>(r) * m_denoising_weights[vid];
            }
        }
    }
    
    denoised.copyTo(slice);
}

void RadarCPU::applyMIMOSeparation(cv::Mat& polar_image)
{
    cv::Mat separated_signals = cv::Mat::zeros(m_cfg.n_cells, m_virtual_array_size, CV_32FC1);
    
    // For each virtual antenna (TX-RX pair)
    for(int v = 0; v < m_virtual_array_size; ++v) {
        int tx_id = v / m_cfg.num_rx;
        
        // Correlate with TX's orthogonal code
        for(int r = 0; r < m_cfg.n_cells; ++r) {
            int code_idx = r % m_cfg.code_length;
            separated_signals.at<float>(r, v) = 
                polar_image.at<float>(r, v) * std::real(m_orthogonal_codes[tx_id][code_idx]);
        }
    }
    
    // Apply matched filtering (code compression)
    cv::Mat compressed;
    cv::reduce(separated_signals, compressed, 1, cv::REDUCE_SUM);
    
    // Update polar image with separated signals
    polar_image = cv::repeat(compressed, 1, m_virtual_array_size);
}

void RadarCPU::applyBeamforming(cv::Mat& polar_image)
{
    cv::Mat beams = cv::Mat::zeros(polar_image.rows, 180, CV_32FC1);
    
    // Beamform for each angle
    for(int angle = -90; angle < 90; angle++) {
        const float theta = angle * M_PI / 180.0f;
        const int beam_col = angle + 90;
        
        // Calculate steering vector
        std::vector<std::complex<float>> weights(m_virtual_array_size);
        for(int i = 0; i < m_virtual_array_size; i++) {
            float phase = 2 * M_PI * m_antennas[i].position.y() * sin(theta) / m_wavelength;
            weights[i] = std::polar(1.0f, -phase);
        }
        
        // Apply to all range bins
        for(int r = 0; r < polar_image.rows; r++) {
            std::complex<float> sum(0,0);
            for(int a = 0; a < polar_image.cols; a++) {
                sum += polar_image.at<float>(r,a) * weights[a];
            }
            beams.at<float>(r, beam_col) = std::abs(sum);
        }
    }
    
    polar_image = beams;
}

sensor_msgs::msg::Image::SharedPtr RadarCPU::simulate(rclcpp::Time stamp)
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

    // Simulate all TX-RX paths
    #pragma omp parallel for collapse(2)
    for(int tx = 0; tx < m_cfg.num_tx; tx++) {
        for(int rx = 0; rx < m_cfg.num_rx; rx++) {
            int virt_idx = tx * m_cfg.num_rx + rx;
            
            // Configure base wave
            DirectedWave wave;
            wave.energy = 1.0f;
            wave.polarization = 0.5f;
            wave.frequency = 76.5f; // GHz
            wave.velocity = 0.3f;   // m/ns
            wave.material_id = m_material_id_air;
            wave.time = 0.0f;
            
            // Set antenna positions
            const auto& tx_ant = m_antennas[tx * m_cfg.num_rx];
            const auto& rx_ant = m_antennas[tx * m_cfg.num_rx + rx];
            
            wave.ray.orig = {tx_ant.position.x(), tx_ant.position.y(), tx_ant.position.z()};
            rm::Vector dir = {
                rx_ant.position.x() - tx_ant.position.x(),
                rx_ant.position.y() - tx_ant.position.y(),
                rx_ant.position.z() - tx_ant.position.z()
            };
            dir.normalizeInplace();
            wave.ray.dir = dir;

            // Simulate reflections
            auto signals = simulateReflections(wave, m_simulator);
            
            // Create range profile for this antenna pair
            cv::Mat slice = cv::Mat::zeros(m_cfg.n_cells, 1, CV_32FC1);
            
            // Convert signals to range bins
            for(const auto& signal : signals) {
                float signal_dist = signal.time * wave.velocity / 2.0f;
                int cell = static_cast<int>(signal_dist / m_cfg.resolution);
                
                if(cell >= 0 && cell < m_cfg.n_cells) {
                    // Apply phase information to complex signal
                    std::complex<float> sig_val = 
                        tx_signals[tx][cell] * 
                        std::polar(signal.strength, signal.phase_shift);
                    
                    slice.at<float>(cell) += std::abs(sig_val);
                }
            }
            
            // Apply denoising to this antenna pair's signal
            applyDenoising(slice);
            
            // Apply ambient noise
            applyAmbientNoise(slice, virt_idx);
            
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

    RCLCPP_DEBUG(m_node->get_logger(), "Simulation took %.3f ms", sw()*1000.0);

    return msg;
}

} // namespace radarays_ros