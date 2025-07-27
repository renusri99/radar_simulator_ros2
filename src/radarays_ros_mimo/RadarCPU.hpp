#ifndef RADARAYS_ROS_RADARCPU_HPP
#define RADARAYS_ROS_RADARCPU_HPP

#include "radarays_ros/Radar.hpp"
#include <rmagine/simulation/OnDnSimulatorEmbree.hpp>
#include <rmagine/types/sensor_models.h>
#include <rmagine/map/EmbreeMap.hpp>
#include <rmagine/util/StopWatch.hpp>
#include <Eigen/Core>

#include <opencv2/core/core.hpp>
#include <vector>
#include <complex>
#include <memory>
#include <unordered_map>
#include <random>

// Forward declarations
namespace rmagine {
struct DirectedWave;
struct Transform;
template<typename> struct Memory;
} // namespace rmagine

namespace radarays_ros
{

struct Signal;

class RadarCPU : public Radar
{
public:
    RadarCPU(
        std::shared_ptr<rclcpp::Node> node,
        std::shared_ptr<tf2_ros::Buffer> tf_buffer,
        std::shared_ptr<tf2_ros::TransformListener> tf_listener,
        std::string map_frame,
        std::string sensor_frame,
        rmagine::EmbreeMapPtr map);

    sensor_msgs::msg::Image::SharedPtr simulate(rclcpp::Time stamp) override;

protected:
    // Configuration structure
    struct Config {
        int num_tx = 3;
        int num_rx = 4;
        int n_cells = 512;
        double resolution = 0.05;
        double signal_max = 255.0;
        double energy_max = 1.0;
        int code_length = 8;
        
        // Material and reflection
        int n_reflections = 3;
        bool include_motion = false;
        bool record_multi_reflection = true;
        bool record_multi_path = false;
        double multipath_threshold = 0.7;
        
        // Denoising
        int signal_denoising = 1; // 0-off, 1-triangular, 2-gaussian, 3-maxwell-boltzmann
        int signal_denoising_triangular_width = 5;
        int signal_denoising_triangular_mode = 2;
        int signal_denoising_gaussian_width = 5;
        int signal_denoising_gaussian_mode = 2;
        int signal_denoising_mb_width = 5;
        int signal_denoising_mb_mode = 2;
        
        // Noise
        int ambient_noise = 2; // 0-off, 1-uniform, 2-perlin
        double ambient_noise_at_signal_0 = 0.1;
        double ambient_noise_at_signal_1 = 0.01;
        double ambient_noise_energy_max = 0.2;
        double ambient_noise_energy_min = 0.01;
        double ambient_noise_energy_loss = 0.1;
    };

    // Antenna configuration
    struct AntennaConfig {
        Eigen::Vector3f position;
        bool is_transmitter;
    };

    // Signal with phase information
    struct Signal {
        float time;
        float strength;
        float phase_shift;
    };

private:
    // Core components
    rmagine::EmbreeMapPtr m_map;
    std::shared_ptr<rmagine::OnDnSimulatorEmbree> m_simulator;
    Config m_cfg;
    
    // Radar properties
    float m_wavelength = 0.0039f; // 76.5GHz wavelength
    unsigned int m_material_id_air = 0;
    float m_wave_energy_threshold = 1e-6f;
    
    // MIMO configuration
    std::vector<AntennaConfig> m_antennas;
    std::vector<std::vector<std::complex<float>>> m_orthogonal_codes;
    int m_virtual_array_size;
    
    // Signal processing
    std::vector<float> m_denoising_weights;
    int m_denoising_mode = 0;
    std::unordered_map<unsigned int, unsigned int> m_object_materials;

    // Initialization
    void initializeMIMOArray();
    void generateOrthogonalCodes();
    void updateDenoisingWeights();

    // Core simulation
    std::vector<std::complex<float>> generateWaveform(int tx_id);
    std::vector<Signal> simulateReflections(
        const rmagine::DirectedWave& wave,
        const std::shared_ptr<rmagine::OnDnSimulatorEmbree>& sim);
    
    // Processing pipeline
    void applyAmbientNoise(cv::Mat& slice, int angle_id);
    void applyDenoising(cv::Mat& slice);
    void applyMIMOSeparation(cv::Mat& polar_image);
    void applyBeamforming(cv::Mat& polar_image);

    // Helpers
    rmagine::OnDnModel make_model(const std::vector<rmagine::DirectedWave>& waves);
    bool updateTsm(rclcpp::Time stamp = rclcpp::Time(0));
    
    // Noise generation
    double perlin_noise(double x, double y);
    double back_reflection_shader(
        double incidence_angle,
        double energy,
        double ambient,
        double diffuse,
        double specular);
};

} // namespace radarays_ros

#endif // RADARAYS_ROS_RADARCPU_HPP