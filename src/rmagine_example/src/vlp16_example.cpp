#include <rmagine/map/EmbreeMap.hpp>
#include <rmagine/types/sensor_models.h>
#include <rmagine/types/sensors.h>
#include <rmagine/simulation/SphereSimulatorEmbree.hpp>
#include <iostream>

namespace rm = rmagine;

int main(int argc, char ** argv)
{
    std::string path_to_mesh = "/home/ldj-rv/ros_ws/src/rmagine_example/meshes/sphere.ply";
    rm::EmbreeMapPtr map = rm::import_embree_map(path_to_mesh);
    
    rm::SphericalModel sensor_model = rm::vlp16_900();
    
    rm::SphereSimulatorEmbree sim;
    sim.setModel(sensor_model);
    sim.setMap(map);
    
    rm::Memory<rm::Transform, rm::RAM> Tbm(100);
    for(size_t i=0; i < Tbm.size(); i++)
    {
        rm::Transform T = rm::Transform::Identity();
        T.t = {2.0, 0.0, 0.0};
        rm::EulerAngles e = {0.0, 0.0, 0.0};
        T.R.set(e);
        Tbm[i] = T;
    }
    
    using ResultT = rm::Bundle<
        rm::Hits<rm::RAM>, 
        rm::Ranges<rm::RAM>
    >;

    ResultT result = sim.simulate<ResultT>(Tbm);
    
    std::cout << "First ray's range: " << result.ranges[0] << std::endl;

     // Print all rays' ranges from the fifth scan
    auto ranges5 = result.ranges(5 * sensor_model.size(), 6 * sensor_model.size());
    std::cout << "--- All ranges of the fifth scan ---" << std::endl;
    for(size_t i = 0; i < ranges5.size(); i++)
    {
       std::cout << "Ray[" << i << "] range: " << ranges5[i] << std::endl;
}

    
    return 0;
}

