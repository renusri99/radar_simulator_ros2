#include <radarays_ros/ros_helper.hpp>
#include <rclcpp/rclcpp.hpp>

namespace radarays_ros
{

RadarMaterial loadRadarMaterialFromParams(const rclcpp::Node::SharedPtr& node, const std::string& prefix)
{
    RadarMaterial mat;
    node->get_parameter_or(prefix + ".velocity", mat.velocity, 0.0);
    node->get_parameter_or(prefix + ".ambient", mat.ambient, 0.0);
    node->get_parameter_or(prefix + ".diffuse", mat.diffuse, 0.0);
    node->get_parameter_or(prefix + ".specular", mat.specular, 0.0);
    return mat;
}

RadarMaterials loadRadarMaterialsFromNode(const rclcpp::Node::SharedPtr& node)
{
    RadarMaterials materials;
    std::vector<std::string> material_ids;

    if (!node->get_parameter("materials", material_ids)) {
        RCLCPP_WARN(node->get_logger(), "No material list found under 'materials' parameter.");
        return materials;
    }

    for (const auto& id : material_ids) {
        RadarMaterial mat = loadRadarMaterialFromParams(node, "material_properties." + id);
        materials.data.push_back(mat);
    }

    return materials;
}

} // namespace radarays_ros
