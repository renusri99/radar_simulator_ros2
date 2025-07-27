#include <rclcpp/rclcpp.hpp>

#include <mesh_msgs/msg/mesh_geometry.hpp>
#include <mesh_msgs/msg/mesh_geometry_stamped.hpp>
#include <mesh_msgs/msg/triangle_indices.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/header.hpp>

#include <rmagine/map/EmbreeMap.hpp>
#include <rmagine/math/types.h>

#include <unordered_map>
#include <sstream>

using namespace rmagine;

rm::Transform pre_transform;

mesh_msgs::msg::MeshGeometry embreeToRos(rm::EmbreeMeshPtr mesh, rm::Matrix4x4 T)
{
    mesh_msgs::msg::MeshGeometry mesh_ros;

    auto vertices = mesh->verticesTransformed();
    for (const auto& v : vertices)
    {
        auto vt = T * v;
        geometry_msgs::msg::Point pt;
        pt.x = vt.x;
        pt.y = vt.y;
        pt.z = vt.z;
        mesh_ros.vertices.push_back(pt);
    }

    auto faces = mesh->faces();
    for (const auto& f : faces)
    {
        mesh_msgs::msg::TriangleIndices face_msg;
        face_msg.vertex_indices[0] = f.v0;
        face_msg.vertex_indices[1] = f.v1;
        face_msg.vertex_indices[2] = f.v2;
        mesh_ros.faces.push_back(face_msg);
    }

    return mesh_ros;
}

std::unordered_map<unsigned int, mesh_msgs::msg::MeshGeometry> embreeToRos(
    rm::EmbreeScenePtr scene, 
    rm::Matrix4x4 T = rm::Matrix4x4::Identity())
{
    std::unordered_map<unsigned int, mesh_msgs::msg::MeshGeometry> result;

    for (const auto& [geom_id, geom] : scene->geometries())
    {
        auto inst = std::dynamic_pointer_cast<rm::EmbreeInstance>(geom);
        if (inst)
        {
            rm::Matrix4x4 M = rm::compose(pre_transform, rm::Vector3{1.0, 1.0, 1.0});
            rm::Matrix4x4 T_ = T * M * inst->matrix();

            auto submeshes = embreeToRos(inst->scene(), T_);
            result.insert(submeshes.begin(), submeshes.end());
        }
        else
        {
            auto mesh = std::dynamic_pointer_cast<rm::EmbreeMesh>(geom);
            if (mesh)
            {
                unsigned int mesh_id = mesh->id(scene);
                rm::Matrix4x4 M = rm::compose(pre_transform, rm::Vector3{1.0, 1.0, 1.0});
                result[mesh_id] = embreeToRos(mesh, T * M);
            }
        }
    }

    return result;
}


class MeshPublisherNode : public rclcpp::Node
{
public:
    MeshPublisherNode()
    : Node("mesh_publisher")
    {
        declare_parameter<std::string>("file", "/tmp/mesh.ply");
        declare_parameter<std::string>("frame", "map");
        declare_parameter<double>("publish_freq", 1.0);
        declare_parameter<std::vector<double>>("pre_transform", {});

        get_parameter("file", meshfile_);
        get_parameter("frame", map_frame_);
        get_parameter("publish_freq", publish_freq_);
        get_parameter("pre_transform", transform_params_);

        pre_transform = rm::Transform::Identity();
        if (transform_params_.size() == 6)
        {
            pre_transform.t = {transform_params_[0], transform_params_[1], transform_params_[2]};
            pre_transform.R = rm::EulerAngles{
                transform_params_[3],
                transform_params_[4],
                transform_params_[5]
            };
        }
        else if (transform_params_.size() == 7)
        {
            pre_transform.t = {transform_params_[0], transform_params_[1], transform_params_[2]};
            pre_transform.R = rm::Quaternion{
                transform_params_[3],
                transform_params_[4],
                transform_params_[5],
                transform_params_[6]
            };
        }

        map_ = rm::import_embree_map(meshfile_);
        auto meshes = embreeToRos(map_->scene);

        for (const auto& [mesh_id, mesh] : meshes)
        {
            auto topic_name = "mesh/" + std::to_string(mesh_id);

            auto pub = create_publisher<mesh_msgs::msg::MeshGeometryStamped>(topic_name, 10);

            mesh_msgs::msg::MeshGeometryStamped msg;
            msg.mesh_geometry = mesh;
            msg.header.frame_id = map_frame_;
            msg.uuid = topic_name;

            mesh_messages_[mesh_id] = msg;
            mesh_publishers_[mesh_id] = pub;
        }

        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / publish_freq_),
            std::bind(&MeshPublisherNode::publishMeshes, this));

        RCLCPP_INFO(get_logger(), "MeshPublisher initialized with %lu meshes.", mesh_publishers_.size());
    }

private:
    void publishMeshes()
    {
        rclcpp::Time now = get_clock()->now();
        for (const auto& [id, pub] : mesh_publishers_)
        {
            auto msg = mesh_messages_[id];
            msg.header.stamp = now;
            pub->publish(msg);
        }
    }

    std::string map_frame_;
    std::string meshfile_;
    double publish_freq_;
    std::vector<double> transform_params_;

    rm::EmbreeMapPtr map_;

    std::unordered_map<unsigned int, rclcpp::Publisher<mesh_msgs::msg::MeshGeometryStamped>::SharedPtr> mesh_publishers_;
    std::unordered_map<unsigned int, mesh_msgs::msg::MeshGeometryStamped> mesh_messages_;

    rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MeshPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
