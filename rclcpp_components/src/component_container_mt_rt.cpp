#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "rclcpp_components/component_manager_rt.hpp"

int main(int argc, char * argv[])
{
    /// Component container with a multi-threaded RT executor.
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::executors::RTMultiThreadedExecutor> exec;
    auto node = std::make_shared<rclcpp_components::ComponentManagerRT>();
    if (!node->has_parameter("trace_file_path")){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "No trace file path has been set as Component "
        "Manager RT parameter. Set the parameter in your launch file or use normal " 
        "Component Manager for your composition");
    }
    if (node->has_parameter("sched_priority")) {
        const auto sched_priority = node->get_parameter("sched_priority").as_int();
        exec = std::make_shared<rclcpp::executors::RTMultiThreadedExecutor>(
            rclcpp::ExecutorOptions(), 0, sched_priority);
    } else {
        exec = std::make_shared<rclcpp::executors::RTMultiThreadedExecutor>();
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "node launched with no specific sched priority");
    }
    node->set_executor(exec);
    exec->add_node(node);
    exec->spin();
}
