

#include <chrono>
#include <memory>

#include "common/logger.h"
#include "rclcpp/rclcpp.hpp"
#include "chassis/node_manager.h"

#define APP_NAME "naiad_chassis"
#define APP_VERSION  "v0.1.0"


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;

    auto manager = std::make_shared<naiad::chassis::NodeManager>(APP_NAME);
    slog::info("program started, version: {}, build time: {} {}", APP_VERSION,  __DATE__, __TIME__);

    manager->bind(executor);

    manager->start();

    executor.spin();

    manager->stop();

    rclcpp::shutdown();
    return 0;
}
