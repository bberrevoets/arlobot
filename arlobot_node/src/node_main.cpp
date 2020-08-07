#include <chrono>
#include <memory>
#include <string>

#include <rcutils/cmdline_parser.h>
#include <rclcpp/rclcpp.hpp>

#include "arlobot_node/arlobot.hpp"

void help_print()
{
    printf("For arlobot node : \n");
    printf("arlobot_node [-i usb_port] [-h]\n");
    printf("options:\n");
    printf("-h : Print this help function.\n");
    printf("-i usb_port: Connected USB port with OpenCR.");
}

int main(int argc, char *argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    if (rcutils_cli_option_exist(argv, argv + argc, "-h"))
    {
        help_print();
        return 0;
    }

    rclcpp::init(argc, argv);

    std::string usb_port = "/dev/ttyACM0";
    char *cli_options;
    cli_options = rcutils_cli_get_option(argv, argv + argc, "-i");
    if (nullptr != cli_options)
    {
        usb_port = std::string(cli_options);
    }

    rclcpp::executors::SingleThreadedExecutor executor;

    auto arlobot = std::make_shared<berrevoets::arlobot::Arlobot>(usb_port);

    executor.add_node(arlobot);

    executor.spin();

    rclcpp::shutdown();

    return 0;
}
