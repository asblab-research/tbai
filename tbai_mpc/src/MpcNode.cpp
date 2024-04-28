#include <ocs2_anymal_mpc/AnymalInterface.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_quadruped_interface/QuadrupedMpc.h>
#include <ocs2_quadruped_interface/QuadrupedMpcNode.h>
#include <ros/init.h>

int main(int argc, char *argv[]) {
    // Initialize ros node
    ros::init(argc, argv, "anymal_mpc");
    ros::NodeHandle nodeHandle;

    // Description name
    std::string descriptionName = "robot_description";

    // Anymal urdf
    std::string urdfString;
    nodeHandle.getParam(descriptionName, urdfString);

    // Task settings
    std::string taskSettingsFile = "/home/kuba/fun/ocs2_project/src/ocs2_fun/ocs2_anymal_robot/config/task.info";

    // Frame declarations
    std::string frameDeclarationFile =
        "/home/kuba/fun/ocs2_project/src/ocs2_fun/ocs2_anymal_robot/config/frame_declarations.info";

    // SQP settings
    std::string sqpSettingsFile = "/home/kuba/fun/ocs2_project/src/ocs2_fun/ocs2_anymal_robot/config/sqp.info";

    auto anymalInterface =
        anymal::getAnymalInterface(urdfString, switched_model::loadQuadrupedSettings(taskSettingsFile),
                                   anymal::frameDeclarationFromFile(frameDeclarationFile));
    const auto mpcSettings = ocs2::mpc::loadSettings(taskSettingsFile);

    if (anymalInterface->modelSettings().algorithm_ == switched_model::Algorithm::SQP) {
        ROS_INFO_STREAM("[MpcNode] Using SQP MPC");
        const auto sqpSettings = ocs2::sqp::loadSettings(sqpSettingsFile);
        auto mpcPtr = switched_model::getSqpMpc(*anymalInterface, mpcSettings, sqpSettings);
        switched_model::quadrupedMpcNode(nodeHandle, *anymalInterface, std::move(mpcPtr));
    }

    if (anymalInterface->modelSettings().algorithm_ == switched_model::Algorithm::DDP) {
        ROS_INFO_STREAM("[MpcNode] Using DDP MPC");
        const auto ddpSettings = ocs2::ddp::loadSettings(taskSettingsFile);
        auto mpcPtr = switched_model::getDdpMpc(*anymalInterface, mpcSettings, ddpSettings);
        switched_model::quadrupedMpcNode(nodeHandle, *anymalInterface, std::move(mpcPtr));
    }

    return EXIT_SUCCESS;
}