#include "Logger.hpp"
#include "TjServer.hpp"
#include "angle.hpp"
#include <libtj/FxRobot.h>
#include <libtj/MarvinSDK.h>
#include <mutex>
#include <stdexcept>
#include <unistd.h>

namespace arms {

TjArmServer::TjArmServer(const std::string &robot_ip) {
    // Initialize robot configuration file
    char exe_path[1024];
    ssize_t count = readlink("/proc/self/exe", exe_path, sizeof(exe_path));
    if (count == -1 || count == sizeof(exe_path)) {
        throw std::runtime_error("Failed to get executable path");
    }
    std::string exe_dir(exe_path, count);
    std::string config_path =
        exe_dir.substr(0, exe_dir.find_last_of('/')) + "/ccs_m6.MvKDCfg";
    LOG_INFO("Loading robot arms configuration from %s", config_path.c_str());
    if (!TjArmServer::initializedKineParams(config_path)) {
        throw std::runtime_error(
            "Failed to initialize robot arms configuration");
    }

    // Connect to the robot arms
    LOG_INFO("Connecting robot arms at %s", robot_ip.c_str());
    if (!TjArmServer::connectToRobot(robot_ip)) {
        throw std::runtime_error("Failed to connect to robot arms");
    }
}

TjArmServer::~TjArmServer() { OnRelease(); }

grpc::Status
TjArmServer::GetJointPosition(grpc::ServerContext *context,
                              const GetJointPositionRequest *request,
                              GetJointPositionResponse *response) {
    // Check if robot index is valid
    int robot_index = request->robot_index();
    if (robot_index < 0 || robot_index >= 2) {
        response->set_success(false);
        response->set_error_message("无效的机器人索引");
        return grpc::Status::OK;
    }

    // Get robot status
    static DCSS dcss;
    if (!OnGetBuf(&dcss)) {
        response->set_success(false);
        response->set_error_message("获取机器人状态失败");
        return grpc::Status::OK;
    }

    // Fill joint positions in response
    for (int i = 0; i < 7; ++i) {
        response->add_joint_positions(
            angle::degreesToRadians(dcss.m_Out[robot_index].m_FB_Joint_Pos[i]));
    }

    response->set_success(true);
    return grpc::Status::OK;
}

grpc::Status
TjArmServer::GetCartesianPosition(grpc::ServerContext *context,
                                  const GetCartesianPositionRequest *request,
                                  GetCartesianPositionResponse *response) {
    // Check if robot index is valid
    int robot_index = request->robot_index();
    if (robot_index < 0 || robot_index >= 2) {
        response->set_success(false);
        response->set_error_message("无效的机器人索引");
        return grpc::Status::OK;
    }

    // Get robot state
    static DCSS dcss;
    if (!OnGetBuf(&dcss)) {
        response->set_success(false);
        response->set_error_message("获取机器人状态失败");
        return grpc::Status::OK;
    }

    // Fill joint positions array
    double joints[7];
    for (int i = 0; i < 7; ++i) {
        joints[i] = dcss.m_Out[robot_index].m_FB_Joint_Pos[i];
    }

    // Convert to Cartesian position
    auto fk_result = TjArmServer::fk(robot_index, joints);
    if (!fk_result.has_value()) {
        response->set_success(false);
        response->set_error_message("动力学正解计算失败");
        return grpc::Status::OK;
    }

    // Fill Cartesian position in response
    auto trans = response->mutable_cartesian_pose()->mutable_translation();
    auto rot = response->mutable_cartesian_pose()->mutable_rotation();
    trans->set_x(fk_result.value()[0]);
    trans->set_y(fk_result.value()[1]);
    trans->set_z(fk_result.value()[2]);
    rot->set_rx(angle::degreesToRadians(fk_result.value()[3]));
    rot->set_ry(angle::degreesToRadians(fk_result.value()[4]));
    rot->set_rz(angle::degreesToRadians(fk_result.value()[5]));

    response->set_success(true);
    return grpc::Status::OK;
}

grpc::Status TjArmServer::JointMove(grpc::ServerContext *context,
                                    const JointMoveRequest *request,
                                    JointMoveResponse *response) {
    int robot_index = request->robot_index();
    if (robot_index < 0 || robot_index > 1) {
        response->set_success(false);
        response->set_error_message("无效的机器人手臂索引");
        return grpc::Status::OK;
    }

    if (request->joint_positions_size() != 7) {
        response->set_success(false);
        response->set_error_message("关节位置需要7个值");
        return grpc::Status::OK;
    }

    // Prepare joint position command
    double joints[7];
    for (int i = 0; i < 7; ++i) {
        joints[i] = angle::radiansToDegrees(request->joint_positions(i));
    }

    // If relative movement is requested, adjust target joints accordingly
    if (request->is_relative()) {
        // Get current joint positions
        static DCSS dcss;
        if (!OnGetBuf(&dcss)) {
            response->set_success(false);
            response->set_error_message("获取机器人状态失败");
            return grpc::Status::OK;
        }
        for (int i = 0; i < 7; ++i) {
            joints[i] += dcss.m_Out[robot_index].m_FB_Joint_Pos[i];
        }
    }

    // Send command to robot
    OnClearSet();
    if (robot_index == 0) {
        LEFT(OnSetJointCmdPos)(joints);
    } else {
        RIGHT(OnSetJointCmdPos)(joints);
    }
    OnSetSend();

    // Sleep for a short while to ensure motion has started
    usleep(100000);

    // Wait for motion to complete if blocking is requested
    if (request->is_block() &&
        !TjArmServer::waitForMotionComplete(robot_index)) {
        response->set_success(false);
        response->set_error_message("等待运动完成超时");
        return grpc::Status::OK;
    }

    response->set_success(true);
    return grpc::Status::OK;
}

grpc::Status TjArmServer::DualJointMove(grpc::ServerContext *context,
                                        const DualJointMoveRequest *request,
                                        DualJointMoveResponse *response) {
    // Check joint positions sizes
    if (request->left_joint_positions_size() != 7) {
        response->set_success(false);
        response->set_error_message("左臂需要7个关节位置");
        return grpc::Status::OK;
    }
    if (request->right_joint_positions_size() != 7) {
        response->set_success(false);
        response->set_error_message("右臂需要7个关节位置");
        return grpc::Status::OK;
    }

    double joints[2][7];
    for (int i = 0; i < 7; ++i) {
        joints[0][i] =
            angle::radiansToDegrees(request->left_joint_positions(i));
        joints[1][i] =
            angle::radiansToDegrees(request->right_joint_positions(i));
    }

    // If relative movement is requested, adjust target joints accordingly
    if (request->is_relative()) {
        // Get current joint positions
        static DCSS dcss;
        if (!OnGetBuf(&dcss)) {
            response->set_success(false);
            response->set_error_message("获取机器人状态失败");
            return grpc::Status::OK;
        }
        for (int i = 0; i < 7; ++i) {
            joints[0][i] += dcss.m_Out[0].m_FB_Joint_Pos[i];
            joints[1][i] += dcss.m_Out[1].m_FB_Joint_Pos[i];
        }
    }

    // Send commands to robot
    OnClearSet();
    LEFT(OnSetJointCmdPos)(joints[0]);
    RIGHT(OnSetJointCmdPos)(joints[1]);
    OnSetSend();

    // Sleep for a short while to ensure motion has started
    usleep(100000);

    // Wait for motion to complete if blocking is requested
    if (request->is_block() && !TjArmServer::waitForMotionComplete(-1)) {
        response->set_success(false);
        response->set_error_message("等待双臂运动完成超时");
        return grpc::Status::OK;
    }

    response->set_success(true);
    return grpc::Status::OK;
}

grpc::Status TjArmServer::CartesianMove(grpc::ServerContext *context,
                                        const CartesianMoveRequest *request,
                                        CartesianMoveResponse *response) {
    int robot_index = request->robot_index();
    if (robot_index < 0 || robot_index > 1) {
        response->set_success(false);
        response->set_error_message("无效的机器人手臂索引");
        return grpc::Status::OK;
    }

    // Construct target pose
    FX_DOUBLE xyzabc[6];
    xyzabc[0] = request->cartesian_pose().translation().x();
    xyzabc[1] = request->cartesian_pose().translation().y();
    xyzabc[2] = request->cartesian_pose().translation().z();
    xyzabc[3] =
        angle::radiansToDegrees(request->cartesian_pose().rotation().rx());
    xyzabc[4] =
        angle::radiansToDegrees(request->cartesian_pose().rotation().ry());
    xyzabc[5] =
        angle::radiansToDegrees(request->cartesian_pose().rotation().rz());

    // Use current joint position as IK reference
    static DCSS dcss;
    if (!OnGetBuf(&dcss)) {
        response->set_success(false);
        response->set_error_message("获取机器人状态失败");
        return grpc::Status::OK;
    }

    std::array<double, 7> current_joints{};
    for (int i = 0; i < 7; ++i) {
        current_joints[i] = dcss.m_Out[robot_index].m_FB_Joint_Pos[i];
    }

    // If relative movement is requested, adjust target pose accordingly
    if (request->is_relative()) {
        // Get current Cartesian position
        auto fk_result = TjArmServer::fk(robot_index, current_joints.data());
        if (!fk_result.has_value()) {
            response->set_success(false);
            response->set_error_message("动力学正解计算失败");
            return grpc::Status::OK;
        }
        // Adjust target pose
        for (int i = 0; i < 6; ++i) {
            xyzabc[i] += fk_result.value()[i];
        }
    }

    // Execute IK to get joint positions
    auto ik_result = TjArmServer::ik(robot_index, xyzabc, current_joints);
    if (!ik_result.has_value()) {
        response->set_success(false);
        response->set_error_message("笛卡尔运动逆解失败");
        return grpc::Status::OK;
    }

    // Prepare joint position command
    double joints[7];
    for (int i = 0; i < 7; ++i) {
        joints[i] = ik_result.value()[i];
    }

    // Send command to robot
    OnClearSet();
    if (robot_index == 0) {
        LEFT(OnSetJointCmdPos)(joints);
    } else {
        RIGHT(OnSetJointCmdPos)(joints);
    }
    OnSetSend();

    // Sleep for a short while to ensure motion has started
    usleep(100000);

    // Wait for motion to complete if blocking is requested
    if (request->is_block() &&
        !TjArmServer::waitForMotionComplete(robot_index)) {
        response->set_success(false);
        response->set_error_message("等待笛卡尔运动完成超时");
        return grpc::Status::OK;
    }

    response->set_success(true);
    return grpc::Status::OK;
}

grpc::Status
TjArmServer::DualCartesianMove(grpc::ServerContext *context,
                               const DualCartesianMoveRequest *request,
                               DualCartesianMoveResponse *response) {
    // TODO: Relative movement support
    if (request->is_relative()) {
        response->set_success(false);
        response->set_error_message("暂不支持双臂笛卡尔相对运动");
        LOG_ERROR("Relative movement currently not supported");
        return grpc::Status::OK;
    }

    // Target pose for left arm
    FX_DOUBLE xyzabc_left[6];
    xyzabc_left[0] = request->left_cartesian_pose().translation().x();
    xyzabc_left[1] = request->left_cartesian_pose().translation().y();
    xyzabc_left[2] = request->left_cartesian_pose().translation().z();
    xyzabc_left[3] =
        angle::radiansToDegrees(request->left_cartesian_pose().rotation().rx());
    xyzabc_left[4] =
        angle::radiansToDegrees(request->left_cartesian_pose().rotation().ry());
    xyzabc_left[5] =
        angle::radiansToDegrees(request->left_cartesian_pose().rotation().rz());

    // Target pose for right arm
    FX_DOUBLE xyzabc_right[6];
    xyzabc_right[0] = request->right_cartesian_pose().translation().x();
    xyzabc_right[1] = request->right_cartesian_pose().translation().y();
    xyzabc_right[2] = request->right_cartesian_pose().translation().z();
    xyzabc_right[3] = angle::radiansToDegrees(
        request->right_cartesian_pose().rotation().rx());
    xyzabc_right[4] = angle::radiansToDegrees(
        request->right_cartesian_pose().rotation().ry());
    xyzabc_right[5] = angle::radiansToDegrees(
        request->right_cartesian_pose().rotation().rz());

    // Use current joint positions of both arms as IK reference
    static DCSS dcss;
    if (!OnGetBuf(&dcss)) {
        response->set_success(false);
        response->set_error_message("获取机器人状态失败");
        return grpc::Status::OK;
    }

    std::array<double, 7> current_left{};
    std::array<double, 7> current_right{};
    for (int i = 0; i < 7; ++i) {
        current_left[i] = dcss.m_Out[0].m_FB_Joint_Pos[i];
        current_right[i] = dcss.m_Out[1].m_FB_Joint_Pos[i];
    }

    // If relative movement is requested, adjust target poses accordingly
    if (request->is_relative()) {
        // Get current Cartesian positions
        auto fk_left = TjArmServer::fk(0, current_left.data());
        auto fk_right = TjArmServer::fk(1, current_right.data());
        if (!fk_left.has_value()) {
            response->set_success(false);
            response->set_error_message("左臂动力学正解计算失败");
            return grpc::Status::OK;
        }
        if (!fk_right.has_value()) {
            response->set_success(false);
            response->set_error_message("右臂动力学正解计算失败");
            return grpc::Status::OK;
        }
        // Adjust target poses
        for (int i = 0; i < 6; ++i) {
            xyzabc_left[i] += fk_left.value()[i];
            xyzabc_right[i] += fk_right.value()[i];
        }
    }

    // Execute IK to get joint positions
    auto ik_left = ik(0, xyzabc_left, current_left);
    if (!ik_left.has_value()) {
        response->set_success(false);
        response->set_error_message("左臂笛卡尔运动逆解失败");
        return grpc::Status::OK;
    }

    auto ik_right = ik(1, xyzabc_right, current_right);
    if (!ik_right.has_value()) {
        response->set_success(false);
        response->set_error_message("右臂笛卡尔运动逆解失败");
        return grpc::Status::OK;
    }

    // Prepare joint position commands
    double joints_left[7];
    double joints_right[7];
    for (int i = 0; i < 7; ++i) {
        joints_left[i] = ik_left.value()[i];
        joints_right[i] = ik_right.value()[i];
    }

    // Send commands to robot
    OnClearSet();
    LEFT(OnSetJointCmdPos)(joints_left);
    RIGHT(OnSetJointCmdPos)(joints_right);
    OnSetSend();

    // Sleep for a short while to ensure motion has started
    usleep(100000);

    // Wait for motion to complete if blocking is requested
    if (request->is_block() && !TjArmServer::waitForMotionComplete(-1)) {
        response->set_success(false);
        response->set_error_message("等待双臂笛卡尔运动完成超时");
        return grpc::Status::OK;
    }

    response->set_success(true);
    return grpc::Status::OK;
}

grpc::Status TjArmServer::Connect(grpc::ServerContext *context,
                                  const ConnectRequest *request,
                                  ConnectResponse *response) {
    // Do nothing
    response->set_success(true);
    return grpc::Status::OK;
}

grpc::Status TjArmServer::Disconnect(grpc::ServerContext *context,
                                     const DisconnectRequest *request,
                                     DisconnectResponse *response) {
    // Do nothing
    response->set_success(true);
    return grpc::Status::OK;
}

grpc::Status TjArmServer::Enable(grpc::ServerContext *context,
                                 const EnableRequest *request,
                                 EnableResponse *response) {
    // Clear errors first
    OnClearSet();
    LEFT(OnClearErr)();
    RIGHT(OnClearErr)();
    OnSetSend();
    usleep(100000);

    // Setup position mode
    OnClearSet();
    LEFT(OnSetTargetState)(1);
    RIGHT(OnSetTargetState)(1);
    LEFT(OnSetJointLmt)(30, 30);
    RIGHT(OnSetJointLmt)(30, 30);
    OnSetSend();
    usleep(100000);

    response->set_success(true);
    return grpc::Status::OK;
}

grpc::Status TjArmServer::Disable(grpc::ServerContext *context,
                                  const DisableRequest *request,
                                  DisableResponse *response) {
    // Only initialize once even if multiple threads call this
    static std::mutex init_mutex;
    std::lock_guard<std::mutex> lock(init_mutex);

    // Clear errors first
    OnClearSet();
    LEFT(OnClearErr)();
    RIGHT(OnClearErr)();
    OnSetSend();
    usleep(100000);

    // Setup idle mode
    OnClearSet();
    LEFT(OnSetTargetState)(0);
    RIGHT(OnSetTargetState)(0);
    OnSetSend();
    usleep(100000);

    response->set_success(true);
    return grpc::Status::OK;
}

grpc::Status TjArmServer::MotionAbort(grpc::ServerContext *context,
                                      const MotionAbortRequest *request,
                                      MotionAbortResponse *response) {
    // Trigger emergency stop
    OnEMG_AB();

    // Return success
    response->set_success(true);
    return grpc::Status::OK;
}

grpc::Status TjArmServer::IsInPosition(grpc::ServerContext *context,
                                       const IsInPositionRequest *request,
                                       IsInPositionResponse *response) {
    static DCSS dcss;
    if (!OnGetBuf(&dcss)) {
        response->set_success(false);
        response->set_error_message("获取机器人状态失败");
    }

    response->set_success(true);
    response->set_is_in_position(!dcss.m_Out[0].m_LowSpdFlag ||
                                 !dcss.m_Out[1].m_LowSpdFlag);

    return grpc::Status::OK;
}

grpc::Status TjArmServer::PowerOn(grpc::ServerContext *context,
                                  const PowerOnRequest *request,
                                  PowerOnResponse *response) {
    // Do nothing
    response->set_success(true);
    return grpc::Status::OK;
}

grpc::Status TjArmServer::PowerOff(grpc::ServerContext *context,
                                   const PowerOffRequest *request,
                                   PowerOffResponse *response) {
    // Do nothing
    response->set_success(true);
    return grpc::Status::OK;
}

grpc::Status TjArmServer::EnableServoMode(grpc::ServerContext *context,
                                          const EnableServoModeRequest *request,
                                          EnableServoModeResponse *response) {
    throw std::runtime_error("Not implemented yet");
    return grpc::Status::OK;
}

grpc::Status TjArmServer::IsServoMode(grpc::ServerContext *context,
                                      const IsServoModeRequest *request,
                                      IsServoModeResponse *response) {
    throw std::runtime_error("Not implemented yet");
    return grpc::Status::OK;
}

grpc::Status TjArmServer::ServoJ(grpc::ServerContext *context,
                                 const ServoJRequest *request,
                                 ServoJResponse *response) {
    throw std::runtime_error("Not implemented yet");
    return grpc::Status::OK;
}

grpc::Status TjArmServer::ServoP(grpc::ServerContext *context,
                                 const ServoPRequest *request,
                                 ServoPResponse *response) {
    throw std::runtime_error("Not implemented yet");
    return grpc::Status::OK;
}

grpc::Status TjArmServer::ServoSend(grpc::ServerContext *context,
                                    const ServoSendRequest *request,
                                    ServoSendResponse *response) {
    throw std::runtime_error("Not implemented yet");
    return grpc::Status::OK;
}

// 动力学正解实现
grpc::Status
TjArmServer::ForwardKinematics(grpc::ServerContext *context,
                               const ForwardKinematicsRequest *request,
                               ForwardKinematicsResponse *response) {
    int robot_index = request->robot_index();
    if (robot_index < 0 || robot_index >= 2) {
        response->set_success(false);
        response->set_error_message("无效的机器人手臂索引");
        return grpc::Status::OK;
    }

    if (request->joint_positions_size() != 7) {
        response->set_success(false);
        response->set_error_message("关节位置需要7个值");
        return grpc::Status::OK;
    }

    // 构建关节位置
    double joints[7];
    for (int i = 0; i < 7; ++i) {
        joints[i] = angle::radiansToDegrees(request->joint_positions(i));
    }

    static Matrix4 kine_pg;
    static Vect6 xyzabc;
    if (FX_Robot_Kine_FK(robot_index, joints, kine_pg) == FX_FALSE) {
        response->set_success(false);
        response->set_error_message("动力学正解计算失败");
        return grpc::Status::OK;
    }

    if (FX_Matrix42XYZABCDEG(kine_pg, xyzabc) == FX_FALSE) {
        response->set_success(false);
        response->set_error_message("获取动力学正解结果失败");
        return grpc::Status::OK;
    }

    auto cartesian_pose = response->mutable_cartesian_pose();
    auto translation = cartesian_pose->mutable_translation();
    auto rotation = cartesian_pose->mutable_rotation();
    translation->set_x(xyzabc[0]);
    translation->set_y(xyzabc[1]);
    translation->set_z(xyzabc[2]);
    rotation->set_rx(angle::degreesToRadians(xyzabc[3]));
    rotation->set_ry(angle::degreesToRadians(xyzabc[4]));
    rotation->set_rz(angle::degreesToRadians(xyzabc[5]));

    response->set_success(true);
    return grpc::Status::OK;
}

// 动力学反解实现
grpc::Status
TjArmServer::InverseKinematics(grpc::ServerContext *context,
                               const InverseKinematicsRequest *request,
                               InverseKinematicsResponse *response) {
    int robot_index = request->robot_index();
    if (robot_index < 0 || robot_index >= 2) {
        response->set_success(false);
        response->set_error_message("无效的机器人手臂索引");
        return grpc::Status::OK;
    }

    // 构建笛卡尔位姿
    static Vect6 xyzabc;
    xyzabc[0] = request->cartesian_pose().translation().x();
    xyzabc[1] = request->cartesian_pose().translation().y();
    xyzabc[2] = request->cartesian_pose().translation().z();
    xyzabc[3] =
        angle::radiansToDegrees(request->cartesian_pose().rotation().rx());
    xyzabc[4] =
        angle::radiansToDegrees(request->cartesian_pose().rotation().ry());
    xyzabc[5] =
        angle::radiansToDegrees(request->cartesian_pose().rotation().rz());

    // Convert to transformation matrix
    static Matrix4 kine_pg;
    FX_XYZABC2Matrix4DEG(xyzabc, kine_pg);

    // Fill inverse kinematics solve parameters
    static FX_InvKineSolvePara sp;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            sp.m_Input_IK_TargetTCP[i][j] = kine_pg[i][j];
        }
    }
    if (request->reference_joint_positions_size() == 7) {
        for (int i = 0; i < 7; ++i) {
            sp.m_Input_IK_RefJoint[i] = request->reference_joint_positions(i);
        }
    }

    // Compute inverse kinematics
    if (FX_Robot_Kine_IK(robot_index, &sp) == FX_FALSE) {
        response->set_success(false);
        response->set_error_message("动力学反解计算失败");
        return grpc::Status::OK;
    }

    // Fill joint positions in response
    auto joint_positions = response->mutable_joint_positions();
    for (int i = 0; i < 7; ++i) {
        joint_positions->Add(angle::degreesToRadians(sp.m_Output_RetJoint[i]));
    }

    response->set_success(true);
    return grpc::Status::OK;
}

} // namespace arms
