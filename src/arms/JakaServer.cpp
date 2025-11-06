#include "JakaServer.hpp"
#include "timespec.h"
#include <bits/types/error_t.h>
#include <chrono>
#include <iostream>
#include <libjaka/jktypes.h>
#include <thread>
#include <time.h>

namespace jaka_robot {

const char *JAKA_ROBOT_IP = "192.168.2.200";

JakaServer::JakaServer()
    : is_connected_(false), is_enabled_(false), servo_mode_enabled_(false) {
    auto res = robot_.login_in(JAKA_ROBOT_IP);
    if (res != ERR_SUCC) {
        throw "Failed to login robot";
    }
    is_connected_ = true; // TODO: useless, omit it

    // 初始化伺服命令
    for (int i = 0; i < 2; ++i) {
        servo_commands_[i].is_joint_mode = true;
        servo_commands_[i].robot_index = i;
        servo_commands_[i].has_new_command = false;
        // 初始化关节位置为0
        for (int j = 0; j < 7; ++j) {
            servo_commands_[i].joint_target.jVal[j] = 0.0;
            current_joint_pos_[i].jVal[j] = 0.0;
        }
        // 初始化笛卡尔位置
        current_cartesian_pos_[i].tran.x = 0.0;
        current_cartesian_pos_[i].tran.y = 0.0;
        current_cartesian_pos_[i].tran.z = 0.0;
        current_cartesian_pos_[i].rpy.rx = 0.0;
        current_cartesian_pos_[i].rpy.ry = 0.0;
        current_cartesian_pos_[i].rpy.rz = 0.0;
    }

    // 启动伺服控制线程
    servo_thread_running_ = true;
    servo_thread_ = std::thread(&JakaServer::servoControlThread, this);
}

JakaServer::~JakaServer() {
    // 停止伺服控制线程
    servo_thread_running_ = false;
    if (servo_thread_.joinable()) {
        servo_thread_.join();
    }
}

grpc::Status
JakaServer::GetJointPosition(grpc::ServerContext *context,
                             const GetJointPositionRequest *request,
                             GetJointPositionResponse *response) {
    if (!is_connected_) {
        response->set_success(false);
        response->set_error_message("机器人未连接");
        return grpc::Status::OK;
    }

    int robot_index = request->robot_index();
    if (robot_index < 0 || robot_index >= 2) {
        response->set_success(false);
        response->set_error_message("无效的机器人索引");
        return grpc::Status::OK;
    }

    if (servo_mode_active_.load()) {
        // 伺服模式下使用维护的状态
        std::lock_guard<std::mutex> lock(state_mutex_);
        response->set_success(true);
        for (int i = 0; i < 7; ++i) {
            response->add_joint_positions(
                current_joint_pos_[robot_index].jVal[i]);
        }
    } else {
        // 非伺服模式下直接查询机器人
        JointValue joint_pos;
        ::CartesianPose cartesian_pose;
        timespec next;
        robot_.servo_move_enable(1, -1);
        clock_gettime(CLOCK_MONOTONIC, &next);
        robot_.edg_recv(&next);
        errno_t result =
            robot_.edg_get_stat(robot_index, &joint_pos, &cartesian_pose);
        robot_.servo_move_enable(0, -1);

        if (result == ERR_SUCC) {
            response->set_success(true);
            for (int i = 0; i < 7; ++i) {
                response->add_joint_positions(joint_pos.jVal[i]);
            }
        } else {
            response->set_success(false);
            response->set_error_message("获取关节位置失败");
        }
    }

    return grpc::Status::OK;
}

grpc::Status
JakaServer::GetCartesianPosition(grpc::ServerContext *context,
                                 const GetCartesianPositionRequest *request,
                                 GetCartesianPositionResponse *response) {
    if (!is_connected_) {
        response->set_success(false);
        response->set_error_message("机器人未连接");
        return grpc::Status::OK;
    }

    int robot_index = request->robot_index();
    if (robot_index < 0 || robot_index >= 2) {
        response->set_success(false);
        response->set_error_message("无效的机器人索引");
        return grpc::Status::OK;
    }

    if (servo_mode_active_.load()) {
        // 伺服模式下使用维护的状态
        std::lock_guard<std::mutex> lock(state_mutex_);
        response->set_success(true);

        auto *translation =
            response->mutable_cartesian_pose()->mutable_translation();
        translation->set_x(current_cartesian_pos_[robot_index].tran.x);
        translation->set_y(current_cartesian_pos_[robot_index].tran.y);
        translation->set_z(current_cartesian_pos_[robot_index].tran.z);

        auto *rotation = response->mutable_cartesian_pose()->mutable_rotation();
        rotation->set_rx(current_cartesian_pos_[robot_index].rpy.rx);
        rotation->set_ry(current_cartesian_pos_[robot_index].rpy.ry);
        rotation->set_rz(current_cartesian_pos_[robot_index].rpy.rz);
    } else {
        // 非伺服模式下直接查询机器人
        JointValue joint_pos;
        ::CartesianPose cartesian_pose;

        errno_t result =
            robot_.edg_get_stat(robot_index, &joint_pos, &cartesian_pose);

        if (result == ERR_SUCC) {
            response->set_success(true);

            auto *translation =
                response->mutable_cartesian_pose()->mutable_translation();
            translation->set_x(cartesian_pose.tran.x);
            translation->set_y(cartesian_pose.tran.y);
            translation->set_z(cartesian_pose.tran.z);

            auto *rotation =
                response->mutable_cartesian_pose()->mutable_rotation();
            rotation->set_rx(cartesian_pose.rpy.rx);
            rotation->set_ry(cartesian_pose.rpy.ry);
            rotation->set_rz(cartesian_pose.rpy.rz);
        } else {
            response->set_success(false);
            response->set_error_message("获取笛卡尔位置失败");
        }
    }

    return grpc::Status::OK;
}

grpc::Status JakaServer::JointMove(grpc::ServerContext *context,
                                   const JointMoveRequest *request,
                                   JointMoveResponse *response) {
    if (!is_connected_) {
        response->set_success(false);
        response->set_error_message("机器人未连接或未使能");
        return grpc::Status::OK;
    }

    int robot_index = request->robot_index();
    if (robot_index < 0 || robot_index > 1) {
        response->set_success(false);
        response->set_error_message("无效的机器人索引（0=左臂，1=右臂）");
        return grpc::Status::OK;
    }

    if (request->joint_positions_size() != 7) {
        response->set_success(false);
        response->set_error_message("需要7个关节位置");
        return grpc::Status::OK;
    }

    JointValue joint_pos;
    for (int i = 0; i < 7; ++i) {
        joint_pos.jVal[i] = request->joint_positions(i);
    }

    MoveMode move_mode = request->is_relative() ? INCR : ABS;
    double vel = request->velocity() > 0 ? request->velocity() : 1.0;
    double acc = request->acceleration() > 0 ? request->acceleration() : 2.0;

    robot_.clear_error();
    robot_.set_collision_level(robot_index, 0);
    errno_t result = robot_.robot_run_multi_movj(
        robot_index, &move_mode, request->is_block(), &joint_pos, &vel, &acc);

    response->set_success(result == ERR_SUCC);
    if (result != ERR_SUCC) {
        std::cerr << "关节运动失败 Code:" << result << std::endl;
        response->set_error_message("关节运动失败");
    }

    return grpc::Status::OK;
}

grpc::Status JakaServer::DualJointMove(grpc::ServerContext *context,
                                       const DualJointMoveRequest *request,
                                       DualJointMoveResponse *response) {
    if (!is_connected_) {
        response->set_success(false);
        response->set_error_message("机器人未连接或未使能");
        return grpc::Status::OK;
    }

    // 检查关节位置数量
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

    JointValue joint_pos[2];
    for (int i = 0; i < 7; ++i) {
        joint_pos[0].jVal[i] = request->left_joint_positions(i);  // 左臂
        joint_pos[1].jVal[i] = request->right_joint_positions(i); // 右臂
    }

    MoveMode move_mode = request->is_relative() ? INCR : ABS;
    MoveMode move_mode_arr[2] = {move_mode, move_mode};
    double vel = request->velocity() > 0 ? request->velocity() : 1.0;
    double vel_arr[2] = {vel, vel};
    double acc = request->acceleration() > 0 ? request->acceleration() : 2.0;
    double acc_arr[2] = {acc, acc};

    robot_.clear_error();
    errno_t result = robot_.robot_run_multi_movj(
        -1, move_mode_arr, request->is_block(), joint_pos, vel_arr, acc_arr);

    response->set_success(result == ERR_SUCC);
    if (result != ERR_SUCC) {
        std::cerr << "双臂关节运动失败 Code:" << result << std::endl;
        response->set_error_message("双臂关节运动失败");
    }

    return grpc::Status::OK;
}

grpc::Status JakaServer::CartesianMove(grpc::ServerContext *context,
                                       const CartesianMoveRequest *request,
                                       CartesianMoveResponse *response) {
    if (!is_connected_) {
        response->set_success(false);
        response->set_error_message("机器人未连接或未使能");
        return grpc::Status::OK;
    }

    int robot_index = request->robot_index();
    if (robot_index < 0 || robot_index > 1) {
        response->set_success(false);
        response->set_error_message("无效的机器人索引（0=左臂，1=右臂）");
        return grpc::Status::OK;
    }

    ::CartesianPose cartesian_pose;
    cartesian_pose.tran.x = request->cartesian_pose().translation().x();
    cartesian_pose.tran.y = request->cartesian_pose().translation().y();
    cartesian_pose.tran.z = request->cartesian_pose().translation().z();
    cartesian_pose.rpy.rx = request->cartesian_pose().rotation().rx();
    cartesian_pose.rpy.ry = request->cartesian_pose().rotation().ry();
    cartesian_pose.rpy.rz = request->cartesian_pose().rotation().rz();

    MoveMode move_mode = request->is_relative() ? INCR : ABS;
    double vel = request->velocity() > 0 ? request->velocity() : 50.0;
    double acc = request->acceleration() > 0 ? request->acceleration() : 100.0;

    errno_t result = robot_.robot_run_multi_movl(robot_index, &move_mode,
                                                 request->is_block(),
                                                 &cartesian_pose, &vel, &acc);

    response->set_success(result == ERR_SUCC);
    if (result != ERR_SUCC) {
        response->set_error_message("笛卡尔运动失败");
    }

    return grpc::Status::OK;
}

grpc::Status
JakaServer::DualCartesianMove(grpc::ServerContext *context,
                              const DualCartesianMoveRequest *request,
                              DualCartesianMoveResponse *response) {
    if (!is_connected_) {
        response->set_success(false);
        response->set_error_message("机器人未连接或未使能");
        return grpc::Status::OK;
    }

    ::CartesianPose cartesian_pose[2];

    // 左臂笛卡尔位姿
    cartesian_pose[0].tran.x = request->left_cartesian_pose().translation().x();
    cartesian_pose[0].tran.y = request->left_cartesian_pose().translation().y();
    cartesian_pose[0].tran.z = request->left_cartesian_pose().translation().z();
    cartesian_pose[0].rpy.rx = request->left_cartesian_pose().rotation().rx();
    cartesian_pose[0].rpy.ry = request->left_cartesian_pose().rotation().ry();
    cartesian_pose[0].rpy.rz = request->left_cartesian_pose().rotation().rz();

    // 右臂笛卡尔位姿
    cartesian_pose[1].tran.x =
        request->right_cartesian_pose().translation().x();
    cartesian_pose[1].tran.y =
        request->right_cartesian_pose().translation().y();
    cartesian_pose[1].tran.z =
        request->right_cartesian_pose().translation().z();
    cartesian_pose[1].rpy.rx = request->right_cartesian_pose().rotation().rx();
    cartesian_pose[1].rpy.ry = request->right_cartesian_pose().rotation().ry();
    cartesian_pose[1].rpy.rz = request->right_cartesian_pose().rotation().rz();

    MoveMode move_mode[2] = {request->is_relative() ? INCR : ABS,
                             request->is_relative() ? INCR : ABS};
    double vel[2] = {request->velocity() > 0 ? request->velocity() : 50.0,
                     request->velocity() > 0 ? request->velocity() : 50.0};
    double acc[2] = {
        request->acceleration() > 0 ? request->acceleration() : 100.0,
        request->acceleration() > 0 ? request->acceleration() : 100.0};

    errno_t result = robot_.robot_run_multi_movl(
        -1, move_mode, request->is_block(), cartesian_pose, vel, acc);
    response->set_success(result == ERR_SUCC);
    if (result != ERR_SUCC) {
        response->set_error_message("双臂笛卡尔运动失败");
    }

    return grpc::Status::OK;
}

grpc::Status JakaServer::Connect(grpc::ServerContext *context,
                                 const ConnectRequest *request,
                                 ConnectResponse *response) {
    // Do nothing
    return grpc::Status::OK;
}

grpc::Status JakaServer::Disconnect(grpc::ServerContext *context,
                                    const DisconnectRequest *request,
                                    DisconnectResponse *response) {
    // Do nothing
    return grpc::Status::OK;
}

grpc::Status JakaServer::Enable(grpc::ServerContext *context,
                                const EnableRequest *request,
                                EnableResponse *response) {
    if (!is_connected_) {
        response->set_success(false);
        response->set_error_message("机器人未连接");
        return grpc::Status::OK;
    }

    errno_t result = robot_.power_on();
    if (result == ERR_SUCC) {
        // is_enabled_ = true; // 移除is_enabled_赋值
        response->set_success(true);
    } else {
        response->set_success(false);
        response->set_error_message("机器人上电失败");
        return grpc::Status::OK;
    }

    result = robot_.enable_robot();

    if (result == ERR_SUCC) {
        // is_enabled_ = true; // 移除is_enabled_赋值
        response->set_success(true);
    } else {
        response->set_success(false);
        response->set_error_message("机器人使能失败");
    }

    return grpc::Status::OK;
}

grpc::Status JakaServer::Disable(grpc::ServerContext *context,
                                 const DisableRequest *request,
                                 DisableResponse *response) {
    errno_t result = robot_.disable_robot();

    // is_enabled_ = false; // 移除is_enabled_赋值

    response->set_success(result == ERR_SUCC);
    if (result != ERR_SUCC) {
        response->set_error_message("机器人下使能失败");
        return grpc::Status::OK;
    }

    result = robot_.power_off();

    response->set_success(result == ERR_SUCC);
    if (result != ERR_SUCC) {
        response->set_error_message("机器人下电失败");
    }

    return grpc::Status::OK;
}

grpc::Status JakaServer::MotionAbort(grpc::ServerContext *context,
                                     const MotionAbortRequest *request,
                                     MotionAbortResponse *response) {
    errno_t result = robot_.motion_abort();

    response->set_success(result == ERR_SUCC);
    if (result != ERR_SUCC) {
        response->set_error_message("停止运动失败");
    }

    return grpc::Status::OK;
}

grpc::Status JakaServer::IsInPosition(grpc::ServerContext *context,
                                      const IsInPositionRequest *request,
                                      IsInPositionResponse *response) {
    int inpos[2];
    errno_t result = robot_.robot_is_inpos(inpos);

    if (result == ERR_SUCC) {
        response->set_success(true);
        response->set_is_in_position(inpos[0] == 1 && inpos[1] == 1);
    } else {
        response->set_success(false);
        response->set_error_message("获取到位状态失败");
    }

    return grpc::Status::OK;
}

// 新增的伺服模式相关接口实现
grpc::Status JakaServer::PowerOn(grpc::ServerContext *context,
                                 const PowerOnRequest *request,
                                 PowerOnResponse *response) {
    if (!is_connected_) {
        response->set_success(false);
        response->set_error_message("机器人未连接");
        return grpc::Status::OK;
    }

    errno_t result = robot_.power_on();

    response->set_success(result == ERR_SUCC);
    if (result != ERR_SUCC) {
        response->set_error_message("机器人上电失败");
    }

    return grpc::Status::OK;
}

grpc::Status JakaServer::PowerOff(grpc::ServerContext *context,
                                  const PowerOffRequest *request,
                                  PowerOffResponse *response) {
    if (!is_connected_) {
        response->set_success(false);
        response->set_error_message("机器人未连接");
        return grpc::Status::OK;
    }

    errno_t result = robot_.power_off();

    response->set_success(result == ERR_SUCC);
    if (result != ERR_SUCC) {
        response->set_error_message("机器人断电失败");
    }

    return grpc::Status::OK;
}

grpc::Status JakaServer::EnableServoMode(grpc::ServerContext *context,
                                         const EnableServoModeRequest *request,
                                         EnableServoModeResponse *response) {
    if (!is_connected_) {
        response->set_success(false);
        response->set_error_message("机器人未连接");
        return grpc::Status::OK;
    }

    if (request->enable()) {
        // 启用伺服模式
        robot_.servo_move_enable(0, -1); // 先关闭所有机器人的伺服模式
        robot_.servo_move_use_joint_LPF(2.0);
        robot_.motion_abort();
        robot_.power_on();
        robot_.enable_robot();
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // 启用伺服模式
        robot_.servo_move_enable(1, -1);
        servo_mode_enabled_ = true;
        servo_mode_active_ = true;

        std::cout << "伺服模式已启用" << std::endl;
    } else {
        // 禁用伺服模式
        robot_.servo_move_enable(0, -1);
        servo_mode_enabled_ = false;
        servo_mode_active_ = false;

        std::cout << "伺服模式已禁用" << std::endl;
    }

    response->set_success(true);
    return grpc::Status::OK;
}

grpc::Status JakaServer::IsServoMode(grpc::ServerContext *context,
                                     const IsServoModeRequest *request,
                                     IsServoModeResponse *response) {
    if (!is_connected_) {
        response->set_success(false);
        response->set_error_message("机器人未连接");
        return grpc::Status::OK;
    }

    response->set_success(true);
    RobotState state;
    robot_.get_robot_state(&state);
    response->set_is_servo_mode(state.servoEnabled);

    return grpc::Status::OK;
}

grpc::Status JakaServer::ServoJ(grpc::ServerContext *context,
                                const ServoJRequest *request,
                                ServoJResponse *response) {
    if (!is_connected_) {
        response->set_success(false);
        response->set_error_message("机器人未连接");
        return grpc::Status::OK;
    }

    if (!servo_mode_enabled_) {
        response->set_success(false);
        response->set_error_message("伺服模式未启用");
        return grpc::Status::OK;
    }

    int robot_index = request->robot_index();
    if (robot_index < 0 || robot_index >= 2) {
        response->set_success(false);
        response->set_error_message("无效的机器人索引");
        return grpc::Status::OK;
    }

    // 更新伺服控制线程的目标位置
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        servo_commands_[robot_index].is_joint_mode = true;
        for (int i = 0; i < 7 && i < request->joint_positions_size(); ++i) {
            servo_commands_[robot_index].joint_target.jVal[i] =
                request->joint_positions(i);
        }
        servo_commands_[robot_index].has_new_command = true;
    }

    response->set_success(true);
    return grpc::Status::OK;
}

grpc::Status JakaServer::ServoP(grpc::ServerContext *context,
                                const ServoPRequest *request,
                                ServoPResponse *response) {
    if (!is_connected_) {
        response->set_success(false);
        response->set_error_message("机器人未连接");
        return grpc::Status::OK;
    }

    if (!servo_mode_enabled_) {
        response->set_success(false);
        response->set_error_message("伺服模式未启用");
        return grpc::Status::OK;
    }

    int robot_index = request->robot_index();
    if (robot_index < 0 || robot_index >= 2) {
        response->set_success(false);
        response->set_error_message("无效的机器人索引");
        return grpc::Status::OK;
    }

    // 更新伺服控制线程的目标位置
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        servo_commands_[robot_index].is_joint_mode = false;
        servo_commands_[robot_index].cartesian_target.tran.x =
            request->cartesian_pose().translation().x();
        servo_commands_[robot_index].cartesian_target.tran.y =
            request->cartesian_pose().translation().y();
        servo_commands_[robot_index].cartesian_target.tran.z =
            request->cartesian_pose().translation().z();
        servo_commands_[robot_index].cartesian_target.rpy.rx =
            request->cartesian_pose().rotation().rx();
        servo_commands_[robot_index].cartesian_target.rpy.ry =
            request->cartesian_pose().rotation().ry();
        servo_commands_[robot_index].cartesian_target.rpy.rz =
            request->cartesian_pose().rotation().rz();
        servo_commands_[robot_index].has_new_command = true;
    }

    response->set_success(true);
    return grpc::Status::OK;
}

grpc::Status JakaServer::ServoSend(grpc::ServerContext *context,
                                   const ServoSendRequest *request,
                                   ServoSendResponse *response) {
    if (!is_connected_) {
        response->set_success(false);
        response->set_error_message("机器人未连接");
        return grpc::Status::OK;
    }

    if (!servo_mode_enabled_) {
        response->set_success(false);
        response->set_error_message("伺服模式未启用");
        return grpc::Status::OK;
    }

    // 伺服数据发送由伺服控制线程自动处理，这里只返回成功状态
    response->set_success(true);
    return grpc::Status::OK;
}

// 伺服控制线程实现
void JakaServer::servoControlThread() {
    std::cout << "伺服控制线程启动" << std::endl;

    // 设置线程优先级为实时优先级
    sched_param sch;
    sch.sched_priority = 90;
    int pr = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch);
    if (pr != 0) {
        std::cerr << "警告: 设置实时调度失败, code=" << pr
                  << ". 线程可能无法获得实时优先级。" << std::endl;
    }

    timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    int control_cycle = 0;

    while (servo_thread_running_.load()) {
        // 等待伺服模式激活
        if (!servo_mode_active_.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // 1000Hz控制循环
        robot_.edg_recv(&next);

        // 获取当前机器人状态
        JointValue actjpos[2];
        ::CartesianPose actcpos[2];
        robot_.edg_get_stat(0, &actjpos[0], &actcpos[0]);
        robot_.edg_get_stat(1, &actjpos[1], &actcpos[1]);

        // 发送伺服命令（无论有无新命令都要调用）
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            for (int i = 0; i < 2; ++i) {
                if (servo_commands_[i].is_joint_mode) {
                    // 关节模式
                    error_t result = robot_.edg_servo_j(
                        i, &servo_commands_[i].joint_target, ABS);
                    if (result != ERR_SUCC) {
                        std::cerr << "伺服关节控制失败 robot " << i
                                  << " error: " << result << std::endl;
                    }
                } else {
                    // 笛卡尔模式
                    error_t result = robot_.edg_servo_p(
                        i, &servo_commands_[i].cartesian_target, ABS);
                    if (result != ERR_SUCC) {
                        std::cerr << "伺服笛卡尔控制失败 robot " << i
                                  << " error: " << result << std::endl;
                    }
                }
                servo_commands_[i].has_new_command = false;
            }
        }

        // 更新当前机器人状态
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            current_joint_pos_[0] = actjpos[0];
            current_joint_pos_[1] = actjpos[1];
            current_cartesian_pos_[0] = actcpos[0];
            current_cartesian_pos_[1] = actcpos[1];
        }

        // 发送伺服数据
        error_t send_result = robot_.edg_send();
        if (send_result != ERR_SUCC) {
            std::cerr << "伺服数据发送失败: " << send_result << std::endl;
        }

        // 打印调试信息（降频）
        if (control_cycle % 1000 == 0) {
            std::cout << "伺服控制周期: " << control_cycle << std::endl;
        }

        control_cycle++;

        // 等待下一个控制周期（1ms）
        timespec dt;
        dt.tv_nsec = 1000000; // 1ms
        dt.tv_sec = 0;
        next = timespec_add(next, dt);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
    }

    std::cout << "伺服控制线程结束" << std::endl;
}

} // namespace jaka_robot
