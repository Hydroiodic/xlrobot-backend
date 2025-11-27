#include "JakaErrorParser.hpp"
#include "JakaServer.hpp"
#include "timespec.h"
#include <bits/types/error_t.h>
#include <chrono>
#include <iostream>
#include <libjaka/JAKAZuRobot.h>
#include <libjaka/jktypes.h>
#include <stdexcept>
#include <thread>
#include <time.h>

namespace jaka_robot {

JakaServer::JakaServer(const std::string &robot_ip)
    : is_enabled_(false), servo_mode_enabled_(false) {
    // Login to the robot
    auto res = robot_.login_in(robot_ip.c_str());
    if (res != ERR_SUCC) {
        throw std::runtime_error("Failed to login robot");
    }

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
        clock_gettime(CLOCK_REALTIME, &next);
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
            response->set_error_message(
                JakaErrorParser::GetErrorMessage(result));
        }
    }

    return grpc::Status::OK;
}

static bool getJointPositionWithRetry(JAKAZuRobot *robot, JointValue &jVal,
                                      int robot_index) {
    ::CartesianPose cartesian_pose;
    timespec next;
    robot->servo_move_enable(1, -1);
    int cnt = 0;
    while (cnt < 10) {
        cnt++;
        for (int i = 0; i < 7; ++i) {
            jVal.jVal[i] = 0;
        }
        clock_gettime(CLOCK_REALTIME, &next);
        robot->edg_recv(&next);
        errno_t result =
            robot->edg_get_stat(robot_index, &jVal, &cartesian_pose);

        std::cout << "重试获取" << robot_index << "伺服命令角度 ";
        for (int j = 0; j < 7; j++) {
            std::cout << jVal.jVal[j] << " ";
        }
        std::cout << std::endl;

        if (result == ERR_SUCC) {
            float total = 0;
            for (int i = 0; i < 7; ++i) {
                total += abs(jVal.jVal[i]);
            }
            if (total < 0.01) {
                continue;
            }
            break;
        } else {
            continue;
        }
    }
    robot->servo_move_enable(0, -1);
    if (cnt == 10) {
        std::cout << "获取角度超过重试次数";
        return false;
    }
    return true;
}

grpc::Status
JakaServer::GetCartesianPosition(grpc::ServerContext *context,
                                 const GetCartesianPositionRequest *request,
                                 GetCartesianPositionResponse *response) {
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
            response->set_error_message(
                JakaErrorParser::GetErrorMessage(result));
        }
    }

    return grpc::Status::OK;
}

grpc::Status JakaServer::JointMove(grpc::ServerContext *context,
                                   const JointMoveRequest *request,
                                   JointMoveResponse *response) {
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

    JointValue joint_pos[2];
    for (int arm = 0; arm < 2; ++arm) {
        for (int i = 0; i < 7; ++i) {
            if (arm == robot_index) {
                joint_pos[arm].jVal[i] = request->joint_positions(i);
            } else {
                joint_pos[arm].jVal[i] = 0;
            }
        }
    }

    MoveMode move_mode = request->is_relative() ? INCR : ABS;
    MoveMode move_mode_arr[2] = {move_mode, move_mode};
    double vel = request->velocity() > 0 ? request->velocity() : 1.0;
    double vel_arr[2] = {vel, vel};
    double acc = request->acceleration() > 0 ? request->acceleration() : 2.0;
    double acc_arr[2] = {acc, acc};

    robot_.clear_error();
    robot_.set_collision_level(robot_index, 0);
    errno_t result = robot_.robot_run_multi_movj(robot_index, move_mode_arr,
                                                 request->is_block(), joint_pos,
                                                 vel_arr, acc_arr);

    response->set_success(result == ERR_SUCC);
    if (result != ERR_SUCC) {
        std::cerr << "关节运动失败 Code:" << result << " - "
                  << JakaErrorParser::GetErrorMessage(result) << std::endl;
        response->set_error_message(JakaErrorParser::GetErrorMessage(result));
    }

    return grpc::Status::OK;
}

grpc::Status JakaServer::DualJointMove(grpc::ServerContext *context,
                                       const DualJointMoveRequest *request,
                                       DualJointMoveResponse *response) {
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
        std::cerr << "双臂关节运动失败 Code:" << result << " - "
                  << JakaErrorParser::GetErrorMessage(result) << std::endl;
        response->set_error_message(JakaErrorParser::GetErrorMessage(result));
    }

    return grpc::Status::OK;
}

grpc::Status JakaServer::CartesianMove(grpc::ServerContext *context,
                                       const CartesianMoveRequest *request,
                                       CartesianMoveResponse *response) {
    int robot_index = request->robot_index();
    if (robot_index < 0 || robot_index > 1) {
        response->set_success(false);
        response->set_error_message("无效的机器人索引（0=左臂，1=右臂）");
        return grpc::Status::OK;
    }

    ::CartesianPose cartesian_pose[2];

    // 笛卡尔位姿
    cartesian_pose[robot_index].tran.x =
        request->cartesian_pose().translation().x();
    cartesian_pose[robot_index].tran.y =
        request->cartesian_pose().translation().y();
    cartesian_pose[robot_index].tran.z =
        request->cartesian_pose().translation().z();
    cartesian_pose[robot_index].rpy.rx =
        request->cartesian_pose().rotation().rx();
    cartesian_pose[robot_index].rpy.ry =
        request->cartesian_pose().rotation().ry();
    cartesian_pose[robot_index].rpy.rz =
        request->cartesian_pose().rotation().rz();

    MoveMode move_mode[2] = {request->is_relative() ? INCR : ABS,
                             request->is_relative() ? INCR : ABS};
    double vel[2] = {request->velocity() > 0 ? request->velocity() : 50.0,
                     request->velocity() > 0 ? request->velocity() : 50.0};
    double acc[2] = {
        request->acceleration() > 0 ? request->acceleration() : 100.0,
        request->acceleration() > 0 ? request->acceleration() : 100.0};

    errno_t result = robot_.robot_run_multi_movl(
        robot_index, move_mode, request->is_block(), cartesian_pose, vel, acc);

    response->set_success(result == ERR_SUCC);
    if (result != ERR_SUCC) {
        response->set_error_message(JakaErrorParser::GetErrorMessage(result));
    }

    return grpc::Status::OK;
}

grpc::Status
JakaServer::DualCartesianMove(grpc::ServerContext *context,
                              const DualCartesianMoveRequest *request,
                              DualCartesianMoveResponse *response) {
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
        response->set_error_message(JakaErrorParser::GetErrorMessage(result));
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
    errno_t result = robot_.power_on();
    if (result == ERR_SUCC) {
        // is_enabled_ = true; // 移除is_enabled_赋值
        response->set_success(true);
    } else {
        response->set_success(false);
        response->set_error_message(JakaErrorParser::GetErrorMessage(result));
        return grpc::Status::OK;
    }

    result = robot_.enable_robot();

    if (result == ERR_SUCC) {
        // is_enabled_ = true; // 移除is_enabled_赋值
        response->set_success(true);
    } else {
        response->set_success(false);
        response->set_error_message(JakaErrorParser::GetErrorMessage(result));
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
        response->set_error_message(JakaErrorParser::GetErrorMessage(result));
        return grpc::Status::OK;
    }

    result = robot_.power_off();

    response->set_success(result == ERR_SUCC);
    if (result != ERR_SUCC) {
        response->set_error_message(JakaErrorParser::GetErrorMessage(result));
    }

    return grpc::Status::OK;
}

grpc::Status JakaServer::MotionAbort(grpc::ServerContext *context,
                                     const MotionAbortRequest *request,
                                     MotionAbortResponse *response) {
    errno_t result = robot_.motion_abort();

    response->set_success(result == ERR_SUCC);
    if (result != ERR_SUCC) {
        response->set_error_message(JakaErrorParser::GetErrorMessage(result));
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
        response->set_error_message(JakaErrorParser::GetErrorMessage(result));
    }

    return grpc::Status::OK;
}

// 新增的伺服模式相关接口实现
grpc::Status JakaServer::PowerOn(grpc::ServerContext *context,
                                 const PowerOnRequest *request,
                                 PowerOnResponse *response) {
    errno_t result = robot_.power_on();

    response->set_success(result == ERR_SUCC);
    if (result != ERR_SUCC) {
        response->set_error_message(JakaErrorParser::GetErrorMessage(result));
    }

    return grpc::Status::OK;
}

grpc::Status JakaServer::PowerOff(grpc::ServerContext *context,
                                  const PowerOffRequest *request,
                                  PowerOffResponse *response) {
    errno_t result = robot_.power_off();

    response->set_success(result == ERR_SUCC);
    if (result != ERR_SUCC) {
        response->set_error_message(JakaErrorParser::GetErrorMessage(result));
    }

    return grpc::Status::OK;
}

grpc::Status JakaServer::EnableServoMode(grpc::ServerContext *context,
                                         const EnableServoModeRequest *request,
                                         EnableServoModeResponse *response) {
    if (request->enable()) {
        if (servo_mode_enabled_ && servo_mode_active_) {
            std::cout << "重复使能伺服模式" << std::endl;
            response->set_success(true);
            return grpc::Status::OK;
        }

        // 启用伺服模式
        robot_.servo_move_enable(0, -1); // 先关闭所有机器人的伺服模式
        robot_.servo_move_use_joint_LPF(2.0);
        robot_.motion_abort();
        robot_.power_on();
        robot_.enable_robot();
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // 获取当前位置
        for (int i = 0; i < 2; i++) {
            JointValue joint_pos;
            bool result = getJointPositionWithRetry(&robot_, joint_pos, i);
            if (!result) {
                response->set_success(false);
                response->set_error_message("初始化伺服模式失败");
                robot_.servo_move_enable(0, -1);
                return grpc::Status::OK;
            }

            servo_commands_[i].is_joint_mode = true;
            std::cout << "初始化" << i << "伺服命令角度 ";
            for (int j = 0; j < 7; j++) {
                servo_commands_[i].joint_target.jVal[j] = joint_pos.jVal[j];
                std::cout << joint_pos.jVal[j] << " ";
            }
            std::cout << std::endl;
            servo_commands_[i].has_new_command = true;
            servo_commands_[i].robot_index = i;
        }

        // 启用伺服模式
        robot_.servo_move_enable(1, -1);

        // 通知伺服控制线程
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
    response->set_success(true);
    RobotState state;
    robot_.get_robot_state(&state);
    response->set_is_servo_mode(state.servoEnabled);

    return grpc::Status::OK;
}

grpc::Status JakaServer::ServoJ(grpc::ServerContext *context,
                                const ServoJRequest *request,
                                ServoJResponse *response) {
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
    if (!servo_mode_enabled_) {
        response->set_success(false);
        response->set_error_message("伺服模式未启用");
        return grpc::Status::OK;
    }

    // 伺服数据发送由伺服控制线程自动处理，这里只返回成功状态
    response->set_success(true);
    return grpc::Status::OK;
}

// 动力学正解实现
grpc::Status
JakaServer::ForwardKinematics(grpc::ServerContext *context,
                              const ForwardKinematicsRequest *request,
                              ForwardKinematicsResponse *response) {
    int robot_index = request->robot_index();
    if (robot_index < 0 || robot_index >= 2) {
        response->set_success(false);
        response->set_error_message("无效的机器人索引");
        return grpc::Status::OK;
    }

    if (request->joint_positions_size() != 7) {
        response->set_success(false);
        response->set_error_message("需要7个关节位置");
        return grpc::Status::OK;
    }

    // 构建关节位置
    JointValue joint_pos;
    for (int i = 0; i < 7; ++i) {
        joint_pos.jVal[i] = request->joint_positions(i);
    }

    // 调用Jaka SDK的正解函数
    ::CartesianPose cartesian_pose;
    errno_t result =
        robot_.kine_forward(robot_index, &joint_pos, &cartesian_pose);

    if (result == ERR_SUCC) {
        response->set_success(true);

        // 设置笛卡尔位姿
        auto *translation =
            response->mutable_cartesian_pose()->mutable_translation();
        translation->set_x(cartesian_pose.tran.x);
        translation->set_y(cartesian_pose.tran.y);
        translation->set_z(cartesian_pose.tran.z);

        auto *rotation = response->mutable_cartesian_pose()->mutable_rotation();
        rotation->set_rx(cartesian_pose.rpy.rx);
        rotation->set_ry(cartesian_pose.rpy.ry);
        rotation->set_rz(cartesian_pose.rpy.rz);
    } else {
        response->set_success(false);
        response->set_error_message(JakaErrorParser::GetErrorMessage(result));
    }

    return grpc::Status::OK;
}

// 动力学反解实现
grpc::Status
JakaServer::InverseKinematics(grpc::ServerContext *context,
                              const InverseKinematicsRequest *request,
                              InverseKinematicsResponse *response) {
    int robot_index = request->robot_index();
    if (robot_index < 0 || robot_index >= 2) {
        response->set_success(false);
        response->set_error_message("无效的机器人索引");
        return grpc::Status::OK;
    }

    // 构建笛卡尔位姿
    ::CartesianPose cartesian_pose;
    cartesian_pose.tran.x = request->cartesian_pose().translation().x();
    cartesian_pose.tran.y = request->cartesian_pose().translation().y();
    cartesian_pose.tran.z = request->cartesian_pose().translation().z();
    cartesian_pose.rpy.rx = request->cartesian_pose().rotation().rx();
    cartesian_pose.rpy.ry = request->cartesian_pose().rotation().ry();
    cartesian_pose.rpy.rz = request->cartesian_pose().rotation().rz();

    // 构建参考关节位置（如果提供）
    JointValue reference_joint_pos;
    if (request->reference_joint_positions_size() == 7) {
        for (int i = 0; i < 7; ++i) {
            reference_joint_pos.jVal[i] = request->reference_joint_positions(i);
        }
    } else {
        // 如果没有提供参考关节位置，使用当前关节位置
        JointValue current_joint_pos;
        ::CartesianPose current_cartesian_pose;
        errno_t get_stat_result = robot_.edg_get_stat(
            robot_index, &current_joint_pos, &current_cartesian_pose);
        if (get_stat_result != ERR_SUCC) {
            response->set_success(false);
            response->set_error_message(
                JakaErrorParser::GetErrorMessage(get_stat_result));
            return grpc::Status::OK;
        }
        reference_joint_pos = current_joint_pos;
    }

    // 调用Jaka SDK的反解函数
    JointValue result_joint_pos;
    errno_t result = robot_.kine_inverse(robot_index, &reference_joint_pos,
                                         &cartesian_pose, &result_joint_pos);

    if (result == ERR_SUCC) {
        response->set_success(true);

        // 设置关节位置
        for (int i = 0; i < 7; ++i) {
            response->add_joint_positions(result_joint_pos.jVal[i]);
        }
    } else {
        response->set_success(false);
        response->set_error_message(JakaErrorParser::GetErrorMessage(result));
    }

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
                                  << " error: " << result << " - "
                                  << JakaErrorParser::GetErrorMessage(result)
                                  << std::endl;
                    }
                } else {
                    // 笛卡尔模式
                    error_t result = robot_.edg_servo_p(
                        i, &servo_commands_[i].cartesian_target, ABS);
                    if (result != ERR_SUCC) {
                        std::cerr << "伺服笛卡尔控制失败 robot " << i
                                  << " error: " << result << " - "
                                  << JakaErrorParser::GetErrorMessage(result)
                                  << std::endl;
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
            std::cerr << "伺服数据发送失败: " << send_result << " - "
                      << JakaErrorParser::GetErrorMessage(send_result)
                      << std::endl;
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
