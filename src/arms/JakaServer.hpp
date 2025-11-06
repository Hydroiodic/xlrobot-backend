#pragma once

#include "jaka_robot.grpc.pb.h"
#include "libjaka/JAKAZuRobot.h"
#include <atomic>
#include <grpcpp/grpcpp.h>
#include <mutex>
#include <thread>

namespace jaka_robot {

class JakaServer final : public JakaRobotService::Service {
  public:
    JakaServer();
    ~JakaServer();

    // GRPC服务方法实现
    grpc::Status GetJointPosition(grpc::ServerContext *context,
                                  const GetJointPositionRequest *request,
                                  GetJointPositionResponse *response) override;

    grpc::Status
    GetCartesianPosition(grpc::ServerContext *context,
                         const GetCartesianPositionRequest *request,
                         GetCartesianPositionResponse *response) override;

    grpc::Status JointMove(grpc::ServerContext *context,
                           const JointMoveRequest *request,
                           JointMoveResponse *response) override;

    grpc::Status DualJointMove(grpc::ServerContext *context,
                               const DualJointMoveRequest *request,
                               DualJointMoveResponse *response) override;

    grpc::Status CartesianMove(grpc::ServerContext *context,
                               const CartesianMoveRequest *request,
                               CartesianMoveResponse *response) override;

    grpc::Status
    DualCartesianMove(grpc::ServerContext *context,
                      const DualCartesianMoveRequest *request,
                      DualCartesianMoveResponse *response) override;

    grpc::Status Connect(grpc::ServerContext *context,
                         const ConnectRequest *request,
                         ConnectResponse *response) override;

    grpc::Status Disconnect(grpc::ServerContext *context,
                            const DisconnectRequest *request,
                            DisconnectResponse *response) override;

    grpc::Status PowerOn(grpc::ServerContext *context,
                         const PowerOnRequest *request,
                         PowerOnResponse *response) override;

    grpc::Status PowerOff(grpc::ServerContext *context,
                          const PowerOffRequest *request,
                          PowerOffResponse *response) override;

    grpc::Status Enable(grpc::ServerContext *context,
                        const EnableRequest *request,
                        EnableResponse *response) override;

    grpc::Status Disable(grpc::ServerContext *context,
                         const DisableRequest *request,
                         DisableResponse *response) override;

    grpc::Status MotionAbort(grpc::ServerContext *context,
                             const MotionAbortRequest *request,
                             MotionAbortResponse *response) override;

    grpc::Status IsInPosition(grpc::ServerContext *context,
                              const IsInPositionRequest *request,
                              IsInPositionResponse *response) override;

    // 新增的伺服模式相关接口
    grpc::Status EnableServoMode(grpc::ServerContext *context,
                                 const EnableServoModeRequest *request,
                                 EnableServoModeResponse *response) override;

    grpc::Status IsServoMode(grpc::ServerContext *context,
                             const IsServoModeRequest *request,
                             IsServoModeResponse *response) override;

    grpc::Status ServoJ(grpc::ServerContext *context,
                        const ServoJRequest *request,
                        ServoJResponse *response) override;

    grpc::Status ServoP(grpc::ServerContext *context,
                        const ServoPRequest *request,
                        ServoPResponse *response) override;

    grpc::Status ServoSend(grpc::ServerContext *context,
                           const ServoSendRequest *request,
                           ServoSendResponse *response) override;

  private:
    // 伺服控制线程函数
    void servoControlThread();

    // 伺服控制相关数据结构
    struct ServoCommand {
        bool is_joint_mode; // true: 关节模式, false: 笛卡尔模式
        JointValue joint_target;
        ::CartesianPose cartesian_target;
        int robot_index;
        bool has_new_command;
    };

    JAKAZuRobot robot_;
    bool is_connected_;
    bool is_enabled_;
    bool servo_mode_enabled_;

    // 伺服控制线程相关
    std::thread servo_thread_;
    std::atomic<bool> servo_thread_running_{false};
    std::atomic<bool> servo_mode_active_{false};
    std::mutex servo_mutex_;

    // 伺服命令存储
    ServoCommand servo_commands_[2]; // 支持两个机器人
    std::mutex command_mutex_;

    // 当前机器人实际状态
    JointValue current_joint_pos_[2];
    ::CartesianPose current_cartesian_pos_[2];
    std::mutex state_mutex_;
};

} // namespace jaka_robot
