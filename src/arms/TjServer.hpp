#pragma once

#include "Logger.hpp"
#include "arms.grpc.pb.h"
#include <array>
#include <grpcpp/grpcpp.h>
#include <libtj/FxRobot.h>
#include <libtj/MarvinSDK.h>
#include <mutex>
#include <optional>

#undef LEFT
#undef RIGHT
#define LEFT(func) func##_A
#define RIGHT(func) func##_B

namespace arms {

class TjArmServer final : public arms::RobotArmService::Service {
  public:
    TjArmServer(const std::string &);
    ~TjArmServer();

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

    // 伺服模式相关接口
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

    // 动力学正反解接口
    grpc::Status
    ForwardKinematics(grpc::ServerContext *context,
                      const ForwardKinematicsRequest *request,
                      ForwardKinematicsResponse *response) override;

    grpc::Status
    InverseKinematics(grpc::ServerContext *context,
                      const InverseKinematicsRequest *request,
                      InverseKinematicsResponse *response) override;

  private:
    static bool initializedKineParams(std::string &config_path) {
        // Whether the kinematics parameters have been initialized
        static bool initialized = false;

        // Only initialize once even if multiple threads call this
        static std::mutex init_mutex;
        std::lock_guard<std::mutex> lock(init_mutex);

        if (initialized) {
            return true;
        }

        // 1. 导入运动学参数
        FX_INT32L TYPE[2];
        FX_DOUBLE GRV[2][3];
        FX_DOUBLE DH[2][8][4];
        FX_DOUBLE PNVA[2][7][4];
        FX_DOUBLE BD[2][4][3];

        FX_DOUBLE Mass[2][7];
        FX_DOUBLE MCP[2][7][3];
        FX_DOUBLE I[2][7][6];

        // Load configuration file
        if (LOADMvCfg(config_path.data(), TYPE, GRV, DH, PNVA, BD, Mass, MCP,
                      I) == FX_TRUE) {
            LOG_INFO("Robot Load CFG Success");
        } else {
            LOG_ERROR("Robot Load CFG Error");
            return false;
        }

        // 2. 初始化运动学参数
        for (int arm = 0; arm < 2; ++arm) {
            // Initialize robot type
            if (FX_Robot_Init_Type(arm, TYPE[arm]) == FX_FALSE) {
                LOG_ERROR("Robot Init Type Error\n");
                return false;
            } else {
                LOG_INFO("Robot Init Type Success\n");
            }

            // Initialize DH parameters
            if (FX_Robot_Init_Kine(arm, DH[arm]) == FX_FALSE) {
                LOG_ERROR("Robot Init DH Parameters Error\n");
                return false;
            } else {
                LOG_INFO("Robot Init DH Parameters Success\n");
            }

            // Initialize limit parameters
            if (FX_Robot_Init_Lmt(arm, PNVA[arm], BD[arm]) == FX_FALSE) {
                LOG_ERROR("Robot Init Limit Parameters Error\n");
                return false;
            } else {
                LOG_INFO("Robot Init Limit Parameters Success\n");
            }
        }

        initialized = true;
        return true;
    }

    // Static method to connect to the robot arms
    static bool connectToRobot(const std::string &ip_address) {
        // Whether the connection is successful
        static bool connected = false;

        // Only initialize connection once even if multiple threads call this
        static std::mutex connect_mutex;
        std::lock_guard<std::mutex> lock(connect_mutex);

        if (connected) {
            return true;
        }

        // Parse the IP address
        int octets[4];
        if (sscanf(ip_address.c_str(), "%d.%d.%d.%d", &octets[0], &octets[1],
                   &octets[2], &octets[3]) != 4) {
            return false;
        }

        for (int i = 0; i < 4; ++i) {
            if (octets[i] < 0 || octets[i] > 255) {
                return false;
            }
        }

        // Connect to the robot
        bool init = OnLinkTo(octets[0], octets[1], octets[2], octets[3]);
        if (!init) {
            LOG_ERROR("Robot arms initialization failed: port already in use");
            return false;
        }

        // Clear errors and verify connection
        usleep(100000);
        OnClearSet();
        OnClearErr_A();
        OnClearErr_B();
        OnSetSend();
        usleep(100000);

        int motion_tag = 0;
        int frame_update = 0;
        DCSS dcss;

        for (int i = 0; i < 5; i++) {
            OnGetBuf(&dcss);
            LOG_INFO("Connect frames: %d", dcss.m_Out[0].m_OutFrameSerial);

            if (dcss.m_Out[0].m_OutFrameSerial != 0 &&
                frame_update != dcss.m_Out[0].m_OutFrameSerial) {
                motion_tag++;
                frame_update = dcss.m_Out[0].m_OutFrameSerial;
            }
            usleep(100000);
        }

        if (motion_tag > 0) {
            LOG_INFO("Robot arms initialization succeed.");
            connected = true;
            return true;
        } else {
            LOG_ERROR("Robot arms initialization failed: no data received");
            return false;
        }
    }

    static bool waitForMotion(int robot_index, int timeout_ms = 10000) {
        // robot_index: -1 - both arms, 0 - left arm, 1 - right arm
        if (robot_index < -1 || robot_index > 1) {
            return false;
        }

        // Current time
        timespec start_time;
        clock_gettime(CLOCK_MONOTONIC, &start_time);

        // Simple polling mechanism to wait for motion completion
        while (true) {
            // Check timeout
            timespec current_time;
            clock_gettime(CLOCK_MONOTONIC, &current_time);
            int elapsed_ms =
                (current_time.tv_sec - start_time.tv_sec) * 1000 +
                (current_time.tv_nsec - start_time.tv_nsec) / 1000000;
            if (elapsed_ms >= timeout_ms) {
                return false; // Timeout
            }

            static DCSS dcss;
            if (!OnGetBuf(&dcss)) {
                usleep(10000);
                continue;
            }

            bool is_in_position = robot_index > 0
                                      ? !dcss.m_Out[robot_index].m_LowSpdFlag
                                      : !dcss.m_Out[0].m_LowSpdFlag ||
                                            !dcss.m_Out[1].m_LowSpdFlag;
            if (is_in_position) {
                return true; // Motion complete
            }

            // Sleep for 10ms before checking again
            usleep(10000);
        }
    }

    static bool waitForMotionComplete(int robot_index, int timeout_ms = 10000) {
        // robot_index: -1 - both arms, 0 - left arm, 1 - right arm
        if (robot_index < -1 || robot_index > 1) {
            return false;
        }

        // Current time
        timespec start_time;
        clock_gettime(CLOCK_MONOTONIC, &start_time);

        // Simple polling mechanism to wait for motion completion
        while (true) {
            // Check timeout
            timespec current_time;
            clock_gettime(CLOCK_MONOTONIC, &current_time);
            int elapsed_ms =
                (current_time.tv_sec - start_time.tv_sec) * 1000 +
                (current_time.tv_nsec - start_time.tv_nsec) / 1000000;
            if (elapsed_ms >= timeout_ms) {
                return false; // Timeout
            }

            static DCSS dcss;
            if (!OnGetBuf(&dcss)) {
                usleep(10000);
                continue;
            }

            bool is_in_position = robot_index > 0
                                      ? !dcss.m_Out[robot_index].m_LowSpdFlag
                                      : !dcss.m_Out[0].m_LowSpdFlag ||
                                            !dcss.m_Out[1].m_LowSpdFlag;
            if (!is_in_position) {
                return true; // Motion complete
            }

            // Sleep for 10ms before checking again
            usleep(10000);
        }
    }

    static std::optional<std::array<double, 6>> fk(FX_INT32L robot_index,
                                                   FX_DOUBLE joints[7]) {
        // robot_index must be 0 or 1
        if (robot_index < 0 || robot_index >= 2) {
            return std::nullopt;
        }

        // Compute forward kinematics into a Matrix4
        static Matrix4 kine_pg;
        static Vect6 xyzabc; // Vect6 is a C-style array expected by the FX API

        if (FX_Robot_Kine_FK(robot_index, joints, kine_pg) == FX_FALSE) {
            return std::nullopt;
        }

        if (FX_Matrix42XYZABCDEG(kine_pg, xyzabc) == FX_FALSE) {
            return std::nullopt;
        }

        // Convert Vect6 (C array) to std::array for returning via std::optional
        std::array<double, 6> result;
        for (int i = 0; i < 6; ++i) {
            result[i] = xyzabc[i];
        }

        return result;
    }

    static std::optional<std::array<double, 7>>
    ik(FX_INT32L robot_index, FX_DOUBLE xyzabc[6],
       std::optional<std::array<double, 7>> ref_joints = std::nullopt) {
        // robot_index must be 0 or 1
        if (robot_index < 0 || robot_index >= 2) {
            return std::nullopt;
        }

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
        if (ref_joints.has_value()) {
            for (int i = 0; i < 7; ++i) {
                sp.m_Input_IK_RefJoint[i] = ref_joints.value()[i];
            }
        }

        // Compute inverse kinematics
        if (FX_Robot_Kine_IK(robot_index, &sp) == FX_FALSE) {
            return std::nullopt;
        }

        // Fill joint positions in response
        std::array<double, 7> result;
        for (int i = 0; i < 7; ++i) {
            result[i] = sp.m_Output_RetJoint[i];
        }

        return result;
    }

    static bool jointMoveCommand(int robot_index, FX_DOUBLE joints[2][7],
                                 int speed_ratio, int acc_ratio) {
        // NOTE: WRAP THIS FUNCTION WITH `OnClearSet` AND `OnSetSend`!

        // -1: both arms, 0: left arm, 1: right arm
        if (robot_index <= -1 || robot_index >= 2) {
            return false;
        }

        // Set speed and acceleration limits
        if (speed_ratio <= 0 || speed_ratio > 30) {
            speed_ratio = 30;
        }
        if (acc_ratio <= 0 || acc_ratio > 30) {
            acc_ratio = 30;
        }
        LEFT(OnSetJointLmt)(speed_ratio, acc_ratio);
        RIGHT(OnSetJointLmt)(speed_ratio, acc_ratio);

        // Set target joint positions
        if (robot_index == -1 || robot_index == 0) {
            LEFT(OnSetJointCmdPos)(joints[0]);
        }
        if (robot_index == -1 || robot_index == 1) {
            RIGHT(OnSetJointCmdPos)(joints[1]);
        }

        return true;
    }
};

} // namespace arms
