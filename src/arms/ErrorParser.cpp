#include "ErrorParser.hpp"
#include "libjaka/jkerr.h"
#include <map>

namespace arms {

const std::map<int, std::string> JakaErrorMap::error_map_ = {
    {ERR_SUCC, "0: 调用成功"},
    {ERR_FUCTION_CALL_ERROR,
     "2: 异常调用 — 控制器接口调用失败或不支持该操作，检查方法和参数是否正确"},
    {ERR_INVALID_HANDLER,
     "-1: 无效的控制句柄 — 可能未登录/未初始化或句柄已被关闭，检查登录状态"},
    {ERR_INVALID_PARAMETER,
     "-2: 无效的参数 — 检查传入参数范围和类型（如速度/加速度/索引等）"},
    {ERR_COMMUNICATION_ERR,
     "-3: 通信连接错误 — 检查与控制器的网络/串口连接是否正常"},
    {ERR_KINE_INVERSE_ERR,
     "-4: 逆解失败 — "
     "给定目标位姿无法求解逆解，检查目标是否可达或是否超出工作空间"},
    {ERR_EMERGENCY_PRESSED,
     "-5: 急停开关被按下 — 请确认急停是否复位并清除相关错误"},
    {ERR_NOT_POWERED, "-6: 机器人未上电 — 请先上电"},
    {ERR_NOT_ENABLED, "-7: 机器人未使能 — 请先使能机器人"},
    {ERR_DISABLE_SERVOMODE,
     "-8: 机器人没有进入 servo 模式 — 需要在伺服控制前启用伺服模式"},
    {ERR_NOT_OFF_ENABLE, "-9: 机器人没有关闭使能 — 请在下电前先下使能"},
    {ERR_PROGRAM_IS_RUNNING,
     "-10: 程序正在运行 — 当前不允许此操作，等待程序结束或停止程序后重试"},
    {ERR_CANNOT_OPEN_FILE,
     "-11: 无法打开文件 — 检查文件路径和权限，确认文件存在"},
    {ERR_MOTION_ABNORMAL,
     "-12: 运动过程中发生异常 — "
     "检查软限位、碰撞、机械或传感器状态并查看控制器错误日志"},
    {ERR_FTP_PREFROM, "-14: FTP 操作异常 — 检查 FTP 配置和网络连通性"},
    {ERR_VALUE_OVERSIZE,
     "-15: 值过大(socket/message) — 检查上报的数据或消息长度是否超过限制"},
    {ERR_TROQUE_CONTROL_NOT_ENABLE,
     "-22: 电流环未使能 — 需要先启用电流环才能进行相关操作"},
    {ERR_ROBOT_NOT_STOPPED, "-23: 机器人正在运动 — 请停止机器人后再执行该操作"},
    {ERR_INVERSE_OUT_OF_LIMIT,
     "-24: 逆解超出软限位 — 目标位姿违反软限位，请修改目标或软限设置"},
    {ROBOT_IN_ERROR, "0xFF: 机器人处于错误状态 — 请查询控制器错误并清除后重试"},
};

const std::map<int, std::string> TjErrorMap::error_map_ = {
    {ERR_SUCC, "0: 调用成功"},
};

} // namespace arms
