#include <MarvinSDK.h>
#include <iostream>
#include <stdexcept>
#include <thread>

#undef LEFT
#undef RIGHT
#define LEFT(func) func##_A
#define RIGHT(func) func##_B

int main(int argc, char *argv[]) {
    if (argc < 2 || argc > 3) {
        std::cerr << "Usage: " << argv[0] << " <mode-number> [robot-ip]"
                  << std::endl;
        std::cerr << "0: 下使能, 1: 位置, 2: PVT, 3: 扭矩, 4: 协作释放"
                  << std::endl;
        return 1;
    }

    int num;
    try {
        num = std::stoi(argv[1]);
    } catch (const std::invalid_argument &e) {
        std::cerr << "Invalid input number: " << argv[1] << std::endl;
        std::cerr << "0: 下使能, 1: 位置, 2: PVT, 3: 扭矩, 4: 协作释放"
                  << std::endl;
        return 1;
    } catch (const std::out_of_range &e) {
        std::cerr << "Invalid input number: " << argv[1] << std::endl;
        std::cerr << "0: 下使能, 1: 位置, 2: PVT, 3: 扭矩, 4: 协作释放"
                  << std::endl;
        return 1;
    }

    if (num < 0 || num > 4) {
        std::cerr << "Incorrect mode number: " << num << std::endl;
        std::cerr << "0: 下使能, 1: 位置, 2: PVT, 3: 扭矩, 4: 协作释放"
                  << std::endl;
        return 1;
    }

    const char *ip_address = argc == 3 ? argv[2] : "192.168.1.190";
    int octets[4];
    if (sscanf(ip_address, "%d.%d.%d.%d", &octets[0], &octets[1], &octets[2],
               &octets[3]) != 4) {
        std::cerr << "Invalid IP address: " << ip_address << std::endl;
        std::cerr << "Default IP address: " << "192.168.1.190" << std::endl;
        return 1;
    }

    for (int i = 0; i < 4; ++i) {
        if (octets[i] < 0 || octets[i] > 255) {
            std::cerr << "Invalid IP address: " << ip_address << std::endl;
            std::cerr << "Default IP address: " << "192.168.1.190" << std::endl;
            return 1;
        }
    }

    bool init = OnLinkTo(octets[0], octets[1], octets[2], octets[3]);
    if (!init) {
        std::cerr << "Robot arms initialization failed: port already in use"
                  << std::endl;
        return 1;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    OnClearSet();
    OnClearErr_A();
    OnClearErr_B();
    OnSetSend();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    int motion_tag = 0;
    int frame_update = 0;
    DCSS dcss;

    for (int i = 0; i < 5; i++) {
        OnGetBuf(&dcss);
        if (dcss.m_Out[0].m_OutFrameSerial != 0 &&
            frame_update != dcss.m_Out[0].m_OutFrameSerial) {
            motion_tag++;
            frame_update = dcss.m_Out[0].m_OutFrameSerial;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (motion_tag <= 0) {
        std::cerr << "Robot arms initialization failed: no data received"
                  << std::endl;
        return 1;
    }

    OnClearSet();
    LEFT(OnSetTargetState)(num);
    RIGHT(OnSetTargetState)(num);
    OnSetSend();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    return 0;
}
