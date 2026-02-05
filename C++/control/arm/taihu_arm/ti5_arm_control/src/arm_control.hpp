#pragma once
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>
#include <array>
#include <condition_variable>
#include "Ti5BASIC.h"
#include "Ti5CAN_Driver.h"
#include "tool.h"

struct ArmState {
    std::array<float, 7> joint_angles;
    std::array<float, 7> end_pose;
};

class MyArmController {
public:
    MyArmController(ArmSide side, int deviceInd, int canInd, uint8_t* canIdList);
    ~MyArmController();

    bool init();
    void stop();

    // 状态获取
    ArmState getState();

    // 控制接口
    void setPositionTarget(const std::vector<float>& target);
    void setSpeedTarget(const std::vector<float>& target);

    // 线程控制
    void startStateThread();
    void startPositionThread();
    void startSpeedThread();
    void stopAllThreads();

private:
    void stateThreadFunc();
    void positionThreadFunc();
    void speedThreadFunc();

    ArmSide side_;
    int deviceInd_, canInd_;
    uint8_t* canIdList_;

    std::atomic<bool> running_state_, running_pos_, running_spd_;
    std::thread th_state_, th_pos_, th_spd_;

    ArmState state_;
    std::mutex mtx_state_, mtx_pos_, mtx_spd_;
    std::vector<float> pos_target_, spd_target_;
};