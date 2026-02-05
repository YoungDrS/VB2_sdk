#include "arm_control.hpp"
#include <iostream>
#include <sstream>

MyArmController::MyArmController(ArmSide side, int deviceInd, int canInd, uint8_t* canIdList)
    : side_(side), deviceInd_(deviceInd), canInd_(canInd), canIdList_(canIdList),
      running_state_(false), running_pos_(false), running_spd_(false) {}

MyArmController::~MyArmController() {
    stopAllThreads();
    stop();
}

bool MyArmController::init() {
    if (!Start()) return false;
    mechanical_arm_origin(side_, deviceInd_, canInd_);
    return true;
}

void MyArmController::stop() {
    brake(side_, deviceInd_, canInd_);
    Exit();
}

ArmState MyArmController::getState() {
    std::lock_guard<std::mutex> lock(mtx_state_);
    return state_;
}

void MyArmController::setPositionTarget(const std::vector<float>& target) {
    std::lock_guard<std::mutex> lock(mtx_pos_);
    pos_target_ = target;
}

void MyArmController::setSpeedTarget(const std::vector<float>& target) {
    std::lock_guard<std::mutex> lock(mtx_spd_);
    spd_target_ = target;
}

void MyArmController::startStateThread() {
    running_state_ = true;
    th_state_ = std::thread(&MyArmController::stateThreadFunc, this);
}

void MyArmController::startPositionThread() {
    running_pos_ = true;
    th_pos_ = std::thread(&MyArmController::positionThreadFunc, this);
}

void MyArmController::startSpeedThread() {
    running_spd_ = true;
    th_spd_ = std::thread(&MyArmController::speedThreadFunc, this);
}

void MyArmController::stopAllThreads() {
    running_state_ = false;
    running_pos_ = false;
    running_spd_ = false;
    if (th_state_.joinable()) th_state_.join();
    if (th_pos_.joinable()) th_pos_.join();
    if (th_spd_.joinable()) th_spd_.join();
}

void MyArmController::stateThreadFunc() {
    while (running_state_) {
        float joints[7], pose[7];
        get_current_angle(side_, joints, deviceInd_, canInd_);
        get_current_pose(side_, pose, deviceInd_, canInd_);
        {
            std::lock_guard<std::mutex> lock(mtx_state_);
            for (int i = 0; i < 7; ++i) state_.joint_angles[i] = joints[i];
            for (int i = 0; i < 7; ++i) state_.end_pose[i] = pose[i];
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void MyArmController::positionThreadFunc() {
    while (running_pos_) {
        std::vector<float> target;
        {
            std::lock_guard<std::mutex> lock(mtx_pos_);
            target = pos_target_;
        }
        if (target.size() == 7)
            joint_to_move(side_, target.data(), deviceInd_, canInd_);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

void MyArmController::speedThreadFunc() {
    while (running_spd_) {
        std::vector<float> target;
        {
            std::lock_guard<std::mutex> lock(mtx_spd_);
            target = spd_target_;
        }
        if (target.size() == 7) {
            uint32_t speed_cmd[7];
            for (int i = 0; i < 7; ++i)
                speed_cmd[i] = static_cast<uint32_t>(target[i] * 101 * 100 / 360.0);
            sendCanCommand(deviceInd_, canInd_, 7, canIdList_, SET_MOTOR_SPEED, speed_cmd);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

extern uint8_t l_id[7], r_id[7];

void print_menu() {
    std::cout << "==== 机械臂控制测试 ====\n";
    std::cout << "1. 获取左臂状态\n";
    std::cout << "2. 获取右臂状态\n";
    std::cout << "3. 左臂位置模式控制\n";
    std::cout << "4. 右臂位置模式控制\n";
    std::cout << "5. 左臂速度模式控制\n";
    std::cout << "6. 右臂速度模式控制\n";
    std::cout << "0. 退出\n";
    std::cout << "请输入指令编号: ";
}


int main() {
    MyArmController left_ctrl(LEFT_ARM, 0, 0, l_id);
    MyArmController right_ctrl(RIGHT_ARM, 1, 1, r_id);

    if (!left_ctrl.init() || !right_ctrl.init()) {
        std::cerr << "机械臂初始化失败！" << std::endl;
        return -1;
    }

    left_ctrl.startStateThread();
    right_ctrl.startStateThread();
    left_ctrl.startPositionThread();
    right_ctrl.startPositionThread();
    left_ctrl.startSpeedThread();
    right_ctrl.startSpeedThread();

    bool running = true;
    while (running) {
        print_menu();
        int cmd = -1;
        std::cin >> cmd;
        if (cmd == 0) {
            running = false;
        } else if (cmd == 1) {
            auto state = left_ctrl.getState();
            std::cout << "左臂关节角度: ";
            for (auto v : state.joint_angles) std::cout << v << " ";
            std::cout << "\n左臂末端位姿: ";
            for (auto v : state.end_pose) std::cout << v << " ";
            std::cout << std::endl;
        } else if (cmd == 2) {
            auto state = right_ctrl.getState();
            std::cout << "右臂关节角度: ";
            for (auto v : state.joint_angles) std::cout << v << " ";
            std::cout << "\n右臂末端位姿: ";
            for (auto v : state.end_pose) std::cout << v << " ";
            std::cout << std::endl;
        } else if (cmd == 3) {
            std::cout << "输入7个目标角度（空格分隔，单位rad）: ";
            std::vector<float> target(7);
            for (int i = 0; i < 7; ++i) std::cin >> target[i];
            left_ctrl.setPositionTarget(target);
        } else if (cmd == 4) {
            std::cout << "输入7个目标角度（空格分隔，单位rad）: ";
            std::vector<float> target(7);
            for (int i = 0; i < 7; ++i) std::cin >> target[i];
            right_ctrl.setPositionTarget(target);
        } else if (cmd == 5) {
            std::cout << "输入7个目标速度（空格分隔，单位deg/s）: ";
            std::vector<float> target(7);
            for (int i = 0; i < 7; ++i) std::cin >> target[i];
            left_ctrl.setSpeedTarget(target);
        } else if (cmd == 6) {
            std::cout << "输入7个目标速度（空格分隔，单位deg/s）: ";
            std::vector<float> target(7);
            for (int i = 0; i < 7; ++i) std::cin >> target[i];
            right_ctrl.setSpeedTarget(target);
        } else {
            std::cout << "未知指令！" << std::endl;
        }
    }

    left_ctrl.stopAllThreads();
    right_ctrl.stopAllThreads();
    left_ctrl.stop();
    right_ctrl.stop();
    std::cout << "退出程序。" << std::endl;
    return 0;
}