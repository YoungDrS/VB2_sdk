#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <thread>
#include <mutex>
#include <iomanip>
#include <ctime>
#include "Ti5BASIC.h"
#include "Ti5CAN_Driver.h"
#include "tool.h"

using namespace std;

class ServoSpeedLogger {
public:
    ServoSpeedLogger(const string& filename) : log_file(filename, ios::app) {}
    ~ServoSpeedLogger() { if (log_file.is_open()) log_file.close(); }

    void log(const string& msg) {
        lock_guard<mutex> lock(log_mutex);
        auto now = chrono::system_clock::to_time_t(chrono::system_clock::now());
        log_file << put_time(localtime(&now), "%F %T") << " " << msg << endl;
    }

    void log_speed(const vector<float>& speeds) {
        lock_guard<mutex> lock(log_mutex);
        auto now = chrono::system_clock::to_time_t(chrono::system_clock::now());
        log_file << put_time(localtime(&now), "%F %T") << " SPEED:";
        for (auto s : speeds) log_file << " " << fixed << setprecision(3) << s;
        log_file << endl;
    }

private:
    ofstream log_file;
    mutex log_mutex;
};

class ServoSpeedController {
public:
    ServoSpeedController(ArmSide side, int deviceInd, int canInd, uint8_t* canIdList, const string& logname)
        : side(side), deviceInd(deviceInd), canInd(canInd), canIdList(canIdList), logger(logname) {}

    bool init() {
        if (!Start()) {
            logger.log("CAN初始化失败");
            return false;
        }
        logger.log("CAN初始化成功");
        mechanical_arm_origin(side, deviceInd, canInd);
        logger.log("机械臂回零完成");
        return true;
    }

    // bool check_error() {
    //     int32_t error[IDNUM] = {0};
    //     get_mechanicalarm_status(side, deviceInd, canInd, error);
    //     for (int i = 0; i < IDNUM; ++i) {
    //         if (error[i] != 0) {
    //             logger.log("检测到电机异常，ID=" + to_string(i) + " 错误码=" + to_string(error[i]));
    //             clear_elc_error(side, deviceInd, canInd);
    //             logger.log("已尝试清除错误");
    //             return false;
    //         }
    //     }
    //     return true;
    // }

    // 设置为速度模式并下发目标速度（单位：度/秒）
    void set_speed(const vector<float>& speeds) {
        uint32_t speed_cmd[IDNUM] = {0};
        for (int i = 0; i < IDNUM; ++i) {
            // 速度单位转换：(目标转速（度每秒）*减速比*100)/360
            speed_cmd[i] = static_cast<uint32_t>(speeds[i] * 101 * 100 / 360.0); // 101为减速比
        }
        sendCanCommand(deviceInd, canInd, IDNUM, canIdList, SET_MOTOR_SPEED, speed_cmd);
        logger.log_speed(speeds);
    }

    // 速度采集与日志（通过角度差分估算速度，单位：rad/s）
    void monitor_speed(int interval_ms = 10, int duration_ms = 5000) {
        int times = duration_ms / interval_ms;
        for (int t = 0; t < times; ++t) {
            float angle1[IDNUM], angle2[IDNUM];
            get_current_angle(side, angle1, deviceInd, canInd);
            this_thread::sleep_for(chrono::milliseconds(interval_ms));
            get_current_angle(side, angle2, deviceInd, canInd);
            vector<float> speeds;
            for (int i = 0; i < IDNUM; ++i) {
                speeds.push_back((angle2[i] - angle1[i]) / (interval_ms / 1000.0)); // rad/s
            }
            logger.log_speed(speeds);
        }
    }

private:
    ArmSide side;
    int deviceInd, canInd;
    uint8_t* canIdList;
    ServoSpeedLogger logger;
};

int main() {
    // 获取canId数组（由SDK全局变量提供）
    extern uint8_t l_id[IDNUM], r_id[IDNUM];

    // 左臂控制器
    ServoSpeedController left_ctrl(LEFT_ARM, 0, 0, l_id, "left_speed.log");
    // 右臂控制器
    ServoSpeedController right_ctrl(RIGHT_ARM, 1, 1, r_id, "right_speed.log");

    // 初始化
    if (!left_ctrl.init()) return -1;
    if (!right_ctrl.init()) return -1;

    // // 异常检测
    // if (!left_ctrl.check_error()) return -2;
    // if (!right_ctrl.check_error()) return -2;

    // 设置目标速度（单位：度/秒，7个关节）
    vector<float> left_speed = {0, 10, 0, 0, 0, 0, 0};
    vector<float> right_speed = {0, 15, 0, 0, 0, 0, 0};
    left_ctrl.set_speed(left_speed);
    right_ctrl.set_speed(right_speed);

    // 启动两个线程分别监控左右臂速度
    std::thread left_monitor([&](){ left_ctrl.monitor_speed(5, 2000); });
    std::thread right_monitor([&](){ right_ctrl.monitor_speed(5, 2000); });

    left_monitor.join();
    right_monitor.join();

    Exit();
    return 0;
}