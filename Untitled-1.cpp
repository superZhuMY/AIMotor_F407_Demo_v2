#include <cmath>
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "jaka_controller_tcp/Command.h"
#include "modbus/modbus.h"
#include <csignal>
#include <boost/thread.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/thread/mutex.hpp>
#include <cstring> 
#include <numeric> 
#include <serial/serial.h>
#include <cerrno>
boost::mutex mtx;
boost::condition_variable cv;
bool received_command = false;
const double deg2rad = M_PI / 180;
modbus_t* ctx[5] = {nullptr};
serial::Serial serial_client;
uint16_t close_power[1] = {0x0000};
uint16_t on_power[1] = {0x0001};
std::string command_Topic_Name = "command_L";
ros::Subscriber cmd_sub;
sensor_msgs::JointState joint_state_msg;
ros::Publisher jaka_joint_state_pub;
constexpr const char* ARM_LABEL = "L";
constexpr const char* MODBUS_PORT = "/dev/ttyUSB1";
constexpr const char* SERIAL_PORT = "/dev/ttyUSB0";
constexpr double LINEAR_FACTORS[3] = {4.0, 4.75, 2.0};

std::string generate_command(int motor_id, int data) {
    // 固定前导标识和电机号
    std::vector<uint8_t> command = {0x3E, 0xA3, static_cast<uint8_t>(motor_id & 0xFF), 0x08};
    
    // 校验和
    uint8_t checksum = (command[0] + command[1] + command[2] + command[3]) & 0xFF;
    command.push_back(checksum);

    // 将 data 转换为 4 字节小端存储并添加额外字节
    std::vector<uint8_t> data_bytes(4);
    memcpy(data_bytes.data(), &data, sizeof(data));
    data_bytes.insert(data_bytes.end(), 4, data >= 0 ? 0x00 : 0xFF); // 添加额外字节

    // 计算第15位和最终命令
    uint8_t sum_data = std::accumulate(data_bytes.begin(), data_bytes.end(), uint8_t(0)) & 0xFF;
    command.insert(command.end(), data_bytes.begin(), data_bytes.end());
    command.push_back(sum_data);

    // 转换为 unsigned char 字符串
    return std::string(command.begin(), command.end());
}
// Function to generate a command to read data from the motor
std::string generate_command_read(int motor_id) {
    std::vector<uint8_t> command = {0x3E, 0x92, static_cast<uint8_t>(motor_id & 0xFF), 0x00};
    uint8_t checksum = (command[0] + command[1] + command[2] + command[3]) & 0xFF;
    command.push_back(checksum);
    return std::string(command.begin(), command.end());
}
// Function to parse a command (hex string) and extract motor ID, data, and checksum
int parse_command(const std::string &command_bin) {

    uint8_t bytes[6] = {
        static_cast<uint8_t>(command_bin[5]),
        static_cast<uint8_t>(command_bin[6]),
        static_cast<uint8_t>(command_bin[7]),
        static_cast<uint8_t>(command_bin[8]),
        static_cast<uint8_t>(command_bin[9]),
        static_cast<uint8_t>(command_bin[10])
    };
    int num = 0;
    num |= bytes[0];  // 最低字节
    num |= bytes[1] << 8;
    num |= bytes[2] << 16;
    num |= bytes[3] << 24;
    return  num;
}

bool read_serial_motor(int motor_id, int& raw_value) {
    serial_client.write(generate_command_read(motor_id));
    ros::Duration(0.001).sleep();
    const std::string response = serial_client.read(14);
    if (response.size() < 11) {
        ROS_WARN_THROTTLE(1.0, "[%s] short serial packet motor=%d bytes=%zu",
                          ARM_LABEL, motor_id, response.size());
        return false;
    }
    raw_value = parse_command(response);
    return true;
}




void close_modbus_connections() {
    for (auto& c : ctx) {
        if (c) {
            modbus_write_registers(c,0x0303,0x01,close_power);
            ros::Duration(0.11).sleep();
            modbus_write_registers(c,0x0303,0x01,close_power);
            ros::Duration(0.11).sleep();
            modbus_close(c);
            modbus_free(c);
            c = nullptr;
        }
    }
}
void signalHandler(int signum) {
    ROS_INFO("Received signal %d, closing Modbus connections", signum);
    close_modbus_connections();
    exit(signum);
}
void decimalToHexs(unsigned int decimal,uint16_t *data){
	unsigned int high = (decimal >> 16) & 0xFFFF;
	unsigned int low = decimal & 0xFFFF; 
	data[0]=low;
	data[1]=high;
	return;
}

void initialize_modbus() {
    for (int i = 0; i < 3; ++i) {
        ctx[i] = modbus_new_rtu(MODBUS_PORT, 57600, 'N', 8,2);
        if (!ctx[i] || modbus_set_slave(ctx[i], i + 1) == -1 || modbus_connect(ctx[i]) == -1) {
            ROS_ERROR("[%s] Modbus motor J%d connection failed: port=%s slave=%d error=%s",
                      ARM_LABEL, i + 1, MODBUS_PORT, i + 1, modbus_strerror(errno));
            close_modbus_connections();
            exit(1);
        }
        ROS_INFO("[%s] Modbus motor J%d connected: port=%s slave=%d",
                 ARM_LABEL, i + 1, MODBUS_PORT, i + 1);
        uint16_t buffer[1];
        modbus_read_registers(ctx[i], 0x0303, 0x01, buffer);
        if (buffer[0] == 0) {
            modbus_write_registers(ctx[i], 0x0303, 0x01, on_power);
        }
    }

    try
    {
        // 串口初始化
        serial_client.setPort(SERIAL_PORT);
        serial_client.setBaudrate(115200);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(20);
        serial_client.setTimeout(timeout);
        serial_client.open();

        if (serial_client.isOpen())
        {
            ROS_INFO("[%s] Serial bus connected: port=%s motors=J4,J5,J6", ARM_LABEL, SERIAL_PORT);
        }
        else
        {
            ROS_ERROR("Failed to open serial port");
            exit(1);
        }
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR("Failed to connect to serial port: %s", e.what());
        exit(1);
    }

    joint_state_msg.name.resize(6);
    joint_state_msg.position.resize(6);//determin the number of the vector size
    for(int i=0;i<6;i++)
    {
    joint_state_msg.name[i]="L_Joint_"+std::to_string(i+1);  // named the joints name
    joint_state_msg.position[i]=0;  // init the position of joints
    }
}
unsigned int hexToDecimal1(uint16_t *data) {
        unsigned int low = data[0];
        unsigned int high = data[1];
        return (high << 16) | low;
    }
int hexToDecimal(uint16_t *data) 
    {
        uint16_t low = data[0];
        uint16_t high = data[1];
        uint16_t data1[2];
        if (high>61440){
            data1[0]=0xFFFF-data[0]+0x0001;
            data1[1]=0xFFFF-data[1];
            int aa=hexToDecimal1(data1);
            int bb=-aa;
            return bb;
        }
        else{
            return hexToDecimal1(data);
        }
    }

void update_modbus_joint(int index) {
    uint16_t buffer[2];
    ros::Duration(0.001).sleep();
    if (modbus_read_registers(ctx[index], 0x0B07, 0x02, buffer) == 2) {
        joint_state_msg.position[index] = hexToDecimal(buffer) / 10000.0 * LINEAR_FACTORS[index] / 1000.0;
    } else {
        ROS_WARN_THROTTLE(1.0, "[%s] Modbus motor J%d read failed: %s",
                          ARM_LABEL, index + 1, modbus_strerror(errno));
    }
}

void write_modbus_joint(int index, float command) {
    uint16_t buffer[2];
    modbus_write_registers(ctx[index], 0x0305, 1, close_power);
    ros::Duration(0.001).sleep();
    decimalToHexs(command, buffer);
    modbus_write_registers(ctx[index], 0x110C, 2, buffer);
    ros::Duration(0.001).sleep();
    modbus_write_registers(ctx[index], 0x0305, 1, on_power);
    update_modbus_joint(index);
}

void update_serial_joint(int index, int motor_id) {
    int raw = 0;
    if (read_serial_motor(motor_id, raw)) {
        joint_state_msg.position[index] = raw / 1000.0 * deg2rad;
    }
}

void update_coupled_serial_joints() {
    int joint5 = 0;
    int joint6 = 0;
    const bool ok5 = read_serial_motor(5, joint5);
    ros::Duration(0.001).sleep();
    const bool ok6 = read_serial_motor(6, joint6);

    if (ok5) {
        joint_state_msg.position[4] = joint5 / 1000.0 / (5.0 / 3.0) * deg2rad;
    }
    if (ok5 && ok6) {
        joint_state_msg.position[5] = (joint6 + joint5) / 1000.0 / (20.0 / 9.0) * deg2rad;
    }
}

void write_serial_motor(int motor_id, float command) {
    serial_client.write(generate_command(motor_id, command));
    ros::Duration(0.001).sleep();
}

void ReadThread(){
    ros::Rate loop_rate1(20);
    while (ros::ok()) 
    {
        boost::unique_lock<boost::mutex> lock(mtx);
        cv.wait(lock, []{return !received_command;});  // Wait for the signal to continue
        lock.unlock();
        for (int i = 0; i < 3; ++i) {
            update_modbus_joint(i);
        }
        update_serial_joint(3, 4);
        update_coupled_serial_joints();
        joint_state_msg.header.stamp=ros::Time::now();
        jaka_joint_state_pub.publish(joint_state_msg);
        loop_rate1.sleep();   
    }
}  



void write_joint_data(jaka_controller_tcp::Command::ConstPtr msg) {
    write_serial_motor(5, msg->joint[4]);
    write_serial_motor(6, msg->joint[5]);
    update_coupled_serial_joints();

    write_serial_motor(4, msg->joint[3]);
    update_serial_joint(3, 4);

    for (int i = 2; i >= 0; --i) {
        write_modbus_joint(i, msg->joint[i]);
    }
    ros::Duration(0.001).sleep();
    joint_state_msg.header.stamp=ros::Time::now();
    jaka_joint_state_pub.publish(joint_state_msg);
}



void CommandCallback(const jaka_controller_tcp::Command::ConstPtr &msg){
    boost::lock_guard<boost::mutex> guard(mtx);
    if (msg->io == 1) {
        received_command = false;
        ROS_INFO("[L] Received trajectory-end command io=1, resuming state reads");
        cv.notify_one();
        return;
    }

    if (msg->joint.size() < 6) {
        ROS_ERROR("[L] Received motion command with too few joints: %zu", msg->joint.size());
        return;
    }
    ROS_INFO("[L] Received motion command io=%d joints=[%.3f %.3f %.3f %.3f %.3f %.3f]",
             msg->io,
             msg->joint[0], msg->joint[1], msg->joint[2],
             msg->joint[3], msg->joint[4], msg->joint[5]);
    received_command = true;
    write_joint_data(msg);
}

/* =====================================================================
 * STM32 二进制协议解析器（与下位机 docs/README_J456_DRIVER.md §6 一致）
 *
 * 帧格式: AA 55 | VERSION(1)=0x01 | CMD(1) | SEQ(2,LE) | LEN(2,LE)
 *         | PAYLOAD(LEN) | CRC16(2,LE)
 *   STATE 帧: CMD=0x81, LEN=60, 总长 70
 *     PAYLOAD: seq(2) | control_state(1) | fault(1) | valid(2) | enabled(2)
 *              | axis_status(4,uint32) | 12×int32(LE) 关节侧位置
 *              L_J1µm..L_J6µrad, R_J1µm..R_J6µrad
 *   axis_status: uint32，每轴 2bit（bit0=online, bit1=fault），
 *                bit = side*6 + joint（L_J1..L_J6, R_J1..R_J6）
 *   CRC16: Modbus（poly 0xA001，初值 0xFFFF），覆盖 VERSION..PAYLOAD 末尾
 *          = 6+LEN 字节，线上低字节在前。
 * ===================================================================== */
namespace stm32_bin {

constexpr uint8_t  BIN_SOF0     = 0xAA;
constexpr uint8_t  BIN_SOF1     = 0x55;
constexpr uint8_t  BIN_VERSION  = 0x01;
constexpr uint8_t  CMD_STATE    = 0x81;
constexpr uint16_t STATE_LEN    = 60;   // 新版 STATE 帧长度
constexpr uint16_t STATE_TOTAL  = 10 + STATE_LEN;   // 总长 70

struct StateFrame {
    uint16_t seq;
    uint8_t  control_state;   // 0=DISABLED 1=ENABLED 2=STOPPED 3=FAULT
    uint8_t  fault;           // bit0 超时 bit1 溢出 bit2 ENABLE失败 bit3 未确认停止 bit4 内部
    uint16_t valid;           // bit0..11: L_J1..L_J6, R_J1..R_J6（真实回读）
    uint16_t enabled;         // bit0..11 同序（回包确认使能）
    uint32_t axis_status;     // 每轴 2bit: bit0=online bit1=fault（24bit 有效）
    int32_t  pos[12];         // L_J1µm..L_J6µrad, R_J1µm..R_J6µrad（LE）
    bool     ok;
    StateFrame() : seq(0), control_state(0), fault(0), valid(0), enabled(0),
                   axis_status(0), ok(false) { memset(pos, 0, sizeof(pos)); }
};

// Modbus CRC-16（与 Aimotor_CRC16 一致）
inline uint16_t crc16(const uint8_t* buf, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= buf[i];
        for (int b = 0; b < 8; ++b)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
    return crc;
}

// 解析下位机返回的 STATE 帧；非法帧返回 ok=false。
// 仅接受固定 LEN=60（拒绝旧 LEN=56/58，避免协议漂移）。
inline bool parse_state(const std::vector<uint8_t>& frame, StateFrame& st) {
    if (frame.size() < STATE_TOTAL) return false;
    if (frame[0] != BIN_SOF0 || frame[1] != BIN_SOF1) return false;
    if (frame[2] != BIN_VERSION || frame[3] != CMD_STATE) return false;
    uint16_t len = static_cast<uint16_t>(frame[6]) | (static_cast<uint16_t>(frame[7]) << 8);
    if (len != STATE_LEN) return false;                       // 只接受 LEN=60
    if (frame.size() < static_cast<size_t>(STATE_TOTAL)) return false;
    uint16_t got = static_cast<uint16_t>(frame[8 + len]) |
                   (static_cast<uint16_t>(frame[9 + len]) << 8);
    if (crc16(frame.data() + 2, 6 + len) != got) return false;

    st.seq = static_cast<uint16_t>(frame[8]) | (static_cast<uint16_t>(frame[9]) << 8);
    st.control_state = frame[10];
    st.fault = frame[11];
    st.valid = static_cast<uint16_t>(frame[12]) | (static_cast<uint16_t>(frame[13]) << 8);
    st.enabled = static_cast<uint16_t>(frame[14]) | (static_cast<uint16_t>(frame[15]) << 8);
    st.axis_status = static_cast<uint32_t>(frame[16]) |
                     (static_cast<uint32_t>(frame[17]) << 8) |
                     (static_cast<uint32_t>(frame[18]) << 16) |
                     (static_cast<uint32_t>(frame[19]) << 24);
    for (int i = 0; i < 12; ++i) {
        size_t o = 20 + 4 * static_cast<size_t>(i);
        uint32_t raw = static_cast<uint32_t>(frame[o]) |
                       (static_cast<uint32_t>(frame[o + 1]) << 8) |
                       (static_cast<uint32_t>(frame[o + 2]) << 16) |
                       (static_cast<uint32_t>(frame[o + 3]) << 24);
        st.pos[i] = static_cast<int32_t>(raw);   // 关节侧 µm/µrad，LE
    }
    st.ok = true;
    return true;
}

} // namespace stm32_bin
int main(int argc, char **argv)
{
	ros::init(argc, argv, "jaka_send_node_L");
	ros::NodeHandle nh;
    signal(SIGINT, signalHandler);
    initialize_modbus();
	cmd_sub=nh.subscribe(command_Topic_Name,20,CommandCallback);
    jaka_joint_state_pub=nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    // ros::Rate loop_rate(30);
    boost::thread modbus_thread(ReadThread);
    while (ros::ok())
    {
        ros::spinOnce();
        // loop_rate.sleep();
    }
    // ros::spin();
    modbus_thread.join();
    close_modbus_connections();
    return 0;
}