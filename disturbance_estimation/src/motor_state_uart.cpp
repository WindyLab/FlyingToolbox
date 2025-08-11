#include "motor_state_uart.h"

disturbance_estimation::motor_state motor_data;
serial::Serial serial_;
void Motor::parseData(const std::vector<uint8_t>& data) {
  if (data.size() < 18) {
    std::cerr << "Invalid data frame." << std::endl;
    return;
  }
  uint16_t temp;
  channel = data[0];
  packetNumber_ = (data[1] << 8) | data[2];
  temp = (data[3] << 8) | data[4];
  rcThrottle_ = std::round(temp * 100 / 1024 * 10.0) / 10.0; 
  temp = (data[5] << 8) | data[6];
  actualThrottle_ =
      std::round(temp * 100 / 1024 * 10.0) / 10.0; 
  temp = ((data[7] << 8) | data[8]);
  electricalSpeed_ = temp * 10 / 14;
  temp = (data[9] << 8) | data[10];
  busVoltage_ = temp / 10.0; 
  temp = ((data[11] << 8) | data[12]);
  if (temp >= 32768) temp -= 65536;
  busCurrent_ = std::round(temp / 64.0 * 10) / 10.0; 
  busCurrent_ = busCurrent_ < 0 ? 0 : busCurrent_;
  busCurrent_ =
      busCurrent_ > 100 ? 0 : busCurrent_; 
  temp = (data[13] << 8) | data[14];
  if (temp >= 32768) temp -= 65536;
  phaseCurrent_ = std::round(temp / 64.0 * 10) / 10.0; 
  phaseCurrent_ = phaseCurrent_ < 0 ? 0 : phaseCurrent_;
  phaseCurrent_ = phaseCurrent_ > 100
                      ? 0
                      : phaseCurrent_; 
  mosTemperature_ = data[15];
  capacitorTemperature_ = data[16];
  statusCode_ = (data[17] << 8) | data[18];
}

void Motor::printData() const {
  std::cout << "Channel: " << static_cast<int>(channel) << std::endl;
  std::cout << "Packet Number: " << packetNumber_ << std::endl;
  std::cout << "RC Throttle: " << rcThrottle_ << std::endl;
  std::cout << "Actual Throttle: " << actualThrottle_ << std::endl;
  std::cout << "Electrical Speed: " << electricalSpeed_ << std::endl;
  std::cout << "Bus Voltage: " << busVoltage_ << std::endl;
  std::cout << "Bus Current: " << busCurrent_ << std::endl;
  std::cout << "Phase Current: " << phaseCurrent_ << std::endl;
  std::cout << "MOS Temperature: " << static_cast<int>(mosTemperature_)
            << std::endl;
  std::cout << "Capacitor Temperature: "
            << static_cast<int>(capacitorTemperature_) << std::endl;
  std::cout << "Status Code: " << statusCode_ << std::endl;
}

uint16_t UpdateCRC16(uint16_t crcIn, uint8_t byte) {
  uint32_t crc = crcIn;
  uint32_t in = byte | 0x100;
  do {
    crc <<= 1;
    in <<= 1;
    if (in & 0x100) ++crc;
    if (crc & 0x10000) crc ^= 0x1021;
  } while (!(in & 0x10000));
  return crc & 0xffffu;
}

uint16_t CalCRC16(const uint8_t* data, uint32_t size) {
  uint32_t crc = 0;
  const uint8_t* dataEnd = data + size;
  while (data < dataEnd) crc = UpdateCRC16(crc, *data++);
  crc = UpdateCRC16(crc, 0);
  crc = UpdateCRC16(crc, 0);
  return crc & 0xffffu;
}

void ros_pub_data(Motor* motor) {
  motor_data.stamp = ros::Time::now();
  for (int i = 0; i < 8; i++) {
    motor_data.packetNumber[i] = motor[i].get_packetNumber();
    motor_data.rcThrottle[i] = motor[i].get_rcThrottle();
    motor_data.actualThrottle[i] = motor[i].get_actualThrottle();
    motor_data.electricalSpeed[i] = motor[i].get_electricalSpeed();
    motor_data.busVoltage[i] = motor[i].get_busVoltage();
    motor_data.busCurrent[i] = motor[i].get_busCurrent();
    motor_data.phaseCurrent[i] = motor[i].get_phaseCurrent();
    motor_data.mosTemperature[i] = motor[i].get_mosTemperature();
    motor_data.capacitorTemperature[i] = motor[i].get_capacitorTemperature();
    motor_data.statusCode[i] = motor[i].get_statusCode();
  }
}
void parseSerialData(const std::vector<uint8_t>& data) {
  uint8_t frameLength = data[1];
  uint8_t protocolVersion = data[2];
  uint8_t command = data[3];
  uint16_t packetCount = (data[4] << 8) | data[5];

  // std::cout << "Frame Length: " << static_cast<int>(frameLength) <<
  // std::endl; std::cout << "Protocol Version: " <<
  // static_cast<int>(protocolVersion) << std::endl; std::cout << "Command: " <<
  // static_cast<int>(command) << std::endl; std::cout << "Packet Count: " <<
  // packetCount << std::endl;

  Motor motor[8];
  motor[0].parseData(std::vector<uint8_t>(data.begin() + 6, data.begin() + 25));
  // std::cout << std::endl << "Motor 1 Data: " << std::endl;
  // motor[0].printData();

  motor[1].parseData(
      std::vector<uint8_t>(data.begin() + 25, data.begin() + 44));
  motor[2].parseData(
      std::vector<uint8_t>(data.begin() + 44, data.begin() + 63));
  motor[3].parseData(
      std::vector<uint8_t>(data.begin() + 63, data.begin() + 82));
  motor[4].parseData(
      std::vector<uint8_t>(data.begin() + 82, data.begin() + 101));
  motor[5].parseData(
      std::vector<uint8_t>(data.begin() + 101, data.begin() + 120));
  motor[6].parseData(
      std::vector<uint8_t>(data.begin() + 120, data.begin() + 139));
  motor[7].parseData(
      std::vector<uint8_t>(data.begin() + 139, data.begin() + 158));

  motor_data.frameLength = frameLength;
  motor_data.protocolVersion = protocolVersion;
  motor_data.command = command;
  motor_data.packetCount = packetCount;
  ros_pub_data(motor);
}

int main(int argc, char** argv) {
  // ROS
  ros::init(argc, argv, "motor_state_uart_node");
  ros::NodeHandle nh;
  std::string port;
  int baud;
  ros::param::param<std::string>("~port", port, "/dev/ttyUSB0");
  ros::param::param<int>("~baud", baud, 115200);
  std::cout << "port is: " << port << std::endl;
  std::cout << "baud is: " << baud << std::endl;

  serial::Timeout to = serial::Timeout::simpleTimeout(100);
  serial_.setPort(port);
  serial_.setBaudrate(baud);
  serial_.setTimeout(to);
  serial_.open();

  ros::Publisher Motor_pub_ =
      nh.advertise<disturbance_estimation::motor_state>("/motor_state", 10);

  std::vector<uint8_t> receivedData;
  uint16_t CRC16 = 0;
  uint16_t CRC_byte[2];
  ros::Rate loop_rate(100);
  while (ros::ok()) {
    uint8_t byte;
    static uint8_t rev_step = 0;
    static uint16_t connect_flag = 0, connect_count = 0;
    uint16_t packet_num;
    auto available_bytes = serial_.available();
    std::string str_received;
    connect_flag++;
    if (available_bytes) {
      connect_flag = 0;
      serial_.read(str_received, available_bytes);
      const char* p = str_received.data();
      for (auto i = 0; i < available_bytes; i++) {
        byte = *(p + i);
        if (byte == 0x9b && rev_step == 0) {
          receivedData.push_back(byte);
          rev_step++;
        } else if (byte == 0x9e && rev_step == 1) {
          receivedData.push_back(byte);
          rev_step++;
        } else if (byte == 0x01 && rev_step == 2) {
          receivedData.push_back(byte);
          rev_step++;
        } else if (byte == 0x02 && rev_step == 3) {
          receivedData.push_back(byte);
          rev_step++;
        }
        else if (rev_step == 4) {
          receivedData.push_back(byte);
          packet_num = byte << 8;
          rev_step++;
        } else if (rev_step == 5) {
          receivedData.push_back(byte);
          packet_num = packet_num | byte;
          rev_step++;
        }
        else if (rev_step > 5 && rev_step < 158) {
          receivedData.push_back(byte);
          rev_step++;
        }
        else if (rev_step == 158) {
          CRC_byte[0] = byte;
          rev_step++;
        } else if (rev_step == 159) {
          CRC_byte[1] = byte;
          CRC16 = (uint16_t)CRC_byte[1] << 8 | CRC_byte[0];
          uint16_t CRC_data = CalCRC16(receivedData.data(), 158);
          if (CRC16 == CRC_data) {
            parseSerialData(receivedData);
            Motor_pub_.publish(motor_data);
            receivedData.clear();
            rev_step = 0;
          } else {
            printf("crc false\n");
            receivedData.clear();
            rev_step = 0;
          }
        } else {
          printf("receive data false\n");
          receivedData.clear();
          rev_step = 0;
        }
      }
    }
    if (connect_flag > 50) {
      connect_count++;
      printf("uart reopen \n");
      serial_.close();
      serial_.open();
      connect_flag = 0;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
