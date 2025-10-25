#include <sstream>
#include <zumo_serial/zumo_serial_node.hpp>

///*********************************************************************************
/// Constructor
///*********************************************************************************
ZumoSerialNode::ZumoSerialNode() : rclcpp::Node("zumo_serial_node") {}

///*********************************************************************************
/// Deconstructor
///*********************************************************************************
ZumoSerialNode::~ZumoSerialNode() {
  try {
    if (m_SerialPort && m_SerialPort->is_open()) {
      m_SerialPort->close();
    }
    m_IoService.stop();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception occurred during cleanup: %s",
                 e.what());
  }

  if (p_SerialThread) {
    p_SerialThread->join();
    delete p_SerialThread;
  }
}

///*********************************************************************************
/// Initialization
///*********************************************************************************
bool ZumoSerialNode::Init() {
  // Initialize the parameters
  this->declare_parameter<std::string>(SERIAL_PORT_PARAMETER,
                                       SERIAL_PORT_DEFAULT);

  // Get the parameter value
  z_SerialPort = this->get_parameter(SERIAL_PORT_PARAMETER).as_string();
  RCLCPP_INFO(this->get_logger(), "serial port: %s\n", z_SerialPort.c_str());

  try {
    m_SerialPort = std::make_unique<boost::asio::serial_port>(m_IoService);

    boost::system::error_code ec;
    m_SerialPort->open(z_SerialPort, ec);

    if (ec) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port '%s': %s",
                   z_SerialPort.c_str(), ec.message().c_str());
      RCLCPP_INFO(
          this->get_logger(),
          "See docs/README.md for serial port configuration instructions");
      RCLCPP_INFO(
          this->get_logger(),
          "Run 'just select_port' to interactively select a serial port");
      return false;
    }

    m_SerialPort->set_option(boost::asio::serial_port_base::baud_rate(115200));
    m_SerialPort->set_option(boost::asio::serial_port_base::character_size(8));
    m_SerialPort->set_option(boost::asio::serial_port_base::flow_control(
        boost::asio::serial_port_base::flow_control::none));
    m_SerialPort->set_option(boost::asio::serial_port_base::parity(
        boost::asio::serial_port_base::parity::none));
    m_SerialPort->set_option(boost::asio::serial_port_base::stop_bits(
        boost::asio::serial_port_base::stop_bits::one));

    RCLCPP_INFO(this->get_logger(), "Successfully opened serial port '%s'",
                z_SerialPort.c_str());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception occurred: %s", e.what());
    return false;
  }

  // ROS publisher setup
  m_ZumoSensorsPub =
      this->create_publisher<zumo_msgs::msg::ZumoSensors>("zumo_sensors", 10);

  // serial read thread
  p_SerialThread =
      new boost::thread(boost::bind(&ZumoSerialNode::SerialDataThread, this));

  return true;
}

///*********************************************************************************
/// Read serial data
///*********************************************************************************
void ZumoSerialNode::ReadSerialData() {
  try {
    if (!m_SerialPort || !m_SerialPort->is_open()) {
      return;
    }

    boost::asio::streambuf buf;
    boost::asio::read_until(*m_SerialPort, buf, '\n');

    std::istream is(&buf);
    std::string zReceivedData;
    std::getline(is, zReceivedData);

    if (!zReceivedData.empty()) {
      RCLCPP_DEBUG(this->get_logger(), "Received: %s", zReceivedData.c_str());

      std::istringstream iss(zReceivedData);
      std::string token;
      auto msg = zumo_msgs::msg::ZumoSensors();
      msg.header.stamp = this->now();
      int iSensorValues[SENSOR_COUNT];
      int i = 0;

      while (std::getline(iss, token, ',')) {
        std::istringstream(token) >> iSensorValues[i];
        i++;
      }

      msg.ax = iSensorValues[0];
      msg.ay = iSensorValues[1];
      msg.az = iSensorValues[2];

      msg.mx = iSensorValues[3];
      msg.my = iSensorValues[4];
      msg.mz = iSensorValues[5];

      msg.gx = iSensorValues[6];
      msg.gy = iSensorValues[7];
      msg.gz = iSensorValues[8];

      msg.enc_left = iSensorValues[9];
      msg.enc_right = iSensorValues[10];

      m_ZumoSensorsPub->publish(msg);
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(),
                 "Exception occurred during serial port reading: %s", e.what());
  }
}

///*********************************************************************************
/// Thread for reading serial data
///*********************************************************************************
void ZumoSerialNode::SerialDataThread() {
  while (rclcpp::ok()) {
    // Start reading serial port
    ReadSerialData();
    boost::this_thread::sleep(boost::posix_time::milliseconds(10));
  }
}
