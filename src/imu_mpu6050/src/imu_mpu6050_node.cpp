#include <geometry_msgs/Quaternion.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

int main(int argc, char **argv)
{
    // create variable init
    serial::Serial ser;
    std::string serial_port;
    int buadrate;

    // init ros node and set name node
    ros::init(argc, argv, "imu_mpu6050_node");

    // node handle must be int manin() FOR GET PARAM
    ros::NodeHandle private_node_handle("~");

    // getParam from file launch conf
    private_node_handle.getParam("port", serial_port);
    private_node_handle.getParam("buadrate", buadrate);

    // // // show value param get
    // ROS_INFO("port: %s", serial_port.c_str());
    // ROS_INFO("port: %d", buadrate);

    // HANDLE TOPIC IMU
    ros::NodeHandle nh("imu");

    // PUBLISHER TOPIC /imu/data
    ros::Publisher imu_pub = nh.advertise<std_msgs::String>("data", 50);

    try
    {
        ser.setPort(serial_port);
        ser.setBaudrate(buadrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if (ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
        ser.write("a"); //Using the serial port, disable I2C
        sleep(1);
        ser.write("c");
        sleep(1);
    }
    else
    {
        return -1;
    }

    std::string input;
    std::string read;
    int data_packet_start;
    int data_packet_identify;

    ros::Rate loop_rate(200);
    while (ros::ok())
    {
        ros::spinOnce();

        if (ser.available())
        {
            // std_msgs::String result;
            read = ser.read(ser.available());
            // ROS_INFO_STREAM("read " << (int)read.size() << " new characters from serial port, adding to characters of old input." << (int)input.size());
            input += read;
            while (input.length() >= 4095)
            {
                data_packet_start = input.find("\x55");
                data_packet_identify = data_packet_start + 1;
                // ROS_INFO_STREAM(data_packet_start);

                if ((char)input[data_packet_identify] == '\x51') //0x51
                {
                    int16_t wx = ((input[data_packet_start + 2] + (input[data_packet_start + 3] * 256)) / 32768) * 2000;
                }
                else if ((char)input[data_packet_identify] == '\x52') //0x52
                {
                    int16_t wx = (((input[data_packet_start + 2] + (input[data_packet_start + 3]) * 256)) / 32768) * 2000;
                    ROS_INFO_STREAM(wx);
                }
                else if ((char)input[data_packet_identify] == '\x53') //0x53
                {
                    int16_t wx = ((input[data_packet_start + 2] + (input[data_packet_start + 3] * 256)) / 32768) * 2000;
                }
                else
                {
                    input.erase(0, data_packet_start + 4095); // delete everything packet input
                }

                // if ((char)input[data_packet_identify] == '\x51') //0x51
                // {
                //     int16_t x = (((0xff & (char)input[data_packet_start + 2]) << 8) | 0xff & (char)input[data_packet_start + 3]) / 32768 * 156.96;
                //     ROS_INFO_STREAM("aX:" << x);
                // }
                // else if ((char)input[data_packet_identify] == '\x52') //0x52
                // {
                //     int16_t x = (((0xff & (char)input[data_packet_start + 2]) << 8) | 0xff & (char)input[data_packet_start + 3]) / 32768 * ;
                //     ROS_INFO_STREAM("wX:" << x);
                // }
                // else if ((char)input[data_packet_identify] == '\x53') //0x53
                // {
                //     int16_t x = (((0xff & (char)input[data_packet_start + 2]) << 8) | 0xff & (char)input[data_packet_start + 3]);
                //     ROS_INFO_STREAM("X:" << x);
                // }
                // else
                // {
                //     input.erase(0, data_packet_start + 4095); // delete everything packet input
                // }
                // ROS_INFO_STREAM(input[data_packet_start + 1]);
                input.erase(0, data_packet_start + 4095);
            }
            // ROS_INFO_STREAM("Read: " << read);
            // imu_pub.publish(result);
        }

        //loop rate
        loop_rate.sleep();
    }
}