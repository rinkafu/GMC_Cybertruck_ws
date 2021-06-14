#include <geometry_msgs/Quaternion.h>
#include <stdio.h>
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
    char check_Sum;
    int data_packet_start;
    int data_packet_identify;
    float _ax, _ay, _az,
        _wx, _wy, _wz,
        roll, pitch, yaw, T;

    ros::Rate loop_rate(200);
    while (ros::ok())
    {
        ros::spinOnce();

        if (ser.available())
        {
            // std_msgs::String result;
            read = ser.read(ser.available());
            input += read;
            // ROS_INFO_STREAM(read);
            // if (input.length() == 10){
            //     ROS_INFO_STREAM(read);
            // }else
            if (input.length() > 10)
            {
                if (input[0] == 0x55)
                {
                    check_Sum = 0x55 + input[3] + input[2] + input[5] + input[4] + input[7] + input[6] + input[9] + input[8];
                    switch (input[1])
                    {
                    case 0x51:
                        if ((check_Sum + 0x51) == input[10])
                        {
                            _ax = ((float(short((input[3] << 8) | input[2]))) / 32768) * 16;
                            _ay = ((float(short((input[5] << 8) | input[4]))) / 32768) * 16;
                            _az = ((float(short((input[7] << 8) | input[6]))) / 32768) * 16;
                            T = (short((input[9] << 8) | input[8]) / 340) + 36.53;
                        }
                        break;
                    case 0x52:
                        if ((check_Sum + 0x52) == input[10])
                        {
                            _wx = ((float(short((input[3] << 8) | input[2]))) / 32768) * 2000;
                            _wy = ((float(short((input[5] << 8) | input[4]))) / 32768) * 2000;
                            _wz = ((float(short((input[7] << 8) | input[6]))) / 32768) * 2000;
                            T = (short((input[9] << 8) | input[8]) / 340) + 36.53;
                        }
                        break;
                    case 0x53:
                        if ((check_Sum + 0x53) == input[10])
                        {
                            roll = ((float(short((input[3] << 8) | input[2]))) / 32768) * 180;
                            pitch = ((float(short((input[5] << 8) | input[4]))) / 32768) * 180;
                            yaw = ((float(short((input[7] << 8) | input[6]))) / 32768) * 180;
                            T = (short((input[9] << 8) | input[8]) / 340) + 36.53;
                        }
                        break;
                    }

                    ROS_INFO_STREAM("T:" << T);
                }

                ROS_INFO("ax :%f ay :%f az : %f wx : %f wy: %f wz: %f roll: %f picth: %f yaw: %f T: %f", _ax, _ay, _az, _wx, _wy, _wz, roll, pitch, yaw, T);

                input.erase(0);
            }
        }

        //loop rate
        loop_rate.sleep();
    }
}