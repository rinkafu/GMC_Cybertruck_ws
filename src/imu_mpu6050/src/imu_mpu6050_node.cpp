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
    std::string frame_id;
    int buadrate;
    double time_offset_in_seconds;
    double linear_acceleration_stddev;
    double angular_velocity_stddev;
    double orientation_stddev;

    tf::Quaternion orientation;
    tf::Quaternion zero_orientation;

    // init ros node and set name node
    ros::init(argc, argv, "imu_mpu6050_node");

    // node handle must be int manin() FOR GET PARAM
    ros::NodeHandle private_node_handle("~");
    private_node_handle.param<double>("time_offset_in_seconds", time_offset_in_seconds, 0.0);
    private_node_handle.param<double>("linear_acceleration_stddev", linear_acceleration_stddev, 0.0);
    private_node_handle.param<std::string>("frame_id", frame_id, "imu_link");
    private_node_handle.param<double>("angular_velocity_stddev", angular_velocity_stddev, 0.0);
    private_node_handle.param<double>("orientation_stddev", orientation_stddev, 0.0);

    // getParam from file launch conf
    private_node_handle.getParam("port", serial_port);
    private_node_handle.getParam("buadrate", buadrate);

    // HANDLE TOPIC IMU
    ros::NodeHandle nh("imu");

    // PUBLISHER TOPIC /imu/data
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("data", 50);
    ros::Publisher imu_temperature_pub = nh.advertise<sensor_msgs::Temperature>("temperature", 50);

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

    try
    {
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
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Error reading from the serial port " << ser.getPort() << ". Closing connection.");
        ser.close();
    }

    std::string input;
    std::string read;
    char check_Sum;
    int data_packet_start;
    int data_packet_identify;

    double _ax, _ay, _az,
        _wx, _wy, _wz,
        roll, pitch, yaw, T;

    ros::Rate loop_rate(200);

    sensor_msgs::Imu imu;

    imu.linear_acceleration_covariance[0] = linear_acceleration_stddev;
    imu.linear_acceleration_covariance[4] = linear_acceleration_stddev;
    imu.linear_acceleration_covariance[8] = linear_acceleration_stddev;

    imu.angular_velocity_covariance[0] = angular_velocity_stddev;
    imu.angular_velocity_covariance[4] = angular_velocity_stddev;
    imu.angular_velocity_covariance[8] = angular_velocity_stddev;

    imu.orientation_covariance[0] = orientation_stddev;
    imu.orientation_covariance[4] = orientation_stddev;
    imu.orientation_covariance[8] = orientation_stddev;

    sensor_msgs::Temperature temperature_msg;
    temperature_msg.variance = 0;

    while (ros::ok())
    {
        ros::spinOnce();

        if (ser.available())
        {
            // std_msgs::String result;
            read = ser.read(ser.available());
            input += read;
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
                            // linear accleration
                            _ax = ((double(short((input[3] << 8) | input[2]))) / 32768) * 16;
                            _ay = ((double(short((input[5] << 8) | input[4]))) / 32768) * 16;
                            _az = ((double(short((input[7] << 8) | input[6]))) / 32768) * 16;
                            T = (short((input[9] << 8) | input[8]) / 340) + 36.53;
                        }
                        break;
                    case 0x52:
                        if ((check_Sum + 0x52) == input[10])
                        {
                            //
                            _wx = ((double(short((input[3] << 8) | input[2]))) / 32768) * 2000;
                            _wy = ((double(short((input[5] << 8) | input[4]))) / 32768) * 2000;
                            _wz = ((double(short((input[7] << 8) | input[6]))) / 32768) * 2000;
                            T = (short((input[9] << 8) | input[8]) / 340) + 36.53;
                        }
                        break;
                    case 0x53:
                        if ((check_Sum + 0x53) == input[10])
                        {
                            // angular_velocity
                            roll = ((double(short((input[3] << 8) | input[2]))) / 32768) * 180;
                            pitch = ((double(short((input[5] << 8) | input[4]))) / 32768) * 180;
                            yaw = ((double(short((input[7] << 8) | input[6]))) / 32768) * 180;
                            T = (short((input[9] << 8) | input[8]) / 340) + 36.53;
                        }
                        break;
                    }
                }
                input.erase(0);
                // ROS_INFO("ax :%f ay :%f az : %f wx : %f wy: %f wz: %f roll: %f picth: %f yaw: %f T: %f", _ax, _ay, _az, _wx, _wy, _wz, roll, pitch, yaw, T);

                // calculate measurement time
                ros::Time measurement_time = ros::Time::now() + ros::Duration(time_offset_in_seconds);

                // publish imu message
                imu.header.stamp = measurement_time;
                imu.header.frame_id = frame_id;

                imu.angular_velocity.x = roll;
                imu.angular_velocity.y = pitch;
                imu.angular_velocity.z = yaw;

                imu.linear_acceleration.x = _ax;
                imu.linear_acceleration.y = _ay;
                imu.linear_acceleration.z = _az;

                // imu.orientation.x = roll;
                // imu.orientation.y = pitch;
                // imu.orientation.z = yaw;

                imu_pub.publish(imu);

                // publish temperature message
                temperature_msg.header.stamp = measurement_time;
                temperature_msg.header.frame_id = frame_id;
                temperature_msg.temperature = T;

                imu_temperature_pub.publish(temperature_msg);
            }
        }

        //loop
        ros::spinOnce();
        loop_rate.sleep();
    }
}