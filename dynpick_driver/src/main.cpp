#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_srvs/Trigger.h>

#include <stdio.h>
#include <fcntl.h>
#include <time.h>
#include <termios.h>
#include <string.h>
#include <unistd.h>

#include <mutex>
#include <condition_variable>

#define true 1
#define false 0

// TODO: Find out why and fix the multiple try approach
#define RESET_COMMAND_TRY 3 // It only works when send several times.

#define DATA_LENGTH 27
#define CALIB_DATA_LENGTH 46

#define RESOLUTION 1000.0
#define FILTER 0.8

std::mutex m_;
std::condition_variable cv_;
int offset_reset_ = 0;

double senddata[6] = {0};
unsigned short offsetdata[6];
char str[256];
int offsetcounter = 0;

int SetComAttr(int fdc)
{
    int n;

    struct termios term;

    // Set baud rate
    n = tcgetattr(fdc, &term);
    if (n < 0)
        goto over;

    bzero(&term, sizeof(term));

    term.c_cflag = B921600 | CS8 | CLOCAL | CREAD;
    term.c_iflag = IGNPAR;
    term.c_oflag = 0;
    term.c_lflag = 0; /*ICANON;*/

    term.c_cc[VINTR] = 0;  /* Ctrl-c */
    term.c_cc[VQUIT] = 0;  /* Ctrl-? */
    term.c_cc[VERASE] = 0; /* del */
    term.c_cc[VKILL] = 0;  /* @ */
    term.c_cc[VEOF] = 4;   /* Ctrl-d */
    term.c_cc[VTIME] = 0;
    term.c_cc[VMIN] = 0;
    term.c_cc[VSWTC] = 0;    /* '?0' */
    term.c_cc[VSTART] = 0;   /* Ctrl-q */
    term.c_cc[VSTOP] = 0;    /* Ctrl-s */
    term.c_cc[VSUSP] = 0;    /* Ctrl-z */
    term.c_cc[VEOL] = 0;     /* '?0' */
    term.c_cc[VREPRINT] = 0; /* Ctrl-r */
    term.c_cc[VDISCARD] = 0; /* Ctrl-u */
    term.c_cc[VWERASE] = 0;  /* Ctrl-w */
    term.c_cc[VLNEXT] = 0;   /* Ctrl-v */
    term.c_cc[VEOL2] = 0;    /* '?0' */

    //  tcflush(fdc, TCIFLUSH);
    n = tcsetattr(fdc, TCSANOW, &term);
over:
    return (n);
}

bool offsetRequest(std_srvs::Trigger::Request &req,
                   std_srvs::Trigger::Response &res)
{
    std::unique_lock<std::mutex> lock(m_);
    offset_reset_ = RESET_COMMAND_TRY;
    cv_.wait(lock, [] { return offset_reset_ <= 0; });
    lock.unlock();
    res.message = "Reset offset command was send " + std::to_string(RESET_COMMAND_TRY) + " times to the sensor.";
    res.success = true;
    return true;
}

bool clearSocket(const int &fdc, char *leftover)
{
    int len = 0;
    int c = 0;
    int length = 255;
    while (len < length)
    {
        c = read(fdc, leftover + len, length - len);
        if (c > 0)
        {
            len += c;
            ROS_DEBUG("More data to clean up; n = %d (%d) ===", c, len);
        }
        else
        {
            ROS_DEBUG("No more data on socket");
            break;
        }
    }
    // This could actually check if data was received and may return on timeout with false
    return true;
}

bool readCharFromSocket(int fdc, int length, char *reply)
{
    int c = read(fdc, reply, length);
    if (c < length)
        return false;
    // int len = 0;
    // int c = 0;
    // while (len < length)
    // {
    //     c = read(fdc, reply + len, length - len);
    //     if (c >= 0)
    //     {
    //         len += c;
    //     }
    //     else
    //     {
    //         printf("=== need to read more data ... n = %d (%d) ===", c, len);
    //         continue;
    //     }
    // }
    // This could actually check if data was received and may return on timeout with false
    return true;
}

int main(int argc, char **argv)
{
    int fdc;
    int clock = 0;
    double rate = 100;
    std::string devname, frame_id;
    bool auto_adjust = true;
    int frq_div = 1;
    int c;

    fdc = -1;

    ros::init(argc, argv, "dynpick_driver");
    ros::NodeHandle n, nh("~");
    nh.param<std::string>("device", devname, "/dev/ttyUSB0");
    nh.param<std::string>("frame_id", frame_id, "/sensor");
    nh.param<double>("rate", rate, 100);
    nh.param<bool>("acquire_calibration", auto_adjust, true);
    nh.param<int>("frequency_div", frq_div, 1);

    ros::ServiceServer service = n.advertiseService("tare", offsetRequest);
    ros::Publisher pub = n.advertise<geometry_msgs::WrenchStamped>("force", 1000);

    ros::AsyncSpinner spinner(2); // Use 2 threads
    spinner.start();

    // Open COM port
    ROS_INFO("Open %s", devname.c_str());

    fdc = open(devname.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fdc < 0)
    {
        ROS_ERROR("could not open %s\n", devname.c_str());
        return -1;
    }

    // Obtain sampling rate
    ROS_INFO("Sampling time = %f ms\n", 1.0 / rate);

    // Set baud rate of COM port
    SetComAttr(fdc);

    // Clean up
    char trash[255];
    clearSocket(fdc, trash);

    float calib[6] = {1, 1, 1, 1, 1, 1};
    // Autoadjust
    if (auto_adjust)
    {
        write(fdc, "p", 1);
        char reply[CALIB_DATA_LENGTH];
        readCharFromSocket(fdc, CALIB_DATA_LENGTH, reply);
        sscanf(reply, "%f,%f,%f,%f,%f,%f",
               &calib[0], &calib[1], &calib[2], &calib[3], &calib[4], &calib[5]);
        ROS_INFO("Calibration from sensor:\n%.3f LSB/N, %.3f LSB/N, %.3f LSB/N, %.3f LSB/Nm, %.3f LSB/Nm, %.3f LSB/Nm",
                 calib[0], calib[1], calib[2], calib[3], calib[4], calib[5]);
        clearSocket(fdc, trash);
    }

    // Set frequncy divider filter
    if (frq_div == 1 || frq_div == 2 || frq_div == 4 || frq_div == 8)
    {
        ROS_INFO("foo1");
        char cmd[2];
        sprintf(cmd, "%dF", frq_div);
        write(fdc, cmd, 2);
        ROS_INFO("Set the frequency divider to %s", cmd);

        // check if successful
        write(fdc, "0F", 2);
        char repl[3];
        readCharFromSocket(fdc, 3, repl);
        ROS_ERROR_COND(repl[0] - '0' != frq_div, "Response by sensor is not as expected! Current Filter: %dF", repl[0] - '0');
        clearSocket(fdc, trash);
    }
    else
    {
        ROS_WARN("Not setting frequency divider. Parameter out of acceptable values {1,2,4,8}: %d", frq_div);
    }

    // Request for initial single data
    write(fdc, "R", 1);
    // while (read(fdc, str, 27) < 27)
    //     ;
    // sscanf(str, "%1d%4hx%4hx%4hx%4hx%4hx%4hx",
    //        NULL, &offsetdata[0], &offsetdata[1], &offsetdata[2], &offsetdata[3], &offsetdata[4], &offsetdata[5]);
    // write(fdc, "R", 1);

    ros::Rate loop_rate(rate);
    while (ros::ok())
    {

        int tick;
        unsigned short data[6];

        geometry_msgs::WrenchStamped msg;

        std::unique_lock<std::mutex> lock(m_);
        if (offset_reset_ <= 0)
        {
            // Request for initial data (2nd round)
            write(fdc, "R", 1);

            // Obtain single data
            //readCharFromSocket(fdc, DATA_LENGTH, str);
            c = read(fdc, str, 27);
            if (!(c < 27))
            {
                if (offsetcounter < 100)
                {

                    msg.header.frame_id = frame_id;
                    msg.header.stamp = ros::Time::now();
                    msg.header.seq = clock++;
                    sscanf(str, "%1d%4hx%4hx%4hx%4hx%4hx%4hx",
                           &tick, &data[0], &data[1], &data[2], &data[3], &data[4], &data[5]);
                    offsetdata[0] = msg.wrench.force.x = offsetdata[0] * FILTER + data[0] * (1 - FILTER);
                    offsetdata[1] = msg.wrench.force.y = offsetdata[1] * FILTER + data[1] * (1 - FILTER);
                    offsetdata[2] = msg.wrench.force.z = offsetdata[2] * FILTER + data[2] * (1 - FILTER);
                    offsetdata[3] = msg.wrench.torque.x = offsetdata[3] * FILTER + data[3] * (1 - FILTER);
                    offsetdata[4] = msg.wrench.torque.y = offsetdata[4] * FILTER + data[4] * (1 - FILTER);
                    offsetdata[5] = msg.wrench.torque.z = offsetdata[5] * FILTER + data[5] * (1 - FILTER);
                    pub.publish(msg);
                    lock.unlock();
                    offsetcounter++;
                }
                else
                {
                    sscanf(str, "%1d%4hx%4hx%4hx%4hx%4hx%4hx",
                           &tick, &data[0], &data[1], &data[2], &data[3], &data[4], &data[5]);

                    msg.header.frame_id = frame_id;
                    msg.header.stamp = ros::Time::now();
                    msg.header.seq = clock++;

                    senddata[0] = msg.wrench.force.x = senddata[0] * FILTER + ((data[0] - offsetdata[0]) / RESOLUTION) * (1 - FILTER);
                    senddata[1] = msg.wrench.force.y = senddata[1] * FILTER + ((data[1] - offsetdata[1]) / RESOLUTION) * (1 - FILTER);
                    senddata[2] = msg.wrench.force.z = senddata[2] * FILTER + ((data[2] - offsetdata[2]) / RESOLUTION) * (1 - FILTER);
                    senddata[3] = msg.wrench.torque.x = senddata[3] * FILTER + ((data[3] - offsetdata[3]) / RESOLUTION) * (1 - FILTER);
                    senddata[4] = msg.wrench.torque.y = senddata[4] * FILTER + ((data[4] - offsetdata[4]) / RESOLUTION) * (1 - FILTER);
                    senddata[5] = msg.wrench.torque.z = senddata[5] * FILTER + ((data[5] - offsetdata[5]) / RESOLUTION) * (1 - FILTER);
                    // msg.wrench.force.x = (data[0] - 8192) / calib[0];
                    // msg.wrench.force.y = (data[1] - 8192) / calib[1];
                    // msg.wrench.force.z = (data[2] - 8192) / calib[2];
                    // msg.wrench.torque.x = (data[3] - 8192) / calib[3];
                    // msg.wrench.torque.y = (data[4] - 8192) / calib[4];
                    // msg.wrench.torque.z = (data[5] - 8192) / calib[5];
                    // msg.wrench.force.x = data[0];
                    // msg.wrench.force.y = data[1];
                    // msg.wrench.force.z = data[2];
                    // msg.wrench.torque.x = data[3];
                    // msg.wrench.torque.y = data[4];
                    // msg.wrench.torque.z = data[5];

                    pub.publish(msg);
                    lock.unlock();
                }
            }
        }

        else
        {
            // Request for offset reset
            write(fdc, "O", 1);
            offset_reset_--;
            lock.unlock();
            cv_.notify_all();
        }
        loop_rate.sleep();
    }

    return 0;
}