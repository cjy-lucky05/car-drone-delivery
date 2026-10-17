
#include "base_control.h"
#include "wd_arm_moveit_demo/ArmJoint.h"
#include "sensor_msgs/JointState.h"
#include <pthread.h>
#define PI 3.14
using namespace std;
string base_type("NanoCar");
int sonar_num = 0;

int set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop)
{
    struct termios newtio, oldtio;
    if (tcgetattr(fd, &oldtio) != 0)
    {
        perror("SetupSerial 1");
        return -1;
    }
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;

    switch (nBits)
    {
    case 7:
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag |= CS8;
        break;
    }

    switch (nEvent)
    {
    case 'O':
        newtio.c_cflag |= PARENB;
        newtio.c_cflag |= PARODD;
        newtio.c_iflag |= (INPCK | ISTRIP);
        break;
    case 'E':
        newtio.c_iflag |= (INPCK | ISTRIP);
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        break;
    case 'N':
        newtio.c_cflag &= ~PARENB;
        break;
    }

    switch (nSpeed)
    {
    case 2400:
        cfsetispeed(&newtio, B2400);
        cfsetospeed(&newtio, B2400);
        break;
    case 4800:
        cfsetispeed(&newtio, B4800);
        cfsetospeed(&newtio, B4800);
        break;
    case 9600:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    case 115200:
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;
    case 460800:
        cfsetispeed(&newtio, B460800);
        cfsetospeed(&newtio, B460800);
        break;
    case 921600:
        cfsetispeed(&newtio, B921600);
        cfsetospeed(&newtio, B921600);
        break;
    default:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    }
    if (nStop == 1)
        newtio.c_cflag &= ~CSTOPB;
    else if (nStop == 2)
        newtio.c_cflag |= CSTOPB;
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;
    tcflush(fd, TCIFLUSH);
    if ((tcsetattr(fd, TCSANOW, &newtio)) != 0)
    {
        perror("com set error");
        return -1;
    }
    return 0;
    //	printf("set done!\n\r");
}
char crc_1byte(char data)
{
    char crc_1byte = 0;
    for (int i = 0; i < 8; i++)
    {
        if ((crc_1byte ^ data) & 0x01)

        {
            crc_1byte ^= 0x18;
            crc_1byte >>= 1;
            crc_1byte |= 0x80;
        }

        else
        {
            crc_1byte >>= 1;
        }
        data >>= 1;
    }

    return crc_1byte;
}

char crc_byte(char *data, int length)
{
    char ret = 0;
    for (int i = 0; i < length; i++)
        ret = crc_1byte(ret ^ data[i]);

    return ret;
}
int BaseControl::arm_convert_value(int id, double angle)
{
    int val = -1;
    if (id == 1)
        val = (3100 - 900) * (180 - angle) / 180 + 900;
    else if (id == 2)
        val = (3100 - 900) * (180 - angle) / 180 + 900;
    else if (id == 3)
        val = (3100 - 900) * (180 - angle) / 180 + 900;
    else if (id == 4)
        val = (3100 - 900) * (180 - angle) / 180 + 900;
    else if (id == 5)
        val = (3700 - 380) * (angle) / 270 + 380;
    else if (id == 6)
        val = (3100 - 900) * (angle) / 180 + 900;
    return val;
}
int BaseControl::arm_convert_angle(int id, double s_value)
{
    int s_angle = -1;
    if (id == 1)
        s_angle = (s_value - 900) * (0 - 180) / (3100 - 900) + 180 + 0.5;
    else if (id == 2)
        s_angle = (s_value - 900) * (0 - 180) / (3100 - 900) + 180 + 0.5;
    else if (id == 3)
        s_angle = (s_value - 900) * (0 - 180) / (3100 - 900) + 180 + 0.5;
    else if (id == 4)
        s_angle = (s_value - 900) * (0 - 180) / (3100 - 900) + 180 + 0.5;
    else if (id == 5)
        s_angle = (270 - 0) * (s_value - 380) / (3700 - 380) + 0 + 0.5;
    else if (id == 6)
        s_angle = (180 - 0) * (s_value - 900) / (3100 - 900) + 0 + 0.5;
    return s_angle;
}
void BaseControl::set_uart_servo_angle(unsigned int s_id, double s_angle, unsigned int run_time)
{
    if (s_id < 1 || s_id > 6 || run_time < 0)
        return;
    int ret = -1;
    unsigned int value;

    if (s_id == 5)
    {
        if (s_angle>=0&&s_angle<= 270)
            ret = 0;
    }
    else  if (s_angle>=0&&s_angle<= 180)
        ret = 0;
    if (!ret)
    {
      //  ROS_INFO("set_uart_servo_angle");
        value = arm_convert_value(s_id, s_angle);
        if (run_time > 2000)
            run_time = 2000;
        char output[11] = {0x5a, 0x0b, 0x01, 0x41,(char)s_id, (char)((value>> 8) & 0xff),  (char)(value& 0xff),
                          (char)((run_time>> 8) & 0xff), (char)(run_time& 0xff)};
        output[9] = 0;
        char crc_8 = crc_byte(output, sizeof(output) / sizeof(output[0]) - 1);
        output[10] = (crc_8);
        int length = sizeof(output) / sizeof(output[0]);
        ros::Duration dur(0, 10000000);
        while (serialIDLE_flag)
            dur.sleep();
        serialIDLE_flag = 4;
        int len = 0;
        while (len != length)
        {
            int ret = write(serial, output + len, length - len);
            if (ret < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
            {
                // usleep(1000);
                continue;
            }
            if (ret < 0)
                break;

            len += ret;
        }
        if (len != length)
            ROS_ERROR("set_uart_servo_angle Command Send Faild");
        serialIDLE_flag = 0;
    }
}

void BaseControl::set_uart_servo_torque(unsigned int enable)
{
    enable = enable ? 1 : 0;
    char output[7] = {0x5a, 0x7, 0x01, 0x43,(char)enable};

    output[5] = 0;
    char crc_8 = crc_byte(output, sizeof(output) / sizeof(output[0]) - 1);
    output[6] = (crc_8);
    int length = sizeof(output) / sizeof(output[0]);
    ros::Duration dur(0, 10000000);
    while (serialIDLE_flag)
        dur.sleep();
    serialIDLE_flag = 4;
    int len = 0;
    while (len != length)
    {
        int ret = write(serial, output + len, length - len);
        if (ret < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
        {
            // usleep(1000);
            continue;
        }
        if (ret < 0)
            break;

        len += ret;
    }
    if (len != length)
        ROS_ERROR("set_uart_servo_angle Command Send Faild");
    serialIDLE_flag = 0;
}

void BaseControl::set_uart_servo_angle_array(const std::vector<float> &angle_s, unsigned int run_time)
{
    if (angle_s.size() != 6)
        return;
     if (angle_s[0]>=-20&&angle_s[0]<= 200&&angle_s[1]>=-20&&angle_s[1]<= 200&&\
     angle_s[2]>=-20&&angle_s[2]<= 270&&angle_s[3]>=-100&&angle_s[3]<= 180&&\
     angle_s[4]>=-20&&angle_s[4]<= 270&&angle_s[5]>=-20&&angle_s[5]<= 200)    
    {
        if (run_time > 2000)
            run_time = 2000;
        else if (run_time < 0)
            run_time = 0;
        std::vector<int> temp_val = {0, 0, 0, 0, 0, 0};
        for (int i = 0; i < 6; i++)
            temp_val[i] = arm_convert_value(i + 1, angle_s[i]);
        char output[0x14] = {0x5a, 0x14, 0x01, 0x45,
                (char)((temp_val[0]>> 8) & 0xff),  (char)(temp_val[0]& 0xff),
                (char)((temp_val[1]>> 8) & 0xff),  (char)(temp_val[1]& 0xff),
                (char)((temp_val[2]>> 8) & 0xff),  (char)(temp_val[2]& 0xff),
                (char)((temp_val[3]>> 8) & 0xff),  (char)(temp_val[3]& 0xff),
                (char)((temp_val[4]>> 8) & 0xff),  (char)(temp_val[4]& 0xff),
                (char)((temp_val[5]>> 8) & 0xff),  (char)(temp_val[5]& 0xff),
                (char)((run_time>> 8) & 0xff),  (char)(run_time& 0xff)};
        output[18] = 0;
        char crc_8 = crc_byte(output, sizeof(output) / sizeof(output[0]) - 1);
        output[19] = (crc_8);
        int length = sizeof(output) / sizeof(output[0]);
        ros::Duration dur(0, 10000000);
        while (serialIDLE_flag)
            dur.sleep();
        serialIDLE_flag = 4;
        int len = 0;
        while (len != length)
        {
            int ret = write(serial, output + len, length - len);
            if (ret < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
            {
                // usleep(1000);
                continue;
            }
            if (ret < 0)
                break;

            len += ret;
        }
        if (len != length)
            ROS_ERROR("arm_crtl Command Send Faild");
        serialIDLE_flag = 0;
    }
}
void BaseControl::get_uart_servo_value(unsigned int s_id)
{
    if (s_id < 1 || s_id > 6)
        return;
    char output[7] = {0x5a, 0x7, 0x01, 0x47, (char)(s_id & 0xff)};
    output[5] = 0;
    char crc_8 = crc_byte(output, sizeof(output) / sizeof(output[0]) - 1);
    output[6] = (crc_8);
    int length = sizeof(output) / sizeof(output[0]);
    ros::Duration dur(0, 10000000);
    while (serialIDLE_flag)
        dur.sleep();
    serialIDLE_flag = 4;
    int len = 0;
    while (len != length)
    {
        int ret = write(serial, output + len, length - len);
        if (ret < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
        {
            // usleep(1000);
            continue;
        }
        if (ret < 0)
            break;

        len += ret;
    }
    if (len != length)
        ROS_ERROR("set_uart_servo_angle Command Send Faild");
    serialIDLE_flag = 0;
}
int BaseControl::get_uart_servo_angle(unsigned int s_id)
{
    if (s_id < 1 || s_id > 6)
        return -1 ;
    int ret=-1;
    read_id = 0;
    read_val = 0;
    get_uart_servo_value(s_id);

    ros::Duration dur(0, 1000000);
    int timeout = 30;
    while (timeout--)
        if (read_id > 0 && read_id < 7)
            break;
        else
            dur.sleep();
    if (timeout > 0)
        if (read_id == s_id)
        {
            ret = arm_convert_angle(read_id, read_val);
            if (s_id == 5)
            {
                if (ret > 270 || ret < 0)
                    ret = -1;
            }
            else if (ret > 180 || ret < 0)
                  ret = -1;
        }
  

    return ret;
}
int BaseControl::get_uart_servo_angle_array( std::vector<double> &angle)
{
    int ret = -1;
    char output[6] = {0x5a, 0x6, 0x01, 0x49};
    output[4] = 0;
    char crc_8 = crc_byte(output, sizeof(output) / sizeof(output[0]) - 1);
    output[5] = (crc_8);
    int length = sizeof(output) / sizeof(output[0]);

    for (int i = 0; i < 6; i++)
    {
        angle[i] = -1;
        read_arm[i] = -1;
    }

    read_arm_ok = 0;
    ros::Duration dur(0, 1000000);
    while (serialIDLE_flag)
        dur.sleep();
    serialIDLE_flag = 4;
    int len = 0;
    while (len != length)
    {
        int ret = write(serial, output + len, length - len);
        if (ret < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
        {
            // usleep(1000);
            continue;
        }
        if (ret < 0)
            break;

        len += ret;
    }
    if (len != length)
    {
        ROS_ERROR("set_uart_servo_angle Command Send Faild");
        serialIDLE_flag = 0;
        return -1;
    }
    serialIDLE_flag = 0;

    int timeout = 30;
    while (timeout--)
        if (read_arm_ok == 1)
        {
            ret = 0;
            for (int i = 0; i < 6; i++)
                if (read_arm[i] > 0)
                    angle[i] = arm_convert_angle(i + 1, read_arm[i]);
            break;
        }
        else
            dur.sleep();

    return ret;
}

void BaseControl::cmdCB(const geometry_msgs::Twist::ConstPtr msg)
{
    trans_x = msg->linear.x;
    trans_y = msg->linear.y;
    rotat_z = msg->angular.z;
    last_cmd_vel_time = ros::Time::now();
    char output[12] = {0x5a, 0xc, 0x01, 0x01, (char)((((int)(trans_x * 1000.0)) >> 8) & 0xff), (char)(((int)(trans_x * 1000.0) & 0xff)),
                        (char)((((int)(trans_y * 1000.0)) >> 8) & 0xff), (char)(((int)(trans_y * 1000.0) & 0xff)),
                        (char)((((int)(rotat_z * 1000.0)) >> 8) & 0xff), (char)(((int)(rotat_z * 1000.0) & 0xff))};
    int length = sizeof(output) / sizeof(output[0]);
    char outputdata[] = {0x5a, 0x0c, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    outputdata[4] = (int(trans_x * 1000.0) >> 8) & 0xff;
    outputdata[5] = int(trans_x * 1000.0) & 0xff;
    outputdata[6] = (int(trans_y * 1000.0) >> 8) & 0xff;
    outputdata[7] = int(trans_y * 1000.0) & 0xff;
    outputdata[8] = (int(rotat_z * 1000.0) >> 8) & 0xff;
    outputdata[9] = int(rotat_z * 1000.0) & 0xff;
    int crc_8 = crc_byte(outputdata, sizeof(outputdata) / sizeof(outputdata[0]) - 1);
    output[11] = char(crc_8);

    ros::Duration dur(0, 10000000);
    while (serialIDLE_flag)
        dur.sleep();
    serialIDLE_flag = 4;
    int len = 0;
    while (len != length)
    {
        int ret = write(serial, output + len, length - len);
        if (ret < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
        {
            // usleep(1000);
            continue;
        }
        if (ret < 0)
            break;

        len += ret;
    }
    if (len != length)
        ROS_ERROR("Vel Command Send Faild");
    serialIDLE_flag = 0;
}
void BaseControl::timerCommunicationCB(const ros::TimerEvent &event)
{
    // ROS_INFO("%s %d  ",__FUNCTION__,__LINE__);
    int len;
    char buf[1024];
    len = read(serial, buf, 1024);
    //  ROS_INFO("len   %d \n",len);
    if (len < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
        return;
    int i = 0;
    if (len < 0)
    {
        ROS_INFO("read  serial error \n");
        perror("read ");
    }
    else if (len > 0)
        while (i < len)
        {
            queue->enqueue(buf[i]);
            i++;
        }
    if (!queue->is_empty())
    {
        char t;
        queue->get_first(t);
        if (t == 0x5a)
        {
            queue->get_second(t);
            if (t <= queue->get_size())
            {
                char *buf = new char[t];
                queue->dequeue(buf, t);
                if (buf[t - 1] != crc_byte(buf, t - 1))
                {
                    queue->dequeue(t);
                    ROS_INFO("error");
                     return;
                }
                // parse receive data
                if (buf[3] == 0x04)
                {
                    Vx = buf[4] << 8;
                    Vx += buf[5];
                    Vy = buf[6] << 8;
                    Vy += buf[7];
                    Vyaw = buf[8] * 256;
                    Vyaw += buf[9];
                }

                else if (buf[3] == 0x06)
                {
                    Yawz = buf[8] << 8;
                    Yawz += buf[9];
                }

                else if (buf[3] == 0x08)
                {
                    Vvoltage = buf[4] << 8;
                    Vvoltage += buf[5];
                    Icurrent = buf[6] << 8;
                    Icurrent += buf[7];
                }

                else if (buf[3] == 0x0a)
                {
                    Vx = buf[4] << 8;
                    Vx += buf[5];
                    Yawz = buf[6] << 8;
                    Yawz += buf[7];
                    Vyaw = buf[8] << 8;
                    Vyaw += buf[9];
                }

                else if (buf[3] == 0x12)
                {
                    Vx = buf[4] << 8;
                    Vx += buf[5];
                    Vy = buf[6] << 8;
                    Vy += buf[7];
                    Yawz = buf[8] << 8;
                    Yawz += buf[9];
                    Vyaw = (buf[10] << 8);
                    Vyaw += buf[11];
                }

                else if (buf[3] == 0x14)
                {

                    Gyro.x = int(((buf[4] & 0xff) << 24) | ((buf[5] & 0xff) << 16) | ((buf[6] & 0xff) << 8) | (buf[7] & 0xff));
                    Gyro.y = int(((buf[8] & 0xff) << 24) | ((buf[9] & 0xff) << 16) | ((buf[10] & 0xff) << 8) | (buf[11] & 0xff));
                    Gyro.z = int(((buf[12] & 0xff) << 24) | ((buf[13] & 0xff) << 16) | ((buf[14] & 0xff) << 8) | (buf[15] & 0xff));

                    Accel.x = int(((buf[16] & 0xff) << 24) | ((buf[17] & 0xff) << 16) | ((buf[18] & 0xff) << 8) | (buf[19] & 0xff));
                    Accel.y = int(((buf[20] & 0xff) << 24) | ((buf[21] & 0xff) << 16) | ((buf[22] & 0xff) << 8) | (buf[23] & 0xff));
                    Accel.z = int(((buf[24] & 0xff) << 24) | ((buf[25] & 0xff) << 16) | ((buf[26] & 0xff) << 8) | (buf[27] & 0xff));

                    Quat.w = int((buf[28] & 0xff) << 8 | buf[29]);
                    Quat.x = int((buf[30] & 0xff) << 8 | buf[31]);
                    Quat.y = int((buf[32] & 0xff) << 8 | buf[33]);
                    Quat.z = int((buf[34] & 0xff) << 8 | buf[35]);
                }

                else if (buf[3] == 0x1a)
                {

                    Sonar[0] = buf[4];
                    Sonar[1] = buf[5];
                    Sonar[2] = buf[6];
                    Sonar[3] = buf[7];
                }
                else if (buf[3] == 0x48)
                {
                 
                    read_id = buf[4];
                    read_val = ((buf[5] << 8) |(unsigned int )buf[6]);
                 
                }
                else if (buf[3] == 0x4a)
                {
                 //   ROS_INFO("0x48");
                    read_arm[0] = (buf[4] << 8) | (unsigned int )buf[5];
                    read_arm[1] = (buf[6] << 8) | (unsigned int )buf[7] ;
                    read_arm[2] = (buf[8] << 8) | (unsigned int )buf[9] ;
                    read_arm[3] = (buf[10] << 8) | (unsigned int )buf[11];
                    read_arm[4] = (buf[12] << 8) | (unsigned int )buf[13] ;
                    read_arm[5] = (buf[14] << 8) | (unsigned int )buf[15] ;
                    read_arm_ok = 1;
                    
                }
                else if (buf[3] == 0xf2)
                {

                    movebase_hardware_version[0] = buf[4];
                    movebase_hardware_version[1] = buf[5];
                    movebase_hardware_version[2] = buf[6];
                    movebase_firmware_version[0] = buf[7];
                    movebase_firmware_version[1] = buf[8];
                    movebase_firmware_version[2] = buf[9];
                    ROS_INFO("Move Base Hardware Ver %d.%d.%d,Firmware Ver %d.%d.%d", movebase_hardware_version[0], movebase_hardware_version[1], movebase_hardware_version[2],
                             movebase_firmware_version[0], movebase_firmware_version[1], movebase_firmware_version[2]);
                }

                else if (buf[3] == 0xf4)
                {

                    ROS_INFO("SN:%02x %02x %02x %02x %02x %02x ", buf[4] << 8 | buf[5], buf[6] << 8 | buf[7],
                             buf[8] << 8 | buf[9], buf[10] << 8 | buf[11], buf[12] << 8 | buf[13], buf[14] << 8 | buf[15]);
                }

                else if (buf[3] == 0x22)
                {
                    float fRatio = (buf[6] << 8 | buf[7]) / 10.0;
                    float fDiameter = (buf[8] << 8 | buf[9]) / 10;
                    ROS_INFO("Type:%s Motor:%s Ratio:%.01f WheelDiameter:%.01f", movebase_type[buf[4] - 1].c_str(), motor_type[buf[5] - 1].c_str(), fRatio, fDiameter);
                }
                 delete buf;
            }
        }
        else
            queue->dequeue(t);
    }
}

void BaseControl::timerOdomCB(const ros::TimerEvent &event)
{
    //  ROS_INFO("%s %d  ",__FUNCTION__,__LINE__);
    char output[6];
    // Get move base velocity data
    if (movebase_firmware_version[1] == 0)
    {
        // old version firmware have no version info and not support new command below
        output[0] = 0x5a;
        output[1] = 0x06;
        output[2] = 0x01;
        output[3] = 0x09;
        output[4] = 0x00;
        output[5] = 0x38;
    }
    else
    {
        // in firmware version new than v1.1.0,support this command
        output[0] = 0x5a;
        output[1] = 0x06;
        output[2] = 0x01;
        output[3] = 0x11;
        output[4] = 0x00;
        output[5] = 0xa2;
    }
    int length = sizeof(output) / sizeof(output[0]);
    ros::Duration dur(0, 10000000);
    while (serialIDLE_flag)
        dur.sleep();
    serialIDLE_flag = 1;
    int len = 0;
    while (len != length)
    {
        int ret = write(serial, output + len, length - len);
        if (ret < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
        {
            //  usleep(1000);
            continue;
        }
        if (ret < 0)
            break;

        len += ret;
    }
    if (len != length)
        ROS_ERROR("Odom Command Send Faild");
    serialIDLE_flag = 0;

    // calculate odom data
    float Vx = this->Vx / 1000.0;
    float Vy = this->Vy / 1000.0;
    float Vyaw = this->Vyaw / 1000.0;

    pose_yaw = Yawz / 100.0;
    pose_yaw = pose_yaw * PI / 180.0;

    ros::Time current_time = ros::Time::now();
    double dt = (current_time - previous_time).toSec();
    previous_time = current_time;
    pose_x = pose_x + Vx * (cos(pose_yaw)) * dt - Vy * (sin(pose_yaw)) * dt;
    pose_y = pose_y + Vx * (sin(pose_yaw)) * dt + Vy * (cos(pose_yaw)) * dt;

    geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromRollPitchYaw(0, 0, pose_yaw);
    nav_msgs::Odometry msg = nav_msgs::Odometry();
    msg.header.stamp = current_time;
    msg.header.frame_id = odomId;
    msg.child_frame_id = baseId;
    msg.pose.pose.position.x = pose_x;
    msg.pose.pose.position.y = pose_y;
    msg.pose.pose.position.z = 0;
    msg.pose.pose.orientation.x = pose_quat.x;
    msg.pose.pose.orientation.y = pose_quat.y;
    msg.pose.pose.orientation.z = pose_quat.z;
    msg.pose.pose.orientation.w = pose_quat.w;
    msg.twist.twist.linear.x = Vx;
    msg.twist.twist.linear.y = Vy;
    msg.twist.twist.angular.z = Vyaw;
    odom_pub.publish(msg);
    tf_broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::createQuaternionFromRPY(0.0, 0.0, pose_yaw), tf::Vector3(pose_x, pose_y, 0.0)), current_time, odomId, baseId));
    // ROS_INFO("%s %d  ",__FUNCTION__,__LINE__);
}
void BaseControl::timerBatteryCB(const ros::TimerEvent &event)
{
    //  ROS_INFO("%s %d  ",__FUNCTION__,__LINE__);
    unsigned char output[] = {0x5a, 0x06, 0x01, 0x07, 0x00, 0xe4}; // 0xe4 is CRC-8 value
    int length = sizeof(output) / sizeof(output[0]);
    ros::Duration dur(0, 10000000);
    while (serialIDLE_flag)
        dur.sleep();
    serialIDLE_flag = 3;
    int len = 0;
    while (len != length)
    {
        int ret = write(serial, output + len, length - len);
        if (ret < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
        {
            // usleep(1000);
            continue;
        }
        if (ret < 0)
            break;

        len += ret;
    }
    if (len != length)
        ROS_ERROR("Battery Command Send Faild");
    serialIDLE_flag = 0;

    sensor_msgs::BatteryState msg = sensor_msgs::BatteryState();
    msg.header.stamp = current_time;
    msg.header.frame_id = baseId;
    msg.voltage = float(Vvoltage / 1000.0);
    msg.current = float(Icurrent / 1000.0);
    battery_pub.publish(msg);
    //  ROS_INFO("%s %d  ",__FUNCTION__,__LINE__);
}
void BaseControl::timerIMUCB(const ros::TimerEvent &event)
{
    // ROS_INFO("%s %d  ",__FUNCTION__,__LINE__);
    unsigned char output[] = {0x5a, 0x06, 0x01, 0x13, 0x00, 0x33}; // 0x33 is CRC-8 value
    int length = sizeof(output) / sizeof(output[0]);
    ros::Duration dur(0, 10000000);
    while (serialIDLE_flag)
        dur.sleep();
    serialIDLE_flag = 3;
    int len = 0;
    while (len != length)
    {
        int ret = write(serial, output + len, length - len);
        if (ret < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
        {
            // usleep(1000);
            continue;
        }
        if (ret < 0)
            break;

        len += ret;
    }
    if (len != length)
        ROS_ERROR("IMU Command Send Faild");
    serialIDLE_flag = 0;

    sensor_msgs::Imu msg = sensor_msgs::Imu();
    msg.header.stamp = current_time;
    msg.header.frame_id = imuId;

    msg.angular_velocity.x = (Gyro.x) / 100000.0;
    msg.angular_velocity.y = (Gyro.y) / 100000.0;
    msg.angular_velocity.z = (Gyro.z) / 100000.0;

    msg.linear_acceleration.x = (Accel.x) / 100000.0;
    msg.linear_acceleration.y = (Accel.y) / 100000.0;
    msg.linear_acceleration.z = (Accel.z) / 100000.0;

    msg.orientation.w = (Quat.w) / 10000.0;
    msg.orientation.x = (Quat.x) / 10000.0;
    msg.orientation.y = (Quat.y) / 10000.0;
    msg.orientation.z = (Quat.z) / 10000.0;

    imu_pub.publish(msg);
    // TF value calculate from mechanical structure
    if (!base_type.compare("NanoCar"))
        tf_broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0.0, 0.0, 0.0, 1.0), tf::Vector3(0.0, 0.0, 0.09)), current_time, baseId, imuId));
}
void BaseControl::getVersion()
{

    unsigned char output[] = {0x5a, 0x06, 0x01, 0xf1, 0x00, 0xd7}; // 0xd7 is CRC-8 value
    int length = sizeof(output) / sizeof(output[0]);
    ros::Duration dur(0, 10000000);
    while (serialIDLE_flag)
        dur.sleep();
    serialIDLE_flag = 1;
    int len = 0;
    while (len != length)
    {
        int ret = write(serial, output + len, length - len);
        if (ret < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
        {
            //  usleep(1000);
            continue;
        }
        if (ret < 0)
            break;

        len += ret;
    }
    if (len != length)
        ROS_ERROR("Get Version Command Send Faild");
    serialIDLE_flag = 0;
}
void BaseControl::getSN()
{
    unsigned char output[] = {0x5a, 0x06, 0x01, 0xf3, 0x00, 0x46}; // 0x46 is CRC-8 value
    int length = sizeof(output) / sizeof(output[0]);

    ros::Duration dur(0, 10000000);
    while (serialIDLE_flag)
        dur.sleep();
    serialIDLE_flag = 1;
    int len = 0;
    while (len != length)
    {
        int ret = write(serial, output + len, length - len);
        if (ret < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
        {
            //  usleep(1000);
            continue;
        }
        if (ret < 0)
            break;

        len += ret;
    }
    if (len != length)
        ROS_ERROR("Get SN Command Send Faild");
    serialIDLE_flag = 0;
}
void BaseControl::getInfo()
{

    unsigned char output[] = {0x5a, 0x06, 0x01, 0x21, 0x00, 0x8f}; // 0x8f is CRC-8 value
    int length = sizeof(output) / sizeof(output[0]);
    ros::Duration dur(0, 10000000);
    while (serialIDLE_flag)
        dur.sleep();
    serialIDLE_flag = 1;
    int len = 0;
    while (len != length)
    {
        int ret = write(serial, output + len, length - len);
        if (ret < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
        {
            //   usleep(1000);
            continue;
        }
        if (ret < 0)
            break;

        len += ret;
    }
    if (len != length)
        ROS_ERROR("Get info Command Send Faild");
    serialIDLE_flag = 0;
}
void BaseControl::Armcallback(const wd_arm_moveit_demo::ArmJoint::ConstPtr msg)
{
    if (msg->joints.size() != 0)
        for (int i = 0; i < 2; i++)
        {
            set_uart_servo_angle_array(msg->joints, msg->run_time);
            usleep(10000);
        }
    else
        for (int i = 0; i < 2; i++)
        {
            set_uart_servo_angle(msg->id, msg->angle, msg->run_time);
            usleep(10000);
        }
}

BaseControl::BaseControl(ros::NodeHandle *phandle)
{
    this->phandle = phandle;
    // Get params
    phandle->param(string("base_id"), baseId, string("base_footprint"));
    phandle->param(string("port"), device_port, string("/dev/move_base"));
    phandle->param(string("baudrate"), baudrate, 115200);
    phandle->param(string("odom_id"), odomId, string("odom"));
    phandle->param(string("odom_topic"), odom_topic, string("odom"));
    phandle->param(string("odom_freq"), odom_freq, 50);
    phandle->param(string("battery_topic"), battery_topic, string("battery"));
    phandle->param(string("battery_freq"), battery_freq, 1.0f);
    phandle->param(string("cmd_vel_topic"), cmd_vel_topic, string("cmd_vel"));
    phandle->param(string("ackermann_cmd_topic"), ackermann_cmd_topic, string("ackermann_cmd_topic"));
    phandle->param(string("pub_imu"), pub_imu, false);
    if (pub_imu)
    {
        phandle->param(string("imu_id"), imuId, string("imu"));
        phandle->param(string("imu_topic"), imu_topic, string("imu"));
        phandle->param(string("imu_freq"), imu_freq, 50.0f);
        if (imu_freq > 100)
            imu_freq = 100;
    }

    phandle->param(string("pub_sonar"), pub_sonar, false);
    phandle->param(string("sub_ackermann"), sub_ackermann, false);
    //  #define param
    queue = new wd::Queue(1024 * 4);
    current_time = ros::Time::now();
    previous_time = current_time;
    movebase_type[0] = "NanoCar";
    motor_type[0] = "25GA370";
    last_cmd_vel_time = ros::Time::now();
    last_ackermann_cmd_time = ros::Time::now();
    // Serial Communication

    serial = open(device_port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial < 0 || set_opt(serial, baudrate, 8, 'N', 1))
        ROS_INFO("Failed to open %s\n", device_port.c_str());
    else
        ROS_INFO("Opening Serial sucessful ");
    arm_topic="TargetAngle";
    jointstate_topic="joint_states";

    arm_sub = phandle->subscribe(arm_topic.c_str(), 20, &BaseControl::Armcallback, this);
    cmd_sub = phandle->subscribe(cmd_vel_topic.c_str(), 20, &BaseControl::cmdCB, this);
    odom_pub = phandle->advertise<nav_msgs::Odometry>(odom_topic.c_str(), 10);
    battery_pub = phandle->advertise<sensor_msgs::BatteryState>(battery_topic.c_str(), 3);
    tf_broadcaster = tf::TransformBroadcaster();

    timer_odom = phandle->createTimer(ros::Duration(1.0 / odom_freq), &BaseControl::timerOdomCB, this, false, true);
    timer_battery = phandle->createTimer(ros::Duration(1.0 / battery_freq), &BaseControl::timerBatteryCB, this, false, true);
    timer_communication = phandle->createTimer(ros::Duration(1.0 / 500), &BaseControl::timerCommunicationCB, this);

    // inorder to compatibility old version firmware,imu topic is NOT pud in default
    if (pub_imu)
    {
        imu_pub = phandle->advertise<sensor_msgs::Imu>(imu_topic.c_str(), 10);
        timer_imu = phandle->createTimer(ros::Duration(1.0 / imu_freq), &BaseControl::timerIMUCB, this);
    }
    ros::Duration dur(0, 10000000);
    serialIDLE_flag = 0;
    getVersion();
    // move base imu initialization need about 2s, during initialization, move base system is blocked
    // so need this gap

    while (movebase_hardware_version[0] == 0)
    {
        ros::spinOnce();
        dur.sleep();
    }
    getSN();
    dur.sleep();
    getInfo();
}

void MySigintHandler(int sig)
{
    ros::shutdown();
    exit(0);
}

BaseControl::~BaseControl()
{

    timer_communication.stop();
    timer_imu.stop();
    timer_odom.stop();
    if (pub_imu)
        timer_imu.stop();
    close(serial);
    delete queue;
}
extern void test(BaseControl &  base);
ros::Publisher jointstate_pub;
bool isrun=false;
void* JointStateThread(void *arg)
{
    BaseControl*  base=(BaseControl*)arg;
    static  vector<double>  angles(6);
    double  rad2deg=3.14/180; 
    double  rad2deg1=  90.0/(180-30);  
    sensor_msgs::JointState state;
    while(isrun)
    {
        if(!base->get_uart_servo_angle_array(angles))
        {
           // ROS_INFO("status %lf %lf %lf %lf  %lf %lf ",angles[0],angles[1],angles[2],angles[3],angles[4],angles[5]);
            state.header.stamp = ros::Time::now();
            state.header.frame_id = "arm_base_link";
            for(int i=0;i<6;i++)
                if(i==5)
                    angles[5]= ((angles[5]-30)*rad2deg1-90)*rad2deg;
                else  
                    angles[i]= (angles[i]-90)*rad2deg;  
            state.name = {"arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5", "grip_joint"};
            state.position = angles;
            jointstate_pub.publish(state); 
        }
        else  ROS_INFO("error");
        
        usleep(100000);
    }
    
    return NULL;
}
BaseControl * pbase;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "wd_base_control");
    signal(SIGINT, MySigintHandler);
    ros::NodeHandle handle;
    ROS_INFO("%s base control ...", base_type.c_str());
    BaseControl base(&handle);
    pbase=&base;
    jointstate_pub=handle.advertise<sensor_msgs::JointState>("joint_states", 3);
    pthread_t thread;
    isrun=true;
    pthread_create(&thread,NULL,JointStateThread,&base);
 //   test(base);
    ros::spin();
    isrun=false;
    pthread_join(thread, NULL);
}











// #include "base_control.h"
// #include "wd_arm_moveit_demo/ArmJoint.h"
// #include "sensor_msgs/JointState.h"
// #include <pthread.h>
// #define PI 3.14
// using namespace std;
// string base_type("NanoCar");
// int sonar_num = 0;

// int set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop)
// {
//     struct termios newtio, oldtio;
//     if (tcgetattr(fd, &oldtio) != 0)
//     {
//         perror("SetupSerial 1");
//         return -1;
//     }
//     bzero(&newtio, sizeof(newtio));
//     newtio.c_cflag |= CLOCAL | CREAD;
//     newtio.c_cflag &= ~CSIZE;

//     switch (nBits)
//     {
//     case 7:
//         newtio.c_cflag |= CS7;
//         break;
//     case 8:
//         newtio.c_cflag |= CS8;
//         break;
//     }

//     switch (nEvent)
//     {
//     case 'O':
//         newtio.c_cflag |= PARENB;
//         newtio.c_cflag |= PARODD;
//         newtio.c_iflag |= (INPCK | ISTRIP);
//         break;
//     case 'E':
//         newtio.c_iflag |= (INPCK | ISTRIP);
//         newtio.c_cflag |= PARENB;
//         newtio.c_cflag &= ~PARODD;
//         break;
//     case 'N':
//         newtio.c_cflag &= ~PARENB;
//         break;
//     }

//     switch (nSpeed)
//     {
//     case 2400:
//         cfsetispeed(&newtio, B2400);
//         cfsetospeed(&newtio, B2400);
//         break;
//     case 4800:
//         cfsetispeed(&newtio, B4800);
//         cfsetospeed(&newtio, B4800);
//         break;
//     case 9600:
//         cfsetispeed(&newtio, B9600);
//         cfsetospeed(&newtio, B9600);
//         break;
//     case 115200:
//         cfsetispeed(&newtio, B115200);
//         cfsetospeed(&newtio, B115200);
//         break;
//     case 460800:
//         cfsetispeed(&newtio, B460800);
//         cfsetospeed(&newtio, B460800);
//         break;
//     case 921600:
//         cfsetispeed(&newtio, B921600);
//         cfsetospeed(&newtio, B921600);
//         break;
//     default:
//         cfsetispeed(&newtio, B9600);
//         cfsetospeed(&newtio, B9600);
//         break;
//     }
//     if (nStop == 1)
//         newtio.c_cflag &= ~CSTOPB;
//     else if (nStop == 2)
//         newtio.c_cflag |= CSTOPB;
//     newtio.c_cc[VTIME] = 0;
//     newtio.c_cc[VMIN] = 0;
//     tcflush(fd, TCIFLUSH);
//     if ((tcsetattr(fd, TCSANOW, &newtio)) != 0)
//     {
//         perror("com set error");
//         return -1;
//     }
//     return 0;
//     //	printf("set done!\n\r");
// }
// char crc_1byte(char data)
// {
//     char crc_1byte = 0;
//     for (int i = 0; i < 8; i++)
//     {
//         if ((crc_1byte ^ data) & 0x01)

//         {
//             crc_1byte ^= 0x18;
//             crc_1byte >>= 1;
//             crc_1byte |= 0x80;
//         }

//         else
//         {
//             crc_1byte >>= 1;
//         }
//         data >>= 1;
//     }

//     return crc_1byte;
// }

// char crc_byte(char *data, int length)
// {
//     char ret = 0;
//     for (int i = 0; i < length; i++)
//         ret = crc_1byte(ret ^ data[i]);

//     return ret;
// }
// int BaseControl::arm_convert_value(int id, double angle)
// {
//     int val = -1;
//     if (id == 1)
//         val = (3100 - 900) * (180 - angle) / 180 + 900;
//     else if (id == 2)
//         val = (3100 - 900) * (180 - angle) / 180 + 900;
//     else if (id == 3)
//         val = (3100 - 900) * (180 - angle) / 180 + 900;
//     else if (id == 4)
//         val = (3100 - 900) * (180 - angle) / 180 + 900;
//     else if (id == 5)
//         val = (3700 - 380) * (angle) / 270 + 380;
//     else if (id == 6)
//         val = (3100 - 900) * (angle) / 180 + 900;
//     return val;
// }
// int BaseControl::arm_convert_angle(int id, double s_value)
// {
//     int s_angle = -1;
//     if (id == 1)
//         s_angle = (s_value - 900) * (0 - 180) / (3100 - 900) + 180 + 0.5;
//     else if (id == 2)
//         s_angle = (s_value - 900) * (0 - 180) / (3100 - 900) + 180 + 0.5;
//     else if (id == 3)
//         s_angle = (s_value - 900) * (0 - 180) / (3100 - 900) + 180 + 0.5;
//     else if (id == 4)
//         s_angle = (s_value - 900) * (0 - 180) / (3100 - 900) + 180 + 0.5;
//     else if (id == 5)
//         s_angle = (270 - 0) * (s_value - 380) / (3700 - 380) + 0 + 0.5;
//     else if (id == 6)
//         s_angle = (180 - 0) * (s_value - 900) / (3100 - 900) + 0 + 0.5;
//     return s_angle;
// }
// void BaseControl::set_uart_servo_angle(unsigned int s_id, double s_angle, unsigned int run_time)
// {
//     if (s_id < 1 || s_id > 6 || run_time < 0)
//         return;
//     int ret = -1;
//     unsigned int value;

//     if (s_id == 5)
//     {
//         if (s_angle>=0&&s_angle<= 270)
//             ret = 0;
//     }
//     else  if (s_angle>=0&&s_angle<= 180)
//         ret = 0;
//     if (!ret)
//     {
//       //  ROS_INFO("set_uart_servo_angle");
//         value = arm_convert_value(s_id, s_angle);
//         if (run_time > 2000)
//             run_time = 2000;
//         char output[11] = {0x5a, 0x0b, 0x01, 0x41,(char)s_id, (char)((value>> 8) & 0xff),  (char)(value& 0xff),
//                           (char)((run_time>> 8) & 0xff), (char)(run_time& 0xff)};
//         output[9] = 0;
//         char crc_8 = crc_byte(output, sizeof(output) / sizeof(output[0]) - 1);
//         output[10] = (crc_8);
//         int length = sizeof(output) / sizeof(output[0]);
//         ros::Duration dur(0, 10000000);
//         while (serialIDLE_flag)
//             dur.sleep();
//         serialIDLE_flag = 4;
//         int len = 0;
//         while (len != length)
//         {
//             int ret = write(serial, output + len, length - len);
//             if (ret < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
//             {
//                 // usleep(1000);
//                 continue;
//             }
//             if (ret < 0)
//                 break;

//             len += ret;
//         }
//         if (len != length)
//             ROS_ERROR("set_uart_servo_angle Command Send Faild");
//         serialIDLE_flag = 0;
//     }
// }

// void BaseControl::set_uart_servo_torque(unsigned int enable)
// {
//     enable = enable ? 1 : 0;
//     char output[7] = {0x5a, 0x7, 0x01, 0x43,(char)enable};

//     output[5] = 0;
//     char crc_8 = crc_byte(output, sizeof(output) / sizeof(output[0]) - 1);
//     output[6] = (crc_8);
//     int length = sizeof(output) / sizeof(output[0]);
//     ros::Duration dur(0, 10000000);
//     while (serialIDLE_flag)
//         dur.sleep();
//     serialIDLE_flag = 4;
//     int len = 0;
//     while (len != length)
//     {
//         int ret = write(serial, output + len, length - len);
//         if (ret < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
//         {
//             // usleep(1000);
//             continue;
//         }
//         if (ret < 0)
//             break;

//         len += ret;
//     }
//     if (len != length)
//         ROS_ERROR("set_uart_servo_angle Command Send Faild");
//     serialIDLE_flag = 0;
// }

// void BaseControl::set_uart_servo_angle_array(const std::vector<float> &angle_s, unsigned int run_time)
// {
    
//     ROS_INFO("=== set_uart_servo_angle_array called ===");  // 添加
//     ROS_INFO("Angles: %f, %f, %f, %f, %f, %f", 
//              angle_s[0], angle_s[1], angle_s[2], angle_s[3], angle_s[4], angle_s[5]);  // 添加
//     ROS_INFO("Run time: %d", run_time);  // 添加



//     if (angle_s.size() != 6)
//         return;
//      if (angle_s[0]>=-20&&angle_s[0]<= 200&&angle_s[1]>=-20&&angle_s[1]<= 200&&\
//      angle_s[2]>=-20&&angle_s[2]<= 270&&angle_s[3]>=-100&&angle_s[3]<= 180&&\
//      angle_s[4]>=-20&&angle_s[4]<= 270&&angle_s[5]>=-20&&angle_s[5]<= 200)    
//     {
//         if (run_time > 2000)
//             run_time = 2000;
//         else if (run_time < 0)
//             run_time = 0;
//         std::vector<int> temp_val = {0, 0, 0, 0, 0, 0};
//         for (int i = 0; i < 6; i++)
//             temp_val[i] = arm_convert_value(i + 1, angle_s[i]);
//         char output[0x14] = {0x5a, 0x14, 0x01, 0x45,
//                 (char)((temp_val[0]>> 8) & 0xff),  (char)(temp_val[0]& 0xff),
//                 (char)((temp_val[1]>> 8) & 0xff),  (char)(temp_val[1]& 0xff),
//                 (char)((temp_val[2]>> 8) & 0xff),  (char)(temp_val[2]& 0xff),
//                 (char)((temp_val[3]>> 8) & 0xff),  (char)(temp_val[3]& 0xff),
//                 (char)((temp_val[4]>> 8) & 0xff),  (char)(temp_val[4]& 0xff),
//                 (char)((temp_val[5]>> 8) & 0xff),  (char)(temp_val[5]& 0xff),
//                 (char)((run_time>> 8) & 0xff),  (char)(run_time& 0xff)};
//         output[18] = 0;
//         char crc_8 = crc_byte(output, sizeof(output) / sizeof(output[0]) - 1);
//         output[19] = (crc_8);
//         int length = sizeof(output) / sizeof(output[0]);
//         ros::Duration dur(0, 10000000);
//         while (serialIDLE_flag)
//             dur.sleep();
//         serialIDLE_flag = 4;
//         int len = 0;
//         while (len != length)
//         {
            
//             // 打印要发送的数据
//             ROS_INFO("Sending %d bytes:", length);
//             for(int i=0; i<length; i++) {
//                 printf("%02X ", (unsigned char)output[i]);
//             }
//             printf("\n");
            
//             int ret = write(serial, output + len, length - len);
//             if (ret < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
//             {
//                 // usleep(1000);
//                 continue;
//             }
//             if (ret < 0)
//                 break;

//             len += ret;
//         }
//         if (len != length)
//             ROS_ERROR("arm_crtl Command Send Faild");
//         serialIDLE_flag = 0;
//     }
// }
// void BaseControl::get_uart_servo_value(unsigned int s_id)
// {
//     if (s_id < 1 || s_id > 6)
//         return;
//     char output[7] = {0x5a, 0x7, 0x01, 0x47, (char)(s_id & 0xff)};
//     output[5] = 0;
//     char crc_8 = crc_byte(output, sizeof(output) / sizeof(output[0]) - 1);
//     output[6] = (crc_8);
//     int length = sizeof(output) / sizeof(output[0]);
//     ros::Duration dur(0, 10000000);
//     while (serialIDLE_flag)
//         dur.sleep();
//     serialIDLE_flag = 4;
//     int len = 0;
//     while (len != length)
//     {
//         int ret = write(serial, output + len, length - len);
//         if (ret < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
//         {
//             // usleep(1000);
//             continue;
//         }
//         if (ret < 0)
//             break;

//         len += ret;
//     }
//     if (len != length)
//         ROS_ERROR("set_uart_servo_angle Command Send Faild");
//     serialIDLE_flag = 0;
// }
// int BaseControl::get_uart_servo_angle(unsigned int s_id)
// {
//     if (s_id < 1 || s_id > 6)
//         return -1 ;
//     int ret=-1;
//     read_id = 0;
//     read_val = 0;
//     get_uart_servo_value(s_id);

//     ros::Duration dur(0, 1000000);
//     int timeout = 30;
//     while (timeout--)
//         if (read_id > 0 && read_id < 7)
//             break;
//         else
//             dur.sleep();
//     if (timeout > 0)
//         if (read_id == s_id)
//         {
//             ret = arm_convert_angle(read_id, read_val);
//             if (s_id == 5)
//             {
//                 if (ret > 270 || ret < 0)
//                     ret = -1;
//             }
//             else if (ret > 180 || ret < 0)
//                   ret = -1;
//         }
  

//     return ret;
// }
// int BaseControl::get_uart_servo_angle_array( std::vector<double> &angle)
// {
//     int ret = -1;
//     char output[6] = {0x5a, 0x6, 0x01, 0x49};
//     output[4] = 0;
//     char crc_8 = crc_byte(output, sizeof(output) / sizeof(output[0]) - 1);
//     output[5] = (crc_8);
//     int length = sizeof(output) / sizeof(output[0]);

//     for (int i = 0; i < 6; i++)
//     {
//         angle[i] = -1;
//         read_arm[i] = -1;
//     }

//     read_arm_ok = 0;
//     ros::Duration dur(0, 1000000);
//     while (serialIDLE_flag)
//         dur.sleep();
//     serialIDLE_flag = 4;
//     int len = 0;
//     while (len != length)
//     {
//         int ret = write(serial, output + len, length - len);
//         if (ret < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
//         {
//             // usleep(1000);
//             continue;
//         }
//         if (ret < 0)
//             break;

//         len += ret;
//     }
//     if (len != length)
//     {
//         ROS_ERROR("set_uart_servo_angle Command Send Faild");
//         serialIDLE_flag = 0;
//         return -1;
//     }
//     serialIDLE_flag = 0;

//     int timeout = 30;
//     while (timeout--)
//         if (read_arm_ok == 1)
//         {
//             ret = 0;
//             for (int i = 0; i < 6; i++)
//                 if (read_arm[i] > 0)
//                     angle[i] = arm_convert_angle(i + 1, read_arm[i]);
//             break;
//         }
//         else
//             dur.sleep();

//     return ret;
// }

// void BaseControl::cmdCB(const geometry_msgs::Twist::ConstPtr msg)
// {
//     trans_x = msg->linear.x;
//     trans_y = msg->linear.y;
//     rotat_z = msg->angular.z;
//     last_cmd_vel_time = ros::Time::now();
//     char output[12] = {0x5a, 0xc, 0x01, 0x01, (char)((((int)(trans_x * 1000.0)) >> 8) & 0xff), (char)(((int)(trans_x * 1000.0) & 0xff)),
//                         (char)((((int)(trans_y * 1000.0)) >> 8) & 0xff), (char)(((int)(trans_y * 1000.0) & 0xff)),
//                         (char)((((int)(rotat_z * 1000.0)) >> 8) & 0xff), (char)(((int)(rotat_z * 1000.0) & 0xff))};
//     int length = sizeof(output) / sizeof(output[0]);
//     char outputdata[] = {0x5a, 0x0c, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//     outputdata[4] = (int(trans_x * 1000.0) >> 8) & 0xff;
//     outputdata[5] = int(trans_x * 1000.0) & 0xff;
//     outputdata[6] = (int(trans_y * 1000.0) >> 8) & 0xff;
//     outputdata[7] = int(trans_y * 1000.0) & 0xff;
//     outputdata[8] = (int(rotat_z * 1000.0) >> 8) & 0xff;
//     outputdata[9] = int(rotat_z * 1000.0) & 0xff;
//     int crc_8 = crc_byte(outputdata, sizeof(outputdata) / sizeof(outputdata[0]) - 1);
//     output[11] = char(crc_8);

//     ros::Duration dur(0, 10000000);
//     while (serialIDLE_flag)
//         dur.sleep();
//     serialIDLE_flag = 4;
//     int len = 0;
//     while (len != length)
//     {
//         int ret = write(serial, output + len, length - len);
//         if (ret < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
//         {
//             // usleep(1000);
//             continue;
//         }
//         if (ret < 0)
//             break;

//         len += ret;
//     }
//     if (len != length)
//         ROS_ERROR("Vel Command Send Faild");
//     serialIDLE_flag = 0;
// }
// void BaseControl::timerCommunicationCB(const ros::TimerEvent &event)
// {
//     // ROS_INFO("%s %d  ",__FUNCTION__,__LINE__);
//     int len;
//     char buf[1024];
//     len = read(serial, buf, 1024);
//     //  ROS_INFO("len   %d \n",len);
//     if (len < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
//         return;
//     int i = 0;
//     if (len < 0)
//     {
//         ROS_INFO("read  serial error \n");
//         perror("read ");
//     }
//     else if (len > 0)
//         while (i < len)
//         {
//             queue->enqueue(buf[i]);
//             i++;
//         }
//     if (!queue->is_empty())
//     {
//         char t;
//         queue->get_first(t);
//         if (t == 0x5a)
//         {
//             queue->get_second(t);
//             if (t <= queue->get_size())
//             {
//                 char *buf = new char[t];
//                 queue->dequeue(buf, t);
//                 if (buf[t - 1] != crc_byte(buf, t - 1))
//                 {
//                     queue->dequeue(t);
//                     ROS_INFO("error");
//                      return;
//                 }
//                 // parse receive data
//                 if (buf[3] == 0x04)
//                 {
//                     Vx = buf[4] << 8;
//                     Vx += buf[5];
//                     Vy = buf[6] << 8;
//                     Vy += buf[7];
//                     Vyaw = buf[8] * 256;
//                     Vyaw += buf[9];
//                 }

//                 else if (buf[3] == 0x06)
//                 {
//                     Yawz = buf[8] << 8;
//                     Yawz += buf[9];
//                 }

//                 else if (buf[3] == 0x08)
//                 {
//                     Vvoltage = buf[4] << 8;
//                     Vvoltage += buf[5];
//                     Icurrent = buf[6] << 8;
//                     Icurrent += buf[7];
//                 }

//                 else if (buf[3] == 0x0a)
//                 {
//                     Vx = buf[4] << 8;
//                     Vx += buf[5];
//                     Yawz = buf[6] << 8;
//                     Yawz += buf[7];
//                     Vyaw = buf[8] << 8;
//                     Vyaw += buf[9];
//                 }

//                 else if (buf[3] == 0x12)
//                 {
//                     Vx = buf[4] << 8;
//                     Vx += buf[5];
//                     Vy = buf[6] << 8;
//                     Vy += buf[7];
//                     Yawz = buf[8] << 8;
//                     Yawz += buf[9];
//                     Vyaw = (buf[10] << 8);
//                     Vyaw += buf[11];
//                 }

//                 else if (buf[3] == 0x14)
//                 {

//                     Gyro.x = int(((buf[4] & 0xff) << 24) | ((buf[5] & 0xff) << 16) | ((buf[6] & 0xff) << 8) | (buf[7] & 0xff));
//                     Gyro.y = int(((buf[8] & 0xff) << 24) | ((buf[9] & 0xff) << 16) | ((buf[10] & 0xff) << 8) | (buf[11] & 0xff));
//                     Gyro.z = int(((buf[12] & 0xff) << 24) | ((buf[13] & 0xff) << 16) | ((buf[14] & 0xff) << 8) | (buf[15] & 0xff));

//                     Accel.x = int(((buf[16] & 0xff) << 24) | ((buf[17] & 0xff) << 16) | ((buf[18] & 0xff) << 8) | (buf[19] & 0xff));
//                     Accel.y = int(((buf[20] & 0xff) << 24) | ((buf[21] & 0xff) << 16) | ((buf[22] & 0xff) << 8) | (buf[23] & 0xff));
//                     Accel.z = int(((buf[24] & 0xff) << 24) | ((buf[25] & 0xff) << 16) | ((buf[26] & 0xff) << 8) | (buf[27] & 0xff));

//                     Quat.w = int((buf[28] & 0xff) << 8 | buf[29]);
//                     Quat.x = int((buf[30] & 0xff) << 8 | buf[31]);
//                     Quat.y = int((buf[32] & 0xff) << 8 | buf[33]);
//                     Quat.z = int((buf[34] & 0xff) << 8 | buf[35]);
//                 }

//                 else if (buf[3] == 0x1a)
//                 {

//                     Sonar[0] = buf[4];
//                     Sonar[1] = buf[5];
//                     Sonar[2] = buf[6];
//                     Sonar[3] = buf[7];
//                 }
//                 else if (buf[3] == 0x48)
//                 {
                 
//                     read_id = buf[4];
//                     read_val = ((buf[5] << 8) |(unsigned int )buf[6]);
                 
//                 }
//                 else if (buf[3] == 0x4a)
//                 {
//                  //   ROS_INFO("0x48");
//                     read_arm[0] = (buf[4] << 8) | (unsigned int )buf[5];
//                     read_arm[1] = (buf[6] << 8) | (unsigned int )buf[7] ;
//                     read_arm[2] = (buf[8] << 8) | (unsigned int )buf[9] ;
//                     read_arm[3] = (buf[10] << 8) | (unsigned int )buf[11];
//                     read_arm[4] = (buf[12] << 8) | (unsigned int )buf[13] ;
//                     read_arm[5] = (buf[14] << 8) | (unsigned int )buf[15] ;
//                     read_arm_ok = 1;
                    
//                 }
//                 else if (buf[3] == 0xf2)
//                 {

//                     movebase_hardware_version[0] = buf[4];
//                     movebase_hardware_version[1] = buf[5];
//                     movebase_hardware_version[2] = buf[6];
//                     movebase_firmware_version[0] = buf[7];
//                     movebase_firmware_version[1] = buf[8];
//                     movebase_firmware_version[2] = buf[9];
//                     ROS_INFO("Move Base Hardware Ver %d.%d.%d,Firmware Ver %d.%d.%d", movebase_hardware_version[0], movebase_hardware_version[1], movebase_hardware_version[2],
//                              movebase_firmware_version[0], movebase_firmware_version[1], movebase_firmware_version[2]);
//                 }

//                 else if (buf[3] == 0xf4)
//                 {

//                     ROS_INFO("SN:%02x %02x %02x %02x %02x %02x ", buf[4] << 8 | buf[5], buf[6] << 8 | buf[7],
//                              buf[8] << 8 | buf[9], buf[10] << 8 | buf[11], buf[12] << 8 | buf[13], buf[14] << 8 | buf[15]);
//                 }

//                 else if (buf[3] == 0x22)
//                 {
//                     float fRatio = (buf[6] << 8 | buf[7]) / 10.0;
//                     float fDiameter = (buf[8] << 8 | buf[9]) / 10;
//                     ROS_INFO("Type:%s Motor:%s Ratio:%.01f WheelDiameter:%.01f", movebase_type[buf[4] - 1].c_str(), motor_type[buf[5] - 1].c_str(), fRatio, fDiameter);
//                 }
//                  delete buf;
//             }
//         }
//         else
//             queue->dequeue(t);
//     }
// }

// void BaseControl::timerOdomCB(const ros::TimerEvent &event)
// {
//     //  ROS_INFO("%s %d  ",__FUNCTION__,__LINE__);
//     char output[6];
//     // Get move base velocity data
//     if (movebase_firmware_version[1] == 0)
//     {
//         // old version firmware have no version info and not support new command below
//         output[0] = 0x5a;
//         output[1] = 0x06;
//         output[2] = 0x01;
//         output[3] = 0x09;
//         output[4] = 0x00;
//         output[5] = 0x38;
//     }
//     else
//     {
//         // in firmware version new than v1.1.0,support this command
//         output[0] = 0x5a;
//         output[1] = 0x06;
//         output[2] = 0x01;
//         output[3] = 0x11;
//         output[4] = 0x00;
//         output[5] = 0xa2;
//     }
//     int length = sizeof(output) / sizeof(output[0]);
//     ros::Duration dur(0, 10000000);
//     while (serialIDLE_flag)
//         dur.sleep();
//     serialIDLE_flag = 1;
//     int len = 0;
//     while (len != length)
//     {
//         int ret = write(serial, output + len, length - len);
//         if (ret < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
//         {
//             //  usleep(1000);
//             continue;
//         }
//         if (ret < 0)
//             break;

//         len += ret;
//     }
//     if (len != length)
//         ROS_ERROR("Odom Command Send Faild");
//     serialIDLE_flag = 0;

//     // calculate odom data
//     float Vx = this->Vx / 1000.0;
//     float Vy = this->Vy / 1000.0;
//     float Vyaw = this->Vyaw / 1000.0;

//     pose_yaw = Yawz / 100.0;
//     pose_yaw = pose_yaw * PI / 180.0;

//     ros::Time current_time = ros::Time::now();
//     double dt = (current_time - previous_time).toSec();
//     previous_time = current_time;
//     pose_x = pose_x + Vx * (cos(pose_yaw)) * dt - Vy * (sin(pose_yaw)) * dt;
//     pose_y = pose_y + Vx * (sin(pose_yaw)) * dt + Vy * (cos(pose_yaw)) * dt;

//     geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromRollPitchYaw(0, 0, pose_yaw);
//     nav_msgs::Odometry msg = nav_msgs::Odometry();
//     msg.header.stamp = current_time;
//     msg.header.frame_id = odomId;
//     msg.child_frame_id = baseId;
//     msg.pose.pose.position.x = pose_x;
//     msg.pose.pose.position.y = pose_y;
//     msg.pose.pose.position.z = 0;
//     msg.pose.pose.orientation.x = pose_quat.x;
//     msg.pose.pose.orientation.y = pose_quat.y;
//     msg.pose.pose.orientation.z = pose_quat.z;
//     msg.pose.pose.orientation.w = pose_quat.w;
//     msg.twist.twist.linear.x = Vx;
//     msg.twist.twist.linear.y = Vy;
//     msg.twist.twist.angular.z = Vyaw;
//     odom_pub.publish(msg);
//     tf_broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::createQuaternionFromRPY(0.0, 0.0, pose_yaw), tf::Vector3(pose_x, pose_y, 0.0)), current_time, odomId, baseId));
//     // ROS_INFO("%s %d  ",__FUNCTION__,__LINE__);
// }
// void BaseControl::timerBatteryCB(const ros::TimerEvent &event)
// {
//     //  ROS_INFO("%s %d  ",__FUNCTION__,__LINE__);
//     unsigned char output[] = {0x5a, 0x06, 0x01, 0x07, 0x00, 0xe4}; // 0xe4 is CRC-8 value
//     int length = sizeof(output) / sizeof(output[0]);
//     ros::Duration dur(0, 10000000);
//     while (serialIDLE_flag)
//         dur.sleep();
//     serialIDLE_flag = 3;
//     int len = 0;
//     while (len != length)
//     {
//         int ret = write(serial, output + len, length - len);
//         if (ret < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
//         {
//             // usleep(1000);
//             continue;
//         }
//         if (ret < 0)
//             break;

//         len += ret;
//     }
//     if (len != length)
//         ROS_ERROR("Battery Command Send Faild");
//     serialIDLE_flag = 0;

//     sensor_msgs::BatteryState msg = sensor_msgs::BatteryState();
//     msg.header.stamp = current_time;
//     msg.header.frame_id = baseId;
//     msg.voltage = float(Vvoltage / 1000.0);
//     msg.current = float(Icurrent / 1000.0);
//     battery_pub.publish(msg);
//     //  ROS_INFO("%s %d  ",__FUNCTION__,__LINE__);
// }
// void BaseControl::timerIMUCB(const ros::TimerEvent &event)
// {
//     // ROS_INFO("%s %d  ",__FUNCTION__,__LINE__);
//     unsigned char output[] = {0x5a, 0x06, 0x01, 0x13, 0x00, 0x33}; // 0x33 is CRC-8 value
//     int length = sizeof(output) / sizeof(output[0]);
//     ros::Duration dur(0, 10000000);
//     while (serialIDLE_flag)
//         dur.sleep();
//     serialIDLE_flag = 3;
//     int len = 0;
//     while (len != length)
//     {
//         int ret = write(serial, output + len, length - len);
//         if (ret < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
//         {
//             // usleep(1000);
//             continue;
//         }
//         if (ret < 0)
//             break;

//         len += ret;
//     }
//     if (len != length)
//         ROS_ERROR("IMU Command Send Faild");
//     serialIDLE_flag = 0;

//     sensor_msgs::Imu msg = sensor_msgs::Imu();
//     msg.header.stamp = current_time;
//     msg.header.frame_id = imuId;

//     msg.angular_velocity.x = (Gyro.x) / 100000.0;
//     msg.angular_velocity.y = (Gyro.y) / 100000.0;
//     msg.angular_velocity.z = (Gyro.z) / 100000.0;

//     msg.linear_acceleration.x = (Accel.x) / 100000.0;
//     msg.linear_acceleration.y = (Accel.y) / 100000.0;
//     msg.linear_acceleration.z = (Accel.z) / 100000.0;

//     msg.orientation.w = (Quat.w) / 10000.0;
//     msg.orientation.x = (Quat.x) / 10000.0;
//     msg.orientation.y = (Quat.y) / 10000.0;
//     msg.orientation.z = (Quat.z) / 10000.0;

//     imu_pub.publish(msg);
//     // TF value calculate from mechanical structure
//     if (!base_type.compare("NanoCar"))
//         tf_broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0.0, 0.0, 0.0, 1.0), tf::Vector3(0.0, 0.0, 0.09)), current_time, baseId, imuId));
// }
// void BaseControl::getVersion()
// {

//     unsigned char output[] = {0x5a, 0x06, 0x01, 0xf1, 0x00, 0xd7}; // 0xd7 is CRC-8 value
//     int length = sizeof(output) / sizeof(output[0]);
//     ros::Duration dur(0, 10000000);
//     while (serialIDLE_flag)
//         dur.sleep();
//     serialIDLE_flag = 1;
//     int len = 0;
//     while (len != length)
//     {
//         int ret = write(serial, output + len, length - len);
//         if (ret < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
//         {
//             //  usleep(1000);
//             continue;
//         }
//         if (ret < 0)
//             break;

//         len += ret;
//     }
//     if (len != length)
//         ROS_ERROR("Get Version Command Send Faild");
//     serialIDLE_flag = 0;
// }
// void BaseControl::getSN()
// {
//     unsigned char output[] = {0x5a, 0x06, 0x01, 0xf3, 0x00, 0x46}; // 0x46 is CRC-8 value
//     int length = sizeof(output) / sizeof(output[0]);

//     ros::Duration dur(0, 10000000);
//     while (serialIDLE_flag)
//         dur.sleep();
//     serialIDLE_flag = 1;
//     int len = 0;
//     while (len != length)
//     {
//         int ret = write(serial, output + len, length - len);
//         if (ret < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
//         {
//             //  usleep(1000);
//             continue;
//         }
//         if (ret < 0)
//             break;

//         len += ret;
//     }
//     if (len != length)
//         ROS_ERROR("Get SN Command Send Faild");
//     serialIDLE_flag = 0;
// }
// void BaseControl::getInfo()
// {

//     unsigned char output[] = {0x5a, 0x06, 0x01, 0x21, 0x00, 0x8f}; // 0x8f is CRC-8 value
//     int length = sizeof(output) / sizeof(output[0]);
//     ros::Duration dur(0, 10000000);
//     while (serialIDLE_flag)
//         dur.sleep();
//     serialIDLE_flag = 1;
//     int len = 0;
//     while (len != length)
//     {
//         int ret = write(serial, output + len, length - len);
//         if (ret < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
//         {
//             //   usleep(1000);
//             continue;
//         }
//         if (ret < 0)
//             break;

//         len += ret;
//     }
//     if (len != length)
//         ROS_ERROR("Get info Command Send Faild");
//     serialIDLE_flag = 0;
// }
// void BaseControl::Armcallback(const wd_arm_moveit_demo::ArmJoint::ConstPtr msg)
// {
//     if (msg->joints.size() != 0)
//         for (int i = 0; i < 2; i++)
//         {
//             set_uart_servo_angle_array(msg->joints, msg->run_time);
//             usleep(10000);
//         }
//     else
//         for (int i = 0; i < 2; i++)
//         {
//             set_uart_servo_angle(msg->id, msg->angle, msg->run_time);
//             usleep(10000);
//         }
// }

// BaseControl::BaseControl(ros::NodeHandle *phandle)
// {
//     this->phandle = phandle;
//     // Get params
//     phandle->param(string("base_id"), baseId, string("base_footprint"));
//     phandle->param(string("port"), device_port, string("/dev/move_base"));
//     phandle->param(string("baudrate"), baudrate, 115200);
//     phandle->param(string("odom_id"), odomId, string("odom"));
//     phandle->param(string("odom_topic"), odom_topic, string("odom"));
//     phandle->param(string("odom_freq"), odom_freq, 50);
//     phandle->param(string("battery_topic"), battery_topic, string("battery"));
//     phandle->param(string("battery_freq"), battery_freq, 1.0f);
//     phandle->param(string("cmd_vel_topic"), cmd_vel_topic, string("cmd_vel"));
//     phandle->param(string("ackermann_cmd_topic"), ackermann_cmd_topic, string("ackermann_cmd_topic"));
//     phandle->param(string("pub_imu"), pub_imu, false);
//     if (pub_imu)
//     {
//         phandle->param(string("imu_id"), imuId, string("imu"));
//         phandle->param(string("imu_topic"), imu_topic, string("imu"));
//         phandle->param(string("imu_freq"), imu_freq, 50.0f);
//         if (imu_freq > 100)
//             imu_freq = 100;
//     }

//     phandle->param(string("pub_sonar"), pub_sonar, false);
//     phandle->param(string("sub_ackermann"), sub_ackermann, false);
//     //  #define param
//     queue = new wd::Queue(1024 * 4);
//     current_time = ros::Time::now();
//     previous_time = current_time;
//     movebase_type[0] = "NanoCar";
//     motor_type[0] = "25GA370";
//     last_cmd_vel_time = ros::Time::now();
//     last_ackermann_cmd_time = ros::Time::now();
//     // Serial Communication

//     serial = open(device_port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
//     if (serial < 0 || set_opt(serial, baudrate, 8, 'N', 1))
//         ROS_INFO("Failed to open %s\n", device_port.c_str());
//     else
//         ROS_INFO("Opening Serial sucessful ");
//     arm_topic="TargetAngle";
//     jointstate_topic="joint_states";

//     arm_sub = phandle->subscribe(arm_topic.c_str(), 20, &BaseControl::Armcallback, this);
//     cmd_sub = phandle->subscribe(cmd_vel_topic.c_str(), 20, &BaseControl::cmdCB, this);
//     odom_pub = phandle->advertise<nav_msgs::Odometry>(odom_topic.c_str(), 10);
//     battery_pub = phandle->advertise<sensor_msgs::BatteryState>(battery_topic.c_str(), 3);
//     tf_broadcaster = tf::TransformBroadcaster();

//     timer_odom = phandle->createTimer(ros::Duration(1.0 / odom_freq), &BaseControl::timerOdomCB, this, false, true);
//     timer_battery = phandle->createTimer(ros::Duration(1.0 / battery_freq), &BaseControl::timerBatteryCB, this, false, true);
//     timer_communication = phandle->createTimer(ros::Duration(1.0 / 500), &BaseControl::timerCommunicationCB, this);

//     // inorder to compatibility old version firmware,imu topic is NOT pud in default
//     if (pub_imu)
//     {
//         imu_pub = phandle->advertise<sensor_msgs::Imu>(imu_topic.c_str(), 10);
//         timer_imu = phandle->createTimer(ros::Duration(1.0 / imu_freq), &BaseControl::timerIMUCB, this);
//     }
//     ros::Duration dur(0, 10000000);
//     serialIDLE_flag = 0;
//     getVersion();
//     // move base imu initialization need about 2s, during initialization, move base system is blocked
//     // so need this gap

//     while (movebase_hardware_version[0] == 0)
//     {
//         ros::spinOnce();
//         dur.sleep();
//     }
//     getSN();
//     dur.sleep();
//     getInfo();
// }

// void MySigintHandler(int sig)
// {
//     ros::shutdown();
//     exit(0);
// }

// BaseControl::~BaseControl()
// {

//     timer_communication.stop();
//     timer_imu.stop();
//     timer_odom.stop();
//     if (pub_imu)
//         timer_imu.stop();
//     close(serial);
//     delete queue;
// }
// extern void test(BaseControl &  base);
// ros::Publisher jointstate_pub;
// bool isrun=false;
// void* JointStateThread(void *arg)
// {
//     BaseControl*  base=(BaseControl*)arg;
//     static  vector<double>  angles(6);
//     double  rad2deg=3.14/180; 
//     double  rad2deg1=  90.0/(180-30);  
//     sensor_msgs::JointState state;
//     while(isrun)
//     {
//         if(!base->get_uart_servo_angle_array(angles))
//         {
//            // ROS_INFO("status %lf %lf %lf %lf  %lf %lf ",angles[0],angles[1],angles[2],angles[3],angles[4],angles[5]);
//             state.header.stamp = ros::Time::now();
//             state.header.frame_id = "arm_base_link";
//             for(int i=0;i<6;i++)
//                 if(i==5)
//                     angles[5]= ((angles[5]-30)*rad2deg1-90)*rad2deg;
//                 else  
//                     angles[i]= (angles[i]-90)*rad2deg;  
//             state.name = {"arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5", "grip_joint"};
//             state.position = angles;
//             jointstate_pub.publish(state); 

//             ROS_INFO("success");
//         }
//         else
//         {
//             //ROS_INFO("error");
//         }
        
//         usleep(100000);
//     }
    
//     return NULL;
// }
// BaseControl * pbase;
// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "wd_base_control");
//     signal(SIGINT, MySigintHandler);
//     ros::NodeHandle handle;
//     ROS_INFO("%s base control ...", base_type.c_str());
//     BaseControl base(&handle);
//     pbase=&base;
//     jointstate_pub=handle.advertise<sensor_msgs::JointState>("joint_states", 3);
    
    
    
//     pthread_t thread;
//     isrun=true;
//     pthread_create(&thread,NULL,JointStateThread,&base);
 
 
//     //test(base);
//     ros::spin();
    
    
//     isrun=false;
//     pthread_join(thread, NULL);
// }
