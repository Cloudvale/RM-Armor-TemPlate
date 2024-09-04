#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <iostream>
#include <vector>
#include <atomic>
#include <string>

using namespace std;


typedef union
{
    float f;
    unsigned char c[4];
} float2uchar;

typedef struct
{
    string id;
    string alias;
    string path;
} Device;

typedef struct
{
    float2uchar pitch_angle; //偏航角
    float2uchar yaw_angle;   //俯仰角
    // float2uchar yaw_angle;//偏航角
    // float2uchar pitch_angle;//俯仰角
    float2uchar dis;  //目标距离
    int isSwitched;   //目标是否发生切换
    int isFindTarget; //当识别的图片范围内有目标且电控发来的信号不为0x00（关闭视觉）置为1，否则置0
    int isSpinning;   //目标是否处于陀螺状态
    int ismiddle;     //设置1表示目标进入了可以开火的范围，设置0则表示目标尚未进入可开火的范围，默认置0
} VisionData;


class SerialPort
{
public:
    atomic_bool need_init;
    // atomic_bool need_init = true;
    Device device;
    int fd;      //串口号
    int last_fd; //上一次串口号
    int speed;
    int baud;
    int mode;
    int databits, stopbits, parity;
    unsigned char rdata[255];                 // raw_data
    float quat[4]; //四元数
    float acc[3]; //加速度
    float gyro[3]; //角速度
    float bullet_speed;
    SerialPort(const string ID, const int BUAD);
    SerialPort(char *);
    bool initSerialPort();
    bool get_Mode();
    bool withoutSerialPort();
    Device getDeviceInfo(string path);
    Device setDeviceByID(std::vector<Device> devices);
    std::vector<Device> listPorts();
    void TransformData(const VisionData &data); //主要方案
    void send();
    void set_Brate();
    int set_Bit();
    void closePort();
    void TransformDataFirst(int Xpos, int Ypos, int dis); //方案1



    //  int set_disp_mode(int);
    //  void TransformTarPos(const VisionData &data);
private:
    unsigned char Tdata[30];                  // transfrom data

    string serial_id;

    float exchange_data(unsigned char *data); //将4个uchar合并成一个float
    bool getQuat(unsigned char *data);
    bool getGyro(unsigned char *data);
    bool getAcc(unsigned char *data);
    bool getSpeed(unsigned char *data);

};

#endif // SERIALPORT_H