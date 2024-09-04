#include "thread.h"
using namespace std;




/*主要进行相机的打开以及相应视频的保存*/
bool producer(Factory<TaskData> &factory, MessageFilter<MCUData> &receive_factory, std::chrono::_V2::steady_clock::time_point time_start)
{
#ifdef USING_USB_CAMERA   //使用usb相机
    sleep(6);
    VideoCapture cap(0);
    // VideoCapture cap("/home/tup/Desktop/TUP-InfantryVision-2022-buff/RH.avi");
    fmt::print(fmt::fg(fmt::color::green), "[CAMERA] Open USB Camera success\n");
    #ifdef SAVE_LOG_ALL
        LOG(INFO) << "[CAMERA] Open USB Camera success";
    #endif //SAVE_LOG_ALL
    // auto time_start = std::chrono::steady_clock::now();
#endif //USING_USB_CAMERA

#ifdef USING_VIDEO
    sleep(6); //等待6ms
    // VideoCapture cap("../1.mp4");  //打开默认摄像头
    VideoCapture cap("D:/Desktop/lzc-rm/template/1.mp4");  //打开默认摄像头
    // VideoCapture cap("/home/tup/sample.avi");

    fmt::print(fmt::fg(fmt::color::green), "[CAMERA] Set param finished\n");
#endif


#ifdef SAVE_VIDEO   //相机录制视频
     /*============ video_writer ===========*/
    int frame_cnt = 0;
    const std::string &storage_location = "../data/";   //路径
    char now[64];
    std::time_t tt;
    struct tm *ttime;
    int width = 1280;
    int height = 1024;
    tt = time(nullptr);
    ttime = localtime(&tt);
    strftime(now, 64, "%Y-%m-%d_%H_%M_%S", ttime);  // 以时间为名字
    std::string now_string(now);
    std::string path(std::string(storage_location + now_string).append(".avi"));
    auto writer = cv::VideoWriter(path, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30.0, cv::Size(width, height));    // Avi format
    std::future<void> write_video;
    bool is_first_loop = true;
    #ifdef SAVE_LOG_ALL
        LOG(INFO) << "[SAVE_VIDEO] Save video to " << path;
    #endif //SAVE_LOG_ALL
#endif

#ifdef RECORD_DATA
    /*============ video_writer ===========*/
    int frame_cnt = 0;
    char now[64];
    std::time_t tt;
    struct tm *ttime;
    int width = 1280;
    int height = 1024;
    tt = time(nullptr);
    ttime = localtime(&tt);
    strftime(now, 64, "%Y-%m-%d_%H_%M_%S", ttime);  // 以时间为名字
    std::string now_string(now);
    const std::string storage_location = "../record/" + now_string;
    //Create dir to store datas.
    mkdir(storage_location.c_str(), S_IRWXU);
    fmt::print(fmt::fg(fmt::color::green), "[RECORD] Created directory :{}\n",storage_location);
    string data_file_pth = storage_location + "/" + "data.txt";
    string video_file_pth = storage_location + "/" + "video.avi";
    ofstream data;
    data.open(data_file_pth, std::ofstream::app);
    auto writer = cv::VideoWriter(video_file_pth, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 60.0, cv::Size(width, height));    // Avi format
#endif //RECORD_DATA

    if (!cap.isOpened())
    {
        std::cout << "Error opening video stream or file" << std::endl;
        return false;
    }

    while (true) 
    {
    //使用usb相机
    #ifdef USING_USB_CAMERA
            TaskData src;
            auto time_cap = std::chrono::steady_clock::now(); //获取当前时间
            cap >> src.img;  // 从摄像头捕获一帧图像
    #endif
    
    //使用视频
    #ifdef USING_VIDEO
            TaskData src;
            auto time_cap = std::chrono::steady_clock::now(); //获取当前时间
            cap >> src.img;  // 从摄像头捕获一帧图像 
            waitKey(33.3);
    #endif
        // namedWindow("dst",0);
        // imshow("dst",src.img);
        // 检查是否成功获取图像

        // if (src.img.empty()) {
        //     // fmt::print(fmt::emphasis::bold|fmt::fg(fmt::color::red),"[CAMERA] Get empty image\n");
        //     // LOG(ERROR) << "[CAMERA] Get empty image";
        //     continue;
        // }


#ifdef SAVE_VIDEO
        frame_cnt++;

        if(frame_cnt % 10 == 0)
        {
            frame_cnt = 0;
            //异步读写加速,避免阻塞生产者
            if (is_first_loop)
                is_first_loop = false;
            else
                write_video.wait();
            write_video = std::async(std::launch::async, [&, src](){writer.write(src.img);});
        }
#endif //SAVE_VIDEO

        //用于辅助标注
        factory.produce(src);
    }   
    return true;
}


//进行识别线程，根据状态切换来决定某个模式
bool consumer(Factory<TaskData> &task_factory,Factory<VisionData> &transmit_factory)
{
    Autoaim autoaim;
    // Buff buff;
    auto mode = -1;
    auto last_mode = -1;

    while(1)
    {
        TaskData dst;
        VisionData data;

        task_factory.consume(dst);
        mode = dst.mode;
        

        // if((!dst.img.empty())) 
        // {
        //     namedWindow("dst2",0);
        //     imshow("dst2",dst.img);
        //     waitKey(1);//需要加一个waitkey，否则会卡死，可能是刷新频率过高，导致显示有问题，如果只是做数据传输，不用waitkey，这个证明图像已经传递过去.
        //     // std::cerr<<"consume recevie data !!"<<endl;
        //     // fmt::print(fmt::fg(fmt::color::green), "[CAMERA] consume image\n");
        // }
        // fmt::print(fmt::fg(fmt::color::yellow), "[CONSUMER] Mode is :{}\n", mode);      
#ifdef DEBUG_WITHOUT_COM
        mode = 1;
        // dst.mode = mode;
#endif // DEBUG_WITHOUT_COM

#ifdef SAVE_TRANSMIT_LOG
    // cout<<mode<<"..."<<last_mode<<endl;
    if (mode != last_mode)
    {
        LOG(INFO)<<"[CONSUMER] Mode switched to "<< mode;
        fmt::print(fmt::fg(fmt::color::pale_violet_red), "[CONSUMER] Mode switched to {}\n", mode);
        last_mode = mode;
    }
#endif //SAVE_TRANSMIT_LOG
        //1:自瞄模式
        //2:前哨站顶部装甲板识别模式
        //3:小能量机关模式
        //4.大能量机关模式
        if (mode == 1 || mode == 2)
        {
            autoaim.run(dst, data);
            transmit_factory.produce(data);
        }
        else if (mode == 3 || mode == 4)
        {
            // buff.run(dst, data);
            // transmit_factory.produce(data);
        }
    }
    return true;

}

bool dataTransmitter(SerialPort &serial,Factory<VisionData> &transmit_factory)
{
    // while(1)
    // {
    //     VisionData transmit;
    //     transmit_factory.consume(transmit);
    //     //若串口离线则跳过数据发送
    //     //TODO:使用无串口的模式时会导致此线程死循环，浪费CPU性能
    //     if (serial.need_init == true)
    //     {
    //         // cout<<"offline..."<<endl;
    //         #ifndef DEBUG_WITHOUT_COM
    //             #ifdef SAVE_LOG_ALL
    //                 LOG(ERROR) << "[TRANSMITTER] Serial offline, trying to reconnect...";
    //             #endif //SAVE_LOG_ALL
    //         #endif //DEBUG_WITHOUT_COM
    //         usleep(5000);
    //         continue;
    //     }
    //     serial.TransformData(transmit);
    //     serial.send();
    //     // cout<<"transmitting..."<<endl;
    // }
    // return true;

}

bool serialWatcher(SerialPort &serial)
{
//     int last = 0;
// #ifdef DEBUG_WITHOUT_COM
//     #ifdef SAVE_TRANSMIT_LOG
//     LOG(WARNING)<<"[SERIAL] Warning: You are not using Serial port";
//     #endif //SAVE_TRANSMIT_LOG
// #endif // DEBUG_WITHOUT_COM

//     while(1)
//     {
//         sleep(1);
//         //检测文件夹是否存在或串口需要初始化
//         if (access(serial.device.path.c_str(),F_OK) == -1 || serial.need_init)
//         {
//             serial.need_init = true;
// #ifdef DEBUG_WITHOUT_COM
//             int now = clock()/CLOCKS_PER_SEC;
//             if (now - last > 10)
//             {
//                 last = now;
//                 fmt::print(fmt::fg(fmt::color::orange), "[SERIAL] Warning: You are not using Serial port\n");
//             }
//             serial.withoutSerialPort();
// #else
//             serial.initSerialPort();
// #endif //DEBUG_WITHOUT_COM
//         }
//     }
}