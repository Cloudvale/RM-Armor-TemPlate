#include <iostream>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <sys/timeb.h>
#include "sort.h"
#include <opencv2/opencv.hpp>
#include <thread>
#include "thread.h"
#include "serialport.h"
#include "debug.h"

using namespace cv;
using namespace std;

#define GLOG_USE_GLOG_EXPORT
#include <glog/logging.h>

int main(int argc,char* argv[])
{
    // std::cout<<"12345"<<std::endl;
#ifdef SAVE_MAIN_LOG   //设置保存日志
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = false;  //除了日志文件之外是否需要标准输出
    FLAGS_colorlogtostderr = true;  //是否启用不同颜色显示
    google::SetLogDestination(google::GLOG_INFO,"../log/info/");  //设置日志级别
    google::SetLogDestination(google::GLOG_WARNING,"../log/warning/");
    google::SetLogDestination(google::GLOG_ERROR,"../log/error/");

#endif //SAVE_MAIN_LOG

    Factory<TaskData> task_factory(3);
    Factory<VisionData> data_transmit_factory(5); 
    MessageFilter<MCUData> data_receiver(100);

    const string SERIAL_ID = "483/5740/200";
    const int BAUD = 115200;
    SerialPort serial(SERIAL_ID, BAUD);
    auto time_start = std::chrono::steady_clock::now();

    std::thread task_producer(&producer, ref(task_factory), ref(data_receiver), time_start);

    std::thread task_consumer(&consumer, ref(task_factory),ref(data_transmit_factory));

    // std::thread transmitter(&dataTransmitter, ref(serial), ref(data_transmit_factory));

    // std::thread serial_watcher(&serialWatcher, ref(serial));

    task_producer.join();
    // task_consumer.join();
    // serial_watcher.join();
    // transmitter.join();


LOG(INFO) << "info test";  //输出一个Info日志
#ifdef SAVE_MAIN_LOG
    LOG(WARNING) << "[MAIN] task_producer end!";
#endif //SAVE_MAIN_LOG

    task_consumer.join();
#ifdef SAVE_MAIN_LOG
    LOG(WARNING) << "[MAIN] task_consumer end!";
#endif //SAVE_MAIN_LOG

    // serial_watcher.join();
#ifdef SAVE_MAIN_LOG
    LOG(WARNING) << "[MAIN] serial_watcher end!";
#endif //SAVE_MAIN_LOG

    // transmitter.join();
#ifdef SAVE_MAIN_LOG
    LOG(WARNING) << "[MAIN] transmitter end!";
#endif //SAVE_MAIN_LOG
 
// #ifdef USING_IMU
//     receiver.join();
//     #ifdef SAVE_MAIN_LOG    
//         LOG(WARNING) << "[MAIN] IMU receiver end!";
//     #endif //SAVE_MAIN_LOG
// #endif //USING_IMU

#ifdef SAVE_MAIN_LOG
    google::ShutdownGoogleLogging();
#endif //SAVE_MAIN_LOG

    return 0;
}




// int main() {
//     // 读取原始图片
//     // string image_path = "123.jpg";  // 替换为你的图片路径
//     Mat original_image = imread( "123.jpg");
//     if (original_image.empty()) {
//         cerr << "Error: Could not read the image " << endl;
//         return -1;
//     }

//     // 图片保存目录
//     string save_folder = "D:/Desktop/lzc-rm/template/test/";  // 替换为你希望保存的目录路径

//     // 保存图片50次，每次命名递增
//     for (int i = 1; i <= 100; ++i) {
//         // 构造文件名
//         string save_path = save_folder + "120104022L001-" + to_string(i) + ".jpg";

//         // 保存图片
//         bool success = imwrite(save_path, original_image);
//         if (success) {
//             cout << "Saved image " << save_path << endl;
//         } else {
//             cerr << "Error: Failed to save image " << save_path << endl;
//         }
//     }

//     return 0;
// }


// int main() {
//     // 图片所在文件夹路径
//     string folder_path = "D:/Desktop/图片/新建文件夹/";

//     // 遍历读取并重命名图片
//     for (int i = 1; i <= 50; ++i) {
//         // 构造原始图片文件名
//         string original_filename = folder_path + "120104029L001-" + to_string(i) + ".jpg";

//         // 使用OpenCV读取图片
//         Mat image = imread(original_filename, IMREAD_UNCHANGED);
//         if (image.empty()) {
//             cerr << "Error: Could not read image " << original_filename << endl;
//             continue;
//         }

//         // 构造新的命名
//         string new_filename = folder_path + "120104029L001-" + to_string(i) + ".jpg";

//         // 重命名图片
//         if (rename(original_filename.c_str(), new_filename.c_str()) != 0) {
//             cerr << "Error: Failed to rename file " << original_filename << endl;
//         } else {
//             cout << "Renamed " << original_filename << " to " << new_filename << endl;
//         }
//     }

//     return 0;
// }


// int main(int argc, char *argv[])
// {

//     // std::thread task_producer(&producer, ref(task_factory), ref(data_receiver), time_start);   //定义一个多线程任务，执行producer函数，传入参数为task_factory，data_receiver....

// // 读取图片并将其转换为Mat对象
// Mat image = imread("image.png");
// // 检查图片是否读取成功
// if (image.empty()) {
//     printf("Could not open or find the image\n");
//     return -1;
// }
// // 创建一个窗口用于显示图片
// namedWindow("Display window", WINDOW_AUTOSIZE);
// // 使用imshow函数显示图片
// imshow("Display window", image);
// // 等待用户按下键
// waitKey(0);
// return 0;
// }
