#include "buff.h"
using namespace cv;
using namespace std;

Buff::Buff()
{
    detector.initModel(network_path);
    coordsolver.loadParam(camera_param_path,camera_name);
    lost_cnt = 0;
    is_last_target_exists = false;
    // input_size = {640,384};
    input_size = {640, 640};
    last_bullet_speed = 0;
    //fmt::print(fmt::fg(fmt::color::pale_violet_red), "[BUFF] Buff init model success! Size: {} {}\n", input_size.height, input_size.width);

#ifdef SAVE_BUFF_LOG
    LOG(INFO)<<"[BUFF] Buff init model success! Size: "<<input_size.height<<" "<<input_size.width;
#endif //SAVE_BUFF_LOG
}

Buff::~Buff()
{
}


bool Buff::run(TaskData &src, VisionData &data)
{
    auto time_start=std::chrono::steady_clock::now();
    vector<BuffObject> objects;
    vector<Fan> fans;
    auto input = src.img;


     Eigen::Matrix3d rmat_imu = Eigen::Matrix3d::Identity();


    if (!detector.detect(input, objects))
    {
#ifdef SHOW_AIM_CROSS
        line(src.img, Point2f(src.img.size().width / 2, 0), Point2f(src.img.size().width / 2, src.img.size().height), Scalar(0,255,0), 1);
        line(src.img, Point2f(0, src.img.size().height / 2), Point2f(src.img.size().width, src.img.size().height / 2), Scalar(0,255,0), 1);
#endif //SHOW_AIM_CROSS
#ifdef SHOW_IMG
        namedWindow("dst",0);
        imshow("dst",src.img);
        waitKey(1);
#endif //SHOW_IMG
        lost_cnt++;
        is_last_target_exists = false;
        last_target_area = 0;
        data = {(float)0, (float)0, (float)0, 0, 0, 0, 1};
        LOG(WARNING) <<"[BUFF] No target detected!";
        return false;
    }



    return false;
}