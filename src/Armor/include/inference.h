#include <iterator>
#include <memory>
#include <string>
#include <vector>
#include <iostream>
// #include <inference_engine.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include "../../general/general.h"

using namespace std;
using namespace cv;
// using namespace InferenceEngine;

//lzc add 添加
// /*装甲配置文件导入*/

/* 颜色定义 */
enum _COLOR
{
    _RED = 0,
    _BLUE = 1,
    _WHITE = 2
};
enum ARMORTYPE
{
    ARMOR_NONE = 0,
    ARMOR_SMALL = 1,
    ARMOR_LARGE = 2
};
enum DETECTEDTYPE
{
    DETECT_LEFT = 0,
    DETECT_RIGHT = 1,
    DETECT_DOUBLE = 2
};
enum FINDTYPE
{
    NOTFIND = 0,
    FIRST = 1,
    SHOOT = 2
};
enum STATE
{
    SEARCHING = 0,
    TRACKING = 2,
    BUFFER = 1
};
/*装甲配置文件导入*/

class ArmorParam
{
public:
    // 图像预处理参数
    double m_roi_value; // roi的放大系数
    int m_min_thresh_red;
    int m_min_thresh_blue;
    int m_color_thresh;

    // 灯条几何筛选参数
    float m_light_max_area;
    float m_light_min_area;
    float m_light_min_ratio;
    float m_light_max_ratio;
    float m_light_min_angle;
    float m_light_max_angle;
    float m_light_area_ratio;
    float m_min_RBdiff;
    float m_min_BRdiff;

    // 装甲板几何筛选参数
    float m_armor_tiltangle;
    float m_armor_malposition;
    float m_armor_angle_diff;
    float m_armor_ratio_max;
    float m_armor_ratio_min;
    float m_lights_diff;
    float m_armor_min_area;
    float m_angle_diff;

    //敌方颜色
    int m_enemy_color;

    // 无数字识别 灯条几何参数
    float m_dimlight_max_area;
    float m_dimlight_min_area;
    float m_dimlight_min_ratio;
    float m_dimlight_max_ratio;
    float m_dimlight_min_angle;
    float m_dimlight_max_angle;
    float m_dimlight_area_ratio;
    float m_dim_min_RBdiff;
    float m_dim_min_BRdiff;

    // 无数字识别 装甲板几何参数
    float m_dim_armor_tiltangle;
    float m_dim_armor_malposition;
    float m_dim_armor_angle_diff;
    float m_dim_armor_ratio_max;
    float m_dim_armor_ratio_min;
    float m_dim_lights_diff;
    float m_dim_angle_diff;
    float m_dim_armor_min_area;

public:
    ArmorParam(){};
    void setParam(const cv::FileStorage &fs);
    
};

/* 灯条定义 */
class Light
{
public:
    cv::RotatedRect m_rect;           //灯条外接矩形
    cv::Point2f m_center;             //灯条中心
    cv::Rect2d m_rectR;               // 灯条正接矩形
    int m_color;                      // 灯条颜色
    double m_ratio;                   //长宽比
    double m_length;                  //灯条长度
    double m_width;                   //灯条宽度
    double m_area;                    //灯条面积
    double m_area_ratio;              //轮廓面积和最小外接矩形面积之比
    double m_angle;                   //灯条角度
    std::vector<cv::Point> m_contour; //灯条轮廓点集
public:
    Light(){};
    Light(const std::vector<cv::Point> contour); //带参构造函数
    bool isLight(ArmorParam _param, bool dimOrlight);                     //灯条几何条件筛选
    void regularRect(cv::RotatedRect &rect);                              //外接矩形矫正
};
typedef std::vector<Light> Lights; // 灯条容器


/* 装甲板定义 */
class Armor_wmj
{
public:
    cv::Rect2d m_rect;                    //装甲板外接矩形
    cv::RotatedRect m_lightRect;          //灯条构成的矩形
    _COLOR m_color;                       //装甲板颜色
    Lights m_pairs;                       //灯条对
    std::vector<cv::Point2f> m_vertices;  //单目测距用角点
    int m_id;                             //装甲id
    cv::Point3f m_position;               //三维坐标信息
    cv::Point3f m_angle;                  //三维角度坐标，等待运算
    cv::Point2f m_center;                 //中心位置，为了双目识别的
    double m_time_seq;                    //时间戳
    ARMORTYPE m_armor_type = ARMOR_NONE;        //大小装甲
    DETECTEDTYPE m_detectedtype;          //判断是双目识别还是单目识别
    FINDTYPE m_bestArmorStatus = NOTFIND; //判断识别结果
    float m_ratio;                        //装甲长宽比
    double m_yaw_angle;                    //按yaw轴旋转的角度
    float m_area;                         //装甲面积
    float m_width;                        //装甲横向长度
    float m_height;                       //装甲纵向长度
    float m_tiltAngle;                    //装甲roll旋转角度
    float m_lighsRatio;                   //装甲板两个灯条的长度比
    float m_angle_diff;                   //装甲板两个灯条的角度差
    double m_socre;                       //装甲板优先度

public:
    Armor_wmj()
    {
        m_pairs.resize(2);
    };
    Armor_wmj(const Light &left, const Light &right, cv::Point2f targetPoint); //带参构造
    bool IsArmor(ArmorParam _param, bool dimOrLight);                      //装甲板几何参数
    bool operator>(const Armor_wmj &armor) const;                              //装甲板排序
};
typedef std::vector<Armor_wmj> Armors;


/* hog-svm定义 */
class HOG_SVM
{
private:
    cv::Ptr<cv::ml::SVM> m_svm;
    std::map<int, int> m_label2id;
    cv::HOGDescriptor m_hog;

public:
    HOG_SVM();
    int test(const cv::Mat &src);
};


struct ArmorObject
{
    Point2f apex[4];
    cv::Rect_<float> rect;
    int cls;
    int color;
    int area;
    float prob;
    std::vector<cv::Point2f> pts;
};


class ArmorDetector
{
public:
    ArmorDetector();
    ~ArmorDetector();
    bool detect(Mat &src,vector<ArmorObject>& objects);
    bool initModel(string path);

    //lzc add
    void img_process(Mat &src,Mat &m_binary_gray);
    bool findLights(Mat &src_output);
    bool findArmors(cv::Mat &src);

    bool IsFakeArmor(int i ,int j);
    bool needToDelate(Armor_wmj &armor, Armor_wmj &b_armor);

    bool findTargetArmor();
    bool judgeColor(Light &light, std::vector<cv::Point> Contours);
    int SetArmorId(Armor_wmj &armor);


private:
    // Core ie;
    // CNNNetwork network;                         // 网络
    // ExecutableNetwork executable_network;       // 可执行网络
    // InferRequest infer_request;      // 推理请求
    // MemoryBlob::CPtr moutput;
    string input_name;
    string output_name;
    
    Eigen::Matrix<float,3,3> transfrom_matrix;
    //lzc add
    ArmorParam m_param;
    Lights m_lights;   //识别到的灯条
    Armors m_armors;   //识别到的装甲板

    Armor_wmj m_bestArmor; //找到的最优装甲板
    HOG_SVM m_svm;                  //数字识别类
    Mat m_src;  //原始图像
    bool useNumber = true; //是否使用数字识别

};

inline double getDistance(cv::Point2f point_a, cv::Point2f a_point_b)
{
    return sqrtf(powf((point_a.x - a_point_b.x), 2) + powf((point_a.y - a_point_b.y), 2));

}


