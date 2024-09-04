#include "inference.h"
#include <opencv2/core.hpp>
#include "../debug.h"

#define PI 3.141592653589

using namespace cv::ml;

// static constexpr int INPUT_W = 640;    // Width of input
// static constexpr int INPUT_H = 384;    // Height of input
static constexpr int INPUT_W = 416;    // Width of input
static constexpr int INPUT_H = 416;    // Height of input
static constexpr int NUM_CLASSES = 8;  // Number of classes
static constexpr int NUM_COLORS = 4;   // Number of color
static constexpr int TOPK = 128;       // TopK
static constexpr float NMS_THRESH = 0.3;
static constexpr float BBOX_CONF_THRESH = 0.75;
static constexpr float MERGE_CONF_ERROR = 0.15;
static constexpr float MERGE_MIN_IOU = 0.9;



/**
 * @brief Resize the image using letterbox
 * @param img Image before resize
 * @param transform_matrix Transform Matrix of Resize
 * @return Image after resize
 */

inline cv::Mat scaledResize(cv::Mat& img, Eigen::Matrix<float,3,3> &transform_matrix)
{
    float r = std::min(INPUT_W / (img.cols * 1.0), INPUT_H / (img.rows * 1.0));
    int unpad_w = r * img.cols;
    int unpad_h = r * img.rows;
    
    int dw = INPUT_W - unpad_w;
    int dh = INPUT_H - unpad_h;

    dw /= 2;
    dh /= 2;
    
    transform_matrix << 1.0 / r, 0, -dw / r,
                        0, 1.0 / r, -dh / r,
                        0, 0, 1;
    
    Mat re;
    cv::resize(img, re, Size(unpad_w,unpad_h));
    Mat out;
    cv::copyMakeBorder(re, out, dh, dh, dw, dw, BORDER_CONSTANT);

    return out;
}

Armor_wmj::Armor_wmj(const Light &left, const Light &right, cv::Point2f targetPoint)
{
    m_vertices.resize(4);
    //装甲板的两根灯条
    m_pairs.push_back(left);
    m_pairs.push_back(right);
    //求装甲板ROI，用于下一帧ROI识别
    Point2d leftCenter = left.m_center;
    Point2d rightCenter = right.m_center;
    double Armor_lightRatio = 2.0; //假定的灯条和装甲板长度的比值
    Point2d LTpoint, RDpoint;
    LTpoint.x = leftCenter.x - left.m_width / 2;
    LTpoint.y = leftCenter.y - left.m_length * Armor_lightRatio / 2;
    RDpoint.x = rightCenter.x + right.m_width / 2;
    RDpoint.y = rightCenter.y + right.m_length * Armor_lightRatio / 2;
    //得到装甲板框选区域
    m_rect = Rect(LTpoint, RDpoint);
    m_rect &= Rect2d(Point(0, 0), Point(1280, 1024));
    //计算装甲板中心
    m_center = Point2f((leftCenter.x + rightCenter.x) / 2, (leftCenter.y + rightCenter.y) / 2);
    m_width = getDistance(left.m_center, right.m_center);                                   //横向长度
    m_height = (left.m_length + right.m_length) / 2;                                        //纵向长度
    m_ratio = m_width / m_height;                                                           //长宽比
    m_armor_type = m_ratio > 3 ? ARMORTYPE::ARMOR_LARGE : ARMORTYPE::ARMOR_SMALL;                                   //装甲板分类
    m_tiltAngle = asin(abs(left.m_center.y - right.m_center.y) / m_width) * 180 / PI;       //倾斜角
    m_angle_diff = abs(m_pairs[0].m_angle - m_pairs[1].m_angle);                            //装甲板的灯条角度差
    m_lighsRatio = max(left.m_length, right.m_length) / min(left.m_length, right.m_length); //两个灯条长度的比值
    //框出灯条画出的矩形
    Point2f left_points[4], right_points[4];
    m_pairs[0].m_rect.points(left_points);
    m_pairs[1].m_rect.points(right_points);
    m_vertices[0] = (left_points[0] + left_points[1]) / 2;
    m_vertices[1] = (left_points[3] + left_points[2]) / 2;
    m_vertices[2] = (right_points[2] + right_points[3]) / 2;
    m_vertices[3] = (right_points[1] + right_points[0]) / 2;
    m_lightRect = minAreaRect(m_vertices);
    /*###############################
    根据装甲板到屏幕中心的距离和几何形状规范程度计算装甲板得分
    ################################*/
    m_socre = exp(-getDistance(m_center, targetPoint) / 100) + m_tiltAngle / 5;

}

bool Armor_wmj::IsArmor(ArmorParam _param, bool dimOrLight)
{
    if (dimOrLight)
    {
        if (!(m_tiltAngle < _param.m_armor_tiltangle &&
                m_ratio < _param.m_armor_ratio_max &&
                m_ratio > _param.m_armor_ratio_min &&
                m_lighsRatio < _param.m_lights_diff &&
                m_width > 7 && m_height > 5 &&
                m_angle_diff < _param.m_angle_diff))
        {
            return 0;
        }

        return 1;
        }
        else
        {

            // if(m_tiltAngle < _param.m_dim_armor_tiltangle)
            // {
            //     fmt::print(fmt::emphasis::bold | fg(fmt::color::red), "1\n");
            // }
            // if( m_ratio < _param.m_dim_armor_ratio_max &&
            //     m_ratio > _param.m_dim_armor_ratio_min)
            // {
            //     fmt::print(fmt::emphasis::bold | fg(fmt::color::red), "2\n");
            // }
            // if(m_lighsRatio < _param.m_dim_lights_diff)
            // {
            //     fmt::print(fmt::emphasis::bold | fg(fmt::color::red), "3\n");
            // }
            // if(m_width > _param.m_dim_armor_malposition)
            // {
            //     fmt::print(fmt::emphasis::bold | fg(fmt::color::red), "4\n");
            // }
            // if(m_angle_diff < _param.m_dim_angle_diff)
            // {
            //     fmt::print(fmt::emphasis::bold | fg(fmt::color::red), "5\n");
            // }

            // fmt::print(fmt::emphasis::bold | fg(fmt::color::yellow_green), "return\n");

            // fmt::print(fmt::emphasis::bold | fg(fmt::color::yellow_green), "m_tiltAngle :{},{}\n",m_tiltAngle,_param.m_dim_armor_tiltangle);

        if (!(m_tiltAngle < _param.m_dim_armor_tiltangle &&
                m_ratio < _param.m_dim_armor_ratio_max &&
                m_ratio > _param.m_dim_armor_ratio_min &&
                m_lighsRatio < _param.m_dim_lights_diff &&
                m_width > _param.m_dim_armor_malposition &&
                m_angle_diff < _param.m_dim_angle_diff))
        {
            return 0;
        }

        return 1;
    }
}

bool Armor_wmj::operator>(const Armor_wmj &armor) const
{
    if (m_socre > armor.m_socre)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}


/*#########################################
            Light类函数
########################################*/
Light::Light(const std::vector<cv::Point> contour)
{
    m_rect = fitEllipse(contour);
    regularRect(m_rect);
    m_angle = m_rect.angle;
    m_rectR = boundingRect(contour);
    m_center = m_rect.center;
    m_length = m_rect.size.width;
    m_width = m_rect.size.height;
    m_area = m_rectR.area();
    m_ratio = m_length / m_width;
    m_contour = contour;
    m_area_ratio = contourArea(contour) / (m_width * m_length);
}

bool Light::isLight(ArmorParam _param, bool dimOrlight)
{
    if (dimOrlight)
    {
        return m_width > 2 && m_length > 4.5 &&
                m_ratio < _param.m_light_max_ratio &&
                m_area < _param.m_light_max_area &&
                m_area_ratio > _param.m_light_area_ratio &&
                m_area > _param.m_light_min_area &&
                m_ratio > _param.m_light_min_ratio;
    }
    else
    {
        // fmt::print(fmt::emphasis::bold | fg(fmt::color::gray),"m_area:{}", m_area);
        // fmt::print(fmt::emphasis::bold | fg(fmt::color::gray),"m_ratio: {}, _:{},:{}\n", m_ratio,_param.m_dimlight_min_ratio,_param.m_dimlight_max_ratio);
        // fmt::print(fmt::emphasis::bold | fg(fmt::color::gray),"m_area: {}, :{},:{}\n",m_area, _param.m_dimlight_min_area,_param.m_dimlight_max_area);
        // fmt::print(fmt::emphasis::bold | fg(fmt::color::gray),"m_angle: {}, :{},:{}\n", m_ratio,_param.m_dimlight_min_ratio,_param.m_dimlight_max_angle);
        // fmt::print(fmt::emphasis::bold | fg(fmt::color::gray),"m_area_ratio: {}, _param.m_dimlight_area_ratio:{}\n", m_area_ratio,_param.m_dimlight_area_ratio);
        // fmt::print(fmt::emphasis::bold | fg(fmt::color::blue),"return\n");

        return m_ratio > _param.m_dimlight_min_ratio &&
                m_ratio < _param.m_dimlight_max_ratio &&
                m_area > _param.m_dimlight_min_area &&
                m_area < _param.m_dimlight_max_area &&
                m_angle > _param.m_dimlight_min_angle &&
                m_angle < _param.m_dimlight_max_angle &&
                m_area_ratio > _param.m_dimlight_area_ratio;
    }
}

void Light::regularRect(cv::RotatedRect &rect)
{
    if (rect.size.height > rect.size.width)
    {
        std::swap<float>(rect.size.height, rect.size.width);
        rect.angle =
            rect.angle >= 0.0f ? rect.angle - 90.0f : rect.angle + 90.0f;
    }
    if (rect.angle < 0)
    {
        rect.angle += 180.0f;
    }
}

//svm类函数
HOG_SVM::HOG_SVM()
{
    m_label2id = {{0, 0}, {1, 1}, {2, 2}, {3, 3}, {4, 4}, {5, 5}, {6, 11}, {7, 7}, {8, 8}};
    m_hog.winSize = Size(48, 32);
    m_hog.blockSize = Size(16, 16);
    m_hog.blockStride = Size(8, 8);
    m_hog.cellSize = Size(8, 8);
    m_hog.nbins = 9;
    m_hog.derivAperture = 1;
    m_hog.winSigma = -1;
    m_hog.histogramNormType = HOGDescriptor::L2Hys;
    m_hog.L2HysThreshold = 0.2;
    m_hog.gammaCorrection = false;
    m_hog.free_coef = -1.f;
    m_hog.nlevels = HOGDescriptor::DEFAULT_NLEVELS;
    m_hog.signedGradient = false;
    if (m_svm)
    {
        m_svm->clear();
    }
    m_svm = SVM::load(SVM_XML);
}
int HOG_SVM::test(const Mat &src)
{
    if (m_svm)
    {
        vector<float> descriptors;
        m_hog.compute(src, descriptors, Size(8, 8));
        int label = m_svm->predict(descriptors);
        return m_label2id[label];
    }
    else
    {
        return 0;
    }
}



void ArmorParam::setParam(const cv::FileStorage &fs)
{
    //预处理参数
    fs["ArmorDetector"]["roi_value"] >> m_roi_value;
    fs["ArmorDetector"]["min_thresh_red"] >> m_min_thresh_red;
    fs["ArmorDetector"]["min_thresh_blue"] >> m_min_thresh_blue;
    fs["ArmorDetector"]["color_thresh"] >> m_color_thresh;
    //灯条参数
    fs["ArmorDetector"]["light_max_area"] >> m_light_max_area;
    fs["ArmorDetector"]["light_min_area"] >> m_light_min_area;
    fs["ArmorDetector"]["light_min_ratio"] >> m_light_min_ratio;
    fs["ArmorDetector"]["light_max_ratio"] >> m_light_max_ratio;
    fs["ArmorDetector"]["light_min_angle"] >> m_light_min_angle;
    fs["ArmorDetector"]["light_max_angle"] >> m_light_max_angle;
    fs["ArmorDetector"]["light_area_ratio"] >> m_light_area_ratio;
    fs["ArmorDetector"]["RBdiff"] >> m_min_RBdiff;
    fs["ArmorDetector"]["BRdiff"] >> m_min_BRdiff;
    //装甲板参数
    fs["ArmorDetector"]["armor_tiltangle"] >> m_armor_tiltangle;
    fs["ArmorDetector"]["armor_malposition"] >> m_armor_malposition;
    fs["ArmorDetector"]["armor_ratio_max"] >> m_armor_ratio_max;
    fs["ArmorDetector"]["armor_ratio_min"] >> m_armor_ratio_min;
    fs["ArmorDetector"]["lights_diff_max"] >> m_lights_diff;
    fs["ArmorDetector"]["armor_min_area"] >> m_armor_min_area;
    fs["ArmorDetector"]["angle_diff"] >> m_angle_diff;
    //敌方颜色
    fs["ArmorDetector"]["enemy_color"] >> m_enemy_color;

    fs["dimDetect"]["light_max_area"] >> m_dimlight_max_area;
    fs["dimDetect"]["light_min_area"] >> m_dimlight_min_area;
    fs["dimDetect"]["light_min_ratio"] >> m_dimlight_min_ratio;
    fs["dimDetect"]["light_max_ratio"] >> m_dimlight_max_ratio;
    fs["dimDetect"]["light_min_angle"] >> m_dimlight_min_angle;
    fs["dimDetect"]["light_max_angle"] >> m_dimlight_max_angle;
    fs["dimDetect"]["light_area_ratio"] >> m_dimlight_area_ratio;
    fs["dimDetect"]["RBdiff"] >> m_dim_min_RBdiff;
    fs["dimDetect"]["BRdiff"] >> m_dim_min_BRdiff;

    //装甲板参数
    fs["dimDetect"]["armor_tiltangle"] >> m_dim_armor_tiltangle;
    fs["dimDetect"]["angle_diff"] >> m_dim_angle_diff;
    fs["dimDetect"]["armor_malposition"] >> m_dim_armor_malposition;
    fs["dimDetect"]["armor_ratio_max"] >> m_dim_armor_ratio_max;
    fs["dimDetect"]["armor_ratio_min"] >> m_dim_armor_ratio_min;
    fs["dimDetect"]["lights_diff_max"] >> m_dim_lights_diff;
}

static void decodeOutputs(const float* prob, std::vector<ArmorObject>& objects,
                            Eigen::Matrix<float,3,3> &transform_matrix, const int img_w, const int img_h)
{
        // std::vector<ArmorObject> proposals;
        // std::vector<int> strides = {8, 16, 32};
        // std::vector<GridAndStride> grid_strides;

        // generate_grids_and_stride(INPUT_W, INPUT_H, strides, grid_strides);
        // generateYoloxProposals(grid_strides, prob, transform_matrix, BBOX_CONF_THRESH, proposals);
        // qsort_descent_inplace(proposals);

        // if (proposals.size() >= TOPK) 
        //     proposals.resize(TOPK);
        // std::vector<int> picked;
        // nms_sorted_bboxes(proposals, picked, NMS_THRESH);
        // int count = picked.size();
        // objects.resize(count);

        // for (int i = 0; i < count; i++)
        // {
        //     objects[i] = proposals[picked[i]];
        // }
}



ArmorDetector::ArmorDetector()
{
    FileStorage fs(ARMOR_CFG, cv::FileStorage::READ);
    // 读取参数文件
    m_param.setParam(fs);
}

ArmorDetector::~ArmorDetector()
{

}

void ArmorDetector::img_process(Mat &src,Mat &m_binary_gray)
{
    Mat src_roi = src;
    m_src= src.clone();
    //通道分离
    vector<Mat> rgb; split(src_roi,rgb); int thresh_light = 160;
    //默认识别红色区域
    Mat m_gray = rgb[0];
    threshold(m_gray,m_binary_gray,m_param.m_color_thresh,255,THRESH_BINARY);
    //进行形态学开操作
    Mat element = getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(3,5));
    morphologyEx(m_binary_gray,m_binary_gray,MORPH_OPEN,element);

}

bool ArmorDetector::findLights(Mat &src_output)
{
    vector<vector<Point>> contours_gray;
    vector<Vec4i> lines;  //使用cv::RETR_EXTERNAL参数，只找最外层轮廓
    findContours(src_output,contours_gray,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_NONE);
    for (size_t i = 0; i < contours_gray.size(); i++)
    {
        // if(contours_gray.size()!=0)
        // {
        //     fmt::print(fmt::emphasis::bold|fg(fmt::color::yellow),"contours_gray.size(): {0:d} °\n",contours_gray.size());
        // }
        RotatedRect rect = minAreaRect(contours_gray[i]);
         cv::Rect2d m_roi;               // ROI
        if(contours_gray[i].size()>10 && rect.boundingRect().area()>10)
        {
           Light light(contours_gray[i]);
           if(light.isLight(m_param,false))
            {
                // 几何条件筛选
                if (judgeColor(light, contours_gray[i]) && light.m_rect.angle > 20 && light.m_rect.angle < 160)
                {
                    cv::Vec4f line;
                    vector<Point2f> tmp;
                    Mat(contours_gray[i]).convertTo(tmp, CV_32FC1);
                    resize(tmp, tmp, Size(20, 1));
                    // 线性拟合
                    fitLine(tmp, line, CV_DIST_HUBER, 0, 0.01, 0.01);
                    light.m_angle = atan2(line[1], line[0]) * 180 / PI;
                    light.m_angle = light.m_angle < 0 ? light.m_angle + 180 : light.m_angle;
                    if (light.m_angle > 60 && light.m_angle < 120)
                    {
                        m_lights.push_back(light);
                    }
                }
            }
        }
        // drawContours(src_output,contours_gray,i,Scalar(255),1,8,lines);
    }
    return m_lights.size()>=2;
    // namedWindow("data imshow",0);
    // imshow("data imshow",src_output);
}

bool ArmorDetector::findArmors(cv::Mat &src)
{

    if(m_lights.size() != 0)
    {
        sort(m_lights.begin(),m_lights.end(),[&](const Light &a,const Light &b)
            {return a.m_center.x < b.m_center.x;});
    }
    else return false;

    //遍历灯条，找到每对可能是装甲板的armor，并存储在armors对象里
    for (size_t i = 0; i < m_lights.size()-1; i++)
    {
        for (size_t j = i+1; j < m_lights.size(); j++)
        {
            /* code */
            Light left = m_lights[i];
            Light right = m_lights[j];

            if (left.m_rect.boundingRect2f().br().y < right.m_rect.boundingRect2f().tl().y || right.m_rect.boundingRect2f().br().y < left.m_rect.boundingRect2f().tl().y)
            {
                continue;
            }
        
            cv::Point targetPoint = (left.m_center + right.m_center) / 2;
            Armor_wmj armor(left, right, targetPoint);
            if(armor.IsArmor(m_param,false))
            {
                if(!IsFakeArmor(i,j))
                {
                    m_armors.push_back(armor);
                }
            }
        }
    }

    //筛选出装甲板是否有重叠
    if(m_armors.size() > 1)
    {
        for (size_t i = 0; i < m_armors.size()-1; i++)
        {
            for (size_t j = i+1; j < m_armors.size(); j++)
            {
                if(needToDelate(m_armors[i],m_armors[j]))
                {
                    m_armors[i].m_armor_type = ARMORTYPE::ARMOR_NONE;
                }
            }
        }
    }

    // fmt::print(fmt::emphasis::bold | fg(fmt::color::red),"m_armors.size: {} °\n",m_armors.size());

    //删除所有类型为ARMORTYPE::ARMOR_NONE的装甲板，去除掉重叠装甲板
    m_armors.erase(std::remove_if(m_armors.begin(),m_armors.end(),[&](Armor_wmj a)
                                    { return a.m_armor_type == ARMORTYPE::ARMOR_NONE;}),
                    m_armors.end());


#ifdef DEBUG_SHOW_ARMOR
    //找到目标装甲板......
    if(m_armors.size() != 0)
    {
        // fmt::print(fmt::emphasis::bold | fg(fmt::color::red),"m_armors.size: {} °\n",m_armors.size());
        for(const auto m_armor : m_armors)
        {
            // fmt::print(fmt::emphasis::bold | fg(fmt::color::red),"m_armor.m_rect: {} °\n",m_armor.m_rect);
            cv::rectangle(src, m_armor.m_rect, cv::Scalar(0, 255, 0), 2); // 绿色矩形，线宽为 2
            namedWindow("imshow_armor",0);
            cv::imshow("imshow_armor", src);
            cv::waitKey(1);


            Mat image_armor = src(m_armor.m_rect);
            namedWindow("armor",0);
            imshow("armor",image_armor);
        }
    }
#endif

    //找到一个最好的装甲板
    if (m_armors.size() != 0)
    {
        if (!findTargetArmor() && useNumber)
        {
            m_bestArmor.m_bestArmorStatus = NOTFIND;
        }
        else
        {
            m_bestArmor.m_bestArmorStatus = FIRST;
        }
    }
    else
    {
        m_bestArmor.m_bestArmorStatus = NOTFIND;
    }

    namedWindow("imshow_armor",0);
    imshow("imshow_armor", m_src);
    waitKey(1);

    return !m_armors.empty();

}

bool ArmorDetector::IsFakeArmor(int i, int j)
    {
        Rect2d rect_left = m_lights[i].m_rectR;
        Rect2d rect_right = m_lights[j].m_rectR;
        double min_y, max_y;
        min_y = fmin(rect_left.y, rect_right.y) + 2;
        max_y = fmax(rect_left.y + rect_left.height, rect_right.y + rect_right.height) - 2;

        for (int s = i + 1; s < j; s++)
        {
            if ((m_lights[s].m_center.y < max_y && m_lights[s].m_center.y > min_y) || (m_lights[s].m_rectR.y < max_y && m_lights[s].m_rectR.y > min_y) || ((m_lights[s].m_rectR.y + m_lights[s].m_rectR.height) < max_y && (m_lights[s].m_rectR.y + m_lights[s].m_rectR.height) > min_y))
            {
                return true;
            }
        }
        return false;
    }

bool ArmorDetector::needToDelate(Armor_wmj &armor, Armor_wmj &b_armor)
{
    if (armor.m_rect.x <= b_armor.m_rect.x && armor.m_rect.y <= b_armor.m_rect.y &&
        armor.m_rect.x + armor.m_rect.width >= b_armor.m_rect.x + b_armor.m_rect.width &&
        armor.m_rect.y + armor.m_rect.height >= b_armor.m_rect.y + b_armor.m_rect.height)
    {
        return true;
    }
    return false;
}

bool ArmorDetector::findTargetArmor()
{
    if(m_armors.size() == 1)
    {
        m_bestArmor = m_armors[0];

        //开始数字识别
        int armorID = SetArmorId(m_bestArmor);
        if(armorID != 0)
        {
            fmt::print(fmt::emphasis::bold | fg(fmt::color::red),"armorID: {} °\n",armorID);
            putText(m_src, to_string(armorID), Point(m_bestArmor.m_rect.x, m_bestArmor.m_rect.y), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 2, 8, false);
        }
        cv::rectangle(m_src, m_bestArmor.m_rect, cv::Scalar(0, 255, 0), 2); // 绿色矩形，线宽为 2
    }
}

bool ArmorDetector::judgeColor(Light &light, std::vector<cv::Point> Contours)
{

    Vec3b pixel;
    int red = 0, blue = 0;
    for (int j = 0; j < Contours.size(); j++)
    {
        pixel = m_src.at<Vec3b>(Point(Contours[j]));  //    m_src 是原始图像，BGR图像
        red += pixel[2];
        blue += pixel[0];
    }
    int RB_diff = (red - blue) / int(Contours.size());
    if (useNumber)
    {
        if (RB_diff > m_param.m_min_RBdiff)
        {
            light.m_color = 0;
        }
        else if (RB_diff < (-1 * m_param.m_min_BRdiff))
        {
            light.m_color = 1;
        }
        else
        {
            light.m_color = 2;
        }
    }
    else
    {
        if (RB_diff > m_param.m_dim_min_RBdiff)
        {
            light.m_color = 0;
        }
        else if (RB_diff < (-1 * m_param.m_dim_min_BRdiff))
        {
            light.m_color = 1;
        }
        else
        {
            light.m_color = 2;
        }
    }

    if (light.m_color == m_param.m_enemy_color)
    {

        return true;
    }
    else
    {
        return false;
    }
}

int ArmorDetector::SetArmorId(Armor_wmj &armor)
{
// 选择数字识别的区域
    Rect2d rect_left = armor.m_pairs[0].m_rectR;
    Rect2d rect_right = armor.m_pairs[1].m_rectR;
    double min_x, min_y, max_x, max_y;
    min_x = fmin(rect_left.x + rect_left.width, rect_right.x + rect_right.width) + 4;
    max_x = fmax(rect_left.x, rect_right.x) - 4;
    min_y = fmin(rect_left.y, rect_right.y) - 0.5 * (rect_left.height + rect_right.height) / 2.0;
    max_y = fmax(rect_left.y + rect_left.height, rect_right.y + rect_right.height) + 0.5 * (rect_left.height + rect_right.height) / 2.0;
    // 冗余操作保证不会内存越界
    min_x = min_x <= 0 ? 0 : min_x;
    min_y = min_y <= 0 ? 0 : min_y;
    max_x = max_x >= 1279 ? 1279 : max_x;
    max_y = max_y >= 1023 ? 1023 : max_y;
    if (min_x > max_x)
    {
        swap(min_x, max_x);
    }
    if (min_y > max_y)
    {
        swap(min_y, max_y);
    }
    Rect2d rect = Rect(Point(min_x, min_y), Point(max_x, max_y));
    Rect2d _bounding = Rect(Point(0, 0), Point(m_src.cols, m_src.rows));
    rect &= _bounding;

    //截取数字识别部分图片
    Mat front;
    if (useNumber)
    {
        if (rect.area() > m_param.m_armor_min_area)
        {
            Mat tmp = m_src(rect);
            resize(tmp, front, Size(48, 32));
            //转为灰度图
            cvtColor(front, front, COLOR_RGB2GRAY);
            //转换尺寸
            Rect front_roi(Point(20, 0), Size(10, 32));
            Mat front_roi_img = front(front_roi);
            //加强亮度，便于识别
            double min = 0, max = 0;
            minMaxLoc(front_roi_img, &min, &max);
            front = front * (255.0 / max);
            //识别数字
            armor.m_id = m_svm.test(front);
            // if (m_endebug)
            // {
            //     m_numbers.push_back(front.clone());
            // }
        }
        else
        {
            armor.m_id = 0;
        }
    }
    else
    {
        armor.m_id = 10;
    }

    return armor.m_id;
}

//TODO:change to your dir
bool ArmorDetector::initModel(string path)
{
    // ie.SetConfig({{CONFIG_KEY(CACHE_DIR), "../.cache"}});
    // // ie.SetConfig({{CONFIG_KEY(GPU_THROUGHPUT_STREAMS),"GPU_THROUGHPUT_AUTO"}});
    // ie.SetConfig({{CONFIG_KEY(GPU_THROUGHPUT_STREAMS),"1"}});
    // // Step 1. Read a model in OpenVINO Intermediate Representation (.xml and
    // // .bin files) or ONNX (.onnx file) format
    // network = ie.ReadNetwork(path);
    // if (network.getOutputsInfo().size() != 1)
    //     throw std::logic_error("Sample supports topologies with 1 output only");

    // // Step 2. Configure input & output
    // //  Prepare input blobs
    // InputInfo::Ptr input_info = network.getInputsInfo().begin()->second;
    // input_name = network.getInputsInfo().begin()->first;


    // //  Prepare output blobs
    // if (network.getOutputsInfo().empty())
    // {
    //     std::cerr << "Network outputs info is empty" << std::endl;
    //     return EXIT_FAILURE;
    // }
    // DataPtr output_info = network.getOutputsInfo().begin()->second;
    // output_name = network.getOutputsInfo().begin()->first;

    // // output_info->setPrecision(Precision::FP16);
    // // Step 3. Loading a model to the device
    // // executable_network = ie.LoadNetwork(network, "MULTI:GPU");
    // executable_network = ie.LoadNetwork(network, "GPU");
    // // executable_network = ie.LoadNetwork(network, "CPU");

    // // Step 4. Create an infer request
    // infer_request = executable_network.CreateInferRequest();
    // const Blob::Ptr output_blob = infer_request.GetBlob(output_name);
    // moutput = as<MemoryBlob>(output_blob);
    // // Blob::Ptr input = infer_request.GetBlob(input_name);     // just wrap Mat data by Blob::Ptr
    // if (!moutput)
    // {
    //     throw std::logic_error("We expect output to be inherited from MemoryBlob, "
    //                             "but by fact we were not able to cast output to MemoryBlob");
    // }
    // // locked memory holder should be alive all time while access to its buffer
    // // happens
    // return true;
}

bool ArmorDetector::detect(Mat &src, vector<ArmorObject> &objects)
{
    if (src.empty())
        {
            fmt::print(fmt::fg(fmt::color::red), "[DETECT] ERROR: 传入了空的src\n");
    #ifdef SAVE_AUTOAIM_LOG
            LOG(ERROR) << "[DETECT] ERROR: 传入了空的src";
    #endif // SAVE_AUTOAIM_LOG
            return false;
        }
    cv::Mat pr_img = scaledResize(src,transfrom_matrix);  //设置一个图片尺寸，将该图片填入该框里，其余地方用黑色填充

#ifdef SHOW_INPUT
    namedWindow("network_input",0);
    imshow("network_input",pr_img); 
    waitKey(1);
#endif //SHOW_INPUT

    cv::Mat pre;
    cv::Mat pre_split[3];
    pr_img.convertTo(pre,CV_32F);
    cv::split(pre,pre_split);

    // Blob::Ptr imgBlob = infer_request.GetBlob(input_name);     // just wrap Mat data by Blob::Ptr
    // InferenceEngine::MemoryBlob::Ptr mblob = InferenceEngine::as<InferenceEngine::MemoryBlob>(imgBlob);
    // // locked memory holder should be alive all time while access to its buffer happens
    // auto mblobHolder = mblob->wmap();
    // float *blob_data = mblobHolder.as<float *>();

    // auto img_offset = INPUT_W * INPUT_H;
    // //Copy img into blob
    // for(int c = 0;c < 3;c++)
    // {
    //     memcpy(blob_data, pre_split[c].data, INPUT_W * INPUT_H * sizeof(float));
    //     blob_data += img_offset;
    // }

    // // auto t1 = std::chrono::steady_clock::now();
    // infer_request.Infer();
    // // auto t2 = std::chrono::steady_clock::now();
    // // cout<<(float)(std::chrono::duration<double,std::milli>(t2 - t1).count())<<endl;
    // // infer_request.GetPerformanceCounts();
    // // -----------------------------------------------------------------------------------------------------
    // // --------------------------- Step 8. Process output----------------
    // // const Blob::Ptr output_blob = infer_request.GetBlob(output_name);
    // // MemoryBlob::CPtr moutput = as<MemoryBlob>(output_blob);

    // auto moutputHolder = moutput->rmap();
    // const float* net_pred = moutputHolder.as<const PrecisionTrait<Precision::FP32>::value_type*>();
    int img_w = src.cols;
    int img_h = src.rows;

    // decodeOutputs(net_pred, objects, transfrom_matrix, img_w, img_h);
    

    //init初始化
    m_armors.clear();
    m_lights.clear();

    Mat m_binary_gray;//灰度图片
    Mat m_show_light = src.clone();//灰度图片
    Mat m_show_armor = src.clone();//灰度图片
    /* 图像预处理 */
    img_process(src,m_binary_gray);
    /* 寻找灯条 */
    findLights(m_binary_gray);


    for (auto light_draw : m_lights)
    {
        cv::rectangle(m_show_light, light_draw.m_rectR, cv::Scalar(0, 255, 0), 2); // 绿色矩形，宽度为2
        fmt::print(fmt::fg(fmt::color::green), "light_draw.m_color:{}\n", light_draw.m_color);
    }
    namedWindow("Image with Rectangles",0);
    imshow("Image with Rectangles", m_show_light);
    waitKey(1);

    /*寻找装甲板*/
    findArmors(m_show_armor);

    for (auto object = objects.begin(); object != objects.end(); ++object)
    {
        //对候选框预测角点进行平均,降低误差
        if ((*object).pts.size() >= 8)
        {
            auto N = (*object).pts.size();
            cv::Point2f pts_final[4];

            for (int i = 0; i < N; i++)
            {
                pts_final[i % 4]+=(*object).pts[i];
            }

            for (int i = 0; i < 4; i++)
            {
                pts_final[i].x = pts_final[i].x / (N / 4);
                pts_final[i].y = pts_final[i].y / (N / 4);
            }

            (*object).apex[0] = pts_final[0];
            (*object).apex[1] = pts_final[1];
            (*object).apex[2] = pts_final[2];
            (*object).apex[3] = pts_final[3];
        }
        (*object).area = (int)(calcTetragonArea((*object).apex));
    }
    if (objects.size() != 0)
        return true;
    else return false;
}


