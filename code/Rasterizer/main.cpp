#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

// 移动矩阵，将物体移动到摄像头前
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    // 在定义该矩阵变量时，创建一个同尺寸同数据类型的单位阵，对其初始化。
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

// Create the model matrix for rotating the triangle around the Z axis.
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f rotation;
    float theta = (rotation_angle / 180.0) * MY_PI;//角度转弧度
    rotation << cos(theta), -sin(theta),0,0,
            sin(theta), cos(theta),0,0,
            0,0,1,0,
            0,0,0,1;

    model = rotation * model;
    return model;
}

// 投影矩阵
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    //Eigen::Matrix4f::Identity() 初始化为单位矩阵
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    //透视图，近大远小，是个视锥  此矩阵是一个公式
    Eigen::Matrix4f P2O = Eigen::Matrix4f::Identity();//将透视矩阵挤压成正交矩阵
    P2O << zNear, 0, 0, 0,
            0, zNear, 0, 0,
            0, 0, zNear + zFar, - zFar* zNear,
            0, 0, 1.0, 0;

    float halfEyeAngelRadian = (eye_fov / 2.0 / 180.0) * MY_PI; //视角的一半
    float y_top = -zNear * std::tan(halfEyeAngelRadian);//y轴正方向值 = 显示视口的一半高度 zNear是负值！
    float x_left = -y_top * aspect_ratio;//x轴负方向值 = 显示视口的一半宽度
    float y_down = -y_top;
    float x_right = -x_left;

    //构造缩放矩阵，使视口大小等同窗口大小
    Eigen::Matrix4f scaleMat = Eigen::Matrix4f::Identity();
    scaleMat << 2 / (x_right - x_left), 0, 0, 0,			//将中心视为原点，则窗口的三维方向值域均为[-1,1]
            0, 2 / (y_top - y_down), 0, 0,				//缩放的倍数为 期望值/当前值
            0, 0, 2 / (zNear - zFar), 0,				//所以缩放的倍数为 (1+1)/某一维度的当前值
            0, 0, 0, 1;

    //构造平移矩阵，将视口左下角移动到原点
    Eigen::Matrix4f translateMat = Eigen::Matrix4f::Identity();

    //左下角的点原本为 （x_left,y_down，zNear）
    //注意！此时已经经过了缩放，所以左下角的点的位置已经变化
    //左下角的点现在为 （-1，-1，zNear）
    //即其实可以不用管x和y轴，比较尺寸已经和窗口匹配了
    //但网上其他人却还是右下方注释的那样写的，左侧+右侧或者上侧+下侧，结果不都是0么？
    translateMat << 1, 0, 0, 0,					//-(x_left+x_right)/2
            0, 1, 0, 0,					//-(y_top+y_down)/2
            0, 0, 1, -(zNear+zFar)/2,
            0, 0, 0, 1;
    //注意！此处矩阵相乘，右结合率，必须先压缩矩阵，再缩放，最后平移，顺序一定不可以改！
    projection = translateMat * scaleMat * P2O;

    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
