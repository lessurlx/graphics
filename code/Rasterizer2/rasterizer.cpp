// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include <Eigen/Eigen>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(int x, int y, const Vector3f* _v)
{
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    // 准备三角形各边的的向量
    Eigen::Vector2f side1;
    side1 << _v[1].x() - _v[0].x(), _v[1].y() - _v[0].y();
    Eigen::Vector2f side2;
    side2 << _v[2].x() - _v[1].x(), _v[2].y() - _v[1].y();
    Eigen::Vector2f side3;
    side3 << _v[0].x() - _v[2].x(), _v[0].y() - _v[2].y();

    // 准备测量点和三角形各点连线的向量
    Eigen::Vector2f v1;
    v1 << x - _v[0].x(), y - _v[0].y();
    Eigen::Vector2f v2;
    v2 << x - _v[1].x(), y - _v[1].y();
    Eigen::Vector2f v3;
    v3 << x - _v[2].x(), y - _v[2].y();

    // 三角形各边的的向量叉乘测量点和三角形各点连线的向量
    float z1 = side1.x() * v1.y() - side1.y() * v1.x();
    float z2 = side2.x() * v2.y() - side2.y() * v2.x();
    float z3 = side3.x() * v3.y() - side3.y() * v3.x();

    // 判断叉乘结果是否有相同的符号
    if ((z1 > 0 && z2 > 0 && z3 > 0) || (z1 < 0 && z2 < 0 && z3 < 0))
    {
        return true;
    }
    else
    {
        return false;
    }
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wc++17-extensions"
//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    // 可以理解为返回齐次坐标
    auto v = t.toVector4();
    
    // Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
    // 创建包围盒，检查中心点是否在三角形内
    float max_x = t.v[0].x() > t.v[1].x() ? t.v[0].x() : t.v[1].x();
    max_x = max_x > t.v[2].x() ? max_x : t.v[2].x();
    float min_x = t.v[0].x() < t.v[1].x() ? t.v[0].x() : t.v[1].x();
    min_x = min_x < t.v[2].x() ? min_x : t.v[2].x();
    float max_y = t.v[0].y() > t.v[1].y() ? t.v[0].y() : t.v[1].y();
    max_y = max_y > t.v[2].y() ? max_y : t.v[2].y();
    float min_y = t.v[0].y() < t.v[1].y() ? t.v[0].y() : t.v[1].y();
    min_y = min_y < t.v[2].y() ? min_y : t.v[2].y();

    for (int x = min_x; x < max_x; ++x) {
        for (int y = min_y; y < max_y; ++y) {
            // 不在三角形内就跳过这个点
            if(!insideTriangle(x,y,t.v)){
                continue;
            }
            // If so, use the following code to get the interpolated z value.
            auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
            float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
            float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
            z_interpolated *= w_reciprocal;
            // 该点的深度比缓冲区的还大，跳过该点
            if(z_interpolated > depth_buf[get_index(x,y)]){
                continue;
            }
            //更新深度
            depth_buf[get_index(x, y)] = z_interpolated;
            // set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
            Vector3f point;
            point << x, y, z_interpolated;
            set_pixel(point, t.getColor());
        }
    }
}
#pragma clang diagnostic pop

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on