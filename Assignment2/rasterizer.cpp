// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
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

int orient(Eigen::Vector2f a, Eigen::Vector2f b, Eigen::Vector2f c)
{
    Eigen::Vector2f ab = b - a;
    Eigen::Vector2f ac = c - a;
    float crossProduct = ab.x() * ac.y() - ab.y() * ac.x();
    if (crossProduct > 0)
        return 1;
    else if (crossProduct < 0)
        return -1;
    else
        return 0;
}

static bool insideTriangle(int x, int y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Eigen::Vector2f p = Eigen::Vector2f(x, y);

    Eigen::Vector2f v0 = Eigen::Vector2f(_v[0].x(), _v[0].y());
    Eigen::Vector2f v1 = Eigen::Vector2f(_v[1].x(), _v[1].y());
    Eigen::Vector2f v2 = Eigen::Vector2f(_v[2].x(), _v[2].y());

    int o1 = orient(v0, v1, p);
    int o2 = orient(v1, v2, p);
    int o3 = orient(v2, v0, p);
    int res = (o1 + o2 + o3);
    return (res == 3 || res == -3);

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

    if(ssaa_level > 1){
        set_ssaa_pixels();
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
    float minX = std::min({t.v[0].x(), t.v[1].x(), t.v[2].x()});
    float minY = std::min({t.v[0].y(), t.v[1].y(), t.v[2].y()});
    float maxX = std::max({t.v[0].x(), t.v[1].x(), t.v[2].x()});
    float maxY = std::max({t.v[0].y(), t.v[1].y(), t.v[2].y()});
    float step = 1.0 / ssaa_level;
    float offset = 0.5 * step;

    for (int x = minX; x <= maxX; x++) {
        for (int y = minY; y <= maxY; y++) {
            if (ssaa_level > 1){
                int index = get_index(x, y);
                for (float i = offset; i < 1; i+=step) {
                    for (float j = offset; j < 1; j+=step) {
                        // std::cout << "iss: " << i << " jss: " << j << std::endl;
                        float x_ssaa = x + i;
                        float y_ssaa = y + j;
                        if (insideTriangle(x_ssaa + offset, y_ssaa + offset, t.v)) {
                            auto[alpha, beta, gamma] = computeBarycentric2D(x_ssaa, y_ssaa, t.v);
                            float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                            float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                            z_interpolated *= w_reciprocal;
                            if (z_interpolated < ssaa_depth_buf[index][i * ssaa_level + j]) {
                                ssaa_depth_buf[index][i * ssaa_level + j] = z_interpolated;
                                ssaa_frame_buf[index][i * ssaa_level + j] = t.getColor();
                            }
                        }
                    }
                }                
            }
            else if (insideTriangle(x, y, t.v)) {
                // If so, use the following code to get the interpolated z value.
                auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;
                int index = get_index(x, y);
                if (z_interpolated < depth_buf[index]) {
                    depth_buf[index] = z_interpolated;
                    set_pixel(Eigen::Vector3f(x, y, 1), t.getColor());
                }
            }
        }
    }


    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
}



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

void rst::rasterizer::set_ssaa_level(int level)
{
    ssaa_level = level;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        for (int i = 0; i < ssaa_frame_buf.size(); i++){
            ssaa_frame_buf[i].resize(ssaa_level * ssaa_level);
            std::fill(ssaa_frame_buf[i].begin(), ssaa_frame_buf[i].end(), Eigen::Vector3f{0, 0, 0});
        }
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        for (int i = 0; i < ssaa_depth_buf.size(); i++){
            ssaa_depth_buf[i].resize(ssaa_level * ssaa_level);
            std::fill(ssaa_depth_buf[i].begin(), ssaa_depth_buf[i].end(), std::numeric_limits<float>::infinity());
        }
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    ssaa_frame_buf.resize(w * h);
    ssaa_depth_buf.resize(w * h);
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

void rst::rasterizer::set_ssaa_pixels(){
    for (int i = 0; i < width; i++){
        for (int j = 0; j < height; j++){
            Eigen::Vector3f color(0.0f, 0.0f, 0.0f);
            for (int k = 0; k < ssaa_level; k++){
                for (int l = 0; l < ssaa_level; l++){
                    // std::cout << "k: " << k << " l: " << l << std::endl;
                    color += ssaa_frame_buf[get_index(i, j)][ssaa_level * k + l];
                }
            }
            color /= (ssaa_level * ssaa_level);
            set_pixel(Eigen::Vector3f(i, j, 1), color);
        }
    }
}

// clang-format on