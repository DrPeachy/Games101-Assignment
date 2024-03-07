#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    float sinPart = std::sin(rotation_angle * MY_PI / 180);
    float cosPart = std::cos(rotation_angle * MY_PI / 180);

    model(0, 0) = cosPart;
    model(0, 1) = -sinPart;
    model(1, 0) = sinPart;
    model(1, 1) = cosPart;
    // Then return it.

    return model;
}

// bonus
Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
    Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();
    float sinPart = std::sin(angle * MY_PI / 180);
    float cosPart = std::cos(angle * MY_PI / 180);
    rotation(0, 0) = cosPart + (1 - cosPart) * axis.x() * axis.x();
    rotation(0, 1) = (1 - cosPart) * axis.x() * axis.y() - axis.z() * sinPart;
    rotation(0, 2) = (1 - cosPart) * axis.x() * axis.z() + axis.y() * sinPart;
    rotation(1, 0) = (1 - cosPart) * axis.x() * axis.y() + axis.z() * sinPart;
    rotation(1, 1) = cosPart + (1 - cosPart) * axis.y() * axis.y();
    rotation(1, 2) = (1 - cosPart) * axis.y() * axis.z() - axis.x() * sinPart;
    rotation(2, 0) = (1 - cosPart) * axis.x() * axis.z() - axis.y() * sinPart;
    rotation(2, 1) = (1 - cosPart) * axis.y() * axis.z() + axis.x() * sinPart;
    rotation(2, 2) = cosPart + (1 - cosPart) * axis.z() * axis.z();
    return rotation;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    float top = std::tan(eye_fov / 2 * MY_PI / 180) * zNear;
    float bottom = -top;
    float right = top * aspect_ratio;
    float left = -right;

    Eigen::Matrix4f persp_to_ortho = Eigen::Matrix4f::Identity();
    persp_to_ortho(0, 0) = zNear;
    persp_to_ortho(1, 1) = zNear;
    persp_to_ortho(2, 2) = zNear + zFar;
    persp_to_ortho(2, 3) = -zNear * zFar;
    persp_to_ortho(3, 2) = 1;
    persp_to_ortho(3, 3) = 0;

    Eigen::Matrix4f ortho = Eigen::Matrix4f::Identity();
    ortho(0, 0) = 2 / (right - left);
    ortho(1, 1) = 2 / (top - bottom);
    ortho(2, 2) = 2 / (zNear - zFar);
    ortho(0, 3) = -(right + left) / 2;
    ortho(1, 3) = -(top + bottom) / 2;
    ortho(2, 3) = -(zNear + zFar) / 2;

    projection = ortho * persp_to_ortho;

    // Then return it.

    return projection;
}

int main(int argc, const char **argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3)
    {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4)
        {
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

    if (command_line)
    {
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

    while (key != 27)
    {
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

        if (key == 'a')
        {
            angle += 10;
        }
        else if (key == 'd')
        {
            angle -= 10;
        }
    }

    return 0;
}
