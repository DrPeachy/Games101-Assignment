//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture
{
private:
    cv::Mat image_data;

public:
    Texture(const std::string &name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        Eigen::Vector3f result(0, 0, 0);
        // Map u and v to image coordinates
        float u_img = u * width;
        float v_img = (1 - v) * height;

        // Calculate the coordinates of the top left pixel
        int u1 = std::floor(u_img);
        int v1 = std::floor(v_img);

        // Ensure coordinates are within the image boundaries
        u1 = std::min(u1, width - 2);
        v1 = std::min(v1, height - 2);

        // Calculate the coordinates of the bottom right pixel
        int u2 = u1 + 1;
        int v2 = v1 + 1;

        // Ensure indices are within the valid range
        u1 = std::max(0, std::min(u1, image_data.cols - 1));
        v1 = std::max(0, std::min(v1, image_data.rows - 1));
        u2 = std::max(0, std::min(u2, image_data.cols - 1));
        v2 = std::max(0, std::min(v2, image_data.rows - 1));

        // Calculate the fractional parts of u and v
        float s = u_img - u1;
        float t = v_img - v1;

        // Fetch the colors of the four surrounding pixels
        // std::cout << "u1: " << u1 << " v1: " << v1 << " u2: " << u2 << " v2: " << v2 << std::endl;
        auto color1 = image_data.at<cv::Vec3b>(v1, u1);
        auto color2 = image_data.at<cv::Vec3b>(v1, u2);
        auto color3 = image_data.at<cv::Vec3b>(v2, u1);
        auto color4 = image_data.at<cv::Vec3b>(v2, u2);

        // Perform bilinear interpolation
        result =
            (1 - s) * (1 - t) * Eigen::Vector3f(color1[0], color1[1], color1[2]) +
            s * (1 - t) * Eigen::Vector3f(color2[0], color2[1], color2[2]) +
            (1 - s) * t * Eigen::Vector3f(color3[0], color3[1], color3[2]) +
            s * t * Eigen::Vector3f(color4[0], color4[1], color4[2]);

        return result;
    }
};
#endif // RASTERIZER_TEXTURE_H
