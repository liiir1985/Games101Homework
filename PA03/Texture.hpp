//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;
    Eigen::Vector3f lerp(const Eigen::Vector3f& a, const Eigen::Vector3f& b, float t)
    {
        return a + t * (b - a);
    }
    Eigen::Vector3f lerp(const cv::Vec3b& a, const cv::Vec3b& b, float t)
    {
        Eigen::Vector3f a2(a[0], a[1], a[2]);
        Eigen::Vector3f b2(b[0], b[1], b[2]);
        return a2 + t * (b2 - a2);
    }

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        u = clamp01(u);
        v = clamp01(v);
        
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto u0 = floorf(u_img);
        auto u1 = u0 + 1;
        auto v0 = floorf(v_img);
        auto v1 = v0 + 1;

        auto cuv0 = lerp(image_data.at<cv::Vec3b>(v0,u0), image_data.at<cv::Vec3b>(v0,u1), u_img - u0);
        auto cuv1 = lerp(image_data.at<cv::Vec3b>(v1,u0), image_data.at<cv::Vec3b>(v1,u1), u_img - u0);
        auto cFinal = lerp(cuv0, cuv1, v_img - v0);
        return cFinal;
        //auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        //return Eigen::Vector3f(color[0], color[1], color[2]);
    }

};
#endif //RASTERIZER_TEXTURE_H
