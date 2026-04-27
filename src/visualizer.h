#pragma once

#include <cassert>
#include <string>
#include <filesystem>
#include <format>

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include "agent.h"

namespace fs = std::filesystem;

class Visualizer {
public:
    Visualizer(int img_width, int img_height, double width, double height, std::string directory)
        : img_width(img_width), img_height(img_height), width(width), height(height), directory(directory), img(img_width, img_height, CV_8UC3, cv::Scalar(255, 255, 255)) {
        assert(fs::exists(directory));
        assert(fs::is_directory(directory));
    }

    void ClearFrame() {
        img = cv::Scalar(255, 255, 255);
    }

    void DrawAgent(const Agent& agent) {

    }

    void SaveFrame(int frame) {
        cv::imwrite(img, directory / std::format("frame_{:04}.png", frame));
    }

protected:
    int img_width;
    int img_height;
    double width;
    double height;
    fs::path directory;
    cv::Mat img;
};