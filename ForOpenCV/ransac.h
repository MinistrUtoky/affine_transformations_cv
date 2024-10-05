#pragma once
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>
#include <random>
#include <ctime>
#include <vector>

static std::mt19937 rng;
static std::uniform_real_distribution<float> udist(0.f, 1.f);
static std::normal_distribution<float> ndist(0.f, 10.f);
const int WIDTH = 640;
const int HEIGHT = 480;

class Ransac
{
public:
    static void init_random();
    static float random_number();
    static float random_noise();
    static std::vector<cv::Point2f> generateData(int N, cv::Point2f p, cv::Point2f dir, float inlier_ratio);
    static cv::Point3f RANSAC(std::vector<cv::Point2f>& points, int max_iterations, float sigma);
private:
    static cv::Point3f fitLine(std::vector<cv::Point2f>& points);
    static cv::Point3f fitLine(cv::Point2f p1, cv::Point2f p2);
    static float distance(cv::Point3f& l, cv::Point2f p);
    static int iterationNumber(float confidence, int sampleSize, int inliers, int N);
};

