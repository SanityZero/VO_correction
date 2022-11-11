#pragma once
#include "Types.h" 
#define M_PI (double)3.14159265358979323846
#define zeroPoint3d cv::Point3d(0,0,0)

using namespace std;
using namespace cv;


/////////////////////////////////////////////////////
inline Point3d rotateP3d(Point3d point, Point3d ang) {//rad
    Point3d res;
    Point3d tmp1;
    Point3d tmp2;
    double x = point.x, y = point.y, z = point.z;
    double a = ang.x, b = ang.y, c = ang.z;

    tmp1.x = x * cos(c) + y * sin(c);
    tmp1.y = y * cos(c) - x * sin(c);
    tmp1.z = z;

    x = tmp1.x, y = tmp1.y, z = tmp1.z;

    tmp2.x = x * cos(b) - z * sin(b);
    tmp2.y = y;
    tmp2.z = x * sin(b) + z * cos(b);

    x = tmp2.x, y = tmp2.y, z = tmp2.z;

    res.x = x;
    res.y = y * cos(a) + z * sin(a);
    res.z = z * cos(a) - y * sin(a);

    return res;
};

/////////////////////////////////////////////////////
inline Mat mat_multi(Mat left, Mat rigth) {
    Mat res = left.clone();

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
        {
            res.at<double>(i, j) = left.at<double>(i, 0) * rigth.at<double>(0, j) + left.at<double>(i, 1) * rigth.at<double>(1, j) + left.at<double>(i, 2) * rigth.at<double>(2, j);
        };
    return res;
};

/////////////////////////////////////////////////////
inline Mat mat_add(Mat left, Mat rigth, double a = 1.0, double b = 1.0) {
    Mat res = left.clone();

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
        {
            res.at<double>(i, j) = a * left.at<double>(i, j) + b * rigth.at<double>(i, j);
        };
    return res;
};

/////////////////////////////////////////////////////
Point3d SpheToDec(Point2d a, double radius = 1.0);
double absVec(Point3d vec);
double angle2V(Point3d a, Point3d b);
Point3d poseDelta(Pose_type a, Pose_type b);
Point3d angDelta(Pose_type a, Pose_type b);


double min(double a, double b, double c);
double max(double a, double b, double c);
cv::Point2d rotate2d(cv::Point2d vec, double angle);
double length(cv::Point2d vec);
double length(cv::Point3d vec);
cv::Point2d get_point_vect(cv::Point2d end, cv::Point2d start = cv::Point2d(0, 0));
cv::Point2d get_norm_vect(cv::Point2d end, cv::Point2d start = cv::Point2d(0, 0));
cv::Point2d get_arc_end_point(cv::Point2d cent, cv::Point2d start, double angle);
cv::Point3d toAngle3d(cv::Point3d vec);


inline cv::Point3d normalize(cv::Point3d vec) {
    double length = sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
    if (length == 0) return vec;
    return vec / length;
};

inline cv::Point2d normalize(cv::Point2d vec) {
    double length = sqrt(vec.x * vec.x + vec.y * vec.y);
    if (length == 0) return vec;
    return vec / length;
};

double min(double a, double b, double c);

double max(double a, double b, double c);

cv::Point2d rotate2d(cv::Point2d vec, double angle);

double length(cv::Point2d vec);

cv::Point2d get_point_vect(cv::Point2d end, cv::Point2d start);

cv::Point2d get_norm_vect(cv::Point2d end, cv::Point2d start);

cv::Point2d get_arc_end_point(cv::Point2d cent, cv::Point2d start, double angle);

cv::Point3d toAngle3d(cv::Point3d vec);

cv::Point3d Angle3dtoPA(cv::Point3d vec);