#include <opencv2/core.hpp>
#include <cmath>
#include <iostream>

#include "Types.h" 
#define M_PI (double)3.14159265358979323846
#define zeroPoint3d cv::Point3d(0,0,0)
#include "Test_math.h" 

using namespace std;
using namespace cv;

/////////////////////////////////////////////////////
Point3d SpheToDec(Point2d a, double radius) {
    return  Point3d(radius * sin(a.x) * cos(a.y), radius * sin(a.x) * sin(a.y), radius * cos(a.x));
};

/////////////////////////////////////////////////////
double absVec(Point3d vec) {
    return sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
};

/////////////////////////////////////////////////////
Point3d poseDelta(Pose_type a, Pose_type b) {
    return(a.getPose() - b.getPose());
};

/////////////////////////////////////////////////////
double angle2V(Point3d a, Point3d b) { //значения углов в радианах
    return acos((a.x * b.x + a.y * b.y + a.z * b.z) / (absVec(a) * absVec(b)));
};


/////////////////////////////////////////////////////ы
Point3d angDelta(Pose_type a, Pose_type b) {
    return(a.getOrient() - b.getOrient());
};

double min(double a, double b, double c) {
    if ((a <= b) && (a <= c)) return a;
    if ((b <= a) && (b <= c)) return b;
    if ((c <= b) && (c <= a)) return c;
};

double max(double a, double b, double c) {
    if ((a >= b) && (a >= c)) return a;
    if ((b >= a) && (b >= c)) return b;
    if ((c >= b) && (c >= a)) return c;
};

cv::Point2d rotate2d(cv::Point2d vec, double angle) {
    return cv::Point2d(
        vec.x * cos(angle) - vec.y * sin(angle),
        vec.x * sin(angle) + vec.y * cos(angle)
    );
};

double length(cv::Point2d vec) {
    return sqrt(vec.x * vec.x + vec.y * vec.y);
};

double length(cv::Point3d vec) {
    return sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
};

cv::Point2d get_point_vect(cv::Point2d end, cv::Point2d start) {
    return normalize(end - start);
};

cv::Point2d get_norm_vect(cv::Point2d end, cv::Point2d start) {
    cv::Point2d vec = normalize(end - start);
    return rotate2d(vec, M_PI / 2);
};

cv::Point2d get_arc_end_point(cv::Point2d cent, cv::Point2d start, double angle) {
    return rotate2d(cent - start, angle);
};

cv::Point3d toAngle3d(cv::Point3d vec) {
    cv::Point3d vec_x = normalize(cv::Point3d(0, vec.y, vec.z));    //проекция на YOZ
    cv::Point3d vec_y = normalize(cv::Point3d(vec.x, 0, vec.z));   //проекция на XOZ
    cv::Point3d vec_z = normalize(cv::Point3d(vec.x, vec.y, 0));    //проекция на XOY
    vec = normalize(vec);

    cv::Point3d res(0, 0, 0);
    res.x = vec == vec_x ? 0 : acos(vec.dot(vec_x));
    res.y = vec == vec_y ? 0 : acos(vec.dot(vec_y));
    res.z = vec == vec_z ? 0 : acos(vec.dot(vec_z));

    return res;
};

cv::Point3d Angle3dtoPA(cv::Point3d vec) {
    cv::Point3d vec_x = normalize(cv::Point3d(0, vec.y, vec.z));    //проекция на YOZ
    cv::Point3d vec_y = normalize(cv::Point3d(vec.x, 0, vec.z));   //проекция на XOZ
    cv::Point3d vec_z = normalize(cv::Point3d(vec.x, vec.y, 0));    //проекция на XOY
    vec = normalize(vec);

    cv::Point3d res(0, 0, 0);
    res.x = acos(vec.dot(vec_x));
    res.y = acos(vec.dot(vec_y));
    res.z = acos(vec.dot(vec_z));

    return res;
};