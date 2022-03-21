#pragma once
#define M_PI (double)3.14159265358979323846

typedef struct {
    double lat;
    double lon;
    double alt;

    double roll;
    double pitch;
    double yaw;

    double ax;
    double ay;
    double az;

    double wx;
    double wy;
    double wz;
} Pose_type;

typedef struct {
    double lat;
    double lon;
    double alt;

    double roll;
    double pitch;
    double yaw;
} Pose_T;

typedef struct {
    Mat mat;
    double timestamp;
    Pose_type pose;
} Source_Type;

typedef struct {
    vector<Point3d> w;
    vector<double> timestamp;
    vector<Point3d> angle;
    vector<Point3d> accel;
    vector<Point3d> pose;
} DataSeq_model_Type;
