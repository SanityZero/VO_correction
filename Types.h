#pragma once
#define M_PI (double)3.14159265358979323846
#define MASS (double)2000

class Pose_type{
public:
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

    Pose_type() {};
    Pose_type(cv::Point3d pose_vec, cv::Point3d orient_vec, cv::Point3d accel_vec, cv::Point3d W_vec) {
        this->setPose(pose_vec);
        this->setOrient(orient_vec);
        this->setAccel(accel_vec);
        this->setW(W_vec);
    };

    cv::Point3d getPose() {
        return cv::Point3d(lon, lat, alt);
    };

    cv::Point3d getOrient() {
        return cv::Point3d(roll, pitch, yaw);
    };

    cv::Point3d getAccel() {
        return cv::Point3d(ax, ay, az);
    };

    cv::Point3d getW() {
        return cv::Point3d(wx, wy, wz);
    };

    void setPose(cv::Point3d vec) {
        this->lon = vec.x;
        this->lat = vec.y;
        this->alt = vec.z;
    };

    void setOrient(cv::Point3d vec) {
        this->roll = vec.x;
        this->pitch = vec.y;
        this->yaw = vec.z;
    };

    void setAccel(cv::Point3d vec) {
        this->ax = vec.x;
        this->ay = vec.y;
        this->az = vec.z;
    };

    void setW(cv::Point3d vec) {
        this->wx = vec.x;
        this->wy = vec.y;
        this->wz = vec.z;
    };
};

typedef struct {
    double lat;
    double lon;
    double alt;

    double roll;
    double pitch;
    double yaw;
} Pose_T;

typedef struct {
    cv::Mat mat;
    double timestamp;
    Pose_type pose;
} Source_Type;

typedef struct {
    vector<cv::Point3d> w;
    vector<double> timestamp;
    vector<cv::Point3d> angle;
    vector<cv::Point3d> accel;
    vector<cv::Point3d> pose;
} DataSeq_model_Type;
