#pragma once
#define M_PI (double)3.14159265358979323846
#define MASS (double)2000
#define Pose_type_HEADER(sep) string("\"lat\"") + sep + "\"lon\"" + sep + "\"alt\"" + sep + "\"roll\"" + sep + "\"pitch\"" + sep + "\"yaw\"" + sep + "\"ax\"" + sep + "\"ay\"" + sep + "\"az\"" + sep + "\"wx\"" + sep + "\"wy\"" + sep + "\"wz\""

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

    void read_csv(std::string line, std::string sep = ";") {
        size_t pos = 0;
        std::vector<std::string> values;
        while ((pos = line.find(sep)) != std::string::npos) {
            values.push_back(line.substr(0, pos));
            //std::cout << values << std::endl;
            line.erase(0, pos + sep.length());
        };
        values.push_back(line);
        std::vector<double> double_buffer;
        for (std::string item : values) {
            if (item.find(",") != std::string::npos) {
                item.replace(item.find(","), 1, ".");
                double_buffer.push_back(std::stod(item));
            };
            //cout << "_" << stod(item) << "_" << endl;
        }

        if (double_buffer.size() != 12) {
            std::cout << "Pose_type csv init wrong size:\t" << std::to_string(double_buffer.size()) << std::endl;
            return;
        };

        this->setPose(cv::Point3d(double_buffer[0], double_buffer[1], double_buffer[2]));
        this->setOrient(cv::Point3d(double_buffer[3], double_buffer[4], double_buffer[5]));
        this->setAccel(cv::Point3d(double_buffer[6], double_buffer[7], double_buffer[8]));
        this->setW(cv::Point3d(double_buffer[9], double_buffer[10], double_buffer[11]));
    };

    std::string get_csv_data(std::string sep = ",") {
        std::string result = "";
        result += std::to_string(lat) + sep + std::to_string(lon) + sep + std::to_string(alt) + sep;
        result += std::to_string(roll) + sep + std::to_string(pitch) + sep + std::to_string(yaw) + sep;
        result += std::to_string(ax) + sep + std::to_string(ay) + sep + std::to_string(az) + sep;
        result += std::to_string(wx) + sep + std::to_string(wy) + sep + std::to_string(wz);
        return result;
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

    void setPose(cv::Point2d vec) {
        this->lon = vec.x;
        this->lat = vec.y;
        this->alt = 0;
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
    std::vector<cv::Point3d> w;
    std::vector<double> timestamp;
    std::vector<cv::Point3d> angle;
    std::vector<cv::Point3d> accel;
    std::vector<cv::Point3d> pose;
} DataSeq_model_Type;
