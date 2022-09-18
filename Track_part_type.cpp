#include <random>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <iostream>

#include "Track_part_type.h"
#define GCONST  9.80665

std::string State_type::get_csv_data(std::string sep) {
    std::string result = "";
    result += std::to_string(vel.x) + sep + std::to_string(vel.y) + sep + std::to_string(vel.z) + sep;
    result += std::to_string(accel.x) + sep + std::to_string(accel.y) + sep + std::to_string(accel.z) + sep;
    result += std::to_string(orient.x) + sep + std::to_string(orient.y) + sep + std::to_string(orient.z) + sep;
    result += std::to_string(anqular_vel.x) + sep + std::to_string(anqular_vel.y) + sep + std::to_string(anqular_vel.z) + sep;
    result += std::to_string(anqular_accel.x) + sep + std::to_string(anqular_accel.y) + sep + std::to_string(anqular_accel.z);
    return result;
};

void State_type::read_csv(std::string line, std::string sep) {
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
        item.replace(item.find(","), 1, ".");
        double_buffer.push_back(std::stod(item));
        //cout << "_" << stod(item) << "_" << endl;
    }

    if (double_buffer.size() != 15) {
        std::cout << "State_type csv init wrong size" << std::to_string(double_buffer.size()) << std::endl;
        return;
    };

    this->change_vel(cv::Point3d(double_buffer[0], double_buffer[1], double_buffer[2]));
    this->change_accel(cv::Point3d(double_buffer[3], double_buffer[4], double_buffer[5]));
    this->change_orient(cv::Point3d(double_buffer[6], double_buffer[7], double_buffer[8]));
    this->change_anqular_vel(cv::Point3d(double_buffer[9], double_buffer[10], double_buffer[11]));
    this->change_anqular_accel(cv::Point3d(double_buffer[12], double_buffer[13], double_buffer[14]));
};

State_type Corner_type::orientation(double dist = 0) {
    State_type res;
    // 
    double omega = this->angle / this->time;
    cv::Point3d cent_radius = cv::Point3d(this->part(dist).x, this->part(dist).y, 0) - cv::Point3d(this->center.x, this->center.y, 0);
    cv::Point2d cent_radius2d = cv::Point2d(this->part(dist).x, this->part(dist).y) - cv::Point2d(this->center.x, this->center.y);

    res.change_accel(cv::Point3d(0, 0, GCONST) - cent_radius * omega * omega);

    res.change_anqular_vel(cv::Point3d(0, 0, omega));
    double sing = 1;
    if (this->angle < 0) sing = -1;
    cv::Point2d delta = rotate2d(cent_radius2d, sing * M_PI / 2);
    cv::Point3d n_delta = normalize(cv::Point3d(delta.x, delta.y, 0));

    res.change_vel(n_delta * (this->len() / this->time));
    res.change_orient(toAngle3d(n_delta));
    return res;
};

State_type Line_track_type::orientation(double dist = 0) {
    State_type res;
    // 
    res.change_accel(cv::Point3d(0, 0, GCONST));
    cv::Point2d delta = this->end - this->start;
    cv::Point3d n_delta = normalize(cv::Point3d(delta.x, delta.y, 0));

    res.change_orient(toAngle3d(n_delta));
    res.change_vel(cv::Point3d(delta.x, delta.y, 0) / this->time);


    return res;
};

cv::Point2d Track_part_type::part(double dist) {
    dist -= line.len();
    if (dist == 0) return this->line.end;
    else if (dist < 0) {
        return this->line.part(dist + line.len());
    }
    else if (dist > 0) {
        return this->turn.part(dist);
    };
};

State_type Track_part_type::orientation(double dist) {
    dist -= line.len();
    if (dist == 0) return cv::Point3d(this->line.end.x, this->line.end.y, 0);
    else if (dist < 0) {
        return this->line.orientation(dist + line.len());
    }
    else if (dist > 0) {
        return this->turn.orientation(dist);
    };
};

double Line_track_type::len() {
    return length(start - end);
};

cv::Point2d Line_track_type::part(double dist) {
    double ratio = dist / this->len();
    return start + ((end - start) * ratio);
};

cv::Point2d Corner_type::part(double dist) {
    return this->center + get_arc_end_point(this->start, this->center, this->angle * (dist / this->len()));
};

double Corner_type::len() {
    return length(this->center - this->start) * this->angle;
};

Corner_type::Corner_type(
    cv::Point2d s,
    cv::Point2d e,
    cv::Point2d c,
    double angl,
    double _time,
    cv::Point2d _start_vec
) : start(s), end(e), center(c), angle(angl), time(_time), start_vec(_start_vec) {};

void State_type::change(cv::Point3d _arg, int _arg_num) {
    switch (_arg_num) {
    case 0:
        this->vel = _arg;
        break;
    case 1:
        this->accel = _arg;
        break;
    case 2:
        this->orient = _arg;
        break;
    case 3:
        this->anqular_vel = _arg;
        break;
    case 4:
        this->anqular_accel = _arg;
        break;
    };
};

State_type::State_type(
    cv::Point3d _arg,
    int _arg_num
) {
    switch (_arg_num) {
    case 0:
        this->vel = _arg;
        this->accel = zeroPoint3d;
        this->orient = zeroPoint3d;
        this->anqular_vel = zeroPoint3d;
        this->anqular_accel = zeroPoint3d;
        break;
    case 1:
        this->vel = zeroPoint3d;
        this->accel = _arg;
        this->orient = zeroPoint3d;
        this->anqular_vel = zeroPoint3d;
        this->anqular_accel = zeroPoint3d;
        break;
    case 2:
        this->vel = zeroPoint3d;
        this->accel = zeroPoint3d;
        this->orient = _arg;
        this->anqular_vel = zeroPoint3d;
        this->anqular_accel = zeroPoint3d;
        break;
    case 3:
        this->vel = zeroPoint3d;
        this->accel = zeroPoint3d;
        this->orient = zeroPoint3d;
        this->anqular_vel = _arg;
        this->anqular_accel = zeroPoint3d;
        break;
    case 4:
        this->vel = zeroPoint3d;
        this->accel = zeroPoint3d;
        this->orient = zeroPoint3d;
        this->anqular_vel = zeroPoint3d;
        this->anqular_accel = _arg;
        break;
    };
};

double Track_part_type::len() {
    return line.len() + turn.len();
};


Track_part_type::Track_part_type(
    cv::Point2d start,
    cv::Point2d start_p_vec,
    double min_line_length,
    double max_line_length,
    double min_corner_radius,
    double max_corner_radius,
    double min_corner_angle,
    double max_corner_angle,
    double min_vel,
    double max_vel
) {
    std::random_device rd;
    std::default_random_engine generator(rd());
    std::uniform_real_distribution<double> distribution_line(min_line_length, max_line_length);
    std::uniform_real_distribution<double> distribution_radius(min_corner_radius, max_corner_radius);
    std::uniform_real_distribution<double> distribution_vel(min_vel, max_vel);

    std::uniform_real_distribution<double> distribution_angle(min_corner_angle, max_corner_angle);
   

    double min_radius = 10;
    double max_radius = 500;
    double min_length = 5;
    double max_length = 500;
    double min_angle = 30;

    double vel = distribution_vel(generator);
    //while (vel <= 0) vel = distribution_vel(generator);

    double line_len = distribution_line(generator);
    //while (line_len < 0 || line_len > max_length) line_len = distribution_line(generator);
    double line_time = line_len / vel;
    this->line = Line_track_type(start, cv::Point2d(start.x + start_p_vec.x * line_len, start.y + start_p_vec.y * line_len), line_time);

    double radius = distribution_radius(generator);
    //while (radius < 0) radius = distribution_radius(generator);

    double angle = distribution_angle(generator);
    angle = fmod(angle, 2 * M_PI);
    //angle = abs(angle) > 0 ? angle : min_angle;
    cv::Point2d center = this->line.end + get_norm_vect(this->line.end, this->line.start) * radius;
    cv::Point2d end = center + get_arc_end_point(this->line.end, center, angle);// not sure

    double corner_time = (length(center - this->line.end) * angle) / vel;
    this->turn = Corner_type(this->line.end, end, center, angle, corner_time, start_p_vec);
    this->exit_vec = get_norm_vect(this->turn.end, this->turn.center);
    this->end = this->turn.end;
};

inline cv::Point3d normalize(cv::Point3d vec) {
    double length = sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
    if (length == 0) return vec;
    return vec / length;
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
    double rot_arr[2][2] = {
        {cos(angle), -sin(angle)},
        {sin(angle), cos(angle)}
    };
    cv::Mat rot = cv::Mat(2, 2, CV_64F, rot_arr);
    double radius_vec_arr[2][1] = {
        {vec.x},
        {vec.y}
    };
    cv::Mat radius_vec = cv::Mat(2, 1, CV_64F, radius_vec_arr);
    cv::Mat end_vec = rot * radius_vec;
    return cv::Point2d(end_vec.at<double>(0, 0), end_vec.at<double>(1, 0));
};

double length(cv::Point2d vec) {
    return sqrt(vec.x * vec.x + vec.y * vec.y);
};

cv::Point2d get_point_vect(cv::Point2d end, cv::Point2d start) {
    return cv::Point2d((end.x - start.x) / length(end - start), (end.y - start.y) / length(end - start));
};

cv::Point2d get_norm_vect(cv::Point2d end, cv::Point2d start) {
    return rotate2d(end - start, M_PI / 2) / length(end - start);
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