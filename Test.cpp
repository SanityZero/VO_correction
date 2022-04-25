#include <iostream>
#include <locale>
#include <iomanip> 
#include <fstream>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
#include <algorithm>
#include <random>
#include <cmath>

using namespace cv;
using namespace std;

#include "Types.h"
#include "ErEstVO.h"
#include "DataSequence.h"
#include "Tests.h"


State_type Corner_type::orientation(double dist) {///ВОТ ЭТО Я ДЕЛАЛ
    State_type res;
    // 
    double omega = this->angle / this->time;
    Point3d cent_radius = Point3d(this->part(dist).x, this->part(dist).y, 0) - Point3d(this->center.x, this->center.y, 0);
    Point2d cent_radius2d = Point2d(this->part(dist).x, this->part(dist).y) - Point2d(this->center.x, this->center.y);

    res.change_accel(Point3d(0, 0, GCONST) + cent_radius*omega*omega);

    res.change_anqular_vel(Point3d(0, 0, omega));
    double sing = 1;
    if (this->angle < 0) sing = -1;
    Point2d delta = rotate2d(cent_radius2d, this->angle * (dist / this->len()));
    Point3d n_delta = normalize(Point3d(delta.x, delta.y, 0));

    res.change_vel(sing * n_delta * (this->len() / this->time) + Point3d(this->start_vec.x, this->start_vec.y, 0));
    res.change_orient(toAngle3d(n_delta));
    return res;
};


State_type Line_track_type::orientation(double dist) {
    State_type res;
    // 
    res.change_accel(Point3d(0, 0, GCONST));
    Point2d delta = this->end - this->start;
    Point3d n_delta = normalize(Point3d(delta.x, delta.y, 0));

    res.change_orient(toAngle3d(n_delta));
    res.change_vel(Point3d(delta.x, delta.y, 0) / this->time);


    return res;
};


inline Point3d normalize(Point3d vec) {
    double length = sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
    return vec / length;
};


inline DataSeq_model_Type generate_model(
    double mean_1,
    const int size,
    double stddev,
    Point3d w,
    Point3d ang_0,
    double deltatime,
    double accel_stddev,
    Point3d accel = Point3d(0, 0, 0),
    Point3d vel_0 = Point3d(0, 0, 0)
)
{
    DataSeq_model_Type res;

    default_random_engine generator_angle;
    normal_distribution<double> distribution_angle(mean_1, stddev);
    res.angle.push_back(ang_0);

    default_random_engine generator_accel;
    normal_distribution<double> distribution_accel(mean_1, accel_stddev);
    res.pose.push_back(Point3d(0, 0, 0));
    vector<Point3d> vel;
    vel.push_back(vel_0);

    for (int i = 1; i < size + 1; i++) {
        res.angle.push_back(Point3d(res.angle[i - 1].x + w.x * deltatime, res.angle[i - 1].y + w.y * deltatime, res.angle[i - 1].z + w.z * deltatime));
        res.w.push_back(Point3d(w.x + distribution_angle(generator_angle), w.y + distribution_angle(generator_angle), w.z + distribution_angle(generator_angle)));
        res.timestamp.push_back(deltatime * i);

        res.accel.push_back(Point3d(accel.x + distribution_accel(generator_accel), accel.y + distribution_accel(generator_accel), accel.z + distribution_accel(generator_accel)));
        vel.push_back(Point3d(vel[i - 1].x + accel.x * deltatime, vel[i - 1].y + accel.y * deltatime, vel[i - 1].z + (accel.z + GCONST) * deltatime));
        res.pose.push_back(Point3d(res.pose[i - 1].x + vel[i].x * deltatime, res.pose[i - 1].y + vel[i].y * deltatime, res.pose[i - 1].z + vel[i].z * deltatime));
    }

    return res;
};/*double mean_1,
    const int size,
    double stddev,
    Point3d w,
    Point3d ang_0,
    Point3d ang_0,
    double deltatime,
    double accel_stddev,
    Point3d accel = Point3d(0, 0, 0),
    Point3d vel_0 = Point3d(0, 0, 0)*/


Track_part_type::Track_part_type(
    Point2d start,
    Point2d start_p_vec,
    double mean_line_length,
    double stddev_line,
    double mean_corner_radius,
    double stddev_radius,
    double min_corner_angle,
    double max_corner_angle,
    double average_vel
) {
    random_device rd;
    default_random_engine generator(rd());
    normal_distribution<double> distribution_line(mean_line_length, stddev_line);
    normal_distribution<double> distribution_radius(mean_corner_radius, stddev_radius);
    uniform_real_distribution<double> distribution_angle(min_corner_angle, max_corner_angle);

    double min_radius = 10;
    double max_radius = 500;
    double min_length = 5;
    double max_length = 500;
    double min_angle = 0.1;

    double line_len = distribution_line(generator);
    while (line_len < 0 || line_len > max_length) line_len = distribution_line(generator);
    double line_time = line_len / average_vel;
    this->line = Line_track_type(start, Point2d(start.x + start_p_vec.x * line_len, start.y + start_p_vec.y * line_len), line_time);

    double radius = distribution_radius(generator);
    while (radius < 0) radius = distribution_radius(generator);

    double angle = distribution_angle(generator);
    angle = fmod(angle, 2 * M_PI);
    angle = abs(angle) > 0 ? angle : min_angle;
    Point2d center = this->line.end + get_norm_vect(this->line.end, this->line.start) * radius;
    Point2d end = center + get_arc_end_point(this->line.end, center, angle);// not sure

    double corner_time = (length(center - this->line.end) * angle) / average_vel;
    this->turn = Corner_type(this->line.end, end, center, angle, corner_time, start_p_vec);
    this->exit_vec = get_norm_vect(this->turn.end, this->turn.center);
    this->end = this->turn.end;
};

void Test_model::generate_states(double delta_m, int point_num) {
    point_num = point_num == 0 ? this->total_length / delta_m : point_num;

    for (int i = 0; i < point_num; i++) {
        State_type state = this->orientation(i * delta_m);
        states.push_back(state);
    };
};

void Test_model::generate_gt_points(double delta_m, int point_num) {
    //нужно расставить точки в соответствии с заданной скоростью
    point_num = point_num == 0 ? this->total_length / delta_m : point_num;

    Pose_type pose1_tmp;
    pose1_tmp.lat = 0;
    pose1_tmp.lon = 0;
    pose1_tmp.alt = 0;

    gt_point.push_back(pose1_tmp);

    for (int i = 1; i < point_num; i++) {
        Point2d pose_delta = this->part(i * delta_m) - this->part((i - 1) * delta_m);

        Pose_type pose_tmp;
        pose_tmp.lat = gt_point[i - 1].lat + pose_delta.y;
        pose_tmp.lon = gt_point[i - 1].lon + pose_delta.x;
        pose_tmp.alt = gt_point[i - 1].alt;

        pose_tmp.roll = states[i].orient.x;
        pose_tmp.pitch = states[i].orient.y;
        pose_tmp.yaw = states[i].orient.z;

        pose_tmp.ax = states[i].accel.x;
        pose_tmp.ay = states[i].accel.y;
        pose_tmp.az = states[i].accel.z;

        pose_tmp.wx = states[i].anqular_accel.x;
        pose_tmp.wy = states[i].anqular_accel.y;
        pose_tmp.wz = states[i].anqular_accel.z;

        gt_point.push_back(pose_tmp);
    };
};


void Test_model::show_gt(string mode, bool pause_enable) {
    static Point2i img_size = Point2i(500, 500);
    Mat img(img_size.x, img_size.y, CV_8UC3, Scalar(255, 255, 255));
    int border = 50;
    double max_x = this->gt_point[0].lon;
    double max_y = this->gt_point[0].lat;
    double min_x = this->gt_point[0].lon;
    double min_y = this->gt_point[0].lat;

    for (int i = 0; i < this->gt_point.size(); i++)
    {
        max_x = this->gt_point[i].lon > max_x ? this->gt_point[i].lon : max_x;
        max_y = this->gt_point[i].lat > max_y ? this->gt_point[i].lat : max_y;

        min_x = this->gt_point[i].lon < min_x ? this->gt_point[i].lon : min_x;
        min_y = this->gt_point[i].lat < min_y ? this->gt_point[i].lat : min_y;
    };

    Point2d scale = Point2d(max_x - min_x, max_y - min_y);
    double im_scale = scale.x > scale.y ? (img_size.x - 2 * border) / scale.x : (img_size.y - 2 * border) / scale.y;
    Point2i zero_offset = -Point2i(min_x * im_scale, min_y * im_scale);

    for (int i = 1; i < this->gt_point.size(); i++)
    {
        Point2d next = Point2d(this->gt_point[i].lon, this->gt_point[i].lat);
        Point2d prev = Point2d(this->gt_point[i - 1].lon, this->gt_point[i - 1].lat);

        Point2i beg = Point2i(border + im_scale * (prev.x - min_x), border + im_scale * (prev.y - min_y));
        Point2i end = Point2i(border + im_scale * (next.x - min_x), border + im_scale * (next.y - min_y));
        line(img, beg, end, Scalar(0, 0, 0), 2);
        //circle(img, beg, 6, Scalar(0, 0, 0));
        if (i == 1) circle(img, beg, 6, Scalar(0, 0, 0));
    };

    for (int i = 0; i < this->track.size(); i++) {
        Point2i points_arr[5] = {
            Point2i(border + im_scale * (this->track[i].line.start.x - min_x),     border + im_scale * (this->track[i].line.start.y - min_y)),
            Point2i(border + im_scale * (this->track[i].line.end.x - min_x),       border + im_scale * (this->track[i].line.end.y - min_y)),
            Point2i(border + im_scale * (this->track[i].turn.start.x - min_x),     border + im_scale * (this->track[i].turn.start.y - min_y)),

            Point2i(border + im_scale * (this->track[i].turn.end.x - min_x),       border + im_scale * (this->track[i].turn.end.y - min_y)),
        };
        for (int p_i = 0; p_i < 4; p_i++) circle(img, points_arr[p_i], 4, Scalar(0, 0, 0));
        circle(
            img,
            Point2i(border + im_scale * (this->track[i].turn.center.x - min_x), border + im_scale * (this->track[i].turn.center.y - min_y)),
            4,
            Scalar(255, 0, 0));
    };


    if (mode.find("screen") != std::string::npos)
    {
        putText(img,
            "TEST",
            Point(img.cols - 200, 30),
            0,
            0.5,
            CV_RGB(0, 0, 0),
            0.5
        );
        namedWindow("Traect");
        imshow("Traect", img);
        if (pause_enable) waitKey(0);
    };
    if (mode.find("save") != std::string::npos) imwrite("Traect_TEST.jpg", img);
};


void Test_model::show_gt_measures(bool pause_enable) {
    static Point2i img_size = Point2i(400, 1000);
    Mat img(img_size.x, img_size.y, CV_8UC3, Scalar(255, 255, 255));
    int border = 50;

    double max_roll = this->gt_point[0].roll;
    double max_pitch = this->gt_point[0].pitch;
    double max_yaw = this->gt_point[0].yaw;

    double min_roll = this->gt_point[0].roll;
    double min_pitch = this->gt_point[0].pitch;
    double min_yaw = this->gt_point[0].yaw;

    for (int i = 0; i < this->gt_point.size(); i++)
    {
        max_roll = this->gt_point[i].roll > max_roll ? this->gt_point[i].roll : max_roll;
        max_pitch = this->gt_point[i].pitch > max_pitch ? this->gt_point[i].pitch : max_pitch;
        max_yaw = this->gt_point[i].yaw > max_yaw ? this->gt_point[i].yaw : max_yaw;

        min_roll = this->gt_point[i].roll < min_roll ? this->gt_point[i].roll : min_roll;
        min_pitch = this->gt_point[i].pitch < min_pitch ? this->gt_point[i].pitch : min_pitch;
        min_yaw = this->gt_point[i].yaw < min_yaw ? this->gt_point[i].yaw : min_yaw;
    };

    double bottom = min(min_roll, min_pitch, min_yaw);
    double top = max(max_roll, max_pitch, max_yaw);
    double offset = (top - bottom) / 2;

    double im_scale_x = (img_size.x - 2 * border) / (top - bottom);
    double im_scale_y = (img_size.y - border / 2) / double(this->gt_point.size());

    for (int i = 1; i < this->gt_point.size(); i++)
    {
        Point2d next = Point2d(this->gt_point[i].yaw, i);
        Point2d prev = Point2d(this->gt_point[i - 1].yaw, i);

        Point2i beg = Point2i(im_scale_y * prev.y + border / 2, border + im_scale_x * prev.x);
        Point2i end = Point2i(im_scale_y * next.y + border / 2, border + im_scale_x * next.x);
        line(img, beg, end, Scalar(128, 0, 0), 3);
    };

    for (int i = 1; i < this->gt_point.size(); i++)
    {
        Point2d next = Point2d(this->gt_point[i].pitch, i);
        Point2d prev = Point2d(this->gt_point[i - 1].pitch, i);

        Point2i beg = Point2i(im_scale_y * prev.y + border / 2, border + im_scale_x * prev.x);
        Point2i end = Point2i(im_scale_y * next.y + border / 2, border + im_scale_x * next.x);
        line(img, beg, end, Scalar(0, 128, 0), 2);
    };

    for (int i = 1; i < this->gt_point.size(); i++)
    {
        Point2d next = Point2d(-this->gt_point[i].roll, i);
        Point2d prev = Point2d(-this->gt_point[i - 1].roll, i);

        //Point2i beg = Point2i(border + im_scale_x * (prev.x + bottom/2), im_scale_y * prev.y);
        //Point2i end = Point2i(border + im_scale_x * (next.x + bottom/2), im_scale_y * next.y);
        Point2i beg = Point2i(im_scale_y * prev.y + border / 2, border + im_scale_x * prev.x);
        Point2i end = Point2i(im_scale_y * next.y + border / 2, border + im_scale_x * next.x);
        line(img, beg, end, Scalar(0, 0, 128), 1);
    };

    namedWindow("angles");
    imshow("angles", img);
    if (pause_enable) waitKey(0);
};


void angle_Test(double w_std, Point3d vel_0, double sko, double delta, double duration) {
    cout << endl;
    cout << endl;
    cout << "-------------<<<" << "Testing angle models" << ">>>-------------" << endl;
    cout << ">>>>w_std:\t" << w_std * 180 / M_PI << "deg/sec" << endl;
    cout << ">>>>sko:\t" << sko << endl;
    cout << ">>>>delta:\t" << delta << " sec" << endl;
    cout << ">>>>duration:\t" << duration << " sec" << endl;
    cout << endl;

    Data_seq test_0("Test0 stand still");
    vector<Point3d> test_0_ang_err_vec;
    test_0.load_model(generate_model(0.0, (int)(duration / delta), sko, Point3d(0, 0, 0), Point3d(M_PI / 2, 0, 0), delta, sko));
    for (int i = 0; i < test_0.limit; i++)
        test_0_ang_err_vec.push_back(calcDisplasment_ang(test_0.rot_ang_GT[i], test_0.rot_ang[i], 1));
    Point3d test_0_av_ang_err = Point3d(0, 0, 0);
    Point3d test_0_max_ang_err = Point3d(0, 0, 0);
    for (int i = 0; i < test_0_ang_err_vec.size(); i++)
    {
        test_0_max_ang_err.x = abs(test_0_ang_err_vec[i].x) > test_0_max_ang_err.x ? abs(test_0_ang_err_vec[i].x) : test_0_max_ang_err.x;
        test_0_max_ang_err.y = abs(test_0_ang_err_vec[i].y) > test_0_max_ang_err.y ? abs(test_0_ang_err_vec[i].y) : test_0_max_ang_err.y;
        test_0_max_ang_err.z = abs(test_0_ang_err_vec[i].z) > test_0_max_ang_err.z ? abs(test_0_ang_err_vec[i].z) : test_0_max_ang_err.z;

        test_0_av_ang_err.x += abs(test_0_ang_err_vec[i].x) / test_0_ang_err_vec.size();
        test_0_av_ang_err.y += abs(test_0_ang_err_vec[i].y) / test_0_ang_err_vec.size();
        test_0_av_ang_err.z += abs(test_0_ang_err_vec[i].z) / test_0_ang_err_vec.size();
    };
    cout << test_0.dirname << "\ttime: " << test_0.timestamps[test_0.limit - 1] - test_0.timestamps[0] << "sec" << endl;
    cout << ">>>>" << "Average angle error: roll " << test_0_av_ang_err.x << " deg\tpitch " << test_0_av_ang_err.y << " deg\tyaw " << test_0_av_ang_err.z << " deg" << endl;
    cout << ">>>>" << "Extrime angle error: roll " << test_0_max_ang_err.x << " deg\tpitch " << test_0_max_ang_err.y << " deg\tyaw " << test_0_max_ang_err.z << " deg" << endl;
    cout << endl;

    Data_seq test_1("Test1 yaw rotation");
    vector<Point3d> test_1_ang_err_vec;
    test_1.load_model(generate_model(0.0, (int)(duration / delta), sko, Point3d(w_std, 0, 0), Point3d(M_PI / 2, 0, 0), delta, sko));
    for (int i = 0; i < test_1.limit; i++)
        test_1_ang_err_vec.push_back(calcDisplasment_ang(test_1.rot_ang_GT[i], test_1.rot_ang[i], 1));
    Point3d test_1_av_ang_err = Point3d(0, 0, 0);
    Point3d test_1_max_ang_err = Point3d(0, 0, 0);
    for (int i = 0; i < test_1_ang_err_vec.size(); i++)
    {
        test_1_max_ang_err.x = abs(test_1_ang_err_vec[i].x) > test_1_max_ang_err.x ? abs(test_1_ang_err_vec[i].x) : test_1_max_ang_err.x;
        test_1_max_ang_err.y = abs(test_1_ang_err_vec[i].y) > test_1_max_ang_err.y ? abs(test_1_ang_err_vec[i].y) : test_1_max_ang_err.y;
        test_1_max_ang_err.z = abs(test_1_ang_err_vec[i].z) > test_1_max_ang_err.z ? abs(test_1_ang_err_vec[i].z) : test_1_max_ang_err.z;

        test_1_av_ang_err.x += abs(test_1_ang_err_vec[i].x) / test_1_ang_err_vec.size();
        test_1_av_ang_err.y += abs(test_1_ang_err_vec[i].y) / test_1_ang_err_vec.size();
        test_1_av_ang_err.z += abs(test_1_ang_err_vec[i].z) / test_1_ang_err_vec.size();
    };
    cout << test_1.dirname << "\ttime: " << test_1.timestamps[test_1.limit - 1] - test_1.timestamps[0] << "sec" << endl;
    cout << ">>>>" << "Average angle error: roll " << test_1_av_ang_err.x * 180 / M_PI << " deg\tpitch " << test_1_av_ang_err.y * 180 / M_PI << " deg\tyaw " << test_1_av_ang_err.z * 180 / M_PI << " deg" << endl;
    cout << ">>>>" << "Extrime angle error: roll " << test_1_max_ang_err.x * 180 / M_PI << " deg\tpitch " << test_1_max_ang_err.y * 180 / M_PI << " deg\tyaw " << test_1_max_ang_err.z * 180 / M_PI << " deg" << endl;
    cout << endl;

    Data_seq test_3("Test3 pitch rotation");
    vector<Point3d> test_3_ang_err_vec;
    test_3.load_model(generate_model(0.0, (int)(duration / delta), sko, Point3d(0, 0, w_std), Point3d(M_PI / 2, 0, 0), delta, sko));
    for (int i = 0; i < test_1.limit; i++)
        test_3_ang_err_vec.push_back(calcDisplasment_ang(test_3.rot_ang_GT[i], test_3.rot_ang[i], 1));
    Point3d test_3_av_ang_err = Point3d(0, 0, 0);
    Point3d test_3_max_ang_err = Point3d(0, 0, 0);
    for (int i = 0; i < test_1_ang_err_vec.size(); i++)
    {
        test_3_max_ang_err.x = abs(test_3_ang_err_vec[i].x) > test_3_max_ang_err.x ? abs(test_3_ang_err_vec[i].x) : test_3_max_ang_err.x;
        test_3_max_ang_err.y = abs(test_3_ang_err_vec[i].y) > test_3_max_ang_err.y ? abs(test_3_ang_err_vec[i].y) : test_3_max_ang_err.y;
        test_3_max_ang_err.z = abs(test_3_ang_err_vec[i].z) > test_3_max_ang_err.z ? abs(test_3_ang_err_vec[i].z) : test_3_max_ang_err.z;

        test_3_av_ang_err.x += abs(test_3_ang_err_vec[i].x) / test_3_ang_err_vec.size();
        test_3_av_ang_err.y += abs(test_3_ang_err_vec[i].y) / test_3_ang_err_vec.size();
        test_3_av_ang_err.z += abs(test_3_ang_err_vec[i].z) / test_3_ang_err_vec.size();
    };
    cout << test_3.dirname << "\ttime: " << test_3.timestamps[test_1.limit - 1] - test_3.timestamps[0] << "sec" << endl;
    cout << ">>>>" << "Average angle error: roll " << test_3_av_ang_err.x << " deg\tpitch " << test_3_av_ang_err.y << " deg\tyaw " << test_3_av_ang_err.z << " deg" << endl;
    cout << ">>>>" << "Extrime angle error: roll " << test_3_max_ang_err.x << " deg\tpitch " << test_3_max_ang_err.y << " deg\tyaw " << test_3_max_ang_err.z << " deg" << endl;
    cout << endl;
};


void motion_Test(double accel_std, double sko, double delta, double duration) {

    cout << endl;
    cout << endl;
    cout << "-------------<<<" << "Testing motion models" << ">>>-------------" << endl;
    cout << ">>>>accel_std:\t" << accel_std << "m/sec^2" << endl;
    cout << ">>>>sko:\t" << sko << endl;
    cout << ">>>>delta:\t" << delta << " sec" << endl;
    cout << ">>>>duration:\t" << duration << " sec" << endl;
    cout << endl;

    Data_seq test_1("Test1 stand still"); {
        vector<Pose_type> pose_err_vec;
        test_1.load_model(generate_model(
            0.0,
            (int)(duration / delta),
            sko,
            Point3d(0, 0, 0),
            Point3d(M_PI / 2, 0, 0),
            delta,
            sko,
            Point3d(0, 0, 0),
            Point3d(0, 0, 0)));
        for (int i = 0; i < test_1.limit - 1; i++) {
            Pose_type GT = test_1.dataline[i];
            pose_err_vec.push_back(calcDisplasment(GT, test_1.pose[i], test_1.pose[0], "Disp_test1", 3));
        };

        Pose_type GT = test_1.dataline[test_1.limit - 1];
        pose_err_vec.push_back(calcDisplasment(GT, test_1.pose[test_1.limit - 1], test_1.pose[0], "Disp_test1"));

        Pose_type av_pose_err;
        av_pose_err.lat = 0.0;
        av_pose_err.lon = 0.0;
        av_pose_err.alt = 0.0;
        for (int i = 0; i < pose_err_vec.size(); i++)
        {
            av_pose_err.lat += abs(pose_err_vec[i].lat) / pose_err_vec.size();
            av_pose_err.lon += abs(pose_err_vec[i].lon) / pose_err_vec.size();
            av_pose_err.alt += abs(pose_err_vec[i].alt) / pose_err_vec.size();
        }
        //ds.print_traect(num_sources);

        cout << "Disp_test1" << "\ttime: " << test_1.timestamps[test_1.limit - 1] - test_1.timestamps[0] << "sec" << endl;
        cout << ">>>>" << "Average pose  error: lat  " << av_pose_err.lat << " \tlon " << av_pose_err.lon << " \t alt " << av_pose_err.alt << endl;
        cout << endl;
        test_1.print_traect(1, "screen save", true);
    };

    Data_seq test_2("Test2 ecvi_dist"); {
        vector<Pose_type> pose_err_vec;
        test_2.load_model(generate_model(
            0.0,
            (int)(duration / delta),
            sko,
            Point3d(0, 0, 0),
            Point3d(M_PI / 2, 0, 0),
            delta,
            sko,
            Point3d(0, 0, 0),
            Point3d(1, 0, 0))
        );
        for (int i = 0; i < test_2.limit; i++) {
            Pose_type GT = test_2.dataline[i];
            pose_err_vec.push_back(calcDisplasment(GT, test_2.pose[i], test_2.pose[0], "Disp_test1", 3));
        };

        Pose_type GT = test_2.dataline[test_2.limit - 1];
        pose_err_vec.push_back(calcDisplasment(GT, test_2.pose[test_2.limit - 1], test_2.pose[0], "Disp_test1"));

        Pose_type av_pose_err;
        av_pose_err.lat = 0.0;
        av_pose_err.lon = 0.0;
        av_pose_err.alt = 0.0;
        for (int i = 0; i < pose_err_vec.size(); i++)
        {
            av_pose_err.lat += abs(pose_err_vec[i].lat) / pose_err_vec.size();
            av_pose_err.lon += abs(pose_err_vec[i].lon) / pose_err_vec.size();
            av_pose_err.alt += abs(pose_err_vec[i].alt) / pose_err_vec.size();
        }
        //ds.print_traect(num_sources);

        cout << "Disp_test2" << "\ttime: " << test_2.timestamps[test_2.limit - 1] - test_2.timestamps[0] << "sec" << endl;
        cout << ">>>>" << "Average pose  error: lat  " << av_pose_err.lat << " \tlon " << av_pose_err.lon << " \t alt " << av_pose_err.alt << endl;
        cout << endl;
        test_2.print_traect(2, "screen save", true);
    };

    Data_seq test_3("Test3 accel"); {
        vector<Pose_type> pose_err_vec;
        test_3.load_model(generate_model(
            0.0,
            (int)(duration / delta),
            sko,
            Point3d(0, 0, 0),
            Point3d(M_PI / 2, 0, 0),
            delta,
            sko,
            Point3d(accel_std, 0, 0),
            Point3d(0, 0, 0))
        );
        for (int i = 0; i < test_3.limit; i++) {
            Pose_type GT = test_3.dataline[i];
            pose_err_vec.push_back(calcDisplasment(GT, test_3.pose[i], test_3.pose[0], test_3.dirname, 3));
        };

        Pose_type GT = test_3.dataline[test_3.limit - 1];
        pose_err_vec.push_back(calcDisplasment(GT, test_3.pose[test_3.limit - 1], test_3.pose[0], test_3.dirname));

        Pose_type av_pose_err;
        av_pose_err.lat = 0.0;
        av_pose_err.lon = 0.0;
        av_pose_err.alt = 0.0;
        for (int i = 0; i < pose_err_vec.size(); i++)
        {
            av_pose_err.lat += abs(pose_err_vec[i].lat) / pose_err_vec.size();
            av_pose_err.lon += abs(pose_err_vec[i].lon) / pose_err_vec.size();
            av_pose_err.alt += abs(pose_err_vec[i].alt) / pose_err_vec.size();
        }
        //ds.print_traect(num_sources);

        cout << test_3.dirname << "\ttime: " << test_3.timestamps[test_3.limit - 1] - test_3.timestamps[0] << "sec" << endl;
        cout << ">>>>" << "Average pose  error: lat  " << av_pose_err.lat << " \tlon " << av_pose_err.lon << " \t alt " << av_pose_err.alt << endl;
        cout << endl;
        test_3.print_traect(3, "screen save", true);
    };
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

Point2d rotate2d(Point2d vec, double angle) {
    double rot_arr[2][2] = {
        {cos(angle), -sin(angle)},
        {sin(angle), cos(angle)}
    };
    Mat rot = Mat(2, 2, CV_64F, rot_arr);
    double radius_vec_arr[2][1] = {
        {vec.x},
        {vec.y}
    };
    Mat radius_vec = Mat(2, 1, CV_64F, radius_vec_arr);
    Mat end_vec = rot * radius_vec;
    return Point2d(end_vec.at<double>(0, 0), end_vec.at<double>(1, 0));
};

double length(Point2d vec) {
    return sqrt(vec.x * vec.x + vec.y * vec.y);
};

Point2d get_point_vect(Point2d end, Point2d start) {
    return Point2d((end.x - start.x) / length(end - start), (end.y - start.y) / length(end - start));
};

Point2d get_norm_vect(Point2d end, Point2d start) {
    return rotate2d(end - start, M_PI / 2) / length(end - start);
};

Point2d get_arc_end_point(Point2d cent, Point2d start, double angle) {
    return rotate2d(cent - start, angle);
};

Point3d toAngle3d(Point3d vec) {
    Point3d vec_x = normalize(Point3d(0, vec.y, vec.z));
    Point3d vec_y = normalize(Point3d(vec.x, 0 , vec.z));
    Point3d vec_z = normalize(Point3d(vec.x, vec.y, 0));
    vec = normalize(vec);

    Point3d res(0, 0, 0);
    res.x = acos(vec.dot(vec_x));
    res.y = acos(vec.dot(vec_y));
    res.z = acos(vec.dot(vec_z));

    return res;
};
