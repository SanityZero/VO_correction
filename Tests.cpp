#include <iostream>
#include <locale>
#include <iomanip> 
#include <fstream>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <random>
#include <cmath>

using namespace cv;
using namespace std;

#include "Types.h"
#include "ErEstVO.h"
#include "DataSequence.h"
#include "Track_part_type.h"
#include "Tests.h"

void Test_model::generate_bins_gt(double bins_deltatime) {
    double bins_total_time = 0.0;
    this->bins_timestamps.push_back(0.0);
    while (bins_total_time < (this->total_time - bins_deltatime)) {
        this->bins_timestamps.push_back(this->bins_timestamps[this->bins_timestamps.size() - 1] + bins_deltatime);
        bins_total_time += bins_deltatime;
    };
    //сгенерировать таймстемпы

    //начальное то же самое
    this->bins_gt_points.push_back(Pose_type(this->gt_point[0].getPose(), this->gt_point[0].getOrient(), this->gt_point[0].getAccel(), this->gt_point[0].getW()));
    for (int i = 1; i < this->bins_timestamps.size(); i++) {
        Pose_type tmp;
        double bins_time = bins_deltatime * i;
        int lesser_ts_i = 0;
        double interp_multi = 0;
        for (int ts_i = 0; ts_i < this->timestaps.size() - 1; ts_i++) {
            if ((this->timestaps[ts_i] <= bins_time) && (this->timestaps[ts_i + 1] >= bins_time)) {
                interp_multi = (bins_time - (this->timestaps[ts_i + 1] - this->timestaps[ts_i])) / (this->timestaps[ts_i + 1] - this->timestaps[ts_i]);
                lesser_ts_i = ts_i;
            };
        };
        Pose_type prev = this->gt_point[lesser_ts_i];
        Pose_type next = this->gt_point[lesser_ts_i + 1];

        //интреполировать на них позиции, ориентации, ускорения, угловые скорости
        tmp.setPose(prev.getPose() * (1 - interp_multi) + next.getPose());
        tmp.setOrient(prev.getOrient() * (1 - interp_multi) + next.getOrient());

        //получить проекции в соотв с ориентацией вектора угловых скоростей, вектора ускорений
        Point3d w_curr = this->states[lesser_ts_i].anqular_vel * (1 - interp_multi) + this->states[lesser_ts_i + 1].anqular_vel;
        Point3d accel_curr = this->states[lesser_ts_i].accel * (1 - interp_multi) + this->states[lesser_ts_i + 1].accel;

        tmp.setAccel(rotateP3d(accel_curr, -tmp.getOrient()));
        tmp.setW(w_curr);
        this->bins_gt_points.push_back(tmp);
    };
};

void Test_model::generate_track(int max_track_parts, double mean_line_length, double stddev_line, double mean_corner_radius,
    double stddev_radius, double mean_corner_angle, double stddev_angle, double average_vel)
{
    track.push_back(Track_part_type(
        Point2d(0, 0),
        Point2d(-1, 0),
        mean_line_length,
        stddev_line,
        mean_corner_radius,
        stddev_radius,
        mean_corner_angle,
        stddev_angle,
        average_vel
    ));
    track_length.push_back(track[0].len());
    this->total_length = track_length[0];
    for (int i = 1; i < max_track_parts; i++) {
        track.push_back(Track_part_type(
            this->track[i - 1].end,
            this->track[i - 1].exit_vec,
            mean_line_length,
            stddev_line,
            mean_corner_radius,
            stddev_radius,
            mean_corner_angle,
            stddev_angle,
            average_vel
        ));
        track_length.push_back(track[i].len());
        this->total_length += track_length[i];
    };
};

void  Test_model::regenerate_gt_points() {
    for (int i = 1; i < this->gt_point.size(); i++) {
        this->gt_point[i].lat = this->gt_point[i - 1].lat + this->states[i].vel.y * (this->timestaps[i] - this->timestaps[i - 1]);
        this->gt_point[i].lon = this->gt_point[i - 1].lon + this->states[i].vel.x * (this->timestaps[i] - this->timestaps[i - 1]);
        this->gt_point[i].alt = this->gt_point[i - 1].alt + this->states[i].vel.z * (this->timestaps[i] - this->timestaps[i - 1]);

        this->gt_point[i].roll = states[i].orient.x;
        this->gt_point[i].pitch = states[i].orient.y;
        this->gt_point[i].yaw = states[i].orient.z;

        this->gt_point[i].ax = states[i].accel.x;
        this->gt_point[i].ay = states[i].accel.y;
        this->gt_point[i].az = states[i].accel.z;

        this->gt_point[i].wx = states[i].anqular_accel.x;
        this->gt_point[i].wy = states[i].anqular_accel.y;
        this->gt_point[i].wz = states[i].anqular_accel.z;
    };

};

void Test_model::generate_s_points(
    double border,
    Point2d z_limits,
    Point3d grid_spacing,
    Point2d displacement
) {
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

    double min_z = z_limits.x;
    double max_z = z_limits.y;

    random_device rd;
    default_random_engine generator(rd());
    double mean_displacement = displacement.x;
    double stddev_displacement = displacement.y;
    normal_distribution<double> distribution_displacement(mean_displacement, stddev_displacement);
    for (double x_shift = min_x - border; x_shift < max_x + border; x_shift += grid_spacing.x)
        for (double y_shift = min_y - border; y_shift < max_y + border; y_shift += grid_spacing.y)
            for (double z_shift = min_z - border; z_shift < max_z + border; z_shift += grid_spacing.z)
            {
                Point3d displacement_vec = Point3d(distribution_displacement(generator), distribution_displacement(generator), distribution_displacement(generator));
                Point3d new_s_point = Point3d(x_shift, y_shift, z_shift) + displacement_vec;
                s_points.push_back(new_s_point);
            };
    //потом можно добавить, чтобы точки совсем рядом с дорогой стирались

};

void Test_model::generate_timestaps(double delta_m, double vel) {
    double deltatime = delta_m / vel;
    this->timestaps.push_back(0.0);
    for (int i = 1; i < this->states.size(); i++) {
        this->timestaps.push_back(this->timestaps[i - 1] + deltatime);
    };
    this->total_time = this->timestaps[this->states.size()-1];
};

State_type Test_model::get_state(int number) {
    if (number < 0)
        return this->states[0];
    else if (number > this->states.size())
        return this->states[this->states.size()];
    else
        return this->states[number];
};

void Test_model::smooth_anqular_vel(double T, double U) {
    vector<Point3d> res_orient_vec;
    vector<Point3d> res_ang_vel_vec;

    res_orient_vec.push_back(this->get_state(0).orient);
    res_orient_vec.push_back(this->get_state(1).orient);
    res_ang_vel_vec.push_back(this->get_state(0).anqular_vel);
    res_ang_vel_vec.push_back(this->get_state(1).anqular_vel);
    for (int i = 2; i < this->states.size(); i++) {
        Point3d old = this->get_state(i).orient;
        Point3d anq_tmp =
            -(T / 2) * (this->get_state(i).orient + this->get_state(i - 1).orient) +
            (U * T / 2 + 1) * res_orient_vec[i - 1] +
            (U * T / 2) * res_orient_vec[i - 2];
        res_orient_vec.push_back(anq_tmp);
        res_ang_vel_vec.push_back((res_orient_vec[i] - res_orient_vec[i - 1]) / T);
    };
    for (int i = 1; i < this->states.size(); i++) this->states[i].change_orient(res_orient_vec[i]);
    for (int i = 1; i < this->states.size(); i++) this->states[i].change_anqular_vel(res_ang_vel_vec[i]);
};

void Test_model::smooth_vel(double T, double U) {
    vector<Point3d> res_vel_vec;
    vector<Point3d> res_accel_vec;

    res_vel_vec.push_back(this->get_state(0).vel);
    res_vel_vec.push_back(this->get_state(1).vel);
    res_accel_vec.push_back(this->get_state(0).accel);
    res_accel_vec.push_back(this->get_state(1).accel);
    for (int i = 2; i < this->states.size(); i++) {
        Point3d old = this->get_state(i).vel;
        Point3d anq_tmp =
            -(T / 2) * (this->get_state(i).vel + this->get_state(i - 1).vel) +
            (U * T / 2 + 1) * res_vel_vec[i - 1] +
            (U * T / 2) * res_vel_vec[i - 2];
        res_vel_vec.push_back(anq_tmp);
        res_accel_vec.push_back((res_vel_vec[i] - res_vel_vec[i - 1]) / T);
    };
    for (int i = 1; i < this->states.size(); i++) this->states[i].change_vel(res_vel_vec[i]);
    for (int i = 1; i < this->states.size(); i++) this->states[i].change_accel(res_accel_vec[i]);
};

void Test_model::print_states(string filename) {
    ofstream out;          // поток для записи
    out.open(filename); // окрываем файл для записи
    if (out.is_open())
    {
        //Point3d vel;
        //Point3d accel;
        //Point3d orient;
        //Point3d anqular_vel;
        //Point3d anqular_accel;
        for (int i = 0; i < this->states.size(); i++) {
            out << format(
                "%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",
                this->states[i].vel.x, this->states[i].vel.y, this->states[i].vel.z,
                this->states[i].orient.x, this->states[i].orient.y, this->states[i].orient.z,
                this->states[i].anqular_vel.x, this->states[i].anqular_vel.y, this->states[i].anqular_vel.z,
                this->states[i].accel.x, this->states[i].accel.y, this->states[i].accel.z,
                this->states[i].anqular_accel.x, this->states[i].anqular_accel.y, this->states[i].anqular_accel.z
            );
        };
    }
    out.close();
};

State_type Test_model::orientation(double dist) {
    int i = 0;
    while (true) {
        dist -= this->track[i].len();
        if (dist == 0) this->track[i].orientation(dist + this->track[i].len());
        if (dist < 0) {
            return this->track[i].orientation(dist + this->track[i].len());
        };
        i++;
        if (i == track.size()) return State_type();
    };
};

Point2d Test_model::part(double dist) {
    int i = 0;
    while (true) {
        dist -= this->track[i].len();
        if (dist == 0) return this->track[i].end;
        if (dist < 0) {
            return this->track[i].part(dist + this->track[i].len());
        };
        i++;
        if (i == track.size()) return Point2d(0, 0);
    };
};


inline DataSeq_model_Type generate_old_model(
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

void Test_model::generate_states(double delta_m, int point_num) {
    point_num = point_num == 0 ? this->total_length / delta_m : point_num;

    for (int i = 0; i < point_num; i++) {
        State_type state = this->orientation(i * delta_m);
        Point3d v3_orient = normalize(state.orient);
        double z_rot = (acos((v3_orient.y + v3_orient.x)/2) + asin((v3_orient.y - v3_orient.x)/2)) / 2;
        state.change_orient(Point3d(0, 0, z_rot));
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

    for (int i = 0; i < this->s_points.size(); i++) {
        Point2i cross_size = Point2i(3, 3);
        Point2i s_point_location = Point2i(border + im_scale * (this->s_points[i].x - min_x), border + im_scale * (this->s_points[i].y - min_y));
        Point2i cross_points[4] = {
            s_point_location + Point2i(-cross_size.x / 2, -cross_size.y / 2),
            s_point_location + Point2i(-cross_size.x / 2, cross_size.y / 2),
            s_point_location + Point2i(cross_size.x / 2, cross_size.y / 2),
            s_point_location + Point2i(cross_size.x / 2, -cross_size.y / 2)
        };
        line(img, cross_points[0], cross_points[2], Scalar(255, 0, 0), 1);
        line(img, cross_points[1], cross_points[3], Scalar(255, 0, 0), 1);
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


void old_angle_Test(double w_std, Point3d vel_0, double sko, double delta, double duration) {
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
    test_0.load_model(generate_old_model(0.0, (int)(duration / delta), sko, Point3d(0, 0, 0), Point3d(M_PI / 2, 0, 0), delta, sko));
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
    test_1.load_model(generate_old_model(0.0, (int)(duration / delta), sko, Point3d(w_std, 0, 0), Point3d(M_PI / 2, 0, 0), delta, sko));
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
    test_3.load_model(generate_old_model(0.0, (int)(duration / delta), sko, Point3d(0, 0, w_std), Point3d(M_PI / 2, 0, 0), delta, sko));
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


void old_motion_Test(double accel_std, double sko, double delta, double duration) {

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
        test_1.load_model(generate_old_model(
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
        test_2.load_model(generate_old_model(
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
        test_3.load_model(generate_old_model(
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
////END FILE