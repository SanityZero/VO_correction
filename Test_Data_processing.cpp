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
#include <omp.h>

//using namespace cv;
using namespace std;

#include "Types.h"
#include "ErEstVO.h"
#include "DataSequence.h"
#include "Track_part_type.h"
#include "Test_math.h"
#include "Tests.h"

typedef cv::Point2d Point2d;
typedef cv::Point3d Point3d;
typedef cv::Point2i Point2i;

#define F_ARR(dt) {\
                { 1,0,0,    0,0,0,  dt,0,0, 0,0,0},\
                { 0,1,0,    0,0,0,  0,dt,0, 0,0,0 },\
                { 0,0,1,    0,0,0,  0,0,dt, 0,0,0 },\
                    \
                { 0,0,0,    1,0,0,  0,0,0,  0,0,0 },\
                { 0,0,0,    0,1,0,  0,0,0,  0,0,0 },\
                { 0,0,0,    0,0,1,  0,0,0,  0,0,0 },\
                    \
                { 0,0,0,    0,0,0,  1,0,0,  0,0,0 },\
                { 0,0,0,    0,0,0,  0,1,0,  0,0,0 },\
                { 0,0,0,    0,0,0,  0,0,1,  0,0,0 },\
                    \
                { 0,0,0,    0,0,0,  dt,0,0, 1,0,0 },\
                { 0,0,0,    0,0,0,  0,dt,0, 0,1,0 },\
                { 0,0,0,    0,0,0,  0,0,dt, 0,0,1 }\
            }

#define B_ARR(dt) {\
                { dt * dt / 2,0,0,   0,0,0},\
                { 0,dt * dt / 2,0,   0,0,0 },\
                { 0,0,dt * dt / 2,   0,0,0 },\
\
                { 0,0,0,  dt,0,0 },\
                { 0,0,0,  0,dt,0 },\
                { 0,0,0,  0,0,dt },\
\
                { dt,0,0,  0,0,0 },\
                { 0,dt,0,  0,0,0 },\
                { 0,0,dt,  0,0,0 },\
\
                { -dt * dt / 2,0,0,   0,0,0 },\
                { 0,-dt * dt / 2,0,   0,0,0 },\
                { 0,0,-dt * dt / 2,   0,0,0 }\
}

void Test_model::trail_sequences_estimate(Trail_sequence _trail_sequence, int _mode) {

    Trail_sequence res(
        _trail_sequence.start,
        _trail_sequence.timestamps[0],
        _trail_sequence.state_vector[0],
        _trail_sequence.measurement_vector[0],
        _trail_sequence.control_vector[0]
    );

    Mat P = load_csv_Mat(this->dir_name + "Mats/P_0.csv", Point2i(12, 12));
    Mat Q = load_csv_Mat(this->dir_name + "Mats/Q_0.csv", Point2i(12, 12));

    double R_arr[2][2] = {
        {1, 0},
        {0, 1}
    };

    Mat R = Mat(2, 2, CV_64F, R_arr);

    vector<State_vector_type> state_vector = _trail_sequence.state_vector;
    vector<Measurement_vector_type> measurement_vector = _trail_sequence.measurement_vector;
    vector<Control_vector_type> control_vector = _trail_sequence.control_vector;
    vector<double> timestamps = _trail_sequence.timestamps;

    double prev_state_arr[12][1] = {
        {state_vector[0].get(0)}, {state_vector[0].get(1)}, {state_vector[0].get(2)},
        {state_vector[0].get(3)}, {state_vector[0].get(4)}, {state_vector[0].get(5)},
        {state_vector[0].get(6)}, {state_vector[0].get(7)}, {state_vector[0].get(8)},
        {state_vector[0].get(9)}, {state_vector[0].get(10)}, {state_vector[0].get(11)},
    };

    Mat x = Mat(12, 1, CV_64F, prev_state_arr);

    switch (_mode) {
    case 0:
    {
        // оценка и есть отклонение

        Mat std_dev = load_csv_Mat(this->dir_name + "Mats/std_dev.csv", Point2i(6, 1));

        for (int i = 1; i < timestamps.size(); i++) {
            //estimate x

            Mat H = this->senseMat(
                measurement_vector[i].get_point2d(), state_vector[i].get_s_pose(),
                state_vector[i].get_cam_pose(), state_vector[i].get_orient()
            );

            double dt = timestamps[i] - timestamps[i - 1];

            double F_arr[12][12] = F_ARR(dt);
            Mat F = Mat(12, 12, CV_64F, F_arr);

            double B_arr[12][6] = B_ARR(dt);
            Mat B = Mat(12, 6, CV_64F, B_arr);

            double std_dev_ax = std_dev.at<double>(0, 0);
            double std_dev_ay = std_dev.at<double>(1, 0);
            double std_dev_az = std_dev.at<double>(2, 0);

            double std_dev_wx = std_dev.at<double>(3, 0);
            double std_dev_wy = std_dev.at<double>(4, 0);
            double std_dev_wz = std_dev.at<double>(5, 0);

            std::random_device rd;
            std::default_random_engine generator(rd());
            std::normal_distribution<double> distribution_accel_x(0, std_dev_ax);
            std::normal_distribution<double> distribution_accel_y(0, std_dev_ay);
            std::normal_distribution<double> distribution_accel_z(0, std_dev_az);

            std::normal_distribution<double> distribution_w_x(0, std_dev_wx);
            std::normal_distribution<double> distribution_w_y(0, std_dev_wy);
            std::normal_distribution<double> distribution_w_z(0, std_dev_wz);



            double U_arr[6][1] = {
                {control_vector[i].get(0) + distribution_accel_x(generator)},
                {control_vector[i].get(1) + distribution_accel_x(generator)},
                {control_vector[i].get(2) + distribution_accel_x(generator)},
                {control_vector[i].get(3) + distribution_w_x(generator)},
                {control_vector[i].get(4) + distribution_w_y(generator)},
                {control_vector[i].get(5) + distribution_w_z(generator)}
            };

            Mat U = Mat(6, 1, CV_64F, U_arr);

            Mat x_next = F * x + B * U;
            Mat P_est = F * P * F.t() + Q;
            Mat H_t = H.t();

            Mat tmp = (H * P_est * H.t() + R);

            double tmp_arr[2][2] = {
                {tmp.at<double>(0, 0), tmp.at<double>(0, 1)},
                {tmp.at<double>(1, 0), tmp.at<double>(1, 1)}
            };

            Mat K = P_est * H.t() * tmp.inv();
            Mat P_next = (Mat::eye(12, 12, CV_64F) - K * H) * P_est;

            P = P_next.clone();
            x = x_next.clone();

            State_vector_type estimated_state;
            estimated_state.set_cam_pose(Point3d(x_next.at<double>(0, 0), x_next.at<double>(1, 0), x_next.at<double>(2, 0)));
            estimated_state.set_orient(Point3d(x_next.at<double>(3, 0), x_next.at<double>(4, 0), x_next.at<double>(5, 0)));
            estimated_state.set_cam_vel(Point3d(x_next.at<double>(6, 0), x_next.at<double>(7, 0), x_next.at<double>(8, 0)));
            estimated_state.set_s_pose(Point3d(x_next.at<double>(9, 0), x_next.at<double>(10, 0), x_next.at<double>(11, 0)));

            double time = timestamps[i];

            res.push_back(timestamps[i], estimated_state, measurement_vector[i], control_vector[i]);
        };
    };
    break;
    case 1:
    {
        // нет оценки и есть отклонение

        Mat std_dev = load_csv_Mat(this->dir_name + "Mats/std_dev.csv", Point2i(6, 1));

        for (int i = 1; i < timestamps.size(); i++) {
            //estimate x

            double dt = timestamps[i] - timestamps[i - 1];

            double F_arr[12][12] = F_ARR(dt);
            Mat F = Mat(12, 12, CV_64F, F_arr);

            double B_arr[12][6] = B_ARR(dt);
            Mat B = Mat(12, 6, CV_64F, B_arr);

            double std_dev_ax = std_dev.at<double>(0, 0);
            double std_dev_ay = std_dev.at<double>(1, 0);
            double std_dev_az = std_dev.at<double>(2, 0);

            double std_dev_wx = std_dev.at<double>(3, 0);
            double std_dev_wy = std_dev.at<double>(4, 0);
            double std_dev_wz = std_dev.at<double>(5, 0);

            std::random_device rd;
            std::default_random_engine generator(rd());
            std::normal_distribution<double> distribution_accel_x(0, std_dev_ax);
            std::normal_distribution<double> distribution_accel_y(0, std_dev_ay);
            std::normal_distribution<double> distribution_accel_z(0, std_dev_az);

            std::normal_distribution<double> distribution_w_x(0, std_dev_wx);
            std::normal_distribution<double> distribution_w_y(0, std_dev_wy);
            std::normal_distribution<double> distribution_w_z(0, std_dev_wz);

            double U_arr[6][1] = {
                {control_vector[i].get(0) + distribution_accel_x(generator)},
                {control_vector[i].get(1) + distribution_accel_x(generator)},
                {control_vector[i].get(2) + distribution_accel_x(generator)},
                {control_vector[i].get(3) + distribution_w_x(generator)},
                {control_vector[i].get(4) + distribution_w_y(generator)},
                {control_vector[i].get(5) + distribution_w_z(generator)}
            };

            Mat U = Mat(6, 1, CV_64F, U_arr);

            Mat x_next = F * x + B * U;

            x = x_next.clone();

            State_vector_type estimated_state;
            estimated_state.set_cam_pose(Point3d(x_next.at<double>(0, 0), x_next.at<double>(1, 0), x_next.at<double>(2, 0)));
            estimated_state.set_orient(Point3d(x_next.at<double>(3, 0), x_next.at<double>(4, 0), x_next.at<double>(5, 0)));
            estimated_state.set_cam_vel(Point3d(x_next.at<double>(6, 0), x_next.at<double>(7, 0), x_next.at<double>(8, 0)));
            estimated_state.set_s_pose(Point3d(x_next.at<double>(9, 0), x_next.at<double>(10, 0), x_next.at<double>(11, 0)));

            double time = timestamps[i];

            res.push_back(timestamps[i], estimated_state, measurement_vector[i], control_vector[i]);
        };
    };

    break;
    case 2:
    {
        //  нет оценки и нет отклонение
        for (int i = 1; i < timestamps.size(); i++) {
            //estimate x

            double dt = timestamps[i] - timestamps[i - 1];

            double F_arr[12][12] = F_ARR(dt);
            Mat F = Mat(12, 12, CV_64F, F_arr);

            double B_arr[12][6] = B_ARR(dt);
            Mat B = Mat(12, 6, CV_64F, B_arr);

            double U_arr[6][1] = {
                {control_vector[i].get(0)},
                {control_vector[i].get(1)},
                {control_vector[i].get(2)},
                {control_vector[i].get(3)},
                {control_vector[i].get(4)},
                {control_vector[i].get(5)}
            };

            Mat U = Mat(6, 1, CV_64F, U_arr);

            Mat x_next = F * x + B * U;

            x = x_next.clone();

            State_vector_type estimated_state;
            estimated_state.set_cam_pose(Point3d(x_next.at<double>(0, 0), x_next.at<double>(1, 0), x_next.at<double>(2, 0)));
            estimated_state.set_orient(Point3d(x_next.at<double>(3, 0), x_next.at<double>(4, 0), x_next.at<double>(5, 0)));
            estimated_state.set_cam_vel(Point3d(x_next.at<double>(6, 0), x_next.at<double>(7, 0), x_next.at<double>(8, 0)));
            estimated_state.set_s_pose(Point3d(x_next.at<double>(9, 0), x_next.at<double>(10, 0), x_next.at<double>(11, 0)));

            double time = timestamps[i];

            res.push_back(timestamps[i], estimated_state, measurement_vector[i], control_vector[i]);
        };
    };
    break;
    };

    this->states_estimated.push_back(res);
};

Point2d Test_model::part_der_h(Point2d _P, Point3d _point_pose, Point3d _camera_pose, Point3d _camera_orient, Point3d _delta) {
    Point2d res = (
        this->point_proection_D(_point_pose + _delta, _camera_pose, _camera_orient)
        - this->point_proection_D(_point_pose - _delta, _camera_pose, _camera_orient)
        )
        / (length(_delta) * 2);
    return res;
};

Mat Test_model::senseMat(Point2d _P, Point3d _point_pose, Point3d _camera_pose, Point3d _camera_orient) {
    //delta _point_pose
    Point3d delta_pp_x = Point3d(0.001, 0, 0);
    Point3d delta_pp_y = Point3d(0, 0.001, 0);
    Point3d delta_pp_z = Point3d(0, 0, 0.001);

    //delta _camera_pose
    Point3d delta_cp_x = Point3d(0.001, 0, 0);
    Point3d delta_cp_y = Point3d(0, 0.001, 0);
    Point3d delta_cp_z = Point3d(0, 0, 0.001);

    //delta _camera_orient
    Point3d delta_co_x = Point3d(0.001, 0, 0);
    Point3d delta_co_y = Point3d(0, 0.001, 0);
    Point3d delta_co_z = Point3d(0, 0, 0.001);

    vector<Point2d> point_pose;
    point_pose.push_back(part_der_h(_P, _point_pose, _camera_pose, _camera_orient, delta_pp_x));
    point_pose.push_back(part_der_h(_P, _point_pose, _camera_pose, _camera_orient, delta_pp_y));
    point_pose.push_back(part_der_h(_P, _point_pose, _camera_pose, _camera_orient, delta_pp_z));

    point_pose.push_back(part_der_h(_P, _point_pose, _camera_pose, _camera_orient, delta_cp_x));
    point_pose.push_back(part_der_h(_P, _point_pose, _camera_pose, _camera_orient, delta_cp_y));
    point_pose.push_back(part_der_h(_P, _point_pose, _camera_pose, _camera_orient, delta_cp_z));

    point_pose.push_back(part_der_h(_P, _point_pose, _camera_pose, _camera_orient, delta_co_x));
    point_pose.push_back(part_der_h(_P, _point_pose, _camera_pose, _camera_orient, delta_co_y));
    point_pose.push_back(part_der_h(_P, _point_pose, _camera_pose, _camera_orient, delta_co_z));

    double senseMat_ar[2][12] = {
        {
            point_pose[0].x, point_pose[1].x, point_pose[2].x,
            point_pose[3].x, point_pose[4].x, point_pose[5].x,
            0.0,             0.0,             0.0,
            point_pose[6].x, point_pose[7].x, point_pose[8].x
        },
        {
            point_pose[0].y, point_pose[1].y, point_pose[2].y,
            point_pose[3].y, point_pose[4].y, point_pose[5].y,
            0.0,             0.0,             0.0,
            point_pose[6].y, point_pose[7].y, point_pose[8].y
        }
    };

    return  Mat(2, 12, CV_64F, senseMat_ar).clone();
};
