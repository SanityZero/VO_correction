#pragma once
#include "Track_part_type.h"

class Test_model {
private:
    vector<Track_part_type> track;
    vector<double> track_length;
    vector<Pose_type> gt_point;
    vector<State_type> states;
    vector<double> timestaps;
    vector<Point3d> s_points;

    vector<vector<Point2i>> point_tracks;
    vector<vector<Point2i>> point_camera_proections;

    double total_length;

    Point2d part(double dist);
    State_type orientation(double dist = 0);

    State_type get_state(int number);

    void generate_gt_points(double delta_m, int point_num = 0);
    void generate_states(double delta_m, int point_num = 0);
    void generate_timestaps(double delta_m, double vel);
    void generate_mesured_points(double mean_angle, double stddev_angle, double mean_pose, double pose_stddev) {
        default_random_engine generator;
        normal_distribution<double> distribution_angle(mean_angle, stddev_angle);
        normal_distribution<double> distribution_pose(mean_pose, pose_stddev);
    };//НЕ РЕАЛИЗОВАНО!!!1!1!1!!!!

    void smooth_anqular_vel(double T, double U);
    void generate_s_points(
        double border = 50,
        Point2d z_limits = Point2d(0, 40),
        Point3d grid_spacing = Point3d(10, 10, 10),
        Point2d displacement = Point2d(0, 3)
    );

    Point2i point_proection(Point3d point, Mat Ex_calib, Point2i cam_size) {
        return Point2i(0, 0);
    };

    void generate_camera_proections() {
        //Mat R = 
        //double E_ar[3][4] = {
        //{1, 0, 0, 0},
        //{0, 1, 0, 0},
        //{0, 0, 1, 0}
        //};
        //Mat E = Mat(3, 4, CV_64F, E_ar);

        //Mat R_t = R.t();

        //Mat tmp = -R_t * t;

        //double M_ar[4][4] = {
        //     {R_t.at<double>(0, 0), R_t.at<double>(0, 1), R_t.at<double>(0, 2), tmp.at<double>(0, 0)},
        //     {R_t.at<double>(1, 0), R_t.at<double>(1, 1), R_t.at<double>(1, 2), tmp.at<double>(1, 0)},
        //     {R_t.at<double>(2, 0), R_t.at<double>(2, 1), R_t.at<double>(2, 2), tmp.at<double>(2, 0)},
        //     {0, 0, 0, 1}
        //};
        //Mat M = Mat(4, 4, CV_64F, M_ar);

        //tmp = A * E * M;

    };

public:
    void generate_test_model(
        int max_track_parts, 
        double dicret,
        double mean_line_length,
        double stddev_line,
        double mean_corner_radius,
        double stddev_radius,
        double mean_corner_angle,
        double stddev_angle,
        double average_vel,
        double T,
        double U
    ) {
        track.push_back(Track_part_type(
            Point2d(0,0),
            Point2d(-1,0),
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
        };// сгенерировать трак в соответствии с ограничениями
        
        // сгенерировать гт данные по ограничениям т.е. набор значений 
        generate_states(dicret);
        generate_gt_points(dicret);
        generate_timestaps(dicret, average_vel);
        smooth_anqular_vel(T, U);
        generate_s_points();
        
        // сгенерировать бинс данные по ограничениям, т.е. набор значений
        // расставить точки
        // сгенерировать изображения в соответствии с точками

    };

    void show_gt(string mode = "screen", bool pause_enable = false);
    void show_gt_measures(bool pause_enable = false);
    void print_states(string filename);
};

DataSeq_model_Type generate_model(
    double mean_1,
    const int size,
    double stddev,
    Point3d w,
    Point3d ang_0,
    double deltatime,
    double accel_stddev,
    Point3d accel,
    Point3d vel_0
    );/*double mean_1,
    const int size,
    double stddev,
    Point3d w,
    Point3d ang_0,
    Point3d ang_0,
    double deltatime,
    double accel_stddev,
    Point3d accel = Point3d(0, 0, 0),
    Point3d vel_0 = Point3d(0, 0, 0)*/


void motion_Test(double accel_std = 1, double sko = 0.2, double delta = 0.004, double duration = 10);
void angle_Test(double w_std = 0.0001 * M_PI/180, Point3d vel_0 = Point3d(0,0,0), double sko = 0.000001 * M_PI/180, double delta = 0.004, double duration = 60);
