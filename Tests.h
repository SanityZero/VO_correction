#pragma once
#include "Track_part_type.h"

class Test_model {
private:
    //геометрическа€ модель
    vector<Track_part_type> track;
    vector<double> track_length;
    double total_length;

    Point2d part(double dist);
    State_type orientation(double dist = 0);
    void generate_track(int max_track_parts, double mean_line_length, double stddev_line, double mean_corner_radius,
        double stddev_radius, double mean_corner_angle, double stddev_angle, double average_vel);


    //модель движени€
    vector<Pose_type> gt_point;
    vector<State_type> states;
    vector<double> timestaps;
    double total_time;

    State_type get_state(int number);
    void generate_gt_points(double delta_m, int point_num = 0);
    void generate_states(double delta_m, int point_num = 0);
    void generate_timestaps(double delta_m, double vel);
    void smooth_anqular_vel(double T, double U);
    void smooth_vel(double T, double U);
    void regenerate_gt_points();


    //модель камеры
    vector<Point3d> s_points;

    void generate_s_points(
        double border = 50,
        Point2d z_limits = Point2d(0, 40),
        Point3d grid_spacing = Point3d(10, 10, 10),
        Point2d displacement = Point2d(0, 3)
    );


    //модель движени€ блест€щих точек
    vector<vector<Point2i>> point_tracks;
    vector<vector<Point2i>> point_camera_proections;

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

    //модель Ѕ»Ќ—
    vector<Pose_type> bins_gt_points;
    vector<Pose_type> bins_points;
    vector<double> bins_timestamps;

    void generate_bins_gt(double bins_deltatime) {
        double bins_total_time = 0.0;
        this->bins_timestamps.push_back(0.0);
        while (bins_total_time < (this->total_time - bins_deltatime)) {
            this->bins_timestamps.push_back(this->bins_timestamps[this->bins_timestamps.size() - 1] + bins_deltatime);
            bins_total_time += bins_deltatime;
        };
        //сгенерировать таймстемпы

        //начальное то же самое
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

            //интреполировать на них позиции, ориентации, ускорени€, угловые скорости
            tmp.setPose(prev.getPose()*(1- interp_multi) + next.getPose());
            tmp.setOrient(prev.getOrient()*(1- interp_multi) + next.getOrient());

            //получить проекции в соотв с ориентацией вектора угловых скоростей, вектора ускорений
            Point3d w_curr = this->states[lesser_ts_i].anqular_vel * (1 - interp_multi) + this->states[lesser_ts_i + 1].anqular_vel;
            Point3d accel_curr = this->states[lesser_ts_i].accel * (1 - interp_multi) + this->states[lesser_ts_i + 1].accel;
            
            //tmp.setAccel(rotate3d()));
            tmp.setW(prev.getW() * (1 - interp_multi) + next.getW());


        };      
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
        generate_track(max_track_parts, mean_line_length, stddev_line, mean_corner_radius, stddev_radius, mean_corner_angle, stddev_angle, average_vel);
        // сгенерировать трак в соответствии с ограничени€ми
        
        generate_states(dicret);
        generate_gt_points(dicret);
        generate_timestaps(dicret, average_vel);
        smooth_anqular_vel(T, U);
        //smooth_vel(T, U/10000);
        regenerate_gt_points();
        generate_s_points();
        
        // сгенерировать бинс данные по ограничени€м, т.е. набор значений
        // расставить точки
        // сгенерировать изображени€ в соответствии с точками

    };

    void show_gt(string mode = "screen", bool pause_enable = false);
    void show_gt_measures(bool pause_enable = false);
    void print_states(string filename);
};


DataSeq_model_Type generate_old_model(
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


void old_motion_Test(double accel_std = 1, double sko = 0.2, double delta = 0.004, double duration = 10);
void old_angle_Test(double w_std = 0.0001 * M_PI/180, Point3d vel_0 = Point3d(0,0,0), double sko = 0.000001 * M_PI/180, double delta = 0.004, double duration = 60);
