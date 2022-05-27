#pragma once
#include "Track_part_type.h"

class Test_model {
private:
    //геометрическая модель
    vector<Track_part_type> track;
    vector<double> track_length;
    double total_length;

    Point2d part(double dist);
    State_type orientation(double dist = 0);
    void generate_track(int max_track_parts, double mean_line_length, double stddev_line, double mean_corner_radius,
        double stddev_radius, double mean_corner_angle, double stddev_angle, double average_vel);


    //модель движения
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


    //модель движения блестящих точек
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

    //модель БИНС
    vector<Pose_type> bins_gt_points;
    vector<Pose_type> bins_points;
    vector<double> bins_timestamps;

    void generate_bins_gt(double bins_deltatime);

    

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
        // сгенерировать трак в соответствии с ограничениями
        generate_track(max_track_parts, mean_line_length, stddev_line, mean_corner_radius, stddev_radius, mean_corner_angle, stddev_angle, average_vel);
        generate_states(dicret);
        generate_gt_points(dicret);
        generate_timestaps(dicret, average_vel);
        smooth_anqular_vel(T, U);
        //smooth_vel(T, U/10000);
        regenerate_gt_points();
        

        // сгенерировать бинс данные по ограничениям, т.е. набор значений
        generate_bins_gt(0.1);
        
        // расставить точки
        generate_s_points();

        // сгенерировать изображения в соответствии с точками

    };


    void show_gt(string mode = "screen", bool pause_enable = false);
    void show_gt_measures(bool pause_enable = false);

    void print_states(string filename);
    void print_bins_gts(string filename) {
        ofstream out;          // поток для записи
        out.open(filename); // окрываем файл для записи
        if (out.is_open())
        {
            //Point3d vel;
            //Point3d accel;
            //Point3d orient;
            //Point3d anqular_vel;
            //Point3d anqular_accel;
            for (int i = 0; i < this->bins_timestamps.size(); i++) {
                out << format(
                    "%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t\n",
                    this->bins_timestamps[i], 
                    this->bins_gt_points[i].getPose().x, this->bins_gt_points[i].getPose().y, this->bins_gt_points[i].getPose().z,
                    this->bins_gt_points[i].getOrient().x, this->bins_gt_points[i].getOrient().y, this->bins_gt_points[i].getOrient().z,
                    this->bins_gt_points[i].getAccel().x, this->bins_gt_points[i].getAccel().y, this->bins_gt_points[i].getAccel().z,
                    this->bins_gt_points[i].getW().x, this->bins_gt_points[i].getW().y, this->bins_gt_points[i].getW().z
                );
            };
        }
        out.close();
    };
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
