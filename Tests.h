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
    void smooth_anqular_vel(double T, double U1, double U2);
    void smooth_vel(double T, double U);
    void regenerate_gt_points();


    //модель камеры
    vector<Point3d> s_points;
    Point2i frame_size;
    Point2d cam_range;
    Mat A;
    //Mat EX_calib;

    void generate_s_points(double border, Point3d z_limits, Point3d grid_spacing, Point2d displacement);
    Mat generateExCalibM(int i);
    void setCameraModel(Point2i _frame_size, Point2d _cam_range, Mat _A) {
        this->frame_size = _frame_size;
        this->cam_range = _cam_range;
        this->A = _A;
    };

    //модель движения блестящих точек
    vector<vector<Point2i>> point_tracks;
    vector<vector<Point2i>> point_camera_proections;

    Point2i point_proection(Point3d point_pose, Point3d camera_pose, Mat Ex_calib);
    void generate_camera_proections();

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
        double U1,
        double U2
    ) {
        // сгенерировать трак в соответствии с ограничениями
        generate_track(max_track_parts, mean_line_length, stddev_line, mean_corner_radius, stddev_radius, mean_corner_angle, stddev_angle, average_vel);
        generate_states(dicret);
        generate_gt_points(dicret);
        generate_timestaps(dicret, average_vel);
        smooth_anqular_vel(T, U1, U2);
        smooth_vel(T, U1/10000);
        regenerate_gt_points();
        

        // сгенерировать бинс данные по ограничениям, т.е. набор значений
        generate_bins_gt(T);
        
        // расставить точки
        double grid_step = 10;
        generate_s_points(
            50, //border x y
            Point3d(0, 30, 0), //z_limits min_z max_z 0
            Point3d(grid_step, grid_step, grid_step), //grid_spacing
            Point2d(0, 3) //displacement
        );


        Point2i fr_size = Point2i(1242, 373);
        Point2d cam_limits = Point2d(3, 100000000);
        double IternalCalib_ar[3][3] = {
        {3, 0, fr_size.x / 2},
        {0, 3, fr_size.y / 2},
        {0, 0, 1}
        };
        Mat A = Mat(3, 3, CV_64F, IternalCalib_ar);
        

        setCameraModel(fr_size, cam_limits, A);
        generate_camera_proections();
        // сгенерировать изображения в соответствии с точками

    };


    void show_gt(string mode = "screen", bool pause_enable = false);
    void show_bins_gt(bool pause_enable = false);
    void print_camera_proections() {
        for (int i = 0; i < this->bins_timestamps.size()-1; i++) {
            Mat frame(this->frame_size.y, this->frame_size.x, CV_8UC3, Scalar(255, 255, 255));
            vector<Point2i> frame_points = this->point_camera_proections[i];

            for (int i_points = 0; i_points < frame_points.size(); i_points++) {
                if (frame_points.size() == 0) continue;
                Point2i cross_size = Point2i(3, 3);
                Point2i s_point_location = Point2i(frame_points[i_points].y, frame_points[i_points].x) + this->frame_size/2;
                Point2i cross_points[4] = {
                    s_point_location + Point2i(-cross_size.x / 2, -cross_size.y / 2),
                    s_point_location + Point2i(-cross_size.x / 2, cross_size.y / 2),
                    s_point_location + Point2i(cross_size.x / 2, cross_size.y / 2),
                    s_point_location + Point2i(cross_size.x / 2, -cross_size.y / 2)
                };
                line(frame, cross_points[0], cross_points[2], Scalar(0, 0, 255), 1);
                line(frame, cross_points[1], cross_points[3], Scalar(0, 0, 255), 1);
                };
            //сохранить пикчу 
            string filename = "C:\\ProgStaff\\test_generated_images\\" + to_string(i) + ".jpg";
            imwrite(filename, frame);
        };
    };

    void print_states(string filename);
    void print_bins_gts(string filename);
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
