#pragma once
#include "Track_part_type.h"


using namespace cv;
using namespace std;

class Test_model {
private:
    string dir_name;
    string name;

    //static vector<Point2d> smooth_p2d(vector<Point2d> input_series, double amp_1, double amp_2, Point2d limiter) {
    static vector<Point2d> smooth_p2d() {
        vector<Point2d> result;

        //result.push_back(input_series[0]);
        //result.push_back(input_series[1]);
        //for (int i = 2; i < input_series.size(); i++) {
        //    
        //}
        return result;
    };

    //ограничения генерации
    struct Test_model_restrictions {
        string filename;
        int max_track_parts;

        double dicret;
        double min_line_length;
        double max_line_length;

        double mean_corner_radius;
        double stddev_radius;

        double mean_corner_angle;
        double stddev_angle;

        double average_vel;
        double stddev_vel;

        double T;
        double U1;
        double U2;

        double f;

        double extra_border;

        double z_lim_min;
        double z_lim_max;

        double grid_step_x;
        double grid_step_y;
        double grid_step_z;

        double grid_disp;

        Test_model_restrictions() {};
        void set(string filename, vector<int> int_data, vector<double> float_data);
        void load_restriction_file(string filename);
        void show();       
    } gen_restrictions;


    //геометрическая модель
    struct Test_track_model {
        vector<Track_part_type> track;
        vector<double> track_length;
        double total_length;

        Test_track_model() {};
        Point2d part(double dist);
        State_type orientation(double dist);
        void generate_track(Test_model_restrictions restr);

        void load_csv(string filename, string sep = ";");
        void save_csv(string filename, string sep = ";");
    } track_model;


    //модель движения
    class Test_motion_model {
    public:
        vector<Pose_type> gt_point;
        vector<Pose_type> old_gt_point;
        vector<Pose_type> eval_old_gt_point;
        vector<State_type> states;
        vector<double> timestamps;
        double total_time;

        void save_csv_gt_point(string filename, string sep = ";");
        void save_csv_old_gt_point(string filename, string sep = ";");
        void save_csv_eval_old_gt_point(string filename, string sep = ";");
        void save_csv_states(string filename, string sep = ";");
        void save_csv_timestamps(string filename, string sep = ";");

        void load_csv_gt_point(string filename, string sep = ";");
        void load_csv_old_gt_point(string filename, string sep = ";");
        void load_csv_eval_old_gt_point(string filename, string sep = ";");
        void load_csv_states(string filename, string sep = ";");
        void load_csv_timestamps(string filename, string sep = ";");

        void update_total_time();

        State_type get_state(int number);

        void generate_gt_points(Test_track_model track_model, double delta_m, int point_num = 0);
        void generate_states(Test_track_model track_model, double delta_m, int point_num = 0);
        void generate_timestamps(double delta_m, double vel);

        void smooth_anqular_vel(double T, double U1, double U2);
        void smooth_vel(double T, double U);

        void regenerate_gt_points();
        void integrate_old_gt();
    } motion_model;


    //модель камеры
    class Test_camera_model {
    
    
    };

    vector<Point3d> s_points;
    Point2i frame_size;
    Point2d cam_range;
    Mat A;
    //Mat EX_calib;

    void generate_s_points(double border, Point3d z_limits, Point3d grid_spacing, Point2d displacement) {};

    void generate_s_points();

    Mat generateExCalibM(int i);
    Mat generateTransitionM(int i);
    void setCameraModel(Point2i _frame_size, Point2d _cam_range, Mat _A) {
        this->frame_size = _frame_size;
        this->cam_range = _cam_range;
        this->A = _A;
    };

    //модель движения блестящих точек
    vector<vector<Point2i>> point_tracks;
    vector<vector<Point2i>> point_camera_proections;

    Point2i point_proection(Point3d point_pose, Point3d camera_pose, Mat Ex_calib);

    Point2i point_proection_linear(Point3d point_pose, Point3d camera_pose, Point3d cam_rotation) {

        double a = cam_rotation.x;
        double b = cam_rotation.y;
        double c = cam_rotation.z;

        double rotMat_ar[4][4] = {
            {cos(c) * cos(b),                               -cos(b) * sin(c),                               sin(b),              0},
            {cos(a) * sin(c) + cos(c) * sin(b) * sin(a),    cos(c) * cos(a) - sin(a) * sin(b) * sin(c),     -cos(b) * sin(a),    0},
            {sin(c) * sin(a) - cos(c) * cos(a) * sin(b),    cos(a) * sin(b) * sin(c) + cos(c) * sin(a),     cos(b) * cos(a),     0},
            {0, 0, 0, 1}
        };
        Mat R = Mat(4, 4, CV_64F, rotMat_ar);

        double Translation_ar[4][1] = {
            {camera_pose.x},
            {camera_pose.y},
            {camera_pose.z},
            {0}
        };

        Mat Translation = Mat(4, 1, CV_64F, Translation_ar);

        double Camera_ar[4][1] = {
            {camera_pose.x},
            {camera_pose.y},
            {camera_pose.z},
            {1}
        };

        Mat Camera = Mat(4, 1, CV_64F, Camera_ar);

        double Point_ar[4][1] = {
            {point_pose.x},
            {point_pose.y},
            {point_pose.z},
            {1}
        };
        
        Mat Point = Mat(4, 1, CV_64F, Point_ar);
        Mat Delta = Point - Camera;

        Mat Result = (Point - Camera) * R;

        Point2d aaaaaaa = Point2d(Result.at<double>(0, 0), Result.at<double>(1, 0));

    
    };
    void generate_camera_proections();

    //модель БИНС
    class Test_bins_model {
    public:
        vector<Pose_type> bins_gt_points;
        vector<Pose_type> bins_points;
        vector<Pose_type> bins_eval_points;
        vector<double> bins_timestamps;

        void generate_bins_gt_points(Test_motion_model mothion_model, double bins_deltatime);

        void save_csv_bins_gt_points(string filename, string sep = ";");
        void save_csv_bins_timestamps(string filename, string sep = ";");

        void load_csv_bins_gt_points(string filename, string sep = ";");
        void load_csv_bins_timestamps(string filename, string sep = ";");
        
    } bins_model;


public:
    Test_model(string name, string dir_name) {
        this->name = name;
        this->dir_name = dir_name;
    };

    void generate_track_model() {
        this->track_model.generate_track(this->gen_restrictions);
        this->track_model.save_csv(this->dir_name + "track.csv");
    };

    //void read_restriction_file(string filename);

    void save_test_model(string filename) {

        //vector<Pose_type> gt_point;

        //  cv::Point3d vel;
        //  cv::Point3d accel;
        //  cv::Point3d orient;
        //  cv::Point3d anqular_vel;
        //  cv::Point3d anqular_accel;
        //vector<double> timestamps;
        //double total_time;

        //vector<Point3d> s_points;
        //Point2i frame_size;
        //Point2d cam_range;
        //Mat A;

        //модель движения блестящих точек
        //vector<vector<Point2i>> point_tracks;
        //vector<vector<Point2i>> point_camera_proections;

        //модель БИНС
        //vector<Pose_type> bins_gt_points;
        //vector<Pose_type> bins_points;
        //vector<double> bins_timestamps;
    };

    void generate_test_model(vector<bool> options, string gen_restr_filename = "");// вот эта функция вызывается


    void show_gt(string mode = "screen", bool pause_enable = false);
    void show_bins_gt(bool pause_enable = false);

    void print_camera_proections() {

        string dir_frames = this->dir_name + "frames\\";
        string cmd_clear_image_dir = "del /f /s /q " + dir_frames;
        system(cmd_clear_image_dir.c_str());

        for (int i = 0; i < this->bins_model.bins_timestamps.size()-1; i++) {
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
            string filename = dir_frames + to_string(i) + ".jpg";
            imwrite(filename, frame);
        };

        string cmd_make_vid = "ffmpeg -start_number 0 -y -i " + dir_frames + "%d.jpg -vcodec mpeg4 "+this->dir_name + this->name +".mp4";
        system(cmd_make_vid.c_str());

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
