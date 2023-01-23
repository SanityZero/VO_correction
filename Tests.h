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

        double focus;

        double extra_border;

        double z_lim_min;
        double z_lim_max;

        double grid_step_x;
        double grid_step_y;
        double grid_step_z;

        double grid_disp;

        int camera_matrix_x;
        int camera_matrix_y;
        int camera_frame_size_x;
        int camera_frame_size_y;

        double camera_FOV_xoy;
        double camera_FOV_zoy;

        double camera_fitting_x;
        double camera_fitting_y;
        double camera_fitting_z;


        int s_points_generation_mode;
        int camera_proection_mode;

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

    string get_csv_data_point3d(Point3d point, string sep = ";") {
        string result = to_string(point.x) + sep + to_string(point.y) + sep + to_string(point.z);
        return result;
    };

    void save_csv_s_points(string filename, string sep = ";");

    void generate_s_points(int mode);

    Mat cameraCSMat(Point3d angles, Point3d _cam_pose);
    Mat generateTransitionM(int i);
    void setCameraModel(Point2i _frame_size, Point2d _cam_range, Mat _A) {
        this->frame_size = _frame_size;
        this->cam_range = _cam_range;
        this->A = _A;
    };

    //модель движения блестящих точек
    vector<vector<Point2i>> point_trails;
    vector<vector<Point2i>> point_camera_proections;

    Point2i point_proection(Point3d point_pose, Point3d camera_pose, int track_point);

    Point2i point_proection_linear(Point3d point_pose, Point3d camera_pose, Point3d cam_rotation);

    void generate_camera_proections(int mode);

    void generate_point_trails(int mode) {

        for (int i_points = 0; i_points < this->s_points.size(); i_points++) {

            vector<Point2i> s_point_trail;
            s_point_trail.push_back(Point2i(i_points, 0));

            for (int i = 0; i < this->bins_model.bins_timestamps.size() - 1; i++) {
                //Mat ex_calib_m = cameraCSMat(this->bins_model.bins_gt_points[i].getOrient(), );
                Point3d cam_pose = this->bins_model.bins_gt_points[i].getPose();

                Point2i tmp = Point2i(this->frame_size.x, this->frame_size.y);
                switch (mode) {
                    case 1:
                        tmp = point_proection(this->s_points[i_points], cam_pose, i);
                        break;
                    case 0:
                        tmp = point_proection_linear(this->s_points[i_points], cam_pose, this->bins_model.bins_gt_points[i].getOrient());
                        break;
                };

                if ((tmp.x == this->frame_size.x) && (tmp.y == this->frame_size.y)) tmp = Point2i(-999, -999);
                s_point_trail.push_back(tmp);
            };

            this->point_trails.push_back(s_point_trail);
            s_point_trail.clear();
        };
    };

    void save_csv_point_trails(string dirname, string sep = ";") {

        for (vector<Point2i> trail : this->point_trails) {
            vector<string> csv_data;
            cout << "save_csv_point_trails" << endl;
            for (int i = 1; i < trail.size(); i++) {
                csv_data.push_back(to_string(trail[i].x) + sep + to_string(trail[i].y));
            };

            string filename = dirname + "trails\\" + to_string(trail[0].x) + ".csv";
            //cout << filename << endl;
            ofstream fout(filename);
            fout << "x" + sep + "y" << '\n';

            for (string row : csv_data) {
                size_t start_pos = 0;
                string from = ".";
                string to = ",";
                while ((start_pos = row.find(from, start_pos)) != std::string::npos) {
                    row.replace(start_pos, from.length(), to);
                    start_pos += to.length();
                }
                fout << row << '\n';
            };

            fout.close();
        };
       
    };

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
    };


    ///////////////////aaaAAAaaAA!1!!!1!!!!!
    void generate_test_model(
        vector<bool> options,
        string gen_restr_filename = ""
    );// вот эта функция вызывается


    void show_gt(string mode = "screen", bool pause_enable = false);
    void show_bins_gt(bool pause_enable = false);

    void print_camera_proections();
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
