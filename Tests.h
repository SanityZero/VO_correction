#pragma once
#include "Track_part_type.h"
#include "Test_model_sequence.h"

#include <omp.h>

#include <map>



using namespace cv;
using namespace std;

class Test_model {
private:
    string dir_name;
    string name;

    //static vector<Point2d> smooth_p2d(vector<Point2d> input_series, double amp_1, double amp_2, Point2d limiter) {
    static vector<Point2d> smooth_p2d() {
        vector<Point2d> result;
        return result;
    };

    string get_csv_Point3d(Point3d _point, string _sep = ";");

    Point3d read_csv_Point3d(string _string, string _sep = ";");
    Point2d read_csv_Point2d(string _string, string _sep = ";");
    Point2i read_csv_Point2i(string _string, string _sep = ";");
    Mat load_csv_Mat(string filename, Point2i _size, string _sep = ";");

    //ограничения генерации
    struct Test_model_restrictions {
        string filename;

        map <string, int> int_data;
        map <string, double> double_data;

        Test_model_restrictions() {};

        void set(string _filename, vector<int> _int_data, vector<double> _float_data);
        void load_restriction_file(string filename);
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

        void load_csv_track(string filename, string sep = ";");
        void save_csv_track(string filename, string sep = ";");
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

        void smooth_anqular_vel_states(double T, double U1, double U2);
        void smooth_vel_accel_states(double T, double U);

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

    void generate_s_points(int mode);
    void save_csv_s_points(string filename, string sep = ";");
    void load_csv_s_points(string filename, string sep = ";");
    

    Mat cameraCSMat(Point3d angles, Point3d _cam_pose);
    Mat generateTransitionM(int i);

    void setCameraModel(Test_model_restrictions _gen_restrictions);

    //модель движения блестящих точек
    vector<vector<Point2d>> point_trails;
    vector<vector<Point2i>> point_camera_proections;

    Point2d point_proection_D(Point3d point_pose, Point3d camera_pose, Point3d camera_orient);

    Point2i point_proection(Point3d point_pose, Point3d camera_pose, Point3d camera_orient) {
        Point2d point_d = this->point_proection_D(point_pose, camera_pose, camera_orient);
        return Point2i(point_d.x, point_d.y);
    };

    void generate_camera_proections(int mode);
    void generate_point_trails(int mode);

    void _load_csv_camera_proections_file(string filename, string sep = ";") {
        ifstream fin(filename);
        char buffer[255];
        vector<Point2i> frame_points;
        fin.getline(buffer, 255);

        vector<string> line_buffer;
        while (fin.getline(buffer, 255)) {
            line_buffer.push_back(buffer);
            string line(buffer);

            Point2i tmp = read_csv_Point2i(line, sep);
            frame_points.push_back(tmp);
        };
        fin.close();
        this->point_camera_proections.push_back(frame_points);
    };

    void load_csv_camera_proections(string dirname, string sep = ";"){
        cout << "load_csv_camera_proections" << endl;
        
        //string dirname = "C:\\ProgStaff\\NIRS_models\\test1\\proections\\";
        int i = 0;
        for (;;i++) {
            string filename = dirname + to_string(i) + ".csv";
            ifstream fin(filename);
            if (fin.fail()) {
                //cout << filename << "\tfile dont exist" << endl;
                break;
            }
            else {
                this->_load_csv_camera_proections_file(filename, sep);
            }
            fin.close();
        };
    };

    void _load_csv_point_trail(string filename, int s_point_id, string sep = ";") {
        ifstream fin(filename);
        char buffer[255];
        vector<Point2d> trail_points;
        fin.getline(buffer, 255);
        trail_points.push_back(Point2d(s_point_id, 0));

        vector<string> line_buffer;
        while (fin.getline(buffer, 255)) {
            line_buffer.push_back(buffer);
            string line(buffer);

            Point2d tmp = read_csv_Point2d(line, sep);
            trail_points.push_back(tmp);
        };
        fin.close();
        this->point_trails.push_back(trail_points);
    };

    void load_csv_point_trails(string dirname, string sep = ";") {
        cout << "load_csv_point_trails" << endl;

        //string dirname = "C:\\ProgStaff\\NIRS_models\\test1\\proections\\";
        int i = 0;
        for (;; i++) {
            string filename = dirname + to_string(i) + ".csv";
            ifstream fin(filename);
            if (fin.fail()) {
                //cout << filename << "\tfile dont exist" << endl;
                break;
            }
            else {
                this->_load_csv_point_trail(filename, i, sep);
            }
            fin.close();
        };
    };

    void save_csv_camera_proections(string dirname, string sep = ";");
    void save_csv_point_trails(string dirname, string sep = ";");

    //модель БИНС
    class Test_bins_model {
    public:
        vector<Pose_type> bins_gt_points;
        vector<Pose_type> bins_points;
        vector<Pose_type> bins_eval_points;
        vector<State_type> bins_measured_states;
        vector<double> bins_timestamps;

        void generate_bins_gt_points(Test_motion_model mothion_model, double bins_deltatime);
        void generate_bins_measured_states(Test_motion_model motion_model, double bins_deltatime);

        void save_csv_bins_gt_points(string filename, string sep = ";");
        void save_csv_bins_measured_states(string filename, string sep = ";");
        void save_csv_bins_timestamps(string filename, string sep = ";");

        void load_csv_bins_gt_points(string filename, string sep = ";");
        void load_csv_bins_measured_states(string filename, string sep = ";");
        void load_csv_bins_timestamps(string filename, string sep = ";");

    } bins_model;

    //обработка 
    vector<Point2d> s_sequence;
    vector<Trail_sequence> trail_sequences;
    vector<Trail_sequence> states_estimated;

    vector<vector<Point3d>> pose_err;
    vector<vector<Point3d>> orient_err;
    vector<Point2i> err_times;
    double score_pose;
    double score_orient;

    Point2d part_der_h(Point2d _P, Point3d _point_pose, Point3d _camera_pose, Point3d _camera_orient, Point3d _delta);
    Mat senseMat(Point2d _P, Point3d _point_pose, Point3d _camera_pose, Point3d _camera_orient);

    void generate_trail_sequences();
    void _generate_err_times_for_seq(Trail_sequence _gt, Trail_sequence _est);
    void generate_pose_err_for_seq(Trail_sequence _gt, Trail_sequence _est);
    void generate_orient_err_for_seq(Trail_sequence _gt, Trail_sequence _est);

    void generate_err() {
        for (int i = 0; i < trail_sequences.size(); i++) {
            Trail_sequence gt = this->trail_sequences[i];
            Trail_sequence est = this->states_estimated[i];

            this->_generate_err_times_for_seq(gt, est);
            this->generate_pose_err_for_seq(gt, est);
            this->generate_orient_err_for_seq(gt, est);
        };

        double data_size = 0;

        for (vector<Point3d> vec : this->pose_err) data_size += vec.size();

        double tmp = 0;

        for (vector<Point3d> seq_err : this->pose_err) {
            for (Point3d point : seq_err) {
                tmp += sqrt(point.x * point.x + point.y * point.y)/ data_size;
                //tmp += sqrt(point.x * point.x + point.y * point.y + point.z * point.z)/ data_size;
            };
        };

        this->score_pose = tmp;


        tmp = 0;

        for (vector<Point3d> seq_err : this->orient_err) {
            for (Point3d point : seq_err) {
                //tmp += sqrt(point.x * point.x + point.y * point.y + point.z * point.z)/ data_size;
                tmp += sqrt(point.z * point.z)/ data_size;
            };
        };

        this->score_orient = tmp;
    };

    void save_scopes(string filename);
    void save_csv_err(std::string _dir, std::string _sep = ";");
    void save_csv_pose_err(std::string _dir, std::string _sep = ";");
    void save_csv_orient_err(std::string _dir, std::string _sep = ";");
    void save_csv_time_err(std::string _filename, std::string _sep = ";");

    void trail_sequences_estimate(Trail_sequence _trail_sequence, int _mode=0);

    void Kalman_filter(int _mode) {
        cout << "Kalman_filter" << endl;

        int computing_size = 0;
        int current_progress = 0;
        for (Trail_sequence trail_sequence : trail_sequences) {
            computing_size += trail_sequence.timestamps.size();
        };
        cout << "0%";
        //_trail_sequence.timestamps
        //cout << "asf" << "\033[1K\r" << "12" << "\033[1K\r" << to_string(12) + "%";
        for (Trail_sequence trail_sequence : trail_sequences) {
            current_progress += trail_sequence.timestamps.size();
            cout << "\033[1K\r" << to_string(100 * (double)current_progress/ (double)computing_size) + "%";
            this->trail_sequences_estimate(trail_sequence, _mode);
        };

        cout << "\033[1K\r";
    };

    
    void save_csv_trail_sequences(std::string _dir, std::string _sep = ";");

    void _load_csv_trail_sequence(string filename, string sep = ";") {\
    //    int start;
    //int end;

    //vector<double> timestamps;
    //vector<State_vector> model_state_vector;
    //vector<Measurement_vector> model_measurement_vector;
    //vector<Control_vector> model_control_vector;
        //Trail_sequence
        ifstream fin(filename);
        char buffer[255];
        //vector<Point2d> trail_points;
        fin.getline(buffer, 255);
        Trail_sequence new_ts = Trail_sequence();
        //trail_points.push_back(Point2d(s_point_id, 0));

        //vector<string> line_buffer;
        //while (fin.getline(buffer, 255)) {
        //    line_buffer.push_back(buffer);
        //    string line(buffer);

        //    Point2d tmp = read_csv_Point2d(line, sep);
        //    trail_points.push_back(tmp);
        //};
        //fin.close();
        //this->point_trails.push_back(trail_points);
    };

    void load_csv_trail_sequences(string dirname, string sep = ";") {
        //cout << "load_csv_point_trails" << endl;

        ////string dirname = "C:\\ProgStaff\\NIRS_models\\test1\\proections\\";
        //int i = 0;
        //for (;; i++) {
        //    string filename = dirname + to_string(i) + ".csv";
        //    ifstream fin(filename);
        //    if (fin.fail()) {
        //        //cout << filename << "\tfile dont exist" << endl;
        //        break;
        //    }
        //    else {
        //        this->_load_csv_point_trail(filename, i, sep);
        //    }
        //    fin.close();
        //};
    };

    void save_csv_state_estimated(std::string _dir, std::string _sep = ";");
    void save_csv_vector_Point3d(vector<Point3d> _vec, string _filename,  int _start, int _end, std::string _sep = ";");

public:
    Test_model(string name, string dir_name);
    void generate_track_model();
    void save_test_model(string filename) {};

    void show_score() {
        cout << "Error score pose:\t" << this->score_pose << endl;
        cout << "Error score orient:\t" << this->score_orient << endl;
    };

    ///////////////////aaaAAAaaAA!1!!!1!!!!!
    void generate_test_model(string gen_restr_filename = "");// вот эта функция вызывается

    void show_gt(string mode = "screen", bool pause_enable = false);
    void show_bins_gt(bool pause_enable = false);

    void print_camera_proections();
};

DataSeq_model_Type generate_old_model(
    double mean_1, const int size, double stddev,
    Point3d w,  Point3d ang_0, double deltatime,
    double accel_stddev, Point3d accel, Point3d vel_0
);
void old_motion_Test(double accel_std = 1, double sko = 0.2, double delta = 0.004, double duration = 10);
void old_angle_Test(double w_std = 0.0001 * M_PI/180, Point3d vel_0 = Point3d(0,0,0), double sko = 0.000001 * M_PI/180, double delta = 0.004, double duration = 60);