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

    void _load_csv_camera_proections_file(string filename, string sep = ";");
    void load_csv_camera_proections(string dirname, string sep = ";");
    void _load_csv_point_trail(string filename, int s_point_id, string sep = ";");
    void load_csv_point_trails(string dirname, string sep = ";");

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

    vector<Point3d> final_trajectory;
    vector<Point3d> final_trajectory_orient;

    vector<Point3d> final_trajectory_gt;
    vector<Point3d> final_trajectory_orient_gt;

    vector<vector<Point3d>> pose_err;
    vector<vector<Point3d>> orient_err;

    vector<Point3d> final_pose_err;
    vector<Point3d> final_orient_err;

    vector<Point2i> err_times;
    double score_pose_total;
    double score_orient_total;
    double score_pose_agg;
    double score_orient_agg;

    Point2d part_der_h(Point2d _P, Point3d _point_pose, Point3d _camera_pose, Point3d _camera_orient, Point3d _delta);
    Mat senseMat(Point2d _P, Point3d _point_pose, Point3d _camera_pose, Point3d _camera_orient);

    void generate_trail_sequences();
    void _generate_err_times_for_seq(Trail_sequence _gt, Trail_sequence _est);
    void generate_pose_err_for_seq(Trail_sequence _gt, Trail_sequence _est);
    void generate_orient_err_for_seq(Trail_sequence _gt, Trail_sequence _est);

    //void generate_gt() {
    //    for (Pose_type gt_point: this->bins_model.bins_gt_points) {
    //        this->final_trajectory_gt.push_back(gt_point.getPose());
    //        this->final_trajectory_orient_gt.push_back(gt_point.getOrient());
    //    };
    //};

    void generate_err();

    void save_scopes(string filename);
    void save_csv_err(std::string _dir, std::string _sep = ";");
    void save_csv_pose_err(std::string _dir, std::string _sep = ";");
    void save_csv_orient_err(std::string _dir, std::string _sep = ";");
    void save_csv_time_err(std::string _filename, std::string _sep = ";");

    void trail_sequences_estimate(Trail_sequence _trail_sequence, int _mode=0);

    void save_csv_trail_sequences(std::string _dir, std::string _sep = ";");

    void _load_csv_trail_sequence(string filename, string sep = ";");
    void load_csv_trail_sequences(string dirname, string sep = ";");

    void save_csv_state_estimated(std::string _dir, std::string _sep = ";");
    void save_csv_vector_Point3d(vector<Point3d> _vec, string _filename,  int _start=-1, int _end=-1, std::string _sep = ";");

public:
    Test_model(string name, string dir_name, string gen_restr_filename="");
    void generate_track_model();

    void load_trail_sequence_model() {
        this->load_csv_trail_sequences(this->dir_name + "trail_sequences\\");
    };

    void show_score() {
        cout << "Error score pose:\t" << this->score_pose_total << endl;
        cout << "Error score orient:\t" << this->score_orient_total << endl;
        cout << "Error score pose agg:\t" << this->score_pose_agg << endl;
        cout << "Error score orient agg:\t" << this->score_orient_agg << endl;
    };

    ///////////////////aaaAAAaaAA!1!!!1!!!!!
    void generate_test_model(string gen_restr_filename = "");// вот эта функция вызывается

    void load_test_model(string gen_restr_filename = "") {
        this->gen_restrictions.int_data["save_load_ts_model"] = 1;
        this->gen_restrictions.int_data["save_load_camera_model"] = 1;
        this->gen_restrictions.int_data["save_load_bins_model"] = 1;
        this->gen_restrictions.int_data["save_load_s_points"] = 1;
        this->gen_restrictions.int_data["save_load_motion_model"] = 1;
        this->gen_restrictions.int_data["save_load_track_model"] = 1;

        this->generate_test_model(gen_restr_filename);
    };// вот эта функция вызывается

    void Kalman_filter(int mode = 10) {

        this->states_estimated.clear();
        this->final_trajectory.clear();

        this->final_trajectory_gt.clear();
        this->final_trajectory_orient_gt.clear();

        this->final_trajectory.clear();
        this->final_trajectory_orient.clear();

        mode = (mode != 10) ? mode : this->gen_restrictions.int_data["kalman_mode"];

        std::cout << "\n----<<<< Kalman filter >>>>----" << std::endl;
        cout << "Test model:\t";
        cout << this->name << endl;

        string mode_name = "invalid mode";
        switch (mode) {
        case 0:
            mode_name = "filtered with noise";
            break;
        case 1:
            mode_name = "not filtered";
            break;
        case 2:
            mode_name = "no noise";
            break;
        };
        std::cout << "mode:\t" << mode_name << std::endl;


        int computing_size = 0;
        int current_progress = 0;
        for (Trail_sequence trail_sequence : trail_sequences) {
            computing_size += trail_sequence.timestamps.size();
        };
        cout << "0%";


        for (Trail_sequence trail_sequence : trail_sequences) {
            current_progress += trail_sequence.timestamps.size();
            cout << "\033[1K\r" << to_string(100 * (double)current_progress / (double)computing_size) + "%";
            this->trail_sequences_estimate(trail_sequence, mode);
        };

        cout << "\033[1K\r";
        //this->save_csv_state_estimated(this->dir_name + "states_estimated\\");

        this->aggrigate_estimated();
    };

    void save_final_trajectory(string filename) {
        this->save_csv_vector_Point3d(this->final_trajectory, this->dir_name + "gt" +filename);
    };

    void save_final_trajectory_gt(string filename) {
        this->save_csv_vector_Point3d(this->final_trajectory_gt, this->dir_name + filename);
    };

    void save_final_trajectory_errors(string filename) {
        this->save_csv_vector_Point3d(this->final_pose_err, this->dir_name + "e" + filename);
    };

    void aggrigate_estimated() {
        //найти минимальный/максимальный таймстемп
        int min_tmst = this->states_estimated[0].start;
        int max_tmst = this->states_estimated[0].end;
        for (Trail_sequence item : this->states_estimated) {
            min_tmst = item.start < min_tmst ? item.start : min_tmst;
            max_tmst = item.end > max_tmst ? item.end : max_tmst;
        };

        //сформировать наборы оцененных точек для каждого таймстемпа
        vector<int> tmst_range;
        for (int i = min_tmst; i <= max_tmst; i++) tmst_range.push_back(i);

        vector<vector<Point3d>> estimated_poses(tmst_range.size());
        vector<vector<Point3d>> estimated_orientations(tmst_range.size());

        for (Trail_sequence item : this->states_estimated) {
            for (int i : item.range()) {
                estimated_poses[i].push_back(item.state_vector[i].cam_pose);
                estimated_orientations[i].push_back(item.state_vector[i].cam_orient);
            };
        };

        vector<Point3d> gt_poses(tmst_range.size());
        vector<Point3d> gt_orient(tmst_range.size());

        for (Trail_sequence item : this->trail_sequences) {
            for (int i : item.range()) {
                gt_poses[i] = item.state_vector[i].cam_pose;
                gt_orient[i] = item.state_vector[i].cam_orient;
            };
        };

        this->final_trajectory_gt = gt_poses;
        this->final_trajectory_orient_gt = gt_orient;

        

        for (vector<Point3d> pose_est_frame : estimated_poses) {
            Point3d aggrigated_frame_point = Point3d(0, 0, 0);
            for (Point3d point : pose_est_frame) {
                aggrigated_frame_point += point / double(pose_est_frame.size());
            };
            this->final_trajectory.push_back(aggrigated_frame_point);
        };

        for (vector<Point3d> orient_est_frame : estimated_orientations) {
            Point3d aggrigated_frame_point = Point3d(0, 0, 0);
            for (Point3d point : orient_est_frame) {
                aggrigated_frame_point += point / double(orient_est_frame.size());
            };
            this->final_trajectory_orient.push_back(aggrigated_frame_point);
        }
    };

    void estimate_errors() {
        this->pose_err.clear();
        this->orient_err.clear();

        this->final_pose_err.clear();
        this->final_orient_err.clear();

        this->err_times.clear();
        this->score_pose_total = -1;
        this->score_orient_total = -1;
        this->score_pose_agg = -1;
        this->score_orient_agg = -1;

        this->generate_err();

        this->show_score();
        //this->save_scopes(this->dir_name + "scores.txt");
        //this->save_csv_err(this->dir_name + "errors\\");
    };

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