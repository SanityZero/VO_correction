#pragma once
#include "Track_part_type.h"

class Test_model {
private:
    string dir_name;
    string name;

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

        void save_csv_gt_point(string filename, string sep = ";") {
            vector<string> csv_data;
            cout << "save_csv_gt_point" << endl;
            for (int i = 0; i < gt_point.size(); i++) {
                csv_data.push_back(gt_point[i].get_csv_data(sep));
            };

            ofstream fout(filename);
            fout << Pose_type_HEADER(sep) << '\n';

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

        void save_csv_old_gt_point(string filename, string sep = ";") {};
        void save_csv_eval_old_gt_pointt(string filename, string sep = ";") {};
        void save_csv_states(string filename, string sep = ";") {};
        void save_csv_timestamps(string filename, string sep = ";") {};

        void load_csv_gt_point(string filename, string sep = ";") {
            ifstream fin(filename);
            char buffer[255];
            fin.getline(buffer, 255);

            vector<string> line_buffer;
            while (fin.getline(buffer, 255)) {
                line_buffer.push_back(buffer);
                //cout << "_" << buffer << "_" << endl;

                size_t pos = 0;
                vector<string> values;
                string line(buffer);
                while ((pos = line.find(sep)) != std::string::npos) {
                    values.push_back(line.substr(0, pos));
                    //std::cout << values << std::endl;
                    line.erase(0, pos + sep.length());
                };
                values.push_back(line);

                vector<double> double_buffer;
                for (string item : values) {
                    item.replace(item.find(","), 1, ".");
                    Pose_type tmp;
                    tmp.read_csv(item);
                    this->gt_point.push_back(tmp);
                    //cout << "_" << stod(item) << "_" << endl;
                }
            };
            fin.close();
        };

        void load_csv_old_gt_point(string filename, string sep = ";") {};
        void load_csv_eval_old_gt_pointt(string filename, string sep = ";") {};
        void load_csv_states(string filename, string sep = ";") {};
        void load_csv_timestamps(string filename, string sep = ";") {};
        

        State_type get_state(int number);
        void generate_gt_points(Test_track_model track_model, double delta_m, int point_num = 0);
        void generate_states(Test_track_model track_model, double delta_m, int point_num = 0);
        void generate_timestaps(double delta_m, double vel);
        void smooth_anqular_vel(double T, double U1, double U2);
        void smooth_vel(double T, double U);
        void regenerate_gt_points();
        void integrate_old_gt();
    } motion_model;


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
    vector<Pose_type> bins_eval_points;
    vector<double> bins_timestamps;

    void generate_bins_gt(double bins_deltatime);

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

    void generate_test_model(string gen_restr_filename = "");


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
