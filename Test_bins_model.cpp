#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

//using namespace cv;
using namespace std;

typedef cv::Point2d Point2d;
typedef cv::Point3d Point3d;
typedef cv::Point2i Point2i;

#include "Types.h"
#include "Tests.h"
#include "Test_math.h"

void  Test_model::generate_trail_sequences() {
    //for (vector<Point2d> trail : this->point_trails)

    for (vector<Point2d> trail : this->point_trails) {
        //vector<Point2d> trail = this->point_trails[0];
        Point3d s_point = this->s_points[int(trail[0].x)];

        int i = 1;
        bool seq_valid = false;
        bool seq_broken = false;
        Trail_sequence tmp_seq;

        while (i < trail.size()) {
            Point2d trail_point = trail[i];
            seq_broken = (trail_point.x == -999.0 || trail_point.y == -999.0) ? true : false;

            if (seq_valid && seq_broken) {
                //сохранить
                this->trail_sequences.push_back(tmp_seq);
                tmp_seq = Trail_sequence();
                //очистить tmp_seq
                seq_valid = false;
                seq_broken = false;
            }
            else if (seq_valid && !seq_broken) {
                //продолжить записывать tmp_seq
                State_vector st_vec;
                st_vec.set_cam_pose(this->bins_model.bins_gt_points[i].getPose());
                st_vec.set_orient(this->bins_model.bins_gt_points[i].getOrient());
                st_vec.set_cam_vel(this->bins_model.bins_measured_states[i].get_vel());
                st_vec.set_s_pose(s_point);

                Measurement_vector ms_vec(trail[i]);

                Control_vector ctrl_vec;
                ctrl_vec.set_accel(this->bins_model.bins_measured_states[i].get_accel());
                ctrl_vec.set_w(this->bins_model.bins_measured_states[i].get_anqular_accel());

                double timestamp = this->bins_model.bins_timestamps[i];
                tmp_seq.push_back(timestamp, st_vec, ms_vec, ctrl_vec);
            }
            else if (!seq_valid && !seq_broken) {
                //начать записывать новый tmp_seq
                int start = i - 1;
                State_vector st_vec;
                st_vec.set_cam_pose(this->bins_model.bins_gt_points[i].getPose());
                st_vec.set_orient(this->bins_model.bins_gt_points[i].getOrient());
                st_vec.set_cam_vel(this->bins_model.bins_measured_states[i].get_vel());
                st_vec.set_s_pose(s_point);

                Measurement_vector ms_vec(trail[i]);

                Control_vector ctrl_vec;
                ctrl_vec.set_accel(this->bins_model.bins_measured_states[i].get_accel());
                ctrl_vec.set_w(this->bins_model.bins_measured_states[i].get_anqular_accel());

                double timestamp = this->bins_model.bins_timestamps[i];
                tmp_seq = Trail_sequence(start, timestamp, st_vec, ms_vec, ctrl_vec);
                seq_valid = true;
                seq_broken = false;
            };

            i++;
        }; // while (i < trail.size())

        //если путь был до самого конца
        if (seq_valid && seq_broken) {
            //сохранить
            this->trail_sequences.push_back(tmp_seq);
            tmp_seq = Trail_sequence();
            //очистить tmp_seq
            seq_valid = false;
            seq_broken = false;
        }

    }; //for (vector<Point2d> trail : this->point_trails)
};

void Test_model::Test_bins_model::generate_bins_measured_states(Test_motion_model motion_model, double bins_deltatime) {

    //начальное то же самое
    this->bins_measured_states.push_back(
        State_type(
            motion_model.states[0].get_vel(),
            motion_model.states[0].get_accel(),
            motion_model.states[0].get_orient(),
            motion_model.states[0].get_anqular_vel(),
            motion_model.states[0].get_anqular_accel()
        )
    );
    vector<double> vec_i;

    for (int i = 1; i < this->bins_timestamps.size(); i++) {
        State_type tmp;
        double bins_time = bins_deltatime * i;
        int lesser_ts_i = 0;
        double interp_multi = 0;
        for (int ts_i = 0; ts_i < motion_model.timestamps.size() - 1; ts_i++) {
            if ((motion_model.timestamps[ts_i] <= bins_time) && (motion_model.timestamps[ts_i + 1] >= bins_time)) {
                interp_multi = abs((bins_time - motion_model.timestamps[ts_i]) / (motion_model.timestamps[ts_i + 1] - motion_model.timestamps[ts_i]));
                //interp_multi = abs((bins_time - this->timestaps[ts_i]) / (this->timestaps[ts_i + 1] - bins_time));
                vec_i.push_back(interp_multi);
                lesser_ts_i = ts_i;
                break;
            };
        };

        State_type prev_state = motion_model.states[lesser_ts_i];
        State_type next_state = motion_model.states[lesser_ts_i + 1];

        //интреполировать на них позиции, ориентации, ускорения, угловые скорости


        Point3d vel_curr = prev_state.vel + (next_state.vel - prev_state.vel) * interp_multi;
        Point3d accel_curr = prev_state.accel + (next_state.accel - prev_state.accel) * interp_multi;
        Point3d anqular_vel_curr = prev_state.anqular_vel + (next_state.anqular_vel - prev_state.anqular_vel) * interp_multi;

        //Point3d w_curr = prev.getW() * (1 - interp_multi) + next.getW();
        //Point3d accel_curr = prev.getAccel() * (1 - interp_multi) + next.getAccel();
        Point3d ang_vec = -(prev_state.get_orient() + (next_state.get_orient() - prev_state.get_orient()) * interp_multi);
        Point3d e1 = rotateP3d(Point3d(1, 0, 0), ang_vec);
        Point3d e2 = rotateP3d(Point3d(0, 1, 0), ang_vec);
        Point3d e3 = rotateP3d(Point3d(0, 0, 1), ang_vec);

        double newBase_ar[3][3] = {
        {e1.x, e2.x, e3.x},
        {e1.y, e2.y, e3.y},
        {e1.z, e2.z, e3.z}
        };
        Mat newBase = Mat(3, 3, CV_64F, newBase_ar);

        double accel_vec_ar[3][1] = {
            {accel_curr.x},
            {accel_curr.y},
            {accel_curr.z}
        };

        double vel_vec_ar[3][1] = {
            {vel_curr.x},
            {vel_curr.y},
            {vel_curr.z}
        };

        Mat accel_vec = Mat(3, 1, CV_64F, accel_vec_ar);
        Mat vel_vec = Mat(3, 1, CV_64F, vel_vec_ar);

        Mat accel_pr = newBase.inv() * accel_vec;
        Mat vel_pr = newBase.inv() * vel_vec;
        accel_curr = Point3d(accel_pr.at<double>(0, 0), accel_pr.at<double>(1, 0), accel_pr.at<double>(2, 0));
        vel_curr = Point3d(vel_pr.at<double>(0, 0), vel_pr.at<double>(1, 0), vel_pr.at<double>(2, 0));

        tmp.change_vel(anqular_vel_curr);
        tmp.change_accel(accel_curr);
        tmp.change_orient(ang_vec);
        tmp.change_anqular_vel(anqular_vel_curr);
        tmp.change_anqular_accel(Point3d(0.0, 0.0, 0.0));

        this->bins_measured_states.push_back(tmp);
    };
};


void Test_model::Test_bins_model::generate_bins_gt_points(Test_motion_model motion_model, double bins_deltatime) {

    double bins_total_time = 0.0;
    this->bins_timestamps.push_back(0.0);

    while (bins_total_time < (motion_model.total_time - bins_deltatime)) {
        this->bins_timestamps.push_back(this->bins_timestamps[this->bins_timestamps.size() - 1] + bins_deltatime);
        bins_total_time += bins_deltatime;
    };
    //сгенерировать таймстемпы

    //начальное то же самое
    this->bins_gt_points.push_back(Pose_type(motion_model.gt_point[0].getPose(), motion_model.gt_point[0].getOrient(), motion_model.gt_point[0].getAccel(), motion_model.gt_point[0].getW()));
    vector<double> vec_i;

    for (int i = 1; i < this->bins_timestamps.size(); i++) {
        Pose_type tmp;
        double bins_time = bins_deltatime * i;
        int lesser_ts_i = 0;
        double interp_multi = 0;
        for (int ts_i = 0; ts_i < motion_model.timestamps.size() - 1; ts_i++) {
            if ((motion_model.timestamps[ts_i] <= bins_time) && (motion_model.timestamps[ts_i + 1] >= bins_time)) {
                interp_multi = abs((bins_time - motion_model.timestamps[ts_i]) / (motion_model.timestamps[ts_i + 1] - motion_model.timestamps[ts_i]));
                //interp_multi = abs((bins_time - this->timestaps[ts_i]) / (this->timestaps[ts_i + 1] - bins_time));
                vec_i.push_back(interp_multi);
                lesser_ts_i = ts_i;
                break;
            };
        };
        Pose_type prev_pose = motion_model.gt_point[lesser_ts_i];
        Pose_type next_pose = motion_model.gt_point[lesser_ts_i + 1];

        State_type prev_state = motion_model.states[lesser_ts_i];
        State_type next_state = motion_model.states[lesser_ts_i + 1];

        //интреполировать на них позиции, ориентации, ускорения, угловые скорости
        tmp.setPose(prev_pose.getPose() + (next_pose.getPose() - prev_pose.getPose()) * interp_multi);
        tmp.setOrient(prev_pose.getOrient() + (next_pose.getOrient() - prev_pose.getOrient()) * interp_multi);
    
        Point3d w_curr = prev_state.anqular_vel + (next_state.anqular_vel - prev_state.anqular_vel) * interp_multi;
        Point3d accel_curr = prev_state.accel + (next_state.accel - prev_state.accel) * interp_multi;
        //Point3d w_curr = prev.getW() * (1 - interp_multi) + next.getW();
        //Point3d accel_curr = prev.getAccel() * (1 - interp_multi) + next.getAccel();
        Point3d ang_vec = -tmp.getOrient();
        Point3d e1 = rotateP3d(Point3d(1, 0, 0), ang_vec);
        Point3d e2 = rotateP3d(Point3d(0, 1, 0), ang_vec);
        Point3d e3 = rotateP3d(Point3d(0, 0, 1), ang_vec);

        double newBase_ar[3][3] = {
        {e1.x, e2.x, e3.x},
        {e1.y, e2.y, e3.y},
        {e1.z, e2.z, e3.z}
        };
        Mat newBase = Mat(3, 3, CV_64F, newBase_ar);

        double accel_vec_ar[3][1] = {
            {accel_curr.x},
            {accel_curr.y},
            {accel_curr.z}
        };

        Mat accel_vec = Mat(3, 1, CV_64F, accel_vec_ar);

        Mat accel_pr = newBase.inv() * accel_vec;
        accel_curr = Point3d(accel_pr.at<double>(0, 0), accel_pr.at<double>(1, 0), accel_pr.at<double>(2, 0));

        tmp.setAccel(accel_curr);
        tmp.setW(w_curr);
        this->bins_gt_points.push_back(tmp);
    };
};

void Test_model::Test_bins_model::save_csv_bins_measured_states(string filename, string sep) {
    vector<string> csv_data;
    cout << "save_csv_bins_measured_states" << endl;
    for (int i = 0; i < this->bins_measured_states.size(); i++) {
        csv_data.push_back(this->bins_measured_states[i].get_csv_data(sep));
    };

    ofstream fout(filename);
    fout << State_type_HEADER(sep) << '\n';

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

void Test_model::Test_bins_model::save_csv_bins_gt_points(string filename, string sep) {
    vector<string> csv_data;
    cout << "save_csv_bins_gt_points" << endl;
    for (int i = 0; i < this->bins_gt_points.size(); i++) {
        csv_data.push_back(this->bins_gt_points[i].get_csv_data(sep));
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

void Test_model::Test_bins_model::load_csv_bins_gt_points(string filename, string sep) {
    ifstream fin(filename);
    char buffer[255];
    fin.getline(buffer, 255);

    cout << "load_csv_bins_gt_points" << endl;

    vector<string> line_buffer;
    while (fin.getline(buffer, 255)) {
        line_buffer.push_back(buffer);
        //cout << "_" << buffer << "_" << endl;

        size_t pos = 0;
        vector<string> values;
        string line(buffer);

        Pose_type tmp;
        tmp.read_csv(line);
        this->bins_gt_points.push_back(tmp);
    };
    fin.close();
};

void Test_model::Test_bins_model::load_csv_bins_measured_states(string filename, string sep) {
    ifstream fin(filename);
    char buffer[255];
    fin.getline(buffer, 255);

    cout << "load_csv_bins_measured_states" << endl;

    vector<string> line_buffer;
    while (fin.getline(buffer, 255)) {
        line_buffer.push_back(buffer);
        //cout << "_" << buffer << "_" << endl;

        size_t pos = 0;
        vector<string> values;
        string line(buffer);

        State_type tmp;
        tmp.read_csv(line);
        this->bins_measured_states.push_back(tmp);
    };
    fin.close();
};

void Test_model::Test_bins_model::save_csv_bins_timestamps(string filename, string sep) {
    vector<string> csv_data;
    cout << "save_csv_bins_timestamps" << endl;
    for (int i = 0; i < this->bins_timestamps.size(); i++) {
        csv_data.push_back(to_string(this->bins_timestamps[i]));
    };

    ofstream fout(filename);
    fout << "timest" << '\n';

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

void Test_model::Test_bins_model::load_csv_bins_timestamps(string filename, string sep) {
    ifstream fin(filename);
    char buffer[255];
    fin.getline(buffer, 255);

    cout << "load_csv_bins_timestamps" << endl;

    vector<string> line_buffer;
    while (fin.getline(buffer, 255)) {
        line_buffer.push_back(buffer);
        //cout << "_" << buffer << "_" << endl;

        string line(buffer);
        line.replace(line.find(","), 1, ".");
        this->bins_timestamps.push_back(std::stod(line));
    };
    fin.close();
};