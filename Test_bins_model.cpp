#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;
using namespace std;

#include "Types.h"
#include "Tests.h"
#include "Test_math.h"

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