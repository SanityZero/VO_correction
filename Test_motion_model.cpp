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
#include <omp.h>
//#pragma omp


void Test_model::Test_motion_model::update_total_time() {
    this->total_time = this->timestamps[this->timestamps.size() - 1];
};

void Test_model::Test_motion_model::save_csv_gt_point(string filename, string sep) {
    vector<string> csv_data;
    cout << "save_csv_gt_point" << endl;
    for (int i = 0; i < this->gt_point.size(); i++) {
        csv_data.push_back(this->gt_point[i].get_csv_line(sep));
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

void Test_model::Test_motion_model::save_csv_old_gt_point(string filename, string sep) {
    vector<string> csv_data;
    cout << "save_csv_old_gt_point" << endl;
    for (int i = 0; i < this->old_gt_point.size(); i++) {
        csv_data.push_back(this->old_gt_point[i].get_csv_line(sep));
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

void Test_model::Test_motion_model::save_csv_eval_old_gt_point(string filename, string sep) {
    vector<string> csv_data;
    cout << "save_csv_eval_old_gt_point" << endl;
    for (int i = 0; i < this->eval_old_gt_point.size(); i++) {
        csv_data.push_back(this->eval_old_gt_point[i].get_csv_line(sep));
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

void Test_model::Test_motion_model::save_csv_states(string filename, string sep) {
    vector<string> csv_data;
    cout << "save_csv_states" << endl;
    for (int i = 0; i < this->states.size(); i++) {
        csv_data.push_back(this->states[i].get_csv_line(sep));
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

void Test_model::Test_motion_model::save_csv_timestamps(string filename, string sep) {
    vector<string> csv_data;
    cout << "save_csv_timestamps" << endl;
    for (int i = 0; i < this->timestamps.size(); i++) {
        csv_data.push_back(to_string(this->timestamps[i]));
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

void Test_model::Test_motion_model::load_csv_gt_point(string filename, string sep) {
    ifstream fin(filename);
    char buffer[255];
    fin.getline(buffer, 255);

    cout << "load_csv_gt_point" << endl;

    vector<string> line_buffer;
    while (fin.getline(buffer, 255)) {
        line_buffer.push_back(buffer);
        //cout << "_" << buffer << "_" << endl;

        size_t pos = 0;
        vector<string> values;
        string line(buffer);

        Pose_type tmp;
        tmp.read_csv(line);
        this->gt_point.push_back(tmp);
    };
    fin.close();
};

void Test_model::Test_motion_model::load_csv_old_gt_point(string filename, string sep) {
    ifstream fin(filename);
    char buffer[255];
    fin.getline(buffer, 255);

    cout << "load_csv_old_gt_point" << endl;

    vector<string> line_buffer;
    while (fin.getline(buffer, 255)) {
        line_buffer.push_back(buffer);
        //cout << "_" << buffer << "_" << endl;

        size_t pos = 0;
        vector<string> values;
        string line(buffer);

        Pose_type tmp;
        tmp.read_csv(line);
        this->old_gt_point.push_back(tmp);
    };
    fin.close();
};

void Test_model::Test_motion_model::load_csv_eval_old_gt_point(string filename, string sep) {
    ifstream fin(filename);
    char buffer[255];
    fin.getline(buffer, 255);

    cout << "load_csv_eval_old_gt_point" << endl;

    vector<string> line_buffer;
    while (fin.getline(buffer, 255)) {
        line_buffer.push_back(buffer);
        //cout << "_" << buffer << "_" << endl;

        size_t pos = 0;
        vector<string> values;
        string line(buffer);

        Pose_type tmp;
        tmp.read_csv(line);
        this->old_gt_point.push_back(tmp);
    };
    fin.close();
};

void Test_model::Test_motion_model::load_csv_states(string filename, string sep) {
    ifstream fin(filename);
    char buffer[255];
    fin.getline(buffer, 255);

    cout << "load_csv_states" << endl;

    vector<string> line_buffer;
    while (fin.getline(buffer, 255)) {
        line_buffer.push_back(buffer);
        //cout << "_" << buffer << "_" << endl;

        size_t pos = 0;
        vector<string> values;
        string line(buffer);

        State_type tmp;
        tmp.read_csv(line);
        this->states.push_back(tmp);
    };
    fin.close();
};

void Test_model::Test_motion_model::load_csv_timestamps(string filename, string sep) {
    ifstream fin(filename);
    char buffer[255];
    fin.getline(buffer, 255);

    cout << "load_csv_timestamps" << endl;

    vector<string> line_buffer;
    while (fin.getline(buffer, 255)) {
        line_buffer.push_back(buffer);
        //cout << "_" << buffer << "_" << endl;

        string line(buffer);
        line.replace(line.find(","), 1, ".");
        this->timestamps.push_back(std::stod(line));
    };
    fin.close();
};


State_type Test_model::Test_motion_model::get_state(int number) {
    if (number < 0)
        return this->states[0];
    else if (number > this->states.size())
        return this->states[this->states.size()];
    else
        return this->states[number];
};


void Test_model::Test_motion_model::generate_gt_points(Test_track_model track_model, double delta_length, int point_num) {
    //нужно расставить точки в соответствии с заданой дискретизацией по расстоянию
    point_num = point_num == 0 ? track_model.total_length / delta_length : point_num;

    Pose_type pose1_tmp;
    pose1_tmp.setPose(Point3d(0, 0, 0));
    pose1_tmp.setOrient(states[0].orient);
    pose1_tmp.setAccel(states[0].accel);
    pose1_tmp.setW(states[0].anqular_accel);

    gt_point.push_back(pose1_tmp);

//#pragma omp parallel num_threads(3)
    for (int i = 1; i < point_num; i++) {
        //Point2d pose_delta = track_model.part(i * delta_length) - track_model.part((i - 1) * delta_length);

        Pose_type pose_tmp;
        //pose_tmp.setPose(gt_point[i - 1].getPose() + Point3d(pose_delta.x, pose_delta.y, 0));
        pose_tmp.setPose(track_model.part(i * delta_length));
        pose_tmp.setOrient(states[i].orient);
        pose_tmp.setAccel(states[i].accel);
        pose_tmp.setW(states[i].anqular_accel);

        gt_point.push_back(pose_tmp);
    };
};


void Test_model::Test_motion_model::generate_states(Test_track_model track_model, double delta_m, int point_num) {
    point_num = point_num == 0 ? track_model.total_length / delta_m : point_num;

    for (int i = 0; i < point_num; i++) {
        State_type state = track_model.orientation(i * delta_m);
        Point3d v3_orient = normalize(state.orient);
        // double z_rot = (acos((v3_orient.y + v3_orient.x) / 2) + asin((v3_orient.y - v3_orient.x) / 2)) / 2;

        double z_rot = atan2(v3_orient.y, v3_orient.x);
        state.change_orient(Point3d(0, 0, z_rot));
        states.push_back(state);
    };
};


void Test_model::Test_motion_model::generate_timestamps(double delta_m, double vel) {
    double deltatime = delta_m / vel;
    this->timestamps.push_back(0.0);
    for (int i = 1; i < this->states.size(); i++) {
        this->timestamps.push_back(this->timestamps[i - 1] + deltatime);
    };
    this->total_time = this->timestamps[this->timestamps.size() - 1];
};


void Test_model::Test_motion_model::smooth_anqular_vel_states(double T, double U1, double U2) {
    vector<Point3d> res_orient_vec;
    vector<Point3d> res_ang_vel_vec;
    smooth_p2d();
    //for (int i = 1; i < this->states.size() - 1; i++) res_orient_vec.push_back(this->get_state(i).orient);
    for (int i = 1; i < this->states.size() - 1; i++) res_ang_vel_vec.push_back(this->get_state(i).anqular_vel);

    vector<Point3d> res_ang_vel_vec_last;
    res_ang_vel_vec_last.push_back(res_ang_vel_vec[0]);

    //for (int i = 1; i < res_ang_vel_vec.size(); i++) {
    //    Point3d tmp = U1 * res_ang_vel_vec[i] + (1 - U1) * res_ang_vel_vec_last[i - 1];
    //    res_ang_vel_vec_last.push_back(tmp);
    //};

    int window2 = 100/T;
    int n2 = res_ang_vel_vec.size() - 1;
    int hw2 = (window2 - 1) / 2;
    //vector<Point3d> res_ang_vel_vec_last;
    int k12, k22, z2;
    for (int i = 1; i < n2; i++) {
        Point3d tmp = Point3d(0.0, 0.0, 0.0);
        if (i < hw2) {
            k12 = 0;
            k22 = 2 * i;
            z2 = k22 + 1;
        }
        else if ((i + hw2) > (n2 - 1)) {
            k12 = i - n2 + i + 1;
            k22 = n2 - 1;
            z2 = k22 - k12 + 1;
        }
        else {
            k12 = i - hw2;
            k22 = i + hw2;
            z2 = window2;
        }

        for (int j = k12; j <= k22; j++) {
            tmp = tmp + res_ang_vel_vec[j];
        }
        res_ang_vel_vec_last.push_back(tmp / z2);
    };

    for (int i = 1; i < res_ang_vel_vec_last.size() - 1; i++) this->states[i].change_anqular_vel(res_ang_vel_vec_last[i]);
    //for (int i = 1; i < res_orient_vec_last.size()-1; i++) this->states[i].change_orient(res_orient_vec_last[i]);
};


void Test_model::Test_motion_model::smooth_vel_accel_states(double T, double U) {
    vector<Point3d> res_vel_vec;

    for (State_type tmp_state : this->states) res_vel_vec.push_back(tmp_state.vel);

    vector<Point3d> res_vel_vec_last;
    res_vel_vec_last.push_back(res_vel_vec[0]);

    //for (int i = 1; i < res_vel_vec.size(); i++) {
    //    Point3d tmp = U * res_vel_vec[i] + (1 - U) * res_vel_vec_last[i - 1];
    //    res_vel_vec_last.push_back(tmp);
    //};


    int window = 100/T;
    int n = res_vel_vec.size() - 1;
    if (fmod(window, 2) == 0) window++;
    int hw = (window - 1) / 2;
    //vector<Point3d> res_vel_vec_last;
    int k1, k2, z;
    for (int i = 1; i < n; i++) {
        Point3d tmp = Point3d(0.0, 0.0, 0.0);
        if (i < hw) {
            k1 = 0;
            k2 = 2 * i;
            z = k2 + 1;
        }
        else if ((i + hw) > (n - 1)) {
            k1 = i - n + i + 1;
            k2 = n - 1;
            z = k2 - k1 + 1;
        }
        else {
            k1 = i - hw;
            k2 = i + hw;
            z = window;
        }

        for (int j = k1; j <= k2; j++) {
            tmp = tmp + res_vel_vec[j];
        }
        res_vel_vec_last.push_back(tmp / z);
    }


    for (int i = 1; i < res_vel_vec_last.size(); i++) this->states[i].change_vel(res_vel_vec_last[i]);
    for (int i = 1; i < res_vel_vec_last.size() - 1; i++) {
        double d_time = this->timestamps[i] - this->timestamps[i - 1];
        Point3d d_vec = res_vel_vec_last[i] - res_vel_vec_last[i - 1];
        Point3d g_force = Point3d(0, 0, 9.80665);
        this->states[i].change_accel(g_force + d_vec / d_time);
    };
};


void Test_model::Test_motion_model::regenerate_gt_points() {
    for (int i = 1; i < this->gt_point.size(); i++) {
        double deltatime = (this->timestamps[i] - this->timestamps[i - 1]);
        //Point3d ori = this->gt_point[i - 1].getOrient();
        //Point3d tmp = this->states[i].anqular_vel;
        //if (this->states[i].anqular_vel.z != 0.0) {
        //    tmp = tmp * deltatime;
        //    tmp = ori + tmp;
        //};
        
        
        this->gt_point[i].setPose(this->gt_point[i - 1].getPose() + this->states[i].vel * deltatime);
        this->gt_point[i].setOrient(this->gt_point[i - 1].getOrient() + this->states[i].anqular_vel * deltatime);
        this->gt_point[i].setAccel(states[i].accel);
        this->gt_point[i].setW(states[i].anqular_vel);
    };

};


void Test_model::Test_motion_model::integrate_old_gt() {
    vector<double> deltatime;
    deltatime.push_back(0.0);
    deltatime.push_back(this->timestamps[1] - this->timestamps[0]);
    vector<Point3d> vel;
    vel.push_back(Point3d(0, 0, 0));
    vel.push_back(cv::Point3d(
        this->old_gt_point[1].lat - this->old_gt_point[0].lat,
        (this->old_gt_point[1].lon - this->old_gt_point[0].lon),
        this->old_gt_point[1].alt - this->old_gt_point[0].alt
    ) / deltatime[0]);
    this->eval_old_gt_point.push_back(this->old_gt_point[0]);
    this->eval_old_gt_point.push_back(this->old_gt_point[1]);

    for (int i = 2; i < this->old_gt_point.size() - 1; i++) {

        deltatime.push_back(this->timestamps[i] - this->timestamps[i - 1]);
        Point3d rot_v;
        Point3d rot_vu = Point3d(0, 0, 0);//rad

        // по направляющим косинусам


        cv::Point3d rot_ang;
        double roll = this->eval_old_gt_point[i - 1].getOrient().x;//rad
        double yaw = this->eval_old_gt_point[i - 1].getOrient().y;//rad
        double pitch = this->eval_old_gt_point[i - 1].getOrient().z;//rad

        double wx = this->old_gt_point[i - 1].getW().x;//rad
        double wy = this->old_gt_point[i - 1].getW().y;//rad
        double wz = this->old_gt_point[i - 1].getW().z;//rad

        //cv::Point3d rotw_EC;
        //rotw_EC.x = (sin(pitch) * wx + wy * cos(pitch))/sin(roll);

        Point3d ang_delta(0, 0, 0);


        ang_delta.x = wx * sin(roll) - cos(roll) * wz;
        ang_delta.y = wx * cos(roll) + sin(roll) * wz;
        ang_delta.z = wy - wz * tan(pitch) * cos(roll) + wx * tan(pitch) * sin(roll);

        roll += ang_delta.x * deltatime.back();
        pitch += ang_delta.y * deltatime.back();
        yaw += ang_delta.z * deltatime.back();
        //double tmp = sin(pitch) * wx + cos(pitch) * wy;

        //rotw_EC.y = tmp / sin(theta);
        //psi += rotw_EC.y * this->deltatime.back();
        //rotw_EC.z = tmp * tan(theta);
        //gamma += rotw_EC.z * this->deltatime.back();

        rot_ang = cv::Point3d(roll, pitch, yaw);

        //Нужно вычесть ускорение свободного падения.

        Pose_type pose_tmp;
        cv::Point3d pose_delta;
        cv::Point3d axel = this->old_gt_point[i].getAccel();
        cv::Point3d tmp_vel(0, 0, 0);

        tmp_vel += vel.back() + axel * deltatime.back();
        pose_delta = (tmp_vel + vel.back()) * deltatime.back() / 2;

        pose_tmp.setPose(this->eval_old_gt_point[i - 1].getPose() + pose_delta);
        pose_tmp.setOrient(rot_ang);


        vel.push_back(tmp_vel);
        this->eval_old_gt_point.push_back(pose_tmp);
    }
};