#include <iostream>
#include <locale>
#include <iomanip> 
#include <fstream>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <random>
#include <cmath>

using namespace cv;
using namespace std;

#include "Types.h"
#include "ErEstVO.h"
#include "DataSequence.h"
#include "Track_part_type.h"
#include "Tests.h"

//inline Point3d smooth(Point3d p_0, Point3d p_1, Point3d p_2, double k1 = 1.0, double k2 = 1.0, double dt = 1.0, double limit_1 = -1, double limit_2 = -1) {
//    
//};

void Test_model::Test_track_model::generate_track(Test_model::Test_model_restrictions restr) {

    this->track.push_back(Track_part_type(
        Point2d(0, 0),
        Point2d(-1, 0),
        restr.min_line_length,
        restr.max_line_length,
        restr.mean_corner_radius,
        restr.stddev_radius,
        restr.mean_corner_angle,
        restr.stddev_angle,
        restr.average_vel,
        restr.stddev_vel
    ));

    this->track_length.push_back(this->track[0].len());
    this->total_length = track_length[0];
    for (int i = 1; i < restr.max_track_parts; i++) {
        track.push_back(Track_part_type(
            this->track[i - 1].end,
            this->track[i - 1].exit_vec,
            restr.min_line_length,
            restr.max_line_length,
            restr.mean_corner_radius,
            restr.stddev_radius,
            restr.mean_corner_angle,
            restr.stddev_angle,
            restr.average_vel,
            restr.stddev_vel
        ));
        this->track_length.push_back(this->track[i].len());
        this->total_length += this->track_length[i];
    };
};

State_type Test_model::Test_track_model::orientation(double dist = 0) {
    int i = 0;
    while (true) {
        dist -= this->track[i].len();
        if (dist == 0) this->track[i].orientation(dist + this->track[i].len());
        if (dist < 0) {
            return this->track[i].orientation(dist + this->track[i].len());
        };
        i++;
        if (i == track.size()) return State_type();
    };
};

Point2d Test_model::Test_track_model::part(double dist) {
    int i = 0;
    while (true) {
        dist -= this->track[i].len();
        if (dist == 0) return this->track[i].end;
        if (dist < 0) {
            return this->track[i].part(dist + this->track[i].len());
        };
        i++;
        if (i == track.size()) return Point2d(0, 0);
    };
};


void Test_model::Test_model_restrictions::load_restriction_file(string filename) {
    vector<int> int_data;
    vector<double> float_data;

    string separator = ":\t";

    fstream restr_file;
    restr_file.open(filename, ios::in); //open a file to perform read operation using file object
    if (restr_file.is_open()) { //checking whether the file is open
        string tp;
        while (getline(restr_file, tp)) { //read data from file object and put it into string.
            //cout << "~)\t" << tp.substr(tp.find(separator) + separator.length()) << endl;
            tp = tp.substr(tp.find(separator) + separator.length());
            if (int_data.size() == 0) int_data.push_back(stoi(tp));
            else
                float_data.push_back(stod(tp));
        }
        restr_file.close(); //close the file object.
    };
    //for (int item : int_data) cout << "int)\t" << item << endl;
    //for (double item : float_data) cout << "double)\t" << item << endl;
    this->set(filename, int_data, float_data);
    //this->gen_restrictions.show();
};

void Test_model::Test_model_restrictions::show() {
    cout << this->filename << endl;
    cout << "1)" << this->max_track_parts << endl;
    cout << "2)" << this->dicret << endl;
    cout << "3)" << this->min_line_length << endl;
    cout << "4)" << this->max_line_length << endl;
    cout << "5)" << this->mean_corner_radius << endl;
    cout << "6)" << this->stddev_radius << endl;
    cout << "7)" << this->mean_corner_angle << endl;
    cout << "8)" << this->stddev_angle << endl;
    cout << "9)" << this->average_vel << endl;
    cout << "10)" << this->stddev_vel << endl;
    cout << "11)" << this->T << endl;
    cout << "12)" << this->U1 << endl;
    cout << "13)" << this->U2 << endl;
};

void Test_model::Test_model_restrictions::set(string filename, vector<int> int_data, vector<double> float_data) {
    this->filename = filename;
    if ((int_data.size() == 1) and (float_data.size() == 12)) {
        this->max_track_parts = int_data[0];

        this->dicret = float_data[0];
        this->min_line_length = float_data[1];
        this->max_line_length = float_data[2];
        this->mean_corner_radius = float_data[3];
        this->stddev_radius = float_data[4];
        this->mean_corner_angle = float_data[5] * M_PI / 180;
        this->stddev_angle = float_data[6] * M_PI / 180;
        this->average_vel = float_data[7];
        this->stddev_vel = float_data[8];
        this->T = float_data[9];
        this->U1 = float_data[10];
        this->U2 = float_data[11];
    }
    else
        cout << "Test_model_restrictions initial arrays sizes dont match" << endl;

};

void Test_model::generate_test_model(string gen_restr_filename){
    if (gen_restr_filename != "") { 
        this->gen_restrictions.load_restriction_file(gen_restr_filename);
    }
    else
    {
        this->gen_restrictions.load_restriction_file(this->dir_name + "init.txt");
    }
    // сгенерировать трак в соответствии с ограничени€ми

    this->track_model.generate_track(this->gen_restrictions);
    this->track_model.save_csv(this->dir_name + "track.csv");
    /*generate_track(
        this->gen_restrictions.max_track_parts, 
        this->gen_restrictions.min_line_length, 
        this->gen_restrictions.max_line_length, 
        this->gen_restrictions.mean_corner_radius, 
        this->gen_restrictions.stddev_radius, 
        this->gen_restrictions.mean_corner_angle, 
        this->gen_restrictions.stddev_angle, 
        this->gen_restrictions.average_vel, 
        this->gen_restrictions.stddev_vel);*/
    generate_states(
        this->gen_restrictions.dicret
    );
    generate_gt_points(
        this->gen_restrictions.dicret
    );
    generate_timestaps(
        this->gen_restrictions.dicret, 
        this->gen_restrictions.average_vel
    );



    //show_gt();
    //waitKey(0);
    for (int i = 0; i < this->gt_point.size() - 1; i++) this->old_gt_point.push_back(this->gt_point[i]);
    integrate_old_gt();
    print_states("old_states.txt");

    smooth_anqular_vel(
        this->gen_restrictions.T, 
        this->gen_restrictions.U1, 
        this->gen_restrictions.U2
    );

    smooth_vel(
        this->gen_restrictions.T, 
        this->gen_restrictions.U1 / 10000);
    regenerate_gt_points();

    // расставить точки
    double grid_step = 30;
    generate_s_points(
        50, //border x y
        Point3d(0, 30, 0), //z_limits min_z max_z 0
        Point3d(grid_step, grid_step, grid_step), //grid_spacing
        Point2d(0, 3) //displacement
    );
    // сгенерировать бинс данные по ограничени€м, т.е. набор значений
    generate_bins_gt(
        this->gen_restrictions.T
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
    // сгенерировать изображени€ в соответствии с точками

};

void Test_model::integrate_old_gt() {
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

        // по направл€ющим косинусам


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

        //Ќужно вычесть ускорение свободного падени€.

        Pose_type pose_tmp;
        cv::Point3d pose_delta;
        cv::Point3d axel = this->old_gt_point[i].getAccel();
        cv::Point3d tmp_vel(0, 0, 0);

        tmp_vel += vel.back() + axel * deltatime.back();
        pose_delta = (tmp_vel + vel.back()) * deltatime.back() / 2;

        pose_tmp.setPose(this->eval_old_gt_point[i - 1].getPose() + pose_delta);
        pose_tmp.setOrient(rot_ang);


        vel.push_back(tmp_vel);
        //cv::Point3d tmp_vel0(0, 0, 0);
        //cv::Point3d tmp_vel1(0, 0, 0);

        //vector<cv::Point3d> vel0_k;

        //vel0_k.push_back(this->deltatime.back() * this->interpolatedAxel(this->timestamps[this->this_ds_i], rot_ang));
        //vel0_k.push_back(this->deltatime.back() * this->interpolatedAxel(this->timestamps[this->this_ds_i] + this->deltatime.back() / 2, rot_ang));
        //vel0_k.push_back(this->deltatime.back() * this->interpolatedAxel(this->timestamps[this->this_ds_i] + this->deltatime.back() / 2, rot_ang));
        //vel0_k.push_back(this->deltatime.back() * this->interpolatedAxel(this->timestamps[this->this_ds_i] + this->deltatime.back(), rot_ang));

        //tmp_vel0 += this->vel.back() + 1 / 6 * (vel0_k[0] + 2 * vel0_k[1] + 2 * vel0_k[2] + vel0_k[3]);

        //vector<cv::Point3d> vel1_k;


        //double tmp_time;
        //if (this->this_ds_i < this->limit) {
        //    tmp_time = this->timestamps[this->this_ds_i - 1];
        //}
        //else {
        //    tmp_time = this->timestamps[this->this_ds_i];
        //};
        //vel1_k.push_back(this->deltatime.back() * this->interpolatedAxel(tmp_time, rot_ang));
        //vel1_k.push_back(this->deltatime.back() * this->interpolatedAxel(tmp_time + this->deltatime.back() / 2, rot_ang));
        //vel1_k.push_back(this->deltatime.back() * this->interpolatedAxel(tmp_time + this->deltatime.back() / 2, rot_ang));
        //vel1_k.push_back(this->deltatime.back() * this->interpolatedAxel(tmp_time + this->deltatime.back(), rot_ang));

        //tmp_vel1 += tmp_vel0 + 1 / 6 * (vel1_k[0] + 2 * vel1_k[1] + 2 * vel1_k[2] + vel1_k[3]);

        //vector<cv::Point3d> pose_k;

        //pose_k.push_back(this->deltatime.back() * tmp_vel0);
        //pose_k.push_back(this->deltatime.back() * (tmp_vel0 + tmp_vel1) / 2);
        //pose_k.push_back(this->deltatime.back() * (tmp_vel0 + tmp_vel1) / 2);
        //pose_k.push_back(this->deltatime.back() * tmp_vel1);


        //pose_delta = 1 / 6 * (pose_k[0] + 2 * pose_k[1] + 2 * pose_k[2] + pose_k[3]);
        //pose_tmp.lat = pose[this->this_ds_i - 1].lat + pose_delta.x;
        //pose_tmp.lon = pose[this->this_ds_i - 1].lon + pose_delta.y;
        //pose_tmp.alt = pose[this->this_ds_i - 1].alt + pose_delta.z;

        //pose_tmp.roll = rot_ang.x;
        //pose_tmp.pitch = rot_ang.y;
        //pose_tmp.yaw = rot_ang.z;

        //this->vel.push_back(tmp_vel0);


        this->eval_old_gt_point.push_back(pose_tmp);
    }
};

Point2i Test_model::point_proection(Point3d point_pose, Point3d camera_pose, Mat Ex_calib) {
    Point3d delta_pose = point_pose - camera_pose;
    double length = sqrt(delta_pose.x * delta_pose.x + delta_pose.y * delta_pose.y + delta_pose.z * delta_pose.z);
    if ((length < this->cam_range.x) || (length > this->cam_range.y)) return this->frame_size;
    double point_Pose_ar[4][1] = {
        {point_pose.x},
        {point_pose.y},
        {point_pose.z},
        {1}
    };

    Mat point_Pose = Mat(4, 1, CV_64F, point_Pose_ar);
    Mat cam_point = Ex_calib * point_Pose;
    Point2i point_int = Point2i(cam_point.at<double>(0, 0), cam_point.at<double>(1, 0));

    if ((point_int.x < this->frame_size.x) && (point_int.x > -frame_size.x) && (point_int.y < this->frame_size.y) && (point_int.y > -frame_size.y)) {
        return point_int;
    }
    else {
        return this->frame_size;
    };
};

void Test_model::print_bins_gts(string filename) {
    ofstream out; // поток дл€ записи
    out.open(filename); // окрываем файл дл€ записи
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

void Test_model::generate_camera_proections() {
    for (int i = 0; i < this->bins_timestamps.size() - 1; i++) {
        vector<Point2i> frame_points;
        Mat ex_calib_m = generateExCalibM(i);
        Point3d cam_pose = this->bins_gt_points[i].getPose();
        for (int i_points = 0; i_points < this->s_points.size() - 1; i_points++) {
            Point2i tmp = point_proection(this->s_points[i_points], cam_pose, ex_calib_m);

            if ((tmp.x == this->frame_size.x) && (tmp.y == this->frame_size.y)) continue;
            frame_points.push_back(tmp);
        };
        this->point_camera_proections.push_back(frame_points);
    };
};

Mat Test_model::generateExCalibM(int i) {
    Point3d angles = this->bins_gt_points[i].getOrient();
    double a = angles.x, b = angles.y, c = angles.z + M_PI/2;
    double rotMat_ar[3][3] = {
        {cos(c) * cos(b), -cos(b) * sin(b), sin(b)},
        {cos(a) * sin(b) + cos(c) * sin(b) * sin(a), cos(c) * cos(a) - sin(a) * sin(b) * sin(b), -cos(b) * sin(a)},
        {sin(b) * sin(a) - cos(c) * cos(a) * sin(b), cos(a) * sin(b) * sin(b) + cos(c) * sin(a), cos(b) * cos(a)}
    };
    Mat R = Mat(3, 3, CV_64F, rotMat_ar);

    Point3d delta = this->bins_gt_points[i].getPose();
    double transMat_ar[3][1] = {
    {delta.x},
    {delta.y},
    {delta.z}
    };
    Mat t = Mat(3, 1, CV_64F, transMat_ar);

    double E_ar[3][4] = {
    {1, 0, 0, 0},
    {0, 1, 0, 0},
    {0, 0, 1, 0}
    };
    Mat E = Mat(3, 4, CV_64F, E_ar);

    Mat R_t = R.t();

    Mat tmp = -R_t * t;

    double M_ar[4][4] = {
         {R_t.at<double>(0, 0), R_t.at<double>(0, 1), R_t.at<double>(0, 2), tmp.at<double>(0, 0)},
         {R_t.at<double>(1, 0), R_t.at<double>(1, 1), R_t.at<double>(1, 2), tmp.at<double>(1, 0)},
         {R_t.at<double>(2, 0), R_t.at<double>(2, 1), R_t.at<double>(2, 2), tmp.at<double>(2, 0)},
         {0, 0, 0, 1}
    };
    //double M_ar[4][4] = {
    //     {R_t.at<double>(0, 0), R_t.at<double>(1, 0), R_t.at<double>(2, 0), tmp.at<double>(0, 0)},
    //     {R_t.at<double>(0, 1), R_t.at<double>(1, 1), R_t.at<double>(2, 1), tmp.at<double>(1, 0)},
    //     {R_t.at<double>(0, 2), R_t.at<double>(1, 2), R_t.at<double>(2, 2), tmp.at<double>(2, 0)},
    //     {0, 0, 0, 1}
    //};
    //double M_ar[4][4] = {
    //    {R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0)},
    //    {R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0)},
    //    {R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0)},
    //    {0, 0, 0, 1}
    //};
    Mat M = Mat(4, 4, CV_64F, M_ar);


    return this->A * E * M;
};

void Test_model::generate_bins_gt(double bins_deltatime) {
    double bins_total_time = 0.0;
    this->bins_timestamps.push_back(0.0);
    while (bins_total_time < (this->total_time - bins_deltatime)) {
        this->bins_timestamps.push_back(this->bins_timestamps[this->bins_timestamps.size() - 1] + bins_deltatime);
        bins_total_time += bins_deltatime;
    };
    //сгенерировать таймстемпы

    //начальное то же самое
    this->bins_gt_points.push_back(Pose_type(this->gt_point[0].getPose(), this->gt_point[0].getOrient(), this->gt_point[0].getAccel(), this->gt_point[0].getW()));
    vector<double> vec_i;
    for (int i = 1; i < this->bins_timestamps.size(); i++) {
        Pose_type tmp;
        double bins_time = bins_deltatime * i;
        int lesser_ts_i = 0;
        double interp_multi = 0;
        for (int ts_i = 0; ts_i < this->timestamps.size() - 1; ts_i++) {
            if ((this->timestamps[ts_i] <= bins_time) && (this->timestamps[ts_i + 1] >= bins_time)) {
                interp_multi = abs((bins_time - this->timestamps[ts_i]) / (this->timestamps[ts_i + 1] - this->timestamps[ts_i]));
                //interp_multi = abs((bins_time - this->timestaps[ts_i]) / (this->timestaps[ts_i + 1] - bins_time));
                vec_i.push_back(interp_multi);
                lesser_ts_i = ts_i;
                break;
            };
        };
        Pose_type prev = this->gt_point[lesser_ts_i];
        Pose_type next = this->gt_point[lesser_ts_i + 1];

        //интреполировать на них позиции, ориентации, ускорени€, угловые скорости
        tmp.setPose(prev.getPose() + (next.getPose() - prev.getPose()) * interp_multi);
        tmp.setOrient(this->gt_point[lesser_ts_i].getOrient() + (this->gt_point[lesser_ts_i + 1].getOrient() - this->gt_point[lesser_ts_i].getOrient()) * interp_multi);
        //tmp.setPose((1 / (1 + interp_multi)) * (prev.getPose() + interp_multi * next.getPose()));
        //tmp.setOrient((1 / (1 + interp_multi)) * (this->states[lesser_ts_i].orient + interp_multi * this->states[lesser_ts_i+1].orient));

        //получить проекции в соотв с ориентацией вектора угловых скоростей, вектора ускорений
        //Point3d w_curr = (1 / (1 + interp_multi)) * (this->states[lesser_ts_i].anqular_vel + interp_multi * this->states[lesser_ts_i + 1].anqular_vel);
        //Point3d accel_curr = (1 / (1 + interp_multi)) * (this->states[lesser_ts_i].accel + interp_multi * this->states[lesser_ts_i + 1].accel);
        Point3d w_curr = this->states[lesser_ts_i].anqular_vel + (this->states[lesser_ts_i + 1].anqular_vel - this->states[lesser_ts_i].anqular_vel) * interp_multi;
        Point3d accel_curr = this->states[lesser_ts_i].accel + (this->states[lesser_ts_i + 1].accel - this->states[lesser_ts_i].accel) * interp_multi;
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

//void Test_model::generate_track(int max_track_parts, double min_line_length, double max_line_length, double min_corner_radius,
//    double max_corner_radius, double min_corner_angle, double max_corner_angle, double min_vel, double max_vel)
//{
//    track.push_back(Track_part_type(
//        Point2d(0, 0),
//        Point2d(-1, 0),
//        min_line_length,
//        max_line_length,
//        min_corner_radius,
//        max_corner_radius,
//        min_corner_angle,
//        max_corner_angle,
//        min_vel,
//        max_vel
//    ));
//    track_length.push_back(track[0].len());
//    this->total_length = track_length[0];
//    for (int i = 1; i < max_track_parts; i++) {
//        track.push_back(Track_part_type(
//            this->track[i - 1].end,
//            this->track[i - 1].exit_vec,
//            min_line_length,
//            max_line_length,
//            min_corner_radius,
//            max_corner_radius,
//            min_corner_angle,
//            max_corner_angle,
//            min_vel,
//            max_vel
//        ));
//        track_length.push_back(track[i].len());
//        this->total_length += track_length[i];
//    };
//};

void  Test_model::regenerate_gt_points() {
    for (int i = 1; i < this->gt_point.size(); i++) {
        double deltatime = (this->timestamps[i] - this->timestamps[i - 1]);
        this->gt_point[i].setPose(this->gt_point[i - 1].getPose() + this->states[i].vel * deltatime);
        this->gt_point[i].setOrient(this->gt_point[i - 1].getOrient() + this->states[i].anqular_vel * deltatime);
        this->gt_point[i].setAccel(states[i].accel);
        this->gt_point[i].setW(states[i].anqular_vel);
    };

};

void Test_model::generate_s_points(
    double border,
    Point3d z_limits,
    Point3d grid_spacing,
    Point2d displacement
) {
    double max_x = this->gt_point[0].lon;
    double max_y = this->gt_point[0].lat;
    double min_x = this->gt_point[0].lon;
    double min_y = this->gt_point[0].lat;

    for (int i = 0; i < this->gt_point.size(); i++)
    {
        max_x = this->gt_point[i].lon > max_x ? this->gt_point[i].lon : max_x;
        max_y = this->gt_point[i].lat > max_y ? this->gt_point[i].lat : max_y;

        min_x = this->gt_point[i].lon < min_x ? this->gt_point[i].lon : min_x;
        min_y = this->gt_point[i].lat < min_y ? this->gt_point[i].lat : min_y;
    };

    double min_z = z_limits.x;
    double max_z = z_limits.y;

    random_device rd;
    default_random_engine generator(rd());
    double mean_displacement = displacement.x;
    double stddev_displacement = displacement.y;
    normal_distribution<double> distribution_displacement(mean_displacement, stddev_displacement);
    for (double x_shift = min_x - border; x_shift < max_x + border; x_shift += grid_spacing.x)
        for (double y_shift = min_y - border; y_shift < max_y + border; y_shift += grid_spacing.y)
            for (double z_shift = min_z; z_shift < max_z; z_shift += grid_spacing.z)
            {
                Point3d displacement_vec = Point3d(distribution_displacement(generator), distribution_displacement(generator), distribution_displacement(generator));
                Point3d new_s_point = Point3d(x_shift, y_shift, z_shift) + displacement_vec;
                s_points.push_back(new_s_point);
            };
    //потом можно добавить, чтобы точки совсем р€дом с дорогой стирались

};

void Test_model::generate_timestaps(double delta_m, double vel) {
    double deltatime = delta_m / vel;
    this->timestamps.push_back(0.0);
    for (int i = 1; i < this->states.size(); i++) {
        this->timestamps.push_back(this->timestamps[i - 1] + deltatime);
    };
    this->total_time = this->timestamps[this->states.size()-1];
};

State_type Test_model::get_state(int number) {
    if (number < 0)
        return this->states[0];
    else if (number > this->states.size())
        return this->states[this->states.size()];
    else
        return this->states[number];
};

void Test_model::smooth_anqular_vel(double T, double U1, double U2) {
    vector<Point3d> res_orient_vec;
    vector<Point3d> res_ang_vel_vec;
    
    //for (int i = 1; i < this->states.size() - 1; i++) res_orient_vec.push_back(this->get_state(i).orient);
    for (int i = 1; i < this->states.size() - 1; i++) res_ang_vel_vec.push_back(this->get_state(i).anqular_vel);

    //int window = 30;
    //int n = res_orient_vec.size() - 1;
    //if (fmod(window, 2) == 0) window++;
    //int hw = (window - 1) / 2;
    //vector<Point3d> res_orient_vec_last;
    //int k1, k2, z;
    //for (int i = 1; i < n; i++) {
    //    Point3d tmp = Point3d(0.0, 0.0, 0.0);
    //    if (i < hw) {
    //        k1 = 0;
    //        k2 = 2 * i;
    //        z = k2 + 1;
    //    }
    //    else if ((i + hw) > (n - 1)) {
    //        k1 = i - n + i + 1;
    //        k2 = n - 1;
    //        z = k2 - k1 + 1;
    //    }
    //    else {
    //        k1 = i - hw;
    //        k2 = i + hw;
    //        z = window;
    //    }

    //    for (int j = k1; j <= k2; j++) {
    //        tmp = tmp + res_orient_vec[j];
    //    }
    //    res_orient_vec_last.push_back(tmp / z);
    //}

    int window2 = 100;
    int n2 = res_ang_vel_vec.size() - 1;
    int hw2 = (window2 - 1) / 2;
    vector<Point3d> res_ang_vel_vec_last;
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

    for (int i = 1; i < res_ang_vel_vec_last.size()-1; i++) this->states[i].change_anqular_vel(res_ang_vel_vec_last[i]);
    //for (int i = 1; i < res_orient_vec_last.size()-1; i++) this->states[i].change_orient(res_orient_vec_last[i]);
};

void Test_model::smooth_vel(double T, double U) {
    vector<Point3d> res_vel_vec;

    for (State_type tmp_state: this->states) res_vel_vec.push_back(tmp_state.vel);

    int window = 30;
    int n = res_vel_vec.size() - 1;
    if (fmod(window, 2) == 0) window++;
    int hw = (window - 1) / 2;
    vector<Point3d> res_vel_vec_last;
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

    

    for (int i = 1; i < res_vel_vec_last.size() - 1; i++) this->states[i].change_vel(res_vel_vec_last[i]);
    for (int i = 1; i < res_vel_vec_last.size() - 1; i++) this->states[i].change_accel(
        Point3d(0, 0, 9.80665) + (res_vel_vec_last[i] - res_vel_vec_last[i - 1]) / (this->timestamps[i] - this->timestamps[i - 1]));
};

void Test_model::print_states(string filename) {
    ofstream out;          // поток дл€ записи
    out.open(filename); // окрываем файл дл€ записи
    if (out.is_open())
    {
        for (int i = 0; i < this->states.size(); i++) {
            out << format(
                "%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",
                this->states[i].vel.x, this->states[i].vel.y, this->states[i].vel.z,
                this->states[i].orient.x, this->states[i].orient.y, this->states[i].orient.z,
                this->states[i].anqular_vel.x, this->states[i].anqular_vel.y, this->states[i].anqular_vel.z,
                this->states[i].accel.x, this->states[i].accel.y, this->states[i].accel.z,
                this->states[i].anqular_accel.x, this->states[i].anqular_accel.y, this->states[i].anqular_accel.z
            );
        };
    }
    out.close();
};


inline DataSeq_model_Type generate_old_model(
    double mean_1,
    const int size,
    double stddev,
    Point3d w,
    Point3d ang_0,
    double deltatime,
    double accel_stddev,
    Point3d accel = Point3d(0, 0, 0),
    Point3d vel_0 = Point3d(0, 0, 0)
)
{
    DataSeq_model_Type res;

    default_random_engine generator_angle;
    normal_distribution<double> distribution_angle(mean_1, stddev);
    res.angle.push_back(ang_0);

    default_random_engine generator_accel;
    normal_distribution<double> distribution_accel(mean_1, accel_stddev);
    res.pose.push_back(Point3d(0, 0, 0));
    vector<Point3d> vel;
    vel.push_back(vel_0);

    for (int i = 1; i < size + 1; i++) {
        res.angle.push_back(Point3d(res.angle[i - 1].x + w.x * deltatime, res.angle[i - 1].y + w.y * deltatime, res.angle[i - 1].z + w.z * deltatime));
        res.w.push_back(Point3d(w.x + distribution_angle(generator_angle), w.y + distribution_angle(generator_angle), w.z + distribution_angle(generator_angle)));
        res.timestamp.push_back(deltatime * i);

        res.accel.push_back(Point3d(accel.x + distribution_accel(generator_accel), accel.y + distribution_accel(generator_accel), accel.z + distribution_accel(generator_accel)));
        vel.push_back(Point3d(vel[i - 1].x + accel.x * deltatime, vel[i - 1].y + accel.y * deltatime, vel[i - 1].z + (accel.z + GCONST) * deltatime));
        res.pose.push_back(Point3d(res.pose[i - 1].x + vel[i].x * deltatime, res.pose[i - 1].y + vel[i].y * deltatime, res.pose[i - 1].z + vel[i].z * deltatime));
    }

    return res;
};/*double mean_1,
    const int size,
    double stddev,
    Point3d w,
    Point3d ang_0,
    Point3d ang_0,
    double deltatime,
    double accel_stddev,
    Point3d accel = Point3d(0, 0, 0),
    Point3d vel_0 = Point3d(0, 0, 0)*/

void Test_model::generate_states(double delta_m, int point_num) {
    point_num = point_num == 0 ? this->track_model.total_length / delta_m : point_num;

    for (int i = 0; i < point_num; i++) {
        State_type state = this->track_model.orientation(i * delta_m);
        Point3d v3_orient = normalize(state.orient);
        double z_rot = (acos((v3_orient.y + v3_orient.x)/2) + asin((v3_orient.y - v3_orient.x)/2)) / 2;
        state.change_orient(Point3d(0, 0, z_rot));
        states.push_back(state);
    };
};

void Test_model::generate_gt_points(double delta_m, int point_num) {
    //нужно расставить точки в соответствии с заданной скоростью
    point_num = point_num == 0 ? this->track_model.total_length / delta_m : point_num;

    Pose_type pose1_tmp;
    pose1_tmp.setPose(Point3d(0,0,0));
    pose1_tmp.setOrient(states[0].orient);
    pose1_tmp.setAccel(states[0].accel);
    pose1_tmp.setW(states[0].anqular_accel);

    gt_point.push_back(pose1_tmp);

    for (int i = 1; i < point_num; i++) {
        Point2d pose_delta = this->track_model.part(i * delta_m) - this->track_model.part((i - 1) * delta_m);

        Pose_type pose_tmp;
        pose_tmp.setPose(gt_point[i - 1].getPose() + Point3d(pose_delta.x, pose_delta.y, 0));
        pose_tmp.setOrient(states[i].orient);
        pose_tmp.setAccel(states[i].accel);
        pose_tmp.setW(states[i].anqular_accel);

        gt_point.push_back(pose_tmp);
    };
};


void Test_model::show_gt(string mode, bool pause_enable) {
    static Point2i img_size = Point2i(500, 500);
    Mat img(img_size.x, img_size.y, CV_8UC3, Scalar(255, 255, 255));
    int border = 50;
    double max_x = this->gt_point[0].lon;
    double max_y = this->gt_point[0].lat;
    double min_x = this->gt_point[0].lon;
    double min_y = this->gt_point[0].lat;

    for (int i = 0; i < this->gt_point.size(); i++)
    {
        max_x = this->gt_point[i].lon > max_x ? this->gt_point[i].lon : max_x;
        max_y = this->gt_point[i].lat > max_y ? this->gt_point[i].lat : max_y;

        min_x = this->gt_point[i].lon < min_x ? this->gt_point[i].lon : min_x;
        min_y = this->gt_point[i].lat < min_y ? this->gt_point[i].lat : min_y;
    };

    Point2d scale = Point2d(max_x - min_x, max_y - min_y);
    double im_scale = scale.x > scale.y ? (img_size.x - 2 * border) / scale.x : (img_size.y - 2 * border) / scale.y;
    Point2i zero_offset = -Point2i(min_x * im_scale, min_y * im_scale);

    for (int i = 1; i < this->gt_point.size(); i++)
    {
        Point2d next = Point2d(this->gt_point[i].lon, this->gt_point[i].lat);
        Point2d prev = Point2d(this->gt_point[i - 1].lon, this->gt_point[i - 1].lat);

        Point2i beg = Point2i(border + im_scale * (prev.x - min_x), border + im_scale * (prev.y - min_y));
        Point2i end = Point2i(border + im_scale * (next.x - min_x), border + im_scale * (next.y - min_y));
        line(img, beg, end, Scalar(0, 0, 0), 2);
        //circle(img, beg, 6, Scalar(0, 0, 0));
        if (i == 1) circle(img, beg, 6, Scalar(0, 0, 0));
    };

    for (int i = 0; i < this->track_model.track.size(); i++) {
        Point2i points_arr[5] = {
            Point2i(border + im_scale * (this->track_model.track[i].line.start.x - min_x),     border + im_scale * (this->track_model.track[i].line.start.y - min_y)),
            Point2i(border + im_scale * (this->track_model.track[i].line.end.x - min_x),       border + im_scale * (this->track_model.track[i].line.end.y - min_y)),
            Point2i(border + im_scale * (this->track_model.track[i].turn.start.x - min_x),     border + im_scale * (this->track_model.track[i].turn.start.y - min_y)),

            Point2i(border + im_scale * (this->track_model.track[i].turn.end.x - min_x),       border + im_scale * (this->track_model.track[i].turn.end.y - min_y)),
        };
        for (int p_i = 0; p_i < 4; p_i++) circle(img, points_arr[p_i], 4, Scalar(0, 0, 0));
        circle(
            img,
            Point2i(border + im_scale * (this->track_model.track[i].turn.center.x - min_x), border + im_scale * (this->track_model.track[i].turn.center.y - min_y)),
            4,
            Scalar(255, 0, 0));
    };

    for (int i = 0; i < this->s_points.size(); i++) {
        Point2i cross_size = Point2i(3, 3);
        Point2i s_point_location = Point2i(border + im_scale * (this->s_points[i].x - min_x), border + im_scale * (this->s_points[i].y - min_y));
        Point2i cross_points[4] = {
            s_point_location + Point2i(-cross_size.x / 2, -cross_size.y / 2),
            s_point_location + Point2i(-cross_size.x / 2, cross_size.y / 2),
            s_point_location + Point2i(cross_size.x / 2, cross_size.y / 2),
            s_point_location + Point2i(cross_size.x / 2, -cross_size.y / 2)
        };
        line(img, cross_points[0], cross_points[2], Scalar(255, 0, 0), 1);
        line(img, cross_points[1], cross_points[3], Scalar(255, 0, 0), 1);
    };

    if (mode.find("screen") != std::string::npos)
    {
        putText(img,
            "TEST",
            Point(img.cols - 200, 30),
            0,
            0.5,
            CV_RGB(0, 0, 0),
            0.5
        );
        namedWindow("Traect");
        imshow("Traect", img);
        if (pause_enable) waitKey(0);
    };
    if (mode.find("save") != std::string::npos) imwrite("Traect_TEST.jpg", img);
};


void Test_model::show_bins_gt(bool pause_enable) {
    static Point2i img_size = Point2i(500, 500);
    Mat img(img_size.x, img_size.y, CV_8UC3, Scalar(255, 255, 255));
    int border = 50;
    double max_x = this->bins_gt_points[0].lon;
    double max_y = this->bins_gt_points[0].lat;
    double min_x = this->bins_gt_points[0].lon;
    double min_y = this->bins_gt_points[0].lat;

    for (int i = 0; i < this->bins_gt_points.size(); i++)
    {
        max_x = this->bins_gt_points[i].lon > max_x ? this->bins_gt_points[i].lon : max_x;
        max_y = this->bins_gt_points[i].lat > max_y ? this->bins_gt_points[i].lat : max_y;

        min_x = this->bins_gt_points[i].lon < min_x ? this->bins_gt_points[i].lon : min_x;
        min_y = this->bins_gt_points[i].lat < min_y ? this->bins_gt_points[i].lat : min_y;
    };

    Point2d scale = Point2d(max_x - min_x, max_y - min_y);
    double im_scale = scale.x > scale.y ? (img_size.x - 2 * border) / scale.x : (img_size.y - 2 * border) / scale.y;
    Point2i zero_offset = -Point2i(min_x * im_scale, min_y * im_scale);

    for (int i = 1; i < this->old_gt_point.size()-1; i++)
    {
        Point2d next = Point2d(this->old_gt_point[i].lon, this->old_gt_point[i].lat);
        Point2d prev = Point2d(this->old_gt_point[i - 1].lon, this->old_gt_point[i - 1].lat);

        Point2i beg = Point2i(border + im_scale * (prev.x - min_x), border + im_scale * (prev.y - min_y));
        Point2i end = Point2i(border + im_scale * (next.x - min_x), border + im_scale * (next.y - min_y));
        line(img, beg, end, Scalar(128, 128, 128), 3);
        //circle(img, beg, 6, Scalar(0, 0, 0));
    };

    //for (int i = 1; i < this->eval_old_gt_point.size() - 1; i++)
    //{
    //    Point2d next = Point2d(this->eval_old_gt_point[i].lon, this->eval_old_gt_point[i].lat);
    //    Point2d prev = Point2d(this->eval_old_gt_point[i - 1].lon, this->eval_old_gt_point[i - 1].lat);

    //    Point2i beg = Point2i(border + im_scale * (prev.x - min_x), border + im_scale * (prev.y - min_y));
    //    Point2i end = Point2i(border + im_scale * (next.x - min_x), border + im_scale * (next.y - min_y));
    //    line(img, beg, end, Scalar(256, 128, 128), 3);
    //    //circle(img, beg, 6, Scalar(0, 0, 0));
    //};

    for (int i = 1; i < this->bins_gt_points.size(); i++)
    {
        Point2d next = Point2d(this->bins_gt_points[i].lon, this->bins_gt_points[i].lat);
        Point2d prev = Point2d(this->bins_gt_points[i - 1].lon, this->bins_gt_points[i - 1].lat);

        Point2i beg = Point2i(border + im_scale * (prev.x - min_x), border + im_scale * (prev.y - min_y));
        Point2i end = Point2i(border + im_scale * (next.x - min_x), border + im_scale * (next.y - min_y));
        line(img, beg, end, Scalar(0, 0, 0), 2);
        //circle(img, beg, 6, Scalar(0, 0, 0));
        if (i == 1) circle(img, beg, 6, Scalar(0, 0, 0));
    };

    for (int i = 0; i < this->s_points.size(); i++) {
        Point2i cross_size = Point2i(3, 3);
        Point2i s_point_location = Point2i(border + im_scale * (this->s_points[i].x - min_x), border + im_scale * (this->s_points[i].y - min_y));
        Point2i cross_points[4] = {
            s_point_location + Point2i(-cross_size.x / 2, -cross_size.y / 2),
            s_point_location + Point2i(-cross_size.x / 2, cross_size.y / 2),
            s_point_location + Point2i(cross_size.x / 2, cross_size.y / 2),
            s_point_location + Point2i(cross_size.x / 2, -cross_size.y / 2)
        };
        line(img, cross_points[0], cross_points[2], Scalar(255, 0, 0), 1);
        line(img, cross_points[1], cross_points[3], Scalar(255, 0, 0), 1);
    };

    putText(img,
        "BINS_GT",
        Point(img.cols - 200, 30),
        0,
        0.5,
        CV_RGB(0, 0, 0),
        0.5
    );
    namedWindow("BINS_GT");
    imshow("BINS_GT", img);
    if (pause_enable) waitKey(0);     
 };


void old_angle_Test(double w_std, Point3d vel_0, double sko, double delta, double duration) {
    cout << endl;
    cout << endl;
    cout << "-------------<<<" << "Testing angle models" << ">>>-------------" << endl;
    cout << ">>>>w_std:\t" << w_std * 180 / M_PI << "deg/sec" << endl;
    cout << ">>>>sko:\t" << sko << endl;
    cout << ">>>>delta:\t" << delta << " sec" << endl;
    cout << ">>>>duration:\t" << duration << " sec" << endl;
    cout << endl;

    Data_seq test_0("Test0 stand still");
    vector<Point3d> test_0_ang_err_vec;
    test_0.load_model(generate_old_model(0.0, (int)(duration / delta), sko, Point3d(0, 0, 0), Point3d(M_PI / 2, 0, 0), delta, sko));
    for (int i = 0; i < test_0.limit; i++)
        test_0_ang_err_vec.push_back(calcDisplasment_ang(test_0.rot_ang_GT[i], test_0.rot_ang[i], 1));
    Point3d test_0_av_ang_err = Point3d(0, 0, 0);
    Point3d test_0_max_ang_err = Point3d(0, 0, 0);
    for (int i = 0; i < test_0_ang_err_vec.size(); i++)
    {
        test_0_max_ang_err.x = abs(test_0_ang_err_vec[i].x) > test_0_max_ang_err.x ? abs(test_0_ang_err_vec[i].x) : test_0_max_ang_err.x;
        test_0_max_ang_err.y = abs(test_0_ang_err_vec[i].y) > test_0_max_ang_err.y ? abs(test_0_ang_err_vec[i].y) : test_0_max_ang_err.y;
        test_0_max_ang_err.z = abs(test_0_ang_err_vec[i].z) > test_0_max_ang_err.z ? abs(test_0_ang_err_vec[i].z) : test_0_max_ang_err.z;

        test_0_av_ang_err.x += abs(test_0_ang_err_vec[i].x) / test_0_ang_err_vec.size();
        test_0_av_ang_err.y += abs(test_0_ang_err_vec[i].y) / test_0_ang_err_vec.size();
        test_0_av_ang_err.z += abs(test_0_ang_err_vec[i].z) / test_0_ang_err_vec.size();
    };
    cout << test_0.dirname << "\ttime: " << test_0.timestamps[test_0.limit - 1] - test_0.timestamps[0] << "sec" << endl;
    cout << ">>>>" << "Average angle error: roll " << test_0_av_ang_err.x << " deg\tpitch " << test_0_av_ang_err.y << " deg\tyaw " << test_0_av_ang_err.z << " deg" << endl;
    cout << ">>>>" << "Extrime angle error: roll " << test_0_max_ang_err.x << " deg\tpitch " << test_0_max_ang_err.y << " deg\tyaw " << test_0_max_ang_err.z << " deg" << endl;
    cout << endl;

    Data_seq test_1("Test1 yaw rotation");
    vector<Point3d> test_1_ang_err_vec;
    test_1.load_model(generate_old_model(0.0, (int)(duration / delta), sko, Point3d(w_std, 0, 0), Point3d(M_PI / 2, 0, 0), delta, sko));
    for (int i = 0; i < test_1.limit; i++)
        test_1_ang_err_vec.push_back(calcDisplasment_ang(test_1.rot_ang_GT[i], test_1.rot_ang[i], 1));
    Point3d test_1_av_ang_err = Point3d(0, 0, 0);
    Point3d test_1_max_ang_err = Point3d(0, 0, 0);
    for (int i = 0; i < test_1_ang_err_vec.size(); i++)
    {
        test_1_max_ang_err.x = abs(test_1_ang_err_vec[i].x) > test_1_max_ang_err.x ? abs(test_1_ang_err_vec[i].x) : test_1_max_ang_err.x;
        test_1_max_ang_err.y = abs(test_1_ang_err_vec[i].y) > test_1_max_ang_err.y ? abs(test_1_ang_err_vec[i].y) : test_1_max_ang_err.y;
        test_1_max_ang_err.z = abs(test_1_ang_err_vec[i].z) > test_1_max_ang_err.z ? abs(test_1_ang_err_vec[i].z) : test_1_max_ang_err.z;

        test_1_av_ang_err.x += abs(test_1_ang_err_vec[i].x) / test_1_ang_err_vec.size();
        test_1_av_ang_err.y += abs(test_1_ang_err_vec[i].y) / test_1_ang_err_vec.size();
        test_1_av_ang_err.z += abs(test_1_ang_err_vec[i].z) / test_1_ang_err_vec.size();
    };
    cout << test_1.dirname << "\ttime: " << test_1.timestamps[test_1.limit - 1] - test_1.timestamps[0] << "sec" << endl;
    cout << ">>>>" << "Average angle error: roll " << test_1_av_ang_err.x * 180 / M_PI << " deg\tpitch " << test_1_av_ang_err.y * 180 / M_PI << " deg\tyaw " << test_1_av_ang_err.z * 180 / M_PI << " deg" << endl;
    cout << ">>>>" << "Extrime angle error: roll " << test_1_max_ang_err.x * 180 / M_PI << " deg\tpitch " << test_1_max_ang_err.y * 180 / M_PI << " deg\tyaw " << test_1_max_ang_err.z * 180 / M_PI << " deg" << endl;
    cout << endl;

    Data_seq test_3("Test3 pitch rotation");
    vector<Point3d> test_3_ang_err_vec;
    test_3.load_model(generate_old_model(0.0, (int)(duration / delta), sko, Point3d(0, 0, w_std), Point3d(M_PI / 2, 0, 0), delta, sko));
    for (int i = 0; i < test_1.limit; i++)
        test_3_ang_err_vec.push_back(calcDisplasment_ang(test_3.rot_ang_GT[i], test_3.rot_ang[i], 1));
    Point3d test_3_av_ang_err = Point3d(0, 0, 0);
    Point3d test_3_max_ang_err = Point3d(0, 0, 0);
    for (int i = 0; i < test_1_ang_err_vec.size(); i++)
    {
        test_3_max_ang_err.x = abs(test_3_ang_err_vec[i].x) > test_3_max_ang_err.x ? abs(test_3_ang_err_vec[i].x) : test_3_max_ang_err.x;
        test_3_max_ang_err.y = abs(test_3_ang_err_vec[i].y) > test_3_max_ang_err.y ? abs(test_3_ang_err_vec[i].y) : test_3_max_ang_err.y;
        test_3_max_ang_err.z = abs(test_3_ang_err_vec[i].z) > test_3_max_ang_err.z ? abs(test_3_ang_err_vec[i].z) : test_3_max_ang_err.z;

        test_3_av_ang_err.x += abs(test_3_ang_err_vec[i].x) / test_3_ang_err_vec.size();
        test_3_av_ang_err.y += abs(test_3_ang_err_vec[i].y) / test_3_ang_err_vec.size();
        test_3_av_ang_err.z += abs(test_3_ang_err_vec[i].z) / test_3_ang_err_vec.size();
    };
    cout << test_3.dirname << "\ttime: " << test_3.timestamps[test_1.limit - 1] - test_3.timestamps[0] << "sec" << endl;
    cout << ">>>>" << "Average angle error: roll " << test_3_av_ang_err.x << " deg\tpitch " << test_3_av_ang_err.y << " deg\tyaw " << test_3_av_ang_err.z << " deg" << endl;
    cout << ">>>>" << "Extrime angle error: roll " << test_3_max_ang_err.x << " deg\tpitch " << test_3_max_ang_err.y << " deg\tyaw " << test_3_max_ang_err.z << " deg" << endl;
    cout << endl;
};


void old_motion_Test(double accel_std, double sko, double delta, double duration) {

    cout << endl;
    cout << endl;
    cout << "-------------<<<" << "Testing motion models" << ">>>-------------" << endl;
    cout << ">>>>accel_std:\t" << accel_std << "m/sec^2" << endl;
    cout << ">>>>sko:\t" << sko << endl;
    cout << ">>>>delta:\t" << delta << " sec" << endl;
    cout << ">>>>duration:\t" << duration << " sec" << endl;
    cout << endl;

    Data_seq test_1("Test1 stand still"); {
        vector<Pose_type> pose_err_vec;
        test_1.load_model(generate_old_model(
            0.0,
            (int)(duration / delta),
            sko,
            Point3d(0, 0, 0),
            Point3d(M_PI / 2, 0, 0),
            delta,
            sko,
            Point3d(0, 0, 0),
            Point3d(0, 0, 0)));
        for (int i = 0; i < test_1.limit - 1; i++) {
            Pose_type GT = test_1.dataline[i];
            pose_err_vec.push_back(calcDisplasment(GT, test_1.pose[i], test_1.pose[0], "Disp_test1", 3));
        };

        Pose_type GT = test_1.dataline[test_1.limit - 1];
        pose_err_vec.push_back(calcDisplasment(GT, test_1.pose[test_1.limit - 1], test_1.pose[0], "Disp_test1"));

        Pose_type av_pose_err;
        av_pose_err.lat = 0.0;
        av_pose_err.lon = 0.0;
        av_pose_err.alt = 0.0;
        for (int i = 0; i < pose_err_vec.size(); i++)
        {
            av_pose_err.lat += abs(pose_err_vec[i].lat) / pose_err_vec.size();
            av_pose_err.lon += abs(pose_err_vec[i].lon) / pose_err_vec.size();
            av_pose_err.alt += abs(pose_err_vec[i].alt) / pose_err_vec.size();
        }
        //ds.print_traect(num_sources);

        cout << "Disp_test1" << "\ttime: " << test_1.timestamps[test_1.limit - 1] - test_1.timestamps[0] << "sec" << endl;
        cout << ">>>>" << "Average pose  error: lat  " << av_pose_err.lat << " \tlon " << av_pose_err.lon << " \t alt " << av_pose_err.alt << endl;
        cout << endl;
        test_1.print_traect(1, "screen save", true);
    };

    Data_seq test_2("Test2 ecvi_dist"); {
        vector<Pose_type> pose_err_vec;
        test_2.load_model(generate_old_model(
            0.0,
            (int)(duration / delta),
            sko,
            Point3d(0, 0, 0),
            Point3d(M_PI / 2, 0, 0),
            delta,
            sko,
            Point3d(0, 0, 0),
            Point3d(1, 0, 0))
        );
        for (int i = 0; i < test_2.limit; i++) {
            Pose_type GT = test_2.dataline[i];
            pose_err_vec.push_back(calcDisplasment(GT, test_2.pose[i], test_2.pose[0], "Disp_test1", 3));
        };

        Pose_type GT = test_2.dataline[test_2.limit - 1];
        pose_err_vec.push_back(calcDisplasment(GT, test_2.pose[test_2.limit - 1], test_2.pose[0], "Disp_test1"));

        Pose_type av_pose_err;
        av_pose_err.lat = 0.0;
        av_pose_err.lon = 0.0;
        av_pose_err.alt = 0.0;
        for (int i = 0; i < pose_err_vec.size(); i++)
        {
            av_pose_err.lat += abs(pose_err_vec[i].lat) / pose_err_vec.size();
            av_pose_err.lon += abs(pose_err_vec[i].lon) / pose_err_vec.size();
            av_pose_err.alt += abs(pose_err_vec[i].alt) / pose_err_vec.size();
        }
        //ds.print_traect(num_sources);

        cout << "Disp_test2" << "\ttime: " << test_2.timestamps[test_2.limit - 1] - test_2.timestamps[0] << "sec" << endl;
        cout << ">>>>" << "Average pose  error: lat  " << av_pose_err.lat << " \tlon " << av_pose_err.lon << " \t alt " << av_pose_err.alt << endl;
        cout << endl;
        test_2.print_traect(2, "screen save", true);
    };

    Data_seq test_3("Test3 accel"); {
        vector<Pose_type> pose_err_vec;
        test_3.load_model(generate_old_model(
            0.0,
            (int)(duration / delta),
            sko,
            Point3d(0, 0, 0),
            Point3d(M_PI / 2, 0, 0),
            delta,
            sko,
            Point3d(accel_std, 0, 0),
            Point3d(0, 0, 0))
        );
        for (int i = 0; i < test_3.limit; i++) {
            Pose_type GT = test_3.dataline[i];
            pose_err_vec.push_back(calcDisplasment(GT, test_3.pose[i], test_3.pose[0], test_3.dirname, 3));
        };

        Pose_type GT = test_3.dataline[test_3.limit - 1];
        pose_err_vec.push_back(calcDisplasment(GT, test_3.pose[test_3.limit - 1], test_3.pose[0], test_3.dirname));

        Pose_type av_pose_err;
        av_pose_err.lat = 0.0;
        av_pose_err.lon = 0.0;
        av_pose_err.alt = 0.0;
        for (int i = 0; i < pose_err_vec.size(); i++)
        {
            av_pose_err.lat += abs(pose_err_vec[i].lat) / pose_err_vec.size();
            av_pose_err.lon += abs(pose_err_vec[i].lon) / pose_err_vec.size();
            av_pose_err.alt += abs(pose_err_vec[i].alt) / pose_err_vec.size();
        }
        //ds.print_traect(num_sources);

        cout << test_3.dirname << "\ttime: " << test_3.timestamps[test_3.limit - 1] - test_3.timestamps[0] << "sec" << endl;
        cout << ">>>>" << "Average pose  error: lat  " << av_pose_err.lat << " \tlon " << av_pose_err.lon << " \t alt " << av_pose_err.alt << endl;
        cout << endl;
        test_3.print_traect(3, "screen save", true);
    };
};


// Ё“ќ ѕјћя“Ќ»  „≈Ћќ¬≈„— ќ… √Ћ”ѕќ—“»
//void Test_model::smooth_anqular_vel(double T, double U1, double U2) {
//    vector<Point3d> res_orient_vec;
//    vector<Point3d> res_ang_vel_vec;
//
//    for (int i = 1; i < this->states.size() - 1; i++) res_orient_vec.push_back(this->get_state(i).orient);
//    for (int i = 1; i < this->states.size() - 1; i++) res_ang_vel_vec.push_back(this->get_state(i).anqular_vel);
//
//    //res_orient_vec.push_back(this->get_state(0).orient);
//    //res_orient_vec.push_back(this->get_state(1).orient);
//    //res_orient_vec.push_back(this->get_state(2).orient);
//    //res_orient_vec.push_back(this->get_state(3).orient);
//    //res_ang_vel_vec.push_back(this->get_state(0).anqular_vel);
//    //res_ang_vel_vec.push_back(this->get_state(1).anqular_vel);
//    //res_ang_vel_vec.push_back(this->get_state(2).anqular_vel);
//    //res_ang_vel_vec.push_back(this->get_state(3).anqular_vel);
//
//    //for (int i = 3; i < this->states.size(); i++) {
//    //    double alpha = -(T / 2);
//    //    Point3d tmp =
//    //        alpha * alpha * (this->get_state(i).orient + this->get_state(i - 1).orient)
//    //        - alpha * alpha * (alpha*U2 +  U1) * res_ang_vel_vec[i - 1] - alpha * alpha*(U1 + 2*alpha*U2) * res_ang_vel_vec[i - 2] - U2 * alpha * alpha * alpha * res_ang_vel_vec[i - 3]
//    //        -  res_orient_vec[i - 1] +  U2 * alpha * alpha * res_orient_vec[i - 2] + U2 * alpha * alpha * res_orient_vec[i - 3];
//
//    //    Point3d tmp2 = 
//    //        alpha * (this->get_state(i).orient + this->get_state(i - 1).orient)
//    //        - (1 + alpha * U1) * res_ang_vel_vec[i - 1] - U1 * alpha * res_ang_vel_vec[i - 2]
//    //        - U2 * alpha * res_orient_vec[i - 1] - U2 * alpha * res_orient_vec[i - 2];
//
//    //    res_orient_vec.push_back(tmp);
//    //    res_ang_vel_vec.push_back(tmp2);
//    //};
//
//    int window = 30;
//    int n = res_orient_vec.size() - 1;
//    if (fmod(window, 2) == 0) window++;
//    int hw = (window - 1) / 2;
//    vector<Point3d> res_orient_vec_last;
//    int k1, k2, z;
//    for (int i = 1; i < n; i++) {
//        Point3d tmp = Point3d(0.0, 0.0, 0.0);
//        if (i < hw) {
//            k1 = 0;
//            k2 = 2 * i;
//            z = k2 + 1;
//        }
//        else if ((i + hw) > (n - 1)) {
//            k1 = i - n + i + 1;
//            k2 = n - 1;
//            z = k2 - k1 + 1;
//        }
//        else {
//            k1 = i - hw;
//            k2 = i + hw;
//            z = window;
//        }
//
//        for (int j = k1; j <= k2; j++) {
//            tmp = tmp + res_orient_vec[j];
//        }
//        res_orient_vec_last.push_back(tmp / z);
//    }
//
//    int window2 = 60;
//    int n2 = res_ang_vel_vec.size() - 1;
//    int hw2 = (window2 - 1) / 2;
//    vector<Point3d> res_ang_vel_vec_last;
//    int k12, k22, z2;
//    for (int i = 1; i < n2; i++) {
//        Point3d tmp = Point3d(0.0, 0.0, 0.0);
//        if (i < hw2) {
//            k12 = 0;
//            k22 = 2 * i;
//            z2 = k22 + 1;
//        }
//        else if ((i + hw2) > (n2 - 1)) {
//            k12 = i - n2 + i + 1;
//            k22 = n2 - 1;
//            z2 = k22 - k12 + 1;
//        }
//        else {
//            k12 = i - hw2;
//            k22 = i + hw2;
//            z2 = window2;
//        }
//
//        for (int j = k12; j <= k22; j++) {
//            tmp = tmp + res_ang_vel_vec[j];
//        }
//        res_ang_vel_vec_last.push_back(tmp / z2);
//    }
//    //for (int i = 1; i < this->states.size(); i++) res_ang_vel_vec_reverse.push_back(res_ang_vel_vec[res_ang_vel_vec.size() - i]);
//
//
//    //res_ang_vel_vec_another.push_back(res_ang_vel_vec_reverse[0]);
//    //res_ang_vel_vec_another.push_back(res_ang_vel_vec_reverse[1]);
//    //for (int i = 2; i < res_ang_vel_vec_reverse.size() - 1; i++) {
//    //    Point3d tmp =
//    //        -(T / 2) * (res_ang_vel_vec_reverse[i] + res_ang_vel_vec_reverse[i - 1]) +
//    //        (U * T / 2 - 1) * res_ang_vel_vec_another[i - 1] +
//    //        (U * T / 2) * res_ang_vel_vec_another[i - 2];
//    //    res_ang_vel_vec_another.push_back(tmp);
//    //};
//
//
//    //for (int i = 1; i < res_ang_vel_vec_another.size(); i++) res_ang_vel_vec_last.push_back(res_ang_vel_vec_another[res_ang_vel_vec_another.size() - i]);
//
//    //for (int i = 1; i < this->states.size(); i++) this->states[i].change_orient(res_orient_vec[i]);
//    for (int i = 1; i < res_ang_vel_vec_last.size() - 1; i++) this->states[i].change_anqular_vel(res_ang_vel_vec_last[i]);
//    for (int i = 1; i < res_orient_vec_last.size() - 1; i++) this->states[i].change_orient(res_orient_vec_last[i]);
////END FILE