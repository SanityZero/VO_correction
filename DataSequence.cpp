#include <iostream>
#include <locale>
#include <iomanip> 
#include <fstream>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
#include <algorithm>
#include <random>
using namespace cv;
using namespace std;

#include "Types.h"
#include "ErEstVO.h"
#include "DataSequence.h"

Point3d Data_seq::interpolatedAxel(double time, Point3d rotation_angle) {
    Point3d rot_an = Point3d(rotation_angle.x, rotation_angle.y, rotation_angle.z);
    if (time >= timestamps[this->limit - 1]) {
        Point3d res = Point3d(this->dataline[this->limit - 1].ax, this->dataline[this->limit - 1].ay, this->dataline[this->limit - 1].az);
        Point3d res_rot = rotateP3d(res, rot_an) + Point3d(0, 0, GCONST);
        return res_rot;
    };
    for (int i = 0; i < this->limit; i++) {
        if (this->timestamps[i] > time) {
            Point3d prev(
                this->dataline[i - 1].ax,
                this->dataline[i - 1].ay,
                this->dataline[i - 1].az
            );
            Point3d next(
                this->dataline[i].ax,
                this->dataline[i].ay,
                this->dataline[i].az
            );
            Point3d res = (time - this->timestamps[i - 1]) / (this->timestamps[i] - this->timestamps[i - 1]) * (prev - next) - prev;
            Point3d res_rot = rotateP3d(res, rot_an) + Point3d(0, 0, GCONST);
            return res_rot;
        };
    };
    Point3d res = Point3d(this->dataline[this->limit].ax, this->dataline[this->limit].ay, this->dataline[this->limit].az);
    Point3d res_rot = rotateP3d(res, rot_an) + Point3d(0, 0, GCONST);
    return res_rot;
};

void Data_seq::loadBINS(string angle_type = "", string integration_mode = "E") {
    this->angle_type = angle_type;
    Data_seq res;

    string fn_tmp = this->dirname + "timestamps.txt";
    ifstream in(fn_tmp);
    if (!in) cout << "File in not open\n";

    this->limit = 0;
    do
    {
        string timeline;
        getline(in, timeline);
        if (timeline == "") break;
        this->limit++;


        timeline = timeline.substr(timeline.find(' ') + 1, timeline.length());
        double tmp = 0;

        tmp += 60.0 * 60.0 * stod(timeline.substr(0, timeline.find(':')));
        timeline = timeline.substr(timeline.find(':') + 1, timeline.length());

        tmp += 60.0 * stod(timeline.substr(0, timeline.find(':')));
        timeline = timeline.substr(timeline.find(':') + 1, timeline.length());
        tmp += stod(timeline.substr(0, timeline.find(':')));
        this->timestamps.push_back(tmp);
    } while (!in.eof() /*&& /*l_tmp >=0*/);

    for (int i = 0; i < this->limit; i++)
        this->dataline.push_back(getGTData(this->dirname + "data/", i));

    this->pose.push_back(this->dataline[0]);//начальное время
    this->pose.push_back(this->dataline[1]);

    //while (this->calculate_next_point(integration_mode));
};

bool Data_seq::calculate_next_point(string integr_method) {
    //static Point3d vel;
    static Mat C0;
    static Mat C;
    if (this->this_ds_i >= this->limit) return false;
    if (this->this_ds_i == 0) {
        this->deltatime.push_back(0.0);
        this->deltatime.push_back(this->timestamps[0] - this->timestamps[1]);
        this->vel.push_back(
            Point3d(
                this->dataline[1].lat - this->dataline[0].lat,
                (this->dataline[1].lon - this->dataline[0].lon),
                this->dataline[1].alt - this->dataline[0].alt
            ) / this->deltatime.back());//начальная скорость

        if (this->angle_type == "") {
            double w1 = this->pose[0].roll;//rad
            double w2 = this->pose[0].yaw;//rad
            double w3 = this->pose[0].pitch;//rad

            C0 = (Mat_<double>(3, 3) << cos(w2) * cos(w1), sin(w3) * sin(w1) - cos(w3) * cos(w1) * cos(w2), cos(w3) * sin(w1) + sin(w3) * cos(w1) * sin(w2), sin(w2), cos(w3) * cos(w2), -sin(w3) * cos(w2), -cos(w2) * sin(w1), sin(w3) * cos(w1) + cos(w3) * cos(w1) * sin(w2), cos(w3) * cos(w1) - sin(w3) * sin(w1) * sin(w2));
            w1 = this->pose[1].roll;//rad
            w2 = this->pose[1].yaw;//rad
            w3 = this->pose[1].pitch;//rad
            C = (Mat_<double>(3, 3) << cos(w2) * cos(w1), sin(w3) * sin(w1) - cos(w3) * cos(w1) * cos(w2), cos(w3) * sin(w1) + sin(w3) * cos(w1) * sin(w2), sin(w2), cos(w3) * cos(w2), -sin(w3) * cos(w2), -cos(w2) * sin(w1), sin(w3) * cos(w1) + cos(w3) * cos(w1) * sin(w2), cos(w3) * cos(w1) - sin(w3) * sin(w1) * sin(w2));
        };

        this->rot_ang_GT.push_back(Point3d(this->dataline[0].roll, this->dataline[0].pitch, this->dataline[0].yaw));
        this->rot_ang_GT.push_back(Point3d(this->dataline[1].roll, this->dataline[1].pitch, this->dataline[1].yaw));

        this->rot_ang.push_back(Point3d(this->dataline[0].roll, this->dataline[0].pitch, this->dataline[0].yaw));
        this->rot_ang.push_back(Point3d(this->dataline[1].roll, this->dataline[1].pitch, this->dataline[1].yaw));
        this->this_ds_i = 2;

    };
    if (this->this_ds_i == 1) {

    };
    this->deltatime.push_back(this->timestamps[this->this_ds_i - 1] - this->timestamps[this->this_ds_i]);

    Point3d rot_v;
    Point3d rot_vu = Point3d(0, 0, 0);//rad

    // по направляющим косинусам


    rot_v.x = dataline[1].wx - rot_vu.x;//rad
    rot_v.y = dataline[1].wz - rot_vu.y;//rad
    rot_v.z = dataline[1].wy - rot_vu.y;//rad

    Point3d rot_ang;

    Point3d rot_ang_GT;

    rot_ang_GT.x = this->dataline[this->this_ds_i].roll;
    rot_ang_GT.y = this->dataline[this->this_ds_i].pitch;
    rot_ang_GT.z = this->dataline[this->this_ds_i].yaw;
    this->rot_ang_GT.push_back(rot_ang_GT);

    if (this->angle_type == "") {
        // формирование кососимметричной матрицы
        Mat CSM = (Mat_<double>(3, 3) << 0, -rot_v.z, rot_v.y, rot_v.z, 0, -rot_v.x, -rot_v.y, rot_v.x, 0);
        //double CSM[3][3] = {
        //    {0, -rot_v.z, rot_v.y},
        //    {rot_v.z, 0, rot_v.x},
        //    {-rot_v.y, rot_v.x, 0}
        //};
        //матричное произведение
        Mat Do_Int = mat_multi(C, CSM);
        Mat C_TMP = mat_add(Do_Int, C, this->deltatime.back());
        C = C_TMP.clone();

        double psi = atan(-C.at<double>(2, 0) / C.at<double>(0, 0));//rad
        double theta = asin(-C.at<double>(1, 0));//rad
        double gamma = atan(-C.at<double>(1, 2) / C.at<double>(1, 1));//rad

        rot_ang = Point3d(psi, gamma, -theta);
    }
    else
        if (this->angle_type == "EC") {

            double roll = this->rot_ang[this->this_ds_i - 1].x;//rad
            double yaw = this->rot_ang[this->this_ds_i - 1].z;//rad
            double pitch = this->rot_ang[this->this_ds_i - 1].y;//rad

            double wx = rot_v.x;//rad
            double wy = rot_v.y;//rad
            double wz = rot_v.z;//rad

            //Point3d rotw_EC;
            //rotw_EC.x = (sin(pitch) * wx + wy * cos(pitch))/sin(roll);

            Point3d ang_tmp(0, 0, 0);

            ang_tmp.z += (sin(pitch) * wx + wy * cos(pitch)) / sin(roll) * this->deltatime.back();
            ang_tmp.x += (wx - sin(pitch) * sin(pitch) * wx + sin(pitch) * wy * cos(pitch)) / cos(pitch) * this->deltatime.back();
            ang_tmp.y += (wz - (sin(pitch) * wx + wy * cos(pitch)) / sin(roll) * cos(roll)) * this->deltatime.back();

            roll += ang_tmp.x;
            pitch += ang_tmp.y;
            yaw += ang_tmp.z;
            //double tmp = sin(pitch) * wx + cos(pitch) * wy;

            //rotw_EC.y = tmp / sin(theta);
            //psi += rotw_EC.y * this->deltatime.back();
            //rotw_EC.z = tmp * tan(theta);
            //gamma += rotw_EC.z * this->deltatime.back();

            rot_ang = Point3d(roll, pitch, yaw);
        }
        else
            if (this->angle_type == "EC2") {

                double roll = this->rot_ang[this->this_ds_i - 1].x;//rad
                double yaw = this->rot_ang[this->this_ds_i - 1].z;//rad
                double pitch = this->rot_ang[this->this_ds_i - 1].y;//rad

                double wx = rot_v.x;//rad
                double wy = rot_v.y;//rad
                double wz = rot_v.z;//rad

                //Point3d rotw_EC;
                //rotw_EC.x = (sin(pitch) * wx + wy * cos(pitch))/sin(roll);

                Point3d ang_delta(0, 0, 0);


                ang_delta.x = wx * sin(roll) - cos(roll) * wz;
                ang_delta.y = wx * cos(roll) + sin(roll) * wz;
                ang_delta.z = wy - wz * tan(pitch) * cos(roll) + wx * tan(pitch) * sin(roll);

                roll += ang_delta.x * this->deltatime.back();
                pitch += ang_delta.y * this->deltatime.back();
                yaw += ang_delta.z * this->deltatime.back();
                //double tmp = sin(pitch) * wx + cos(pitch) * wy;

                //rotw_EC.y = tmp / sin(theta);
                //psi += rotw_EC.y * this->deltatime.back();
                //rotw_EC.z = tmp * tan(theta);
                //gamma += rotw_EC.z * this->deltatime.back();

                rot_ang = Point3d(roll, pitch, yaw);
            }
            else
                if (this->angle_type == "GT") {
                    rot_ang = rot_ang_GT;
                    rot_ang.z = rot_ang.z;
                };

    this->rot_ang.push_back(rot_ang);

    //Нужно вычесть ускорение свободного падения.

    Pose_type pose_tmp;
    Point3d pose_delta;
    if (integr_method == "E") {
        Point3d axel = this->interpolatedAxel(this->timestamps[this->this_ds_i], Point3d(rot_ang.x, rot_ang.y, rot_ang.z));
        Point3d tmp_vel(0, 0, 0);

        tmp_vel.x += this->vel.back().x + axel.x * this->deltatime.back();
        tmp_vel.y += this->vel.back().y + axel.y * this->deltatime.back();
        tmp_vel.z += this->vel.back().z + axel.z * this->deltatime.back();

        pose_delta.x = (tmp_vel.x + this->vel.back().x) * this->deltatime.back() / 2;
        pose_delta.y = (tmp_vel.y + this->vel.back().y) * this->deltatime.back() / 2;
        pose_delta.z = (tmp_vel.z + this->vel.back().z) * this->deltatime.back() / 2;

        pose_tmp.lat = pose[this->this_ds_i - 1].lat + pose_delta.x;
        pose_tmp.lon = pose[this->this_ds_i - 1].lon + pose_delta.y;
        pose_tmp.alt = pose[this->this_ds_i - 1].alt + pose_delta.z;

        pose_tmp.roll = rot_ang.x;
        pose_tmp.pitch = rot_ang.y;
        pose_tmp.yaw = rot_ang.z;

        this->vel.push_back(tmp_vel);
    }
    else
        if (integr_method == "RK") {
            Point3d tmp_vel0(0, 0, 0);
            Point3d tmp_vel1(0, 0, 0);

            vector<Point3d> vel0_k;

            vel0_k.push_back(this->deltatime.back() * this->interpolatedAxel(this->timestamps[this->this_ds_i], rot_ang));
            vel0_k.push_back(this->deltatime.back() * this->interpolatedAxel(this->timestamps[this->this_ds_i] + this->deltatime.back() / 2, rot_ang));
            vel0_k.push_back(this->deltatime.back() * this->interpolatedAxel(this->timestamps[this->this_ds_i] + this->deltatime.back() / 2, rot_ang));
            vel0_k.push_back(this->deltatime.back() * this->interpolatedAxel(this->timestamps[this->this_ds_i] + this->deltatime.back(), rot_ang));

            tmp_vel0 += this->vel.back() + 1 / 6 * (vel0_k[0] + 2 * vel0_k[1] + 2 * vel0_k[2] + vel0_k[3]);

            vector<Point3d> vel1_k;


            double tmp_time;
            if (this->this_ds_i < this->limit) {
                tmp_time = this->timestamps[this->this_ds_i - 1];
            }
            else {
                tmp_time = this->timestamps[this->this_ds_i];
            };
            vel1_k.push_back(this->deltatime.back() * this->interpolatedAxel(tmp_time, rot_ang));
            vel1_k.push_back(this->deltatime.back() * this->interpolatedAxel(tmp_time + this->deltatime.back() / 2, rot_ang));
            vel1_k.push_back(this->deltatime.back() * this->interpolatedAxel(tmp_time + this->deltatime.back() / 2, rot_ang));
            vel1_k.push_back(this->deltatime.back() * this->interpolatedAxel(tmp_time + this->deltatime.back(), rot_ang));

            tmp_vel1 += tmp_vel0 + 1 / 6 * (vel1_k[0] + 2 * vel1_k[1] + 2 * vel1_k[2] + vel1_k[3]);

            vector<Point3d> pose_k;

            pose_k.push_back(this->deltatime.back() * tmp_vel0);
            pose_k.push_back(this->deltatime.back() * (tmp_vel0 + tmp_vel1) / 2);
            pose_k.push_back(this->deltatime.back() * (tmp_vel0 + tmp_vel1) / 2);
            pose_k.push_back(this->deltatime.back() * tmp_vel1);


            pose_delta = 1 / 6 * (pose_k[0] + 2 * pose_k[1] + 2 * pose_k[2] + pose_k[3]);
            pose_tmp.lat = pose[this->this_ds_i - 1].lat + pose_delta.x;
            pose_tmp.lon = pose[this->this_ds_i - 1].lon + pose_delta.y;
            pose_tmp.alt = pose[this->this_ds_i - 1].alt + pose_delta.z;

            pose_tmp.roll = rot_ang.x;
            pose_tmp.pitch = rot_ang.y;
            pose_tmp.yaw = rot_ang.z;

            this->vel.push_back(tmp_vel0);
        }
        else if (integr_method == "multistep") {

            //первый шаг будет по эйлеру
            //потом дорастать до 4 порядка по шагам
        };


    this->pose.push_back(pose_tmp);
    this->this_ds_i++;
    return true;
};

void Data_seq::print_traect(int num, string mode = "screen", bool pause_enable = true) {
    static Point2i img_size = Point2i(520, 520);
    Mat img(img_size.x, img_size.y, CV_8UC3, Scalar(255, 255, 255));
    int border = 3;
    double max_x = this->pose[0].lon;
    double max_y = this->pose[0].lat;
    double min_x = this->pose[0].lon;
    double min_y = this->pose[0].lat;

    for (int i = 0; i < this->pose.size(); i++)
    {
        max_x = this->pose[i].lon > max_x ? this->pose[i].lon : max_x;
        max_y = this->pose[i].lat > max_y ? this->pose[i].lat : max_y;

        min_x = this->pose[i].lon < min_x ? this->pose[i].lon : min_x;
        min_y = this->pose[i].lat < min_y ? this->pose[i].lat : min_y;
    };

    for (int i = 0; i < this->pose.size(); i++)
    {
        Point2d tmp(this->dataline[i].lon, this->dataline[i].lat);
        max_x = tmp.x >= max_x ? tmp.x : max_x;
        max_y = tmp.y >= max_y ? tmp.y : max_y;

        min_x = tmp.x <= min_x ? tmp.x : min_x;
        min_y = tmp.y <= min_y ? tmp.y : min_y;
    };

    for (int i = 0; i < this->pose_VOcr.size(); i++)
    {
        max_x = this->pose_VOcr[i].lon > max_x ? this->pose_VOcr[i].lon : max_x;
        max_y = this->pose_VOcr[i].lat > max_y ? this->pose_VOcr[i].lat : max_y;

        min_x = this->pose_VOcr[i].lon < min_x ? this->pose_VOcr[i].lon : min_x;
        min_y = this->pose_VOcr[i].lat < min_y ? this->pose_VOcr[i].lat : min_y;
    };

    Point2d scale = Point2d(max_x - min_x, max_y - min_y);
    double im_scale = scale.x > scale.y ? (img_size.x - 2 * border) / scale.x : (img_size.y - 2 * border) / scale.y;

    for (int i = 1; i < this->pose.size(); i++)
    {
        Point2d next = Point2d(this->dataline[i].lon, this->dataline[i].lat);
        Point2d prev = Point2d(this->dataline[i - 1].lon, this->dataline[i - 1].lat);

        Point2i beg = Point2i(border + im_scale * (prev.x - abs(min_x)), border + im_scale * (prev.y - abs(min_y)));
        Point2i end = Point2i(border + im_scale * (next.x - abs(min_x)), border + im_scale * (next.y - abs(min_y)));
        line(img, beg, end, Scalar(0, 0, 0), 3);
        if (i == 1) circle(img, beg, 6, Scalar(0, 0, 0));
    };

    for (int i = 1; i < this->pose.size(); i++)
    {

        Point2i beg = Point2i(border + im_scale * (this->pose[i - 1].lon - abs(min_x)), border + im_scale * (this->pose[i - 1].lat - abs(min_y)));
        Point2i end = Point2i(border + im_scale * (this->pose[i].lon - abs(min_x)), border + im_scale * (this->pose[i].lat - abs(min_y)));
        line(img, beg, end, Scalar(0, 0, 255), 2);
        if (i == 1) circle(img, beg, 6, Scalar(0, 0, 255));
    };

    for (int i = 1; i < this->pose_VOcr.size(); i++)
    {

        Point2i beg = Point2i(border + im_scale * (this->pose_VOcr[i - 1].lon - abs(min_x)), border + im_scale * (this->pose_VOcr[i - 1].lat - abs(min_y)));
        Point2i end = Point2i(border + im_scale * (this->pose_VOcr[i].lon - abs(min_x)), border + im_scale * (this->pose_VOcr[i].lat - abs(min_y)));
        line(img, beg, end, Scalar(255, 0, 0), 1);
        if (i == 1) circle(img, beg, 6, Scalar(255, 0, 0));
    };

    if (mode.find("screen") != std::string::npos)
    {
        putText(img,
            "black - GT red - BINS",
            Point(img.cols - 200, 30),
            0,
            0.5,
            CV_RGB(0, 0, 0),
            0.5
        );
        namedWindow("Traect" + to_string(num), WINDOW_NORMAL);
        imshow("Traect" + to_string(num), img);
        if (pause_enable) waitKey(0);
    };
    if (mode.find("save") != std::string::npos) imwrite("Traect" + to_string(num) + ".jpg", img);
};

void Data_seq::load_model(DataSeq_model_Type model, string angle_type) {
    this->angle_type = angle_type;
    vector<Point3d> w = model.w;
    vector<double> time = model.timestamp;
    vector<Point3d> GT_angle = model.angle;
    vector<Point3d> pose_GT = model.pose;
    vector<Point3d> accel = model.accel;
    Data_seq res;

    this->limit = time.size() - 1;
    //загрузить данные модели
    for (int i = 0; i < this->limit; i++)
    {
        this->timestamps.push_back(time[i]);

        Pose_type tmp;
        //double lat;
        //double lon;
        //double alt;

        //double roll;
        //double pitch;
        //double yaw;

        tmp.lat = pose_GT[i].x;
        tmp.lon = pose_GT[i].y;
        tmp.alt = pose_GT[i].z;

        //double ax;
        //double ay;
        //double az;
        tmp.wx = w[i].x;
        tmp.wy = w[i].y;
        tmp.wz = w[i].z;

        tmp.ax = accel[i].x;
        tmp.ay = accel[i].x;
        tmp.az = accel[i].x;

        tmp.roll = GT_angle[i].x;
        tmp.pitch = GT_angle[i].y;
        tmp.yaw = GT_angle[i].z;

        Pose_type GT = tmp;

        GT.lat = pose_GT[i].x;
        GT.lon = pose_GT[i].y;
        GT.alt = pose_GT[i].z;

        this->dataline.push_back(tmp);

        Point3d rot_ang_GT;

        rot_ang_GT.x = this->dataline[i].roll;
        rot_ang_GT.y = this->dataline[i].pitch;
        rot_ang_GT.z = this->dataline[i].yaw;
        this->rot_ang_GT.push_back(rot_ang_GT);
    };

    this->rot_ang_GT.push_back(Point3d(this->dataline[0].roll, this->dataline[0].pitch, this->dataline[0].yaw));
    this->rot_ang_GT.push_back(Point3d(this->dataline[1].roll, this->dataline[1].pitch, this->dataline[1].yaw));

    this->pose.push_back(this->dataline[0]);
    this->pose.push_back(this->dataline[1]);


    for (int i = 1; i < this->limit; i++) {

        this->calculate_next_point();
    }
};
