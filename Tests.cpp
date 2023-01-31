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



//using namespace cv;
using namespace std;

#include "Types.h"
#include "ErEstVO.h"
#include "DataSequence.h"
#include "Track_part_type.h"
#include "Test_math.h"
#include "Tests.h"

typedef cv::Point2d Point2d;
typedef cv::Point3d Point3d;
typedef cv::Point2i Point2i;

//inline Point3d smooth(Point3d p_0, Point3d p_1, Point3d p_2, double k1 = 1.0, double k2 = 1.0, double dt = 1.0, double limit_1 = -1, double limit_2 = -1) {
//    
//};

//void Test_model::Kalman_filter() {
//    cout << "Kalman_filter start" << endl;
//
//    #pragma omp parallel for
//    for (Trail_sequence trail_sequence : trail_sequences) { 
//        this->trail_sequences_estimate(trail_sequence);
//    };
//
//    cout << "Kalman_filter start" << endl;
//};
void Test_model::setCameraModel(Test_model_restrictions _gen_restrictions) {

    double focus = _gen_restrictions.focus;
    double yScale = cos(_gen_restrictions.camera_FOV_zoy) / sin(_gen_restrictions.camera_FOV_zoy);
    double xScale = cos(_gen_restrictions.camera_FOV_xoy) / sin(_gen_restrictions.camera_FOV_xoy);

    double IternalCalib_ar[4][4] = {
    {xScale,    0,      0,  0},
    {0,         yScale, 0,  0},
    {0,         0,      1,  1 / focus},
    {0,         0,      0,  0}
    };

    //    double IternalCalib_ar[3][3] = {
    //{xScale,    0,  0},
    //{0, yScale, 0},
    //{0, 0,  1 / focus}
    //    };
    this->A = Mat(4, 4, CV_64F, IternalCalib_ar).clone();
    this->frame_size = Point2i(
        _gen_restrictions.camera_frame_size_x,
        _gen_restrictions.camera_frame_size_y
    );
    this->cam_range = Point2d(0.1, 100000000);
};

void Test_model::trail_sequences_estimate(Trail_sequence _trail_sequence) {

    Trail_sequence res(
        _trail_sequence.start,
        _trail_sequence.timestamps[0],
        _trail_sequence.model_state_vector[0],
        _trail_sequence.model_measurement_vector[0],
        _trail_sequence.model_control_vector[0]
    );

    Mat P = load_csv_Mat(this->dir_name + "Mats/P_0.csv", Point2i(12, 12));
    Mat Q = load_csv_Mat(this->dir_name + "Mats/Q_0.csv", Point2i(12, 12));

    double R_arr[2][2] = {
        {1, 0},
        {0, 1}
    };

    Mat R = Mat(2, 2, CV_64F, R_arr);

    vector<State_vector> state_vector = _trail_sequence.model_state_vector;
    vector<Measurement_vector> measurement_vector = _trail_sequence.model_measurement_vector;
    vector<Control_vector> control_vector = _trail_sequence.model_control_vector;
    vector<double> timestamps = _trail_sequence.timestamps;

    double prev_state_arr[12][1] = {
        {state_vector[0].get(0)}, {state_vector[0].get(1)}, {state_vector[0].get(2)},
        {state_vector[0].get(3)}, {state_vector[0].get(4)}, {state_vector[0].get(5)},
        {state_vector[0].get(6)}, {state_vector[0].get(7)}, {state_vector[0].get(8)},
        {state_vector[0].get(9)}, {state_vector[0].get(10)}, {state_vector[0].get(11)},
    };

    Mat x = Mat(12, 1, CV_64F, prev_state_arr);

    for (int i = 1; i < timestamps.size(); i++) {
        //estimate x

        Mat H = this->senseMat(
            measurement_vector[i].get_point2d(), state_vector[i].get_s_pose(),
            state_vector[i].get_cam_pose(), state_vector[i].get_orient()
        );

        double dt = timestamps[i] - timestamps[i - 1];

        double F_arr[12][12] = F_ARR(dt);
        Mat F = Mat(12, 12, CV_64F, F_arr);

        double B_arr[12][6] = B_ARR(dt);
        Mat B = Mat(12, 6, CV_64F, B_arr);

        double U_arr[6][1] = {
            {control_vector[i].get(0)}, {control_vector[i].get(1)}, {control_vector[i].get(2)},
            {control_vector[i].get(3)}, {control_vector[i].get(4)}, {control_vector[i].get(5)}
        };

        Mat U = Mat(6, 1, CV_64F, U_arr);

        Mat x_next = F * x + B * U;
        Mat P_est = F * P * F.t() + Q;
        Mat H_t = H.t();

        Mat tmp = (H * P_est * H.t() + R);

        double tmp_arr[2][2] = {
            {tmp.at<double>(0, 0), tmp.at<double>(0, 1)},
            {tmp.at<double>(1, 0), tmp.at<double>(1, 1)}
        };

        Mat K = P_est * H.t() * tmp.inv();
        Mat P_next = (Mat::eye(12, 12, CV_64F) - K * H) * P_est;

        P = P_next.clone();
        x = x_next.clone();

        State_vector estimated_state;
        estimated_state.set_cam_pose(Point3d(x_next.at<double>(0, 0), x_next.at<double>(1, 0), x_next.at<double>(2, 0)));
        estimated_state.set_orient(Point3d(x_next.at<double>(3, 0), x_next.at<double>(4, 0), x_next.at<double>(5, 0)));
        estimated_state.set_cam_vel(Point3d(x_next.at<double>(6, 0), x_next.at<double>(7, 0), x_next.at<double>(8, 0)));
        estimated_state.set_s_pose(Point3d(x_next.at<double>(9, 0), x_next.at<double>(10, 0), x_next.at<double>(11, 0)));

        double time = timestamps[i];

        res.push_back(timestamps[i], estimated_state, measurement_vector[i], control_vector[i]);
    };


    this->states_estimated.push_back(res);
};

Point2d Test_model::part_der_h(Point2d _P, Point3d _point_pose, Point3d _camera_pose, Point3d _camera_orient, Point3d _delta) {
    Point2d res = (
        this->point_proection_D(_point_pose + _delta, _camera_pose, _camera_orient)
        - this->point_proection_D(_point_pose - _delta, _camera_pose, _camera_orient)
        )
        / (length(_delta) * 2);
    return res;
};

Mat Test_model::senseMat(Point2d _P, Point3d _point_pose, Point3d _camera_pose, Point3d _camera_orient) {
    //delta _point_pose
    Point3d delta_pp_x = Point3d(0.001, 0, 0);
    Point3d delta_pp_y = Point3d(0, 0.001, 0);
    Point3d delta_pp_z = Point3d(0, 0, 0.001);

    //delta _camera_pose
    Point3d delta_cp_x = Point3d(0.001, 0, 0);
    Point3d delta_cp_y = Point3d(0, 0.001, 0);
    Point3d delta_cp_z = Point3d(0, 0, 0.001);

    //delta _camera_orient
    Point3d delta_co_x = Point3d(0.001, 0, 0);
    Point3d delta_co_y = Point3d(0, 0.001, 0);
    Point3d delta_co_z = Point3d(0, 0, 0.001);

    vector<Point2d> point_pose;
    point_pose.push_back(part_der_h(_P, _point_pose, _camera_pose, _camera_orient, delta_pp_x));
    point_pose.push_back(part_der_h(_P, _point_pose, _camera_pose, _camera_orient, delta_pp_y));
    point_pose.push_back(part_der_h(_P, _point_pose, _camera_pose, _camera_orient, delta_pp_z));

    point_pose.push_back(part_der_h(_P, _point_pose, _camera_pose, _camera_orient, delta_cp_x));
    point_pose.push_back(part_der_h(_P, _point_pose, _camera_pose, _camera_orient, delta_cp_y));
    point_pose.push_back(part_der_h(_P, _point_pose, _camera_pose, _camera_orient, delta_cp_z));

    point_pose.push_back(part_der_h(_P, _point_pose, _camera_pose, _camera_orient, delta_co_x));
    point_pose.push_back(part_der_h(_P, _point_pose, _camera_pose, _camera_orient, delta_co_y));
    point_pose.push_back(part_der_h(_P, _point_pose, _camera_pose, _camera_orient, delta_co_z));

    double senseMat_ar[2][12] = {
        {
            point_pose[0].x, point_pose[1].x, point_pose[2].x,
            point_pose[3].x, point_pose[4].x, point_pose[5].x,
            0.0,             0.0,             0.0,
            point_pose[6].x, point_pose[7].x, point_pose[8].x
        },
        {
            point_pose[0].y, point_pose[1].y, point_pose[2].y,
            point_pose[3].y, point_pose[4].y, point_pose[5].y,
            0.0,             0.0,             0.0,
            point_pose[6].y, point_pose[7].y, point_pose[8].y
        }
    };

    return  Mat(2, 12, CV_64F, senseMat_ar).clone();
};


void Test_model::generate_point_trails(int mode) {
    for (int i_points = 0; i_points < this->s_points.size(); i_points++) {

        vector<Point2d> s_point_trail;
        s_point_trail.push_back(Point2d(i_points, 0));

        for (int i = 0; i < this->bins_model.bins_timestamps.size() - 1; i++) {
            //Mat ex_calib_m = cameraCSMat(this->bins_model.bins_gt_points[i].getOrient(), );
            Point3d cam_pose = this->bins_model.bins_gt_points[i].getPose();
            Point3d cam_orient = this->bins_model.bins_gt_points[i].getOrient();

            Point2d tmp = Point2d(this->frame_size.x, this->frame_size.y);
            switch (mode) {
            case 1:
                tmp = point_proection_D(this->s_points[i_points], cam_pose, cam_orient);
                break;
            case 0:
                //tmp = point_proection_linear(this->s_points[i_points], cam_pose, this->bins_model.bins_gt_points[i].getOrient());
                break;
            };

            if ((tmp.x == this->frame_size.x) && (tmp.y == this->frame_size.y)) tmp = Point2d(-999.0, -999.0);
            s_point_trail.push_back(tmp);
        };

        this->point_trails.push_back(s_point_trail);
        s_point_trail.clear();
    };
};

void Test_model::print_camera_proections() {
    cout << "print_camera_proections start" << endl;
    string dir_frames = this->dir_name + "frames\\";
    string cmd_clear_image_dir = "del /f /q " + dir_frames;
    system(cmd_clear_image_dir.c_str());
    for (int i = 0; i < this->bins_model.bins_timestamps.size() - 1; i++) {
        Mat frame(this->frame_size.y, this->frame_size.x, CV_8UC3, Scalar(255, 255, 255));
        vector<Point2i> frame_points = this->point_camera_proections[i];

        for (int i_points = 0; i_points < frame_points.size(); i_points++) {
            if (frame_points.size() == 0) continue;
            Point2i cross_size = Point2i(3, 3);
            Point2i s_point_location = Point2i(frame_points[i_points].y, frame_points[i_points].x) + this->frame_size / 2;
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
    cout << "print_camera_proections end" << endl;

    string cmd_make_vid = "ffmpeg -start_number 0 -y -i " + dir_frames + "%d.jpg -vcodec mpeg4 " + this->dir_name + this->name + ".mp4";
    
    system(cmd_make_vid.c_str());
    cout << "---<<< video generation ended >>>---" << endl;
};

void Test_model::save_csv_s_points(string filename, string sep) {

    vector<string> csv_data;
    cout << "save_csv_s_points" << endl;
    for (int i = 0; i < this->s_points.size(); i++) {
        csv_data.push_back(this->get_csv_Point3d(this->s_points[i]));
    };

    ofstream fout(filename);
    fout << Point3d_HEADER(sep) << '\n';

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

Point2d Test_model::point_proection_D(Point3d point_pose, Point3d camera_pose, Point3d camera_orient) {

    Point3d delta_pose = point_pose - camera_pose;
    double delta_pose_length = sqrt(delta_pose.x * delta_pose.x + delta_pose.y * delta_pose.y + delta_pose.z * delta_pose.z);
    if ((length(delta_pose) < this->cam_range.x) || (length(delta_pose) > this->cam_range.y)) return this->frame_size;

    //double w = point_pose.x / this->gen_restrictions.focus;
    double point_Pose_ar[4][1] = {
        {point_pose.x},
        {point_pose.y},
        {point_pose.z},
        {1}
    };

    double E_ar[3][4] = {
    {1, 0, 0, 0},
    {0, 1, 0, 0},
    {0, 0, 1, 0}
    };
    Mat E = Mat(3, 4, CV_64F, E_ar);

    //Mat A = this->A;
    //double A_ar[3][3] = {
    //    {A.at<double>(0, 0), A.at<double>(0, 1), A.at<double>(0, 2)},
    //    {A.at<double>(1, 0), A.at<double>(1, 1), A.at<double>(1, 2)},
    //    {A.at<double>(2, 0), A.at<double>(2, 1), A.at<double>(2, 2)}
    //};

    //Mat Ex_calib = cameraCSMat(this->bins_model.bins_gt_points[track_point].getOrient(), camera_pose);

    //double Ex_calib_ar2[4][4] = {
    //    {Ex_calib.at<double>(0, 0), Ex_calib.at<double>(0, 1), Ex_calib.at<double>(0, 2), Ex_calib.at<double>(0, 3)},
    //    {Ex_calib.at<double>(1, 0), Ex_calib.at<double>(1, 1), Ex_calib.at<double>(1, 2), Ex_calib.at<double>(1, 3)},
    //    {Ex_calib.at<double>(2, 0), Ex_calib.at<double>(2, 1), Ex_calib.at<double>(2, 2), Ex_calib.at<double>(2, 3)},
    //    {Ex_calib.at<double>(3, 0), Ex_calib.at<double>(3, 1), Ex_calib.at<double>(3, 2), Ex_calib.at<double>(3, 3)}
    //};

    Mat point_Pose = Mat(4, 1, CV_64F, point_Pose_ar);
    //Mat point_camera_coord = cameraCSMat(this->bins_model.bins_gt_points[track_point].getOrient(), camera_pose) * point_Pose;
    Mat point_camera_coord = cameraCSMat(camera_orient, camera_pose) * point_Pose;

    Point3d local_point_pose = Point3d(point_camera_coord.at<double>(0, 0), point_camera_coord.at<double>(1, 0), point_camera_coord.at<double>(2, 0));
    double w = local_point_pose.x / this->gen_restrictions.focus;
    //double Ex_calib_ar[4][4] = {
    //    {Ex_calib.at<double>(0, 0), Ex_calib.at<double>(0, 1), Ex_calib.at<double>(0, 2), Ex_calib.at<double>(0, 3)},
    //    {Ex_calib.at<double>(1, 0), Ex_calib.at<double>(1, 1), Ex_calib.at<double>(1, 2), Ex_calib.at<double>(1, 3)},
    //    {Ex_calib.at<double>(2, 0), Ex_calib.at<double>(2, 1), Ex_calib.at<double>(2, 2), Ex_calib.at<double>(2, 3)},
    //    {Ex_calib.at<double>(3, 0), Ex_calib.at<double>(3, 1), Ex_calib.at<double>(3, 2), Ex_calib.at<double>(3, 3)}
    //};
    //double E_ar[3][4] = {
    //{1, 0, 0, 0},
    //{0, 1, 0, 0},
    //{0, 0, 1, 0}
    //};

    double point_camera_coord_ar2[4][1] = {
        {local_point_pose.z},
        {local_point_pose.y},
        {local_point_pose.x},
        {w}
    };

    
    point_camera_coord = Mat(4, 1, CV_64F, point_camera_coord_ar2);

    Point2d point_double = Point2d(
        point_camera_coord.at<double>(0, 0) * (this->gen_restrictions.focus / point_camera_coord.at<double>(2, 0)),
        point_camera_coord.at<double>(1, 0) * (this->gen_restrictions.focus / point_camera_coord.at<double>(2, 0))
    );

    if (point_camera_coord.at<double>(2, 0) < 0.0) point_double = this->frame_size;

    Mat cam_point = this->A * point_camera_coord;

    double point_camera_coord_ar[3][1] = {
    {cam_point.at<double>(0, 0)},
    {cam_point.at<double>(1, 0)},
    {cam_point.at<double>(2, 0)}
    };
    //Mat cam_point = this->A * E * point_camera_coord / point_camera_coord.at<double>(2, 0);
    //Point2i point_int = Point2i(
    //    cam_point.at<double>(0, 0) / cam_point.at<double>(2, 0), 
    //    cam_point.at<double>(1, 0) / cam_point.at<double>(2, 0)
    //);

    if ((point_double.x < this->frame_size.x) && (point_double.x > -frame_size.x) && (point_double.y < this->frame_size.y) && (point_double.y > -frame_size.y)) {
        return point_double;
    }
    else {
        return this->frame_size;
    };
};

Point2i Test_model::point_proection_linear(Point3d point_pose, Point3d camera_pose, Point3d cam_rotation) {
    //говнина, не работает

    Point2d FOV_xoy = Point2d(
        -this->gen_restrictions.camera_FOV_xoy / 2,
        this->gen_restrictions.camera_FOV_xoy / 2
    );

    Point2d FOV_zoy = Point2d(
        -this->gen_restrictions.camera_FOV_zoy / 2,
        this->gen_restrictions.camera_FOV_zoy / 2
    );

    int pixels_xoy = this->gen_restrictions.camera_matrix_x;
    int pixels_zoy = this->gen_restrictions.camera_matrix_y;

    double a = cam_rotation.x + this->gen_restrictions.camera_fitting_x;
    double b = cam_rotation.y + this->gen_restrictions.camera_fitting_y;
    double c = cam_rotation.z + this->gen_restrictions.camera_fitting_z;

    //double rotMat_ar[4][4] = {
    //    {cos(c) * cos(b),                               -cos(b) * sin(c),                               sin(b),              0},
    //    {cos(a) * sin(c) + cos(c) * sin(b) * sin(a),    cos(c) * cos(a) - sin(a) * sin(b) * sin(c),     -cos(b) * sin(a),    0},
    //    {sin(c) * sin(a) - cos(c) * cos(a) * sin(b),    cos(a) * sin(b) * sin(c) + cos(c) * sin(a),     cos(b) * cos(a),     0},
    //    {0, 0, 0, 1}
    //};

    double rotMat_ar[4][4] = {
        {cos(c),    -sin(c),    0,  0},
        {sin(c),    cos(c),     0,  0},
        {0,         0,          1,  0},
        {0,         0,          0,  1}
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

    Mat mat_camera_pose = Mat(4, 1, CV_64F, Camera_ar);

    double Point_ar[4][1] = {
        {point_pose.x},
        {point_pose.y},
        {point_pose.z},
        {1}
    };

    Mat mat_point_pose = Mat(4, 1, CV_64F, Point_ar);

    Mat res_vec = R * (mat_point_pose - mat_camera_pose);

    double x = res_vec.at<double>(0, 0);
    double y = res_vec.at<double>(1, 0);
    double z = res_vec.at<double>(2, 0);

    Point2d sphe_coord = Point2d(
        atan2(sqrt(x * x + y * y), z), //угол места
        atan2(y, x) - M_PI//азимут
    );

    Point2d scale = Point2d(
        (FOV_xoy.y - FOV_xoy.x) / pixels_xoy,
        (FOV_zoy.y - FOV_zoy.x) / pixels_zoy
    );

    Point2i cam_poection = Point2i(this->frame_size.x, this->frame_size.y);
    if (
        (sphe_coord.x >= FOV_zoy.x) and (sphe_coord.x <= FOV_zoy.y)
        and (sphe_coord.y >= FOV_xoy.x) and (sphe_coord.y <= FOV_xoy.y)
        )
    {
        cam_poection = Point2i(
            sphe_coord.x * scale.x,
            sphe_coord.y * scale.y
        );
    };
    return cam_poection;
};

void Test_model::generate_camera_proections(int mode = 1) {

    for (int i = 0; i < this->bins_model.bins_timestamps.size() - 1; i++) {
        vector<Point2i> frame_points;
        //Mat ex_calib_m = generateExCalibM(i);

        //double Ex_calib_ar[4][4] = {
        //{ex_calib_m.at<double>(0, 0), ex_calib_m.at<double>(0, 1), ex_calib_m.at<double>(0, 2), ex_calib_m.at<double>(0, 3)},
        //{ex_calib_m.at<double>(1, 0), ex_calib_m.at<double>(1, 1), ex_calib_m.at<double>(1, 2), ex_calib_m.at<double>(1, 3)},
        //{ex_calib_m.at<double>(2, 0), ex_calib_m.at<double>(2, 1), ex_calib_m.at<double>(2, 2), ex_calib_m.at<double>(2, 3)},
        //{ex_calib_m.at<double>(3, 0), ex_calib_m.at<double>(3, 1), ex_calib_m.at<double>(3, 2), ex_calib_m.at<double>(3, 3)}
        //};

        Point3d cam_pose = this->bins_model.bins_gt_points[i].getPose();
        Point3d cam_orient = this->bins_model.bins_gt_points[i].getOrient();

        for (int i_points = 0; i_points < this->s_points.size(); i_points++) {

            Point2i tmp = Point2i(this->frame_size.x, this->frame_size.y);
            switch (mode) {
            case 1:
                tmp = point_proection(this->s_points[i_points], cam_pose, cam_orient);
                break;
            case 0:
                tmp = point_proection_linear(this->s_points[i_points], cam_pose, this->bins_model.bins_gt_points[i].getOrient());
                break;
            };
           

            if ((tmp.x == this->frame_size.x) && (tmp.y == this->frame_size.y)) continue;
            frame_points.push_back(tmp);
        };
        this->point_camera_proections.push_back(frame_points);
        frame_points.clear();
    };
};

Mat Test_model::cameraCSMat(Point3d angles, Point3d _cam_pose) {
    //Point3d angles = this->bins_model.bins_gt_points[i].getOrient();

    double a = angles.x + this->gen_restrictions.camera_fitting_x;
    double b = angles.y + this->gen_restrictions.camera_fitting_y;
    double c = angles.z + this->gen_restrictions.camera_fitting_z;


    double rotMat_ar[3][3] = {
        {cos(c) * cos(b),                               -cos(b) * sin(c),                               sin(b)},
        {cos(a) * sin(c) + cos(c) * sin(b) * sin(a),    cos(c) * cos(a) - sin(a) * sin(b) * sin(c),     -cos(b) * sin(a)},
        {sin(c) * sin(a) - cos(c) * cos(a) * sin(b),    cos(a) * sin(b) * sin(c) + cos(c) * sin(a),     cos(b) * cos(a)}
    };
    Mat R = Mat(3, 3, CV_64F, rotMat_ar);

    //Point3d delta = this->bins_model.bins_gt_points[i].getPose();
    double transMat_ar[3][1] = {
    {_cam_pose.x},
    {_cam_pose.y},
    {_cam_pose.z}
    };
    Mat t = Mat(3, 1, CV_64F, transMat_ar);

    Mat R_t = R.t();

    //Mat tmp = -R_t * t;
    Mat tmp = -R * t;


    double M_ar[4][4] = {
        {R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), tmp.at<double>(0, 0)},
        {R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), tmp.at<double>(1, 0)},
        {R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), tmp.at<double>(2, 0)},
        {0, 0, 0, 1}
    };
    Mat M = Mat(4, 4, CV_64F, M_ar);


    return M.clone();
};

Mat Test_model::generateTransitionM(int i){
    Point3d angles = this->bins_model.bins_gt_points[i].getOrient();
    
    double a = angles.x, b = angles.y, c = angles.z + M_PI / 2;
    double rotMat_ar[3][3] = {
        {cos(c) * cos(b),                               -cos(b) * sin(c),                               sin(b)},
        {cos(a) * sin(c) + cos(c) * sin(b) * sin(a),    cos(c) * cos(a) - sin(a) * sin(b) * sin(c),     -cos(b) * sin(a)},
        {sin(c) * sin(a) - cos(c) * cos(a) * sin(b),    cos(a) * sin(b) * sin(c) + cos(c) * sin(a),     cos(b) * cos(a)}
    };
    Mat R = Mat(3, 3, CV_64F, rotMat_ar);
    
    Point3d delta = this->bins_model.bins_gt_points[i].getPose();
    double transMat_ar[4][14] = {
    {1, 0, 0, delta.x},
    {0, 1, 0, delta.y},
    {0, 0, 1, delta.z},
    {0, 0, 0, 0     }
    };
    Mat t = Mat(3, 1, CV_64F, transMat_ar);
    
    double E_ar[3][4] = {
    {1, 0, 0, 0},
    {0, 1, 0, 0},
    {0, 0, 1, 0}
    };
    Mat E = Mat(3, 4, CV_64F, E_ar);
    
    Mat R_t = R.t();
    
    //Mat tmp = -R_t * t;
    Mat tmp = -R_t * t;
    
    //double M_ar[4][4] = {
    //     {R_t.at<double>(0, 0), R_t.at<double>(0, 1), R_t.at<double>(0, 2), tmp.at<double>(0, 0)},
    //     {R_t.at<double>(1, 0), R_t.at<double>(1, 1), R_t.at<double>(1, 2), tmp.at<double>(1, 0)},
    //     {R_t.at<double>(2, 0), R_t.at<double>(2, 1), R_t.at<double>(2, 2), tmp.at<double>(2, 0)},
    //     {0, 0, 0, 1}
    //};
    //double M_ar[4][4] = {
    //     {R_t.at<double>(0, 0), R_t.at<double>(1, 0), R_t.at<double>(2, 0), tmp.at<double>(0, 0)},
    //     {R_t.at<double>(0, 1), R_t.at<double>(1, 1), R_t.at<double>(2, 1), tmp.at<double>(1, 0)},
    //     {R_t.at<double>(0, 2), R_t.at<double>(1, 2), R_t.at<double>(2, 2), tmp.at<double>(2, 0)},
    //     {0, 0, 0, 1}
    //};
    double M_ar[4][4] = {
        {R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), tmp.at<double>(0, 0)},
        {R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), tmp.at<double>(1, 0)},
        {R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), tmp.at<double>(2, 0)},
        {0, 0, 0, 1}
    };
    Mat M = Mat(4, 4, CV_64F, M_ar);
    
    
    return this->A * E * M;
};


vector<Point3d> generate_cross(
    double height,
    double wight,
    double dot_density,
    Point3d cross_orient,
    Point3d pole_orient,
    Point3d center
) {
    vector<Point3d> res;
    for (double v_shift = -height / 2; v_shift < height / 2; v_shift += dot_density) {
        res.push_back(pole_orient * v_shift + center);
    };
    for (double h_shift = -wight / 2; h_shift < wight / 2; h_shift += dot_density) {
        res.push_back(cross_orient * h_shift + center + pole_orient * (height / 6));
    };
    return res;
};

void Test_model::generate_s_points(
    int mode = 1
    //double border,
    //Point3d z_limits,
    //Point3d grid_spacing,
    //Point2d displacement
) {
    Point3d tmp_point = Point3d(100, 2, 0);
    switch (mode) {
    case 3:
    {
        Point3d point = Point3d(100, 10, 0);
        Point3d point1 = Point3d(100, 0, 0);
        Point3d point2 = Point3d(100, -10, 0);
        s_points.push_back(point);
        s_points.push_back(point1);
        s_points.push_back(point2);
    };
        break;
    case 0:
    {
        for (Point3d cross_point: generate_cross(
            100, 
            50, 
            0.01, 
            Point3d(0, 1, 0), 
            Point3d(0, 0, 1), 
            Point3d(100, 0, 0)
            ))
            s_points.push_back(cross_point);
        //tmp_point = Point3d(0, 100, 0);
        //s_points.push_back(tmp_point);
        };
        break;
    case 1:
    {
        double border = this->gen_restrictions.extra_border;
        Point3d z_limits = Point3d(this->gen_restrictions.z_lim_min, this->gen_restrictions.z_lim_max, 0);
        Point3d grid_spacing = Point3d(this->gen_restrictions.grid_step_x, this->gen_restrictions.grid_step_y, this->gen_restrictions.grid_step_z);
        Point2d displacement = Point2d(0, this->gen_restrictions.grid_disp);

        double max_x = this->motion_model.gt_point[0].lon;
        double max_y = this->motion_model.gt_point[0].lat;
        double min_x = this->motion_model.gt_point[0].lon;
        double min_y = this->motion_model.gt_point[0].lat;

        for (int i = 0; i < this->motion_model.gt_point.size(); i++)
        {
            max_x = this->motion_model.gt_point[i].lon > max_x ? this->motion_model.gt_point[i].lon : max_x;
            max_y = this->motion_model.gt_point[i].lat > max_y ? this->motion_model.gt_point[i].lat : max_y;

            min_x = this->motion_model.gt_point[i].lon < min_x ? this->motion_model.gt_point[i].lon : min_x;
            min_y = this->motion_model.gt_point[i].lat < min_y ? this->motion_model.gt_point[i].lat : min_y;
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
                    tmp_point = Point3d(x_shift, y_shift, z_shift) + displacement_vec;
                    s_points.push_back(tmp_point);
                };
        //потом можно добавить, чтобы точки совсем р€дом с дорогой стирались
    };
        break;
    }
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


void Test_model::show_gt(string mode, bool pause_enable) {
    static Point2i img_size = Point2i(500, 500);
    Mat img(img_size.x, img_size.y, CV_8UC3, Scalar(255, 255, 255));
    int border = 50;
    double max_x = this->motion_model.gt_point[0].lon;
    double max_y = this->motion_model.gt_point[0].lat;
    double min_x = this->motion_model.gt_point[0].lon;
    double min_y = this->motion_model.gt_point[0].lat;

    for (int i = 0; i < this->motion_model.gt_point.size(); i++)
    {
        max_x = this->motion_model.gt_point[i].lon > max_x ? this->motion_model.gt_point[i].lon : max_x;
        max_y = this->motion_model.gt_point[i].lat > max_y ? this->motion_model.gt_point[i].lat : max_y;

        min_x = this->motion_model.gt_point[i].lon < min_x ? this->motion_model.gt_point[i].lon : min_x;
        min_y = this->motion_model.gt_point[i].lat < min_y ? this->motion_model.gt_point[i].lat : min_y;
    };

    Point2d scale = Point2d(max_x - min_x, max_y - min_y);
    double im_scale = scale.x > scale.y ? (img_size.x - 2 * border) / scale.x : (img_size.y - 2 * border) / scale.y;
    Point2i zero_offset = -Point2i(min_x * im_scale, min_y * im_scale);

    for (int i = 1; i < this->motion_model.gt_point.size(); i++)
    {
        Point2d next = Point2d(this->motion_model.gt_point[i].lon, this->motion_model.gt_point[i].lat);
        Point2d prev = Point2d(this->motion_model.gt_point[i - 1].lon, this->motion_model.gt_point[i - 1].lat);

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
    double max_x = this->bins_model.bins_gt_points[0].lon;
    double max_y = this->bins_model.bins_gt_points[0].lat;
    double min_x = this->bins_model.bins_gt_points[0].lon;
    double min_y = this->bins_model.bins_gt_points[0].lat;

    for (int i = 0; i < this->bins_model.bins_gt_points.size(); i++)
    {
        max_x = this->bins_model.bins_gt_points[i].lon > max_x ? this->bins_model.bins_gt_points[i].lon : max_x;
        max_y = this->bins_model.bins_gt_points[i].lat > max_y ? this->bins_model.bins_gt_points[i].lat : max_y;

        min_x = this->bins_model.bins_gt_points[i].lon < min_x ? this->bins_model.bins_gt_points[i].lon : min_x;
        min_y = this->bins_model.bins_gt_points[i].lat < min_y ? this->bins_model.bins_gt_points[i].lat : min_y;
    };

    Point2d scale = Point2d(max_x - min_x, max_y - min_y);
    double im_scale = scale.x > scale.y ? (img_size.x - 2 * border) / scale.x : (img_size.y - 2 * border) / scale.y;
    Point2i zero_offset = -Point2i(min_x * im_scale, min_y * im_scale);

    for (int i = 1; i < this->motion_model.old_gt_point.size() - 1; i++)
    {
        Point2d next = Point2d(this->motion_model.old_gt_point[i].lon, this->motion_model.old_gt_point[i].lat);
        Point2d prev = Point2d(this->motion_model.old_gt_point[i - 1].lon, this->motion_model.old_gt_point[i - 1].lat);

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

    for (int i = 1; i < this->bins_model.bins_gt_points.size(); i++)
    {
        Point2d next = Point2d(this->bins_model.bins_gt_points[i].lon, this->bins_model.bins_gt_points[i].lat);
        Point2d prev = Point2d(this->bins_model.bins_gt_points[i - 1].lon, this->bins_model.bins_gt_points[i - 1].lat);

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
        };
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