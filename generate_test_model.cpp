#include <iostream>
#include <string>
#include <opencv2/core.hpp>

using namespace std;

#include "Types.h"
#include "Tests.h"

typedef cv::Point2d Point2d;
typedef cv::Point3d Point3d;
typedef cv::Point2i Point2i;

void Test_model::generate_test_model(string gen_restr_filename) {

    std::cout << "\n----<<<< gen_restrictions >>>>----" << std::endl;
    if (gen_restr_filename != "") {
        this->gen_restrictions.load_restriction_file(gen_restr_filename);
    }
    else
    {
        this->gen_restrictions.load_restriction_file(this->dir_name + "\\init.txt");
    }

    // сгенерировать трак в соответствии с ограничениями или загрузить его
    std::cout << "\n----<<<< track_model >>>>----" << std::endl;
    switch (this->gen_restrictions.int_data["save_load_track_model"]) {
    case 0:
        this->track_model.generate_track(this->gen_restrictions);
        this->track_model.save_csv_track(this->dir_name + "track.csv");
        break;
    case 1:
        this->track_model.load_csv_track(this->dir_name + "track.csv");
        break;
    };


    // сгенерировать состояний в соответствии с ограничениями или загрузить их
    std::cout << "\n----<<<< motion_model >>>>----" << std::endl;
    switch (this->gen_restrictions.int_data["save_load_motion_model"]) {
    case 0:
        std::cout << "generate_states" << std::endl;
        this->motion_model.generate_states(this->track_model, this->gen_restrictions.double_data["dicret"]);

        std::cout << "generate_timestamps" << std::endl;
        this->motion_model.generate_timestamps(this->gen_restrictions.double_data["dicret"], this->gen_restrictions.double_data["average_vel"]);

        std::cout << "generate_gt_points" << std::endl;
        this->motion_model.generate_gt_points(this->track_model, this->gen_restrictions.double_data["dicret"]);

        std::cout << "smooth_anqular_vel_states" << std::endl;
        this->motion_model.smooth_anqular_vel_states(
            this->gen_restrictions.double_data["dicret"],
            this->gen_restrictions.double_data["U1"],
            this->gen_restrictions.double_data["U2"]
        );

        std::cout << "smooth_vel_accel_states" << std::endl;
        this->motion_model.smooth_vel_accel_states(
            this->gen_restrictions.double_data["dicret"],
            this->gen_restrictions.double_data["U2"]
        );


        
        this->motion_model.regenerate_gt_points();

        
        for (int i = 0; i < this->motion_model.gt_point.size() - 1; i++)
            this->motion_model.old_gt_point.push_back(this->motion_model.gt_point[i]);
        this->motion_model.integrate_old_gt();

        this->motion_model.save_csv_states(this->dir_name + "states.csv");
        this->motion_model.save_csv_gt_point(this->dir_name + "gt_point.csv");
        this->motion_model.save_csv_timestamps(this->dir_name + "timestamps.csv");
        this->motion_model.save_csv_eval_old_gt_point(this->dir_name + "eval_old_gt_point.csv");
        this->motion_model.save_csv_old_gt_point(this->dir_name + "old_gt_point.csv");
        break;
    case 1:
        this->motion_model.load_csv_states(this->dir_name + "states.csv");
        this->motion_model.load_csv_gt_point(this->dir_name + "gt_point.csv");
        this->motion_model.load_csv_timestamps(this->dir_name + "timestamps.csv");
        this->motion_model.update_total_time();
        this->motion_model.load_csv_eval_old_gt_point(this->dir_name + "eval_old_gt_point.csv");
        this->motion_model.load_csv_old_gt_point(this->dir_name + "old_gt_point.csv");
        break;
    };



    // расставить точки
    std::cout << "\n----<<<< s_points >>>>----" << std::endl;
    switch (this->gen_restrictions.int_data["save_load_s_points"]) {
    case 0:
        generate_s_points(this->gen_restrictions.int_data["s_points_generation_mode"]);
        save_csv_s_points(this->dir_name + "s_points.csv");
        break;
    case 1:
        load_csv_s_points(this->dir_name + "s_points.csv");
        break;
    };


    // сгенерировать бинс данные по ограничениям, т.е. набор значений
    std::cout << "\n----<<<< bins_model >>>>----" << std::endl;
    switch (this->gen_restrictions.int_data["save_load_bins_model"]) {
    case 0:
        this->bins_model.generate_bins_gt_points(
            this->motion_model,
            this->gen_restrictions.double_data["T"]
        );
        this->bins_model.generate_bins_measured_states(
            this->motion_model,
            this->gen_restrictions.double_data["T"]
        );
        this->bins_model.save_csv_bins_measured_states(this->dir_name + "bins_measured_states.csv");
        this->bins_model.save_csv_bins_timestamps(this->dir_name + "bins_timestamps.csv");
        this->bins_model.save_csv_bins_gt_points(this->dir_name + "bins_gt_points.csv");
        break;
    case 1:
        this->bins_model.load_csv_bins_gt_points(this->dir_name + "bins_gt_points.csv");
        this->bins_model.load_csv_bins_measured_states(this->dir_name + "bins_measured_states.csv");
        this->bins_model.load_csv_bins_timestamps(this->dir_name + "bins_timestamps.csv");
        break;
    };


    std::cout << "\n----<<<< camera_model >>>>----" << std::endl;
    setCameraModel(this->gen_restrictions);
    generate_camera_proections(this->gen_restrictions.int_data["camera_proection_mode"]);
    generate_point_trails(this->gen_restrictions.int_data["camera_proection_mode"]);

    save_csv_point_trails(this->dir_name);


    std::cout << "\n----<<<< Kalman filter >>>>----" << std::endl;
    generate_trail_sequences();
    std::cout << "mode:\t" << this->gen_restrictions.int_data["kalman_mode"] << std::endl;
    Kalman_filter(this->gen_restrictions.int_data["kalman_mode"]);
    generate_err();

    show_score();
    save_scopes(this->dir_name + "scores.txt");

    save_csv_trail_sequences(this->dir_name + "trail_sequences\\");
    save_csv_state_estimated(this->dir_name + "states_estimated\\");
    save_csv_err(this->dir_name + "errors\\");
};