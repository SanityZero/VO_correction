#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "Tests.h"

using namespace std;
using namespace cv;

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
            if (tp != "") {
                tp = tp.substr(tp.find(separator) + separator.length());
                if (int_data.size() == 0) int_data.push_back(stoi(tp));
                else
                    float_data.push_back(stod(tp));
            };
        }
        restr_file.close(); //close the file object.
    };
    this->set(filename, int_data, float_data);
};

void Test_model::Test_model_restrictions::set(string _filename, vector<int> _int_data, vector<double> _float_data) {
    this->filename = _filename;
    if ((_int_data.size() == 1) and (_float_data.size() == 38)) {
        int_data["max_track_parts"] = _int_data[0];

        double_data["dicret"] = _float_data[0];
        double_data["min_line_length"] = _float_data[1];
        double_data["max_line_length"] = _float_data[2];
        double_data["mean_corner_radius"] = _float_data[3];
        double_data["stddev_radius"] = _float_data[4];
        double_data["mean_corner_angle"] = _float_data[5] * M_PI / 180;
        double_data["stddev_angle"] = _float_data[6] * M_PI / 180;

        double_data["average_vel"] = _float_data[7];
        double_data["stddev_vel"] = _float_data[8];

        double_data["T"] = _float_data[9];
        double_data["U1"] = _float_data[10];
        double_data["U2"] = _float_data[11];

        double_data["focus"] = _float_data[12];

        double_data["extra_border"] = _float_data[13];

        double_data["z_lim_min"] = _float_data[14];
        double_data["z_lim_max"] = _float_data[15];

        double_data["grid_step_x"] = _float_data[16];
        double_data["grid_step_y"] = _float_data[17];
        double_data["grid_step_z"] = _float_data[18];

        double_data["grid_disp"] = _float_data[19];

        int_data["camera_matrix_x"] = int(_float_data[20]);
        int_data["camera_matrix_y"] = int(_float_data[21]);
        int_data["camera_frame_size_x"] = int(_float_data[22]);
        int_data["camera_frame_size_y"] = int(_float_data[23]);

        double_data["camera_FOV_xoy"] = _float_data[24] * M_PI / 180;
        double_data["camera_FOV_zoy"] = _float_data[25] * M_PI / 180;

        double_data["camera_fitting_x"] = _float_data[26] * M_PI / 180;
        double_data["camera_fitting_y"] = _float_data[27] * M_PI / 180;
        double_data["camera_fitting_z"] = _float_data[28] * M_PI / 180;

        int_data["s_points_generation_mode"] = int(_float_data[29]);
        int_data["camera_proection_mode"] = int(_float_data[30]);
        int_data["kalman_mode"] = int(_float_data[31]);

        int_data["save_load_track_model"] = int(_float_data[32]);
        int_data["save_load_motion_model"] = int(_float_data[33]);
        int_data["save_load_s_points"] = int(_float_data[34]);
        int_data["save_load_bins_model"] = int(_float_data[35]);
        int_data["save_load_camera_model"] = int(_float_data[36]);
        int_data["save_load_ts_model"] = int(_float_data[37]);
    }
    else
        cout << "Test_model_restrictions initial arrays sizes dont match" << endl;
        cout << _int_data.size() << " "<< _float_data.size() << endl;

};