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
    cout << "1)\t" << this->max_track_parts << endl;
    cout << "2)\t" << this->dicret << endl;
    cout << "3)\t" << this->min_line_length << endl;
    cout << "4)\t" << this->max_line_length << endl;
    cout << "5)\t" << this->mean_corner_radius << endl;
    cout << "6)\t" << this->stddev_radius << endl;
    cout << "7)\t" << this->mean_corner_angle << endl;
    cout << "8)\t" << this->stddev_angle << endl;
    cout << "9)\t" << this->average_vel << endl;
    cout << "10\t)" << this->stddev_vel << endl;
    cout << "11)\t" << this->T << endl;
    cout << "12)\t" << this->U1 << endl;
    cout << "13)\t" << this->U2 << endl;
    cout << "14)\t" << this->f << endl;
    cout << "15)\t" << this->extra_border << endl;
    cout << "16)\t" << this->z_lim_min << endl;
    cout << "17)\t" << this->z_lim_min << endl;
    cout << "18)\t" << this->grid_step_x << endl;
    cout << "19)\t" << this->grid_step_y << endl;
    cout << "20)\t" << this->grid_step_z << endl;
    cout << "21)\t" << this->grid_disp << endl;
    cout << "22)\t" << this->camera_matrix_x << endl;
    cout << "23)\t" << this->camera_matrix_y << endl;
    cout << "24)\t" << this->camera_frame_size_x << endl;
    cout << "25)\t" << this->camera_frame_size_y << endl;
    cout << "26)\t" << this->camera_FOV_xoy << endl;
    cout << "27)\t" << this->camera_FOV_zoy << endl;
    cout << "28)\t" << this->s_points_generation_mode << endl;
    cout << "29)\t" << this->camera_proection_mode << endl;
};

void Test_model::Test_model_restrictions::set(string filename, vector<int> int_data, vector<double> float_data) {
    this->filename = filename;
    if ((int_data.size() == 1) and (float_data.size() == 28)) {
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

        this->f = float_data[12];

        this->extra_border = float_data[13];
        this->z_lim_min = float_data[14];
        this->z_lim_max = float_data[15];
        this->grid_step_x = float_data[16];
        this->grid_step_y = float_data[17];
        this->grid_step_z = float_data[18];
        this->grid_disp = float_data[19];

        this->camera_matrix_x = int(float_data[20]);
        this->camera_matrix_y = int(float_data[21]);
        this->camera_frame_size_x = int(float_data[22]);
        this->camera_frame_size_y = int(float_data[23]);

        this->camera_FOV_xoy = float_data[24] * M_PI / 180;
        this->camera_FOV_zoy = float_data[25] * M_PI / 180;

        this->s_points_generation_mode = int(float_data[26]);
        this->camera_proection_mode = int(float_data[27]);
    }
    else
        cout << "Test_model_restrictions initial arrays sizes dont match" << endl;

};