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
//using namespace std;

#include "Types.h"
#include "Track_part_type.h"
#include "Tests.h"

void Test_model::Test_track_model::load_csv_track(std::string filename, std::string sep) {
    std::cout << "load_csv_track" << std::endl;
    std::ifstream fin(filename);
    char buffer[255];
    fin.getline(buffer, 255);
    std::cout << "load_csv_track_model" << std::endl;
    std::vector<std::string> line_buffer;
    while (fin.getline(buffer, 255)) {
        line_buffer.push_back(buffer);
        //cout << "_" << buffer << "_" << std::endl;

        size_t pos = 0;
        std::vector<std::string> values;
        std::string line(buffer);
        while ((pos = line.find(sep)) != std::string::npos) {
            values.push_back(line.substr(0, pos));
            //std::cout << values << std::endl;
            line.erase(0, pos + sep.length());
        };
        values.push_back(line);

        std::vector<double> double_buffer;
        for (std::string item : values) {
            item.replace(item.find(","), 1, ".");
            double_buffer.push_back(stod(item));
            //cout << "_" << stod(item) << "_" << std::endl;
        }

        this->track.push_back(Track_part_type(double_buffer));
    };

    this->total_length = 0.0;
    for (Track_part_type tp : this->track) {
        this->track_length.push_back(tp.len());
        this->total_length += tp.len();
    };

    fin.close();
};

void Test_model::Test_track_model::save_csv_track(std::string filename, std::string sep) {
    std::vector<std::string> csv_data;
    std::cout << "save_csv_track" << std::endl;
    for (int i = 0; i < track.size(); i++) {
        csv_data.push_back(track[i].get_csv_line(sep) + sep + std::to_string(track_length[i]));
    };

    std::vector<std::string> header;
    header.push_back("Line_sx");
    header.push_back("Line_sy");
    header.push_back("Line_ex");
    header.push_back("Line_ey");
    header.push_back("Line_time");

    header.push_back("Turn_sx");
    header.push_back("Turn_sy");
    header.push_back("Turn_svx");
    header.push_back("Turn_svy");
    header.push_back("Turn_ex");
    header.push_back("Turn_ey");
    header.push_back("Turn_cx");
    header.push_back("Turn_cy");
    header.push_back("Turn_a");
    header.push_back("Turn_time");

    header.push_back("TP_evx");
    header.push_back("TP_evy");
    header.push_back("TP_ex");
    header.push_back("TP_ey");
    header.push_back("TP_len");

    header.push_back(std::to_string(track.size()));

    std::string header_line = "";
    for (std::string item : header)
        header_line += "\"" + item + "\"" + sep;


    std::ofstream fout(filename);
    fout << header_line << '\n';

    for (std::string row : csv_data) {

        size_t start_pos = 0;
        std::string from = ".";
        std::string to = ",";
        while ((start_pos = row.find(from, start_pos)) != std::string::npos) {
            row.replace(start_pos, from.length(), to);
            start_pos += to.length();
        }
        fout << row << '\n';
    };

    fout.close();
};


void Test_model::Test_track_model::generate_track(Test_model::Test_model_restrictions restr) {

    this->track.push_back(Track_part_type(
        cv::Point2d(0, 0),
        cv::Point2d(1, 0),
        restr.double_data["min_line_length"],
        restr.double_data["max_line_length"],
        restr.double_data["mean_corner_radius"],
        restr.double_data["stddev_radius"],
        restr.double_data["mean_corner_angle"],
        restr.double_data["stddev_angle"],
        restr.double_data["average_vel"],
        restr.double_data["stddev_vel"]
    ));

    this->track_length.push_back(this->track[0].len());
    this->total_length = track_length[0];
    for (int i = 1; i < restr.int_data["max_track_parts"]; i++) {
        track.push_back(Track_part_type(
            this->track[i - 1].end,
            this->track[i - 1].exit_vec,
            restr.double_data["min_line_length"],
            restr.double_data["max_line_length"],
            restr.double_data["mean_corner_radius"],
            restr.double_data["stddev_radius"],
            restr.double_data["mean_corner_angle"],
            restr.double_data["stddev_angle"],
            restr.double_data["average_vel"],
            restr.double_data["stddev_vel"]
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

cv::Point2d Test_model::Test_track_model::part(double dist) {
    int i = 0;
    while (true) {
        dist -= this->track[i].len();
        if (dist == 0) return this->track[i].end;
        if (dist < 0) {
            return this->track[i].part(dist + this->track[i].len());
        };
        i++;
        if (i == track.size()) return cv::Point2d(0, 0);
    };
};