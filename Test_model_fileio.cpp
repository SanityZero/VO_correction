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
#include "Tests.h"

typedef cv::Point2d Point2d;
typedef cv::Point3d Point3d;
typedef cv::Point2i Point2i;

void Test_model::save_csv_time_err(std::string _filename, std::string _sep) {
    std::vector<std::string> csv_data;
    for (Point2i err_time : err_times) {
        csv_data.push_back(std::to_string(err_time.x) + _sep + std::to_string(err_time.y));
    };

    std::vector<std::string> header;
    header.push_back("start");
    header.push_back("end");

    std::string header_line = "";
    for (std::string item : header)
        header_line += "\"" + item + "\"" + _sep;


    ofstream fout(_filename);
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

void Test_model::save_csv_vector_Point3d(vector<Point3d> _vec, string _filename, int _start, int _end, std::string _sep) {
    std::vector<std::string> csv_data;
    for (int i = 0; i < _end - _start; i++) {
        csv_data.push_back(std::to_string(_start + i) + _sep + this->get_csv_Point3d(_vec[i], _sep));
    };

    std::vector<std::string> header;
    header.push_back("timest");

    header.push_back("x");
    header.push_back("y");
    header.push_back("z");

    header.push_back(std::to_string(_start));
    header.push_back(std::to_string(_end));

    std::string header_line = "";
    for (std::string item : header)
        header_line += "\"" + item + "\"" + _sep;


    ofstream fout(_filename);
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

void Test_model::save_csv_pose_err(std::string _dir, std::string _sep) {

    string cmd_clear_image_dir = "del /f /q " + _dir;
    system(cmd_clear_image_dir.c_str());

    int i = 0;
    cout << "save_csv_pose_err start" << endl;
    for (vector<Point3d> err_vec : this->pose_err) {
        this->save_csv_vector_Point3d(err_vec, _dir + to_string(i) + ".csv", this->err_times[i].x, this->err_times[i].y, _sep);
        i++;
    };
    cout << "save_csv_pose_err end" << endl;
};

void Test_model::save_csv_orient_err(std::string _dir, std::string _sep) {

    string cmd_clear_image_dir = "del /f /q " + _dir;
    system(cmd_clear_image_dir.c_str());

    int i = 0;
    cout << "save_csv_pose_err start" << endl;
    for (vector<Point3d> err_vec : this->orient_err) {
        this->save_csv_vector_Point3d(err_vec, _dir + to_string(i) + ".csv", this->err_times[i].x, this->err_times[i].y, _sep);
        i++;
    };
    cout << "save_csv_pose_err end" << endl;
};

void Test_model::save_csv_state_estimated(std::string _dir, std::string _sep) {

    string cmd_clear_image_dir = "del /f /q " + _dir;
    system(cmd_clear_image_dir.c_str());

    int i = 0;
    cout << "save_csv_state_estimated start" << endl;
    for (Trail_sequence state_estimated : states_estimated) {
        state_estimated.save_csv_trail_sequence(_dir + to_string(i) + ".csv", _sep);
        i++;
    };
    cout << "save_csv_state_estimated end" << endl;
};

void Test_model::save_csv_trail_sequences(std::string _dir, std::string _sep) {

    string cmd_clear_image_dir = "del /f /q " + _dir;
    system(cmd_clear_image_dir.c_str());

    int i = 0;
    cout << "save_csv_trail_sequences start" << endl;
    for (Trail_sequence trail_sequence : trail_sequences) {
        trail_sequence.save_csv_trail_sequence(_dir + to_string(i) + ".csv", _sep);
        i++;
    };
    cout << "save_csv_trail_sequences end" << endl;
};

Point2d Test_model::read_csv_Point2d(string _string, string _sep) {
    size_t pos = 0;
    string line(_string);

    std::vector<std::string> values;
    while ((pos = line.find(_sep)) != std::string::npos) {
        values.push_back(line.substr(0, pos));
        //std::cout << values << std::endl;
        line.erase(0, pos + _sep.length());
    };
    values.push_back(line);
    std::vector<double> double_buffer;
    for (std::string item : values) {
        item.replace(item.find(","), 1, ".");
        double_buffer.push_back(std::stod(item));
        //cout << "_" << stod(item) << "_" << endl;
    };
    return Point2d(double_buffer[0], double_buffer[1]);
};

Point2i Test_model::read_csv_Point2i(string _string, string _sep) {
    size_t pos = 0;
    string line(_string);

    std::vector<std::string> values;
    while ((pos = line.find(_sep)) != std::string::npos) {
        values.push_back(line.substr(0, pos));
        //std::cout << values << std::endl;
        line.erase(0, pos + _sep.length());
    };
    values.push_back(line);
    std::vector<int> int_buffer;
    for (std::string item : values) {
        //item.replace(item.find(","), 1, ".");
        int_buffer.push_back(std::stoi(item));
        //cout << "_" << stod(item) << "_" << endl;
    };
    return Point2d(int_buffer[0], int_buffer[1]);
};

string Test_model::get_csv_Point3d(Point3d _point, string _sep) {
    string result = to_string(_point.x) + _sep + to_string(_point.y) + _sep + to_string(_point.z);
    return result;
};

Point3d Test_model::read_csv_Point3d(string _string, string _sep) {
    size_t pos = 0;
    string line(_string);

    std::vector<std::string> values;
    while ((pos = line.find(_sep)) != std::string::npos) {
        values.push_back(line.substr(0, pos));
        //std::cout << values << std::endl;
        line.erase(0, pos + _sep.length());
    };
    values.push_back(line);
    std::vector<double> double_buffer;
    for (std::string item : values) {
        item.replace(item.find(","), 1, ".");
        double_buffer.push_back(std::stod(item));
        //cout << "_" << stod(item) << "_" << endl;
    };
    return Point3d(double_buffer[0], double_buffer[1], double_buffer[2]);
};

Mat Test_model::load_csv_Mat(string filename, Point2i _size, string _sep) {
    ifstream fin(filename);
    char buffer[255];

    Mat mat = Mat::zeros(_size.x, _size.y, CV_64F);

    vector<string> line_buffer;
    std::vector<double> double_buffer;

    while (fin.getline(buffer, 255)) {
        line_buffer.push_back(buffer);
        //cout << "_" << buffer << "_" << endl;

        size_t pos = 0;
        vector<string> values;
        string line(buffer);

        while ((pos = line.find(_sep)) != std::string::npos) {
            values.push_back(line.substr(0, pos));
            //std::cout << values << std::endl;
            line.erase(0, pos + _sep.length());
        };
        values.push_back(line);

        for (std::string item : values) {
            if (item.find(",") != -1) item.replace(item.find(","), 1, ".");
            while (item.find("\"") != -1) item.replace(item.find("\""), 1, "");
            double_buffer.push_back(std::stod(item));
            //cout << "_" << stod(item) << "_" << endl;
        }; //for (std::string item : values)

    }; //while (fin.getline(buffer, 255) && Mat_vec_vec.size() <= _size.x)
    fin.close();

    int cnt = 0;//index starts from 0
    for (double item : double_buffer)
    {
        int temprow = cnt / _size.y;
        int tempcol = cnt % _size.y;
        mat.at<double>(temprow, tempcol) = item;
        cnt++;
    }

    return mat.clone();
};

void Test_model::save_csv_err(std::string _dir, std::string _sep) {
    this->save_csv_time_err(_dir + "time_err.csv", _sep);
    this->save_csv_pose_err(_dir + "pose_err\\", _sep);
    this->save_csv_orient_err(_dir + "orient_err\\", _sep);
};

void Test_model::save_scopes(string filename) {
    vector<string> csv_data;
    cout << "save_scopes" << endl;
    csv_data.push_back(to_string(this->score_pose));
    csv_data.push_back(to_string(this->score_orient));

    ofstream fout(filename);

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

void Test_model::save_csv_camera_proections(string dirname, string sep) {
    cout << "save_csv_camera_proections" << endl;

    int computing_size = 0;
    for (vector<Point2i> frame : this->point_camera_proections) {
        computing_size += frame.size();
    };

    int current_progress = 0;
    //cout << "\033[1K\r" << to_string(100 * (double)current_progress / (double)computing_size) + "%";

    string dir_frames = this->dir_name + "proections\\";
    string dir_create = "mkdir " + dir_frames;
    system(dir_create.c_str());

    string cmd_clear_image_dir = "del /f /q " + dir_frames;
    system(cmd_clear_image_dir.c_str());
    int frame_num = 0;

    for (vector<Point2i> frame : this->point_camera_proections) {
        cout << "\033[1K\r" << to_string(100 * (double)current_progress / (double)computing_size) + "%";
        vector<string> csv_data;
        for (int i = 1; i < frame.size(); i++) {
            csv_data.push_back(to_string(int(frame[i].x)) + sep + to_string(int(frame[i].y)));
        };

        string filename = dirname + "proections\\" + to_string(frame_num++) + ".csv";
        //cout << filename << endl;
        ofstream fout(filename);
        fout << "x" + sep + "y" << '\n';

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
        current_progress += frame.size();
    };
    cout << "\033[1K\r";


};

void Test_model::save_csv_point_trails(string dirname, string sep) {
    cout << "save_csv_point_trails" << endl;

    int computing_size = 0;
    for (vector<Point2d> trail : this->point_trails) {
        computing_size += trail.size();
    };

    int current_progress = 0;
    //cout << "\033[1K\r" << to_string(100 * (double)current_progress / (double)computing_size) + "%";

    string dir_frames = this->dir_name + "trails\\";
    string cmd_clear_image_dir = "del /f /q " + dir_frames;
    system(cmd_clear_image_dir.c_str());

    for (vector<Point2d> trail : this->point_trails) {
        cout << "\033[1K\r" << to_string(100 * (double)current_progress / (double)computing_size) + "%";
        vector<string> csv_data;
        for (int i = 1; i < trail.size(); i++) {
            csv_data.push_back(to_string(double(trail[i].x)) + sep + to_string(double(trail[i].y)));
        };

        string filename = dirname + "trails\\" + to_string(int(trail[0].x)) + ".csv";
        //cout << filename << endl;
        ofstream fout(filename);
        fout << "x" + sep + "y" << '\n';

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
        current_progress += trail.size();
    };
    cout << "\033[1K\r";
};

void Test_model::load_csv_s_points(string filename, string sep) {
    ifstream fin(filename);
    char buffer[255];
    fin.getline(buffer, 255);

    cout << "load_csv_s_points" << endl;

    vector<string> line_buffer;
    while (fin.getline(buffer, 255)) {
        line_buffer.push_back(buffer);
        string line(buffer);

        Point3d tmp = read_csv_Point3d(line, sep);
        this->s_points.push_back(tmp);
    };
    fin.close();
};