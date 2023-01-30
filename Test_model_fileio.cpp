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

void Test_model::save_csv_point_trails(string dirname, string sep) {
    cout << "save_csv_point_trails" << endl;
    string dir_frames = this->dir_name + "trails\\";
    string cmd_clear_image_dir = "del /f /q " + dir_frames;
    system(cmd_clear_image_dir.c_str());

    cout << "start";
    for (vector<Point2d> trail : this->point_trails) {
        cout << ".";
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
    };
    cout << "end" << endl;

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