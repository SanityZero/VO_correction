#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/core.hpp>

typedef cv::Point2d Point2d;
typedef cv::Point3d Point3d;
typedef cv::Point2i Point2i;
//using namespace cv;
using namespace std;
#include "Test_model_sequence.h"
//////////////////////////////////////////////////////////////////////////////////////////////
// Trail_sequence
void Trail_sequence::set_start(int _start, double _timestamp, State_vector _state_vec, Measurement_vector _mes_vec, Control_vector _control_vector) {
	this->push_back(_timestamp, _state_vec, _mes_vec, _control_vector);
	this->start = _start;
	this->end = _start;
};

void Trail_sequence::push_back(double _timestamp, State_vector _state_vec, Measurement_vector _mes_vec, Control_vector _control_vector) {
	this->timestamps.push_back(_timestamp);
	this->model_state_vector.push_back(_state_vec);
	this->model_measurement_vector.push_back(_mes_vec);
	this->model_control_vector.push_back(_control_vector);
	this->end++;
};

vector<int> Trail_sequence::range() {
	vector<int> res;
	int i = 0;
	for (double timestamp : this->timestamps) {
		res.push_back(i);
		i++;
	};
	return res;
};

string Trail_sequence::get_csv_line(int _number, std::string _sep) {
	std::string result = std::to_string(this->timestamps[_number]) + _sep;
	result += this->model_state_vector[_number].get_csv_line() + _sep;
	result += this->model_measurement_vector[_number].get_csv_line() + _sep;
	result += this->model_control_vector[_number].get_csv_line();
	return result;
};

void Trail_sequence::read_csv_line(std::string line, std::string _sep) {
	vector<double> values;

	size_t start_pos = 0;
	std::string from = ",";
	std::string to = ".";
	while ((start_pos = line.find(from, start_pos)) != std::string::npos) {
		line.replace(start_pos, from.length(), to);
		start_pos += to.length();
	}

	int pos;
	while ((pos = line.find(_sep)) != std::string::npos) {
		values.push_back(std::stod(line.substr(0, pos)));
		//std::cout << values << std::endl;
		line.erase(0, pos + _sep.length());
	};
	values.push_back(std::stod(line.substr(0, pos)));

	vector<double> state_init_vec;
	vector<double> measurement_init_vec;
	vector<double> control_init_vec;

	for (int i = 0; i < 12; i++)
		state_init_vec.push_back(values[i + 1]);

	for (int i = 12; i < 14; i++)
		measurement_init_vec.push_back(values[i + 1]);

	for (int i = 14; i < 20; i++)
		control_init_vec.push_back(values[i + 1]);

	this->timestamps.push_back(values[0]);
	this->model_state_vector.push_back(State_vector(state_init_vec));
	this->model_measurement_vector.push_back(Measurement_vector(measurement_init_vec));
	this->model_control_vector.push_back(Control_vector(control_init_vec));
};

void Trail_sequence::update_start_end() {
	this->start = this->timestamps[0];
	this->end = this->timestamps[this->timestamps.size() - 1];
};

void Trail_sequence::read_csv(std::string filename, std::string _sep) {
	ifstream fin(filename);

	if (!fin.is_open())
		cout << "failed to open file\n";

	string line;
	getline(fin, line);
	while (getline(fin, line)) {

		this->read_csv_line(line, _sep);
	};
	this->update_start_end();
	fin.close();
};

std::vector<Point3d> Trail_sequence::get_pose_vec() {
	std::vector<Point3d> res;

	for (int i : this->range()) res.push_back(this->model_state_vector[i].get_cam_pose());
	return res;
};

std::vector<Point3d> Trail_sequence::get_orient_vec() {
	std::vector<Point3d> res;

	for (int i : this->range()) res.push_back(this->model_state_vector[i].get_orient());
	return res;
};

void Trail_sequence::save_csv_trail_sequence(std::string _filename, std::string _sep) {
	std::vector<std::string> csv_data;
	for (int i : this->range()) {
		csv_data.push_back(this->get_csv_line(i));
	};

	std::vector<std::string> header;
	header.push_back("timest");

	header.push_back("sv_cam_posex");
	header.push_back("sv_cam_posey");
	header.push_back("sv_cam_posez");

	header.push_back("sv_orientx");
	header.push_back("sv_orienty");
	header.push_back("sv_orientz");

	header.push_back("sv_cam_velx");
	header.push_back("sv_cam_vely");
	header.push_back("sv_cam_velz");

	header.push_back("sv_s_posex");
	header.push_back("sv_s_posey");
	header.push_back("sv_s_posez");


	header.push_back("mv_proectionx");
	header.push_back("mv_proectiony");


	header.push_back("cv_accelx");
	header.push_back("cv_accely");
	header.push_back("cv_accelz");

	header.push_back("cv_wx");
	header.push_back("cv_wy");
	header.push_back("cv_wz");

	header.push_back(std::to_string(this->start));
	header.push_back(std::to_string(this->end));

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


//////////////////////////////////////////////////////////////////////////////////////////////
// State_vector
void State_vector::set_from_vector(vector<double> csv_data) {
	this->set_cam_pose(Point3d(csv_data[0], csv_data[1], csv_data[2]));
	this->set_orient(Point3d(csv_data[3], csv_data[4], csv_data[5]));
	this->set_cam_vel(Point3d(csv_data[6], csv_data[7], csv_data[8]));
	this->set_s_pose(Point3d(csv_data[9], csv_data[10], csv_data[11]));
};

void State_vector::set(Point3d _cam_pose, Point3d _orient, Point3d _cam_vel, Point3d _s_pose) {
	this->cam_pose = _cam_pose;
	this->orient = _orient;
	this->cam_vel = _cam_vel;
	this->s_pose = _s_pose;
};

void State_vector::set_cam_pose(Point3d _cam_pose) {
	this->cam_pose = _cam_pose;
};

void State_vector::set_orient(Point3d _orient) {
	this->orient = _orient;
};

void State_vector::set_cam_vel(Point3d _cam_vel) {
	this->cam_vel = _cam_vel;
};

void State_vector::set_s_pose(Point3d _s_pose) {
	this->s_pose = _s_pose;
};

Point3d State_vector::get_cam_pose() {
	return this->cam_pose;
};

Point3d State_vector::get_orient() {
	return this->orient;
};

Point3d State_vector::get_cam_vel() {
	return this->cam_vel;
};

Point3d State_vector::get_s_pose() {
	return this->s_pose;
};

double State_vector::get(int _number) {
	switch (_number) {
	case 0:	return this->cam_pose.x;
	case 1: return this->cam_pose.y;
	case 2:	return this->cam_pose.z;

	case 3:	return this->orient.x;
	case 4:	return this->orient.y;
	case 5:	return this->orient.z;

	case 6:	return this->cam_vel.x;
	case 7:	return this->cam_vel.y;
	case 8:	return this->cam_vel.z;

	case 9:	return this->s_pose.x;
	case 10:	return this->s_pose.y;
	case 11:	return this->s_pose.z;
	};
};

string State_vector::get_csv_line(std::string _sep) {
	std::string result = std::to_string(this->get(0)) + _sep;
	result += std::to_string(this->get(1)) + _sep;
	result += std::to_string(this->get(2)) + _sep;
	result += std::to_string(this->get(3)) + _sep;
	result += std::to_string(this->get(4)) + _sep;
	result += std::to_string(this->get(5)) + _sep;
	result += std::to_string(this->get(6)) + _sep;
	result += std::to_string(this->get(7)) + _sep;
	result += std::to_string(this->get(8)) + _sep;
	result += std::to_string(this->get(9)) + _sep;
	result += std::to_string(this->get(10)) + _sep;
	result += std::to_string(this->get(11));
	return result;
};

//////////////////////////////////////////////////////////////////////////////////////////////
// Measurement_vector
void Measurement_vector::set_from_vector(vector<double> csv_data) {
	this->set(Point2d(csv_data[0], csv_data[1]));
};

Measurement_vector::Measurement_vector(Point2d _poect) {
	this->proection = _poect;
};

double Measurement_vector::get(int _number) {
	switch (_number) {
	case 0:
		return this->proection.x;
	case 1:
		return this->proection.y;
	};
};

string Measurement_vector::get_csv_line(std::string _sep) {
	std::string result = std::to_string(this->get(0)) + _sep;
	result += std::to_string(this->get(1));
	return result;
};

Point2d Measurement_vector::get_point2d() {
	return this->proection;
};

void Measurement_vector::set(Point2d _poect) {
	this->proection = _poect;
};

//////////////////////////////////////////////////////////////////////////////////////////////
//Control_vector
void Control_vector::set_from_vector(vector<double> csv_data) {
	this->set_accel(Point3d(csv_data[0], csv_data[1], csv_data[2]));
	this->set_w(Point3d(csv_data[3], csv_data[4], csv_data[5]));
};

double Control_vector::get(int _number) {
	switch (_number) {
	case 0:
		return this->accel.x;
	case 1:
		return this->accel.y;
	case 2:
		return this->accel.z;

	case 3:
		return this->w.x;
	case 4:
		return this->w.y;
	case 5:
		return this->w.z;
	};
};

string Control_vector::get_csv_line(std::string _sep) {
	std::string result = std::to_string(this->get(0)) + _sep;
	result += std::to_string(this->get(1)) + _sep;
	result += std::to_string(this->get(2)) + _sep;
	result += std::to_string(this->get(3)) + _sep;
	result += std::to_string(this->get(4)) + _sep;
	result += std::to_string(this->get(5));
	return result;
};

void Control_vector::set_accel(Point3d _accel) {
	this->accel = _accel;
};

void Control_vector::set_w(Point3d _w) {
	this->w = _w;
};