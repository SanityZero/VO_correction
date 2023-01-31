#pragma once

#include <fstream>

class State_vector {
public:
	Point3d cam_pose;
	Point3d orient;
	Point3d cam_vel;
	Point3d s_pose;

	State_vector() {};
	State_vector(Point3d _cam_pose, Point3d _orient, Point3d _cam_vel, Point3d _s_pose) : cam_pose(_cam_pose), orient(_orient), cam_vel(_cam_vel), s_pose(_s_pose){};

	void set(Point3d _cam_pose, Point3d _orient, Point3d _cam_vel, Point3d _s_pose) {
		this->cam_pose = _cam_pose;
		this->orient = _orient;
		this->cam_vel = _cam_vel;
		this->s_pose = _s_pose;
	};

	void set_cam_pose(Point3d _cam_pose) {
		this->cam_pose = _cam_pose;
	};

	void set_orient(Point3d _orient) {
		this->orient = _orient;
	};

	void set_cam_vel(Point3d _cam_vel) {
		this->cam_vel = _cam_vel;
	};

	void set_s_pose(Point3d _s_pose) {
		this->s_pose = _s_pose;
	};

	Point3d get_cam_pose() {
		return this->cam_pose;
	};

	Point3d get_orient() {
		return this->orient;
	};

	Point3d get_cam_vel() {
		return this->cam_vel;
	};

	Point3d get_s_pose() {
		return this->s_pose;
	};

	double get(int _number) {
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

	string get_csv_line(std::string _sep = ";") {
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
};

class Measurement_vector {
public:

	Point2d proection;

	Measurement_vector() {};
	Measurement_vector(Point2d _poect) {
		this->proection = _poect;
	};

	double get(int _number) {
		switch(_number){
			case 0:
				return this->proection.x;
			case 1:
				return this->proection.y;
		};
	};

	string get_csv_line(std::string _sep = ";") {
		std::string result = std::to_string(this->get(0)) + _sep;
		result += std::to_string(this->get(1));
		return result;
	};

	Point2d get_point2d() {
		return this->proection;
	};

	void set(Point2d _poect) {
		this->proection = _poect;
	};
};

class Control_vector {
public:

	Point3d accel;
	Point3d w;

	Control_vector() {};
	Control_vector(Point3d _accel, Point3d _w) : accel(_accel), w(_w) {};

	double get(int _number) {
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

	string get_csv_line(std::string _sep = ";") {
		std::string result = std::to_string(this->get(0)) + _sep;
		result += std::to_string(this->get(1)) + _sep;
		result += std::to_string(this->get(2)) + _sep;
		result += std::to_string(this->get(3)) + _sep;
		result += std::to_string(this->get(4)) + _sep;
		result += std::to_string(this->get(5));
		return result;
	};

	void set_accel(Point3d _accel) {
		this->accel = _accel;
	};

	void set_w(Point3d _w) {
		this->w = _w;
	};
};

class Trail_sequence {
public:

	int start;
	int end;

	vector<double> timestamps;
	vector<State_vector> model_state_vector;
	vector<Measurement_vector> model_measurement_vector;
	vector<Control_vector> model_control_vector;

	Trail_sequence() {};

	Trail_sequence(int _start, double _timestamp, State_vector _state_vec, Measurement_vector _mes_vec, Control_vector _control_vector): start(_start), end(_start - 1) {
		this->push_back(_timestamp, _state_vec, _mes_vec, _control_vector);
	};

	void set_start(int _start, double _timestamp, State_vector _state_vec, Measurement_vector _mes_vec, Control_vector _control_vector) {
		this->push_back(_timestamp, _state_vec, _mes_vec, _control_vector);
		this->start = _start;
		this->end = _start;
	};

	void push_back(double _timestamp, State_vector _state_vec, Measurement_vector _mes_vec, Control_vector _control_vector) {
		this->timestamps.push_back(_timestamp);
		this->model_state_vector.push_back(_state_vec);
		this->model_measurement_vector.push_back(_mes_vec);
		this->model_control_vector.push_back(_control_vector);
		this->end++;
	};

	vector<int> range() {
		vector<int> res;
		int i = 0;
		for (double timestamp : this->timestamps) {
			res.push_back(i);
			i++;
		};
		return res;
	};

	string get_csv_line(int _number, std::string _sep = ";") {
		std::string result = std::to_string(this->timestamps[_number]) + _sep;
		result += this->model_state_vector[_number].get_csv_line() + _sep;
		result += this->model_measurement_vector[_number].get_csv_line() + _sep;
		result += this->model_control_vector[_number].get_csv_line();
		return result;
	};

	void save_csv_trail_sequence(std::string _filename, std::string _sep = ";") {
		std::vector<std::string> csv_data;
		for (int i: this->range()) {
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
};