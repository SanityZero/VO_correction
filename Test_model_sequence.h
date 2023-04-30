#pragma once

#include <fstream>

//////////////////////////////////////////////////////////////////////////////////////////////
class State_vector {
public:
	Point3d cam_pose;
	Point3d orient;
	Point3d cam_vel;
	Point3d s_pose;

	State_vector() {};
	State_vector(vector<double> csv_data){
		this-> set_from_vector(csv_data);
	};
	State_vector(Point3d _cam_pose, Point3d _orient, Point3d _cam_vel, Point3d _s_pose) : cam_pose(_cam_pose), orient(_orient), cam_vel(_cam_vel), s_pose(_s_pose){};

	void set(Point3d _cam_pose, Point3d _orient, Point3d _cam_vel, Point3d _s_pose);
	void set_cam_pose(Point3d _cam_pose);
	void set_orient(Point3d _orient);
	void set_cam_vel(Point3d _cam_vel);
	void set_s_pose(Point3d _s_pose);

	Point3d get_cam_pose();
	Point3d get_orient();
	Point3d get_cam_vel();
	Point3d get_s_pose();

	double get(int _number);
	string get_csv_line(std::string _sep = ";");

	void set_from_vector(vector<double> csv_data) {
		this->set_cam_pose(Point3d(csv_data[0], csv_data[1], csv_data[2]));
		this->set_orient(Point3d(csv_data[3], csv_data[4], csv_data[5]));
		this->set_cam_vel(Point3d(csv_data[6], csv_data[7], csv_data[8]));
		this->set_s_pose(Point3d(csv_data[9], csv_data[10], csv_data[11]));
	};
};

//////////////////////////////////////////////////////////////////////////////////////////////
class Measurement_vector {
public:
	Point2d proection;

	Measurement_vector() {};
	Measurement_vector(vector<double> csv_data) {
		this->set_from_vector(csv_data);
	};
	Measurement_vector(Point2d _poect);

	double get(int _number);
	string get_csv_line(std::string _sep = ";");
	Point2d get_point2d();

	void set(Point2d _poect);
	void set_from_vector(vector<double> csv_data) {
		this->set(Point2d(csv_data[0], csv_data[1]));
	};
};

class Control_vector {
public:
	Point3d accel;
	Point3d w;

	Control_vector() {};
	Control_vector(vector<double> csv_data){
		this->set_from_vector(csv_data);
	};
	Control_vector(Point3d _accel, Point3d _w) : accel(_accel), w(_w) {};

	double get(int _number);
	string get_csv_line(std::string _sep = ";");
	void set_accel(Point3d _accel);
	void set_w(Point3d _w);

	void set_from_vector(vector<double> csv_data) {
		this->set_accel(Point3d(csv_data[0], csv_data[1], csv_data[2]));
		this->set_w(Point3d(csv_data[3], csv_data[4], csv_data[5]));
	};
};

//////////////////////////////////////////////////////////////////////////////////////////////
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

	void read_csv_line(std::string line, std::string _sep = ";") {
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

	void read_csv(std::string filename, std::string _sep = ";") {
		ifstream fin(filename);
		char buffer[255];
		fin.getline(buffer, 255);

		while (fin.getline(buffer, 255)) {
			string line(buffer);

			this->read_csv_line(line, _sep);
		};
		fin.close();
	};

	std::vector<Point3d> get_pose_vec() {
		std::vector<Point3d> res;

		for (int i : this->range()) res.push_back(this->model_state_vector[i].get_cam_pose());
		return res;
	};

	std::vector<Point3d> get_orient_vec() {
		std::vector<Point3d> res;

		for (int i : this->range()) res.push_back(this->model_state_vector[i].get_orient());
		return res;
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