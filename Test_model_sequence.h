#pragma once

#include <fstream>

//////////////////////////////////////////////////////////////////////////////////////////////
class State_vector_type {
public:
	Point3d cam_pose;
	Point3d cam_orient;
	Point3d cam_vel;
	Point3d s_pose;

	State_vector_type() {};
	State_vector_type(vector<double> csv_data){
		this-> set_from_vector(csv_data);
	};
	State_vector_type(Point3d _cam_pose, Point3d _orient, Point3d _cam_vel, Point3d _s_pose) : cam_pose(_cam_pose), cam_orient(_orient), cam_vel(_cam_vel), s_pose(_s_pose){};

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

	void set_from_vector(vector<double> csv_data);
};

//////////////////////////////////////////////////////////////////////////////////////////////
class Measurement_vector_type {
public:
	Point2d proection;

	Measurement_vector_type() {};
	Measurement_vector_type(vector<double> csv_data) {
		this->set_from_vector(csv_data);
	};
	Measurement_vector_type(Point2d _poect);

	double get(int _number);
	string get_csv_line(std::string _sep = ";");
	Point2d get_point2d();

	void set(Point2d _poect);
	void set_from_vector(vector<double> csv_data);
};

class Control_vector_type {
public:
	Point3d accel;
	Point3d w;

	Control_vector_type() {};
	Control_vector_type(vector<double> csv_data){
		this->set_from_vector(csv_data);
	};
	Control_vector_type(Point3d _accel, Point3d _w) : accel(_accel), w(_w) {};

	double get(int _number);
	string get_csv_line(std::string _sep = ";");
	void set_accel(Point3d _accel);
	void set_w(Point3d _w);

	void set_from_vector(vector<double> csv_data);
};

//////////////////////////////////////////////////////////////////////////////////////////////
class Trail_sequence {
public:

	int start;
	int end;

	vector<double> timestamps;
	vector<State_vector_type> state_vector;
	vector<Measurement_vector_type> measurement_vector;
	vector<Control_vector_type> control_vector;

	Trail_sequence() {};

	Trail_sequence(int _start, double _timestamp, State_vector_type _state_vec, Measurement_vector_type _mes_vec, Control_vector_type _control_vector): start(_start), end(_start - 1) {
		this->push_back(_timestamp, _state_vec, _mes_vec, _control_vector);
	};

	void set_start(int _start, double _timestamp, State_vector_type _state_vec, Measurement_vector_type _mes_vec, Control_vector_type _control_vector);
	void push_back(double _timestamp, State_vector_type _state_vec, Measurement_vector_type _mes_vec, Control_vector_type _control_vector);
	vector<int> range();

	string get_csv_line(int _number, std::string _sep = ";");
	void read_csv_line(std::string line, std::string _sep = ";");
	void update_start_end() ;
	void read_csv(std::string filename, std::string _sep = ";");

	std::vector<Point3d> get_pose_vec();
	std::vector<Point3d> get_orient_vec();

	void save_csv_trail_sequence(std::string _filename, std::string _sep = ";");
};