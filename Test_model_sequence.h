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

	void set_from_vector(vector<double> csv_data);
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
	void set_from_vector(vector<double> csv_data);
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

	void set_from_vector(vector<double> csv_data);
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

	void set_start(int _start, double _timestamp, State_vector _state_vec, Measurement_vector _mes_vec, Control_vector _control_vector);
	void push_back(double _timestamp, State_vector _state_vec, Measurement_vector _mes_vec, Control_vector _control_vector);
	vector<int> range();

	string get_csv_line(int _number, std::string _sep = ";");
	void read_csv_line(std::string line, std::string _sep = ";");
	void update_start_end() ;
	void read_csv(std::string filename, std::string _sep = ";");

	std::vector<Point3d> get_pose_vec();
	std::vector<Point3d> get_orient_vec();

	void save_csv_trail_sequence(std::string _filename, std::string _sep = ";");
};