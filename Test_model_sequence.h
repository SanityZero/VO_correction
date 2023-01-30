#pragma once

class State_vector {
public:
	Point3d cam_pose;
	Point3d orient;
	Point3d cam_vel;
	Point3d s_pose;

	State_vector() {};

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
		double res;
		switch (_number) {
		case 0:
			res = this->cam_pose.x;
			break;
		case 1:
			res = this->cam_pose.y;
			break;
		case 2:
			res = this->cam_pose.z;
			break;

		case 3:
			res = this->orient.x;
			break;
		case 4:
			res = this->orient.y;
			break;
		case 5:
			res = this->orient.z;
			break;

		case 6:
			res = this->cam_vel.x;
			break;
		case 7:
			res = this->cam_vel.y;
			break;
		case 8:
			res = this->cam_vel.z;
			break;

		case 9:
			res = this->s_pose.x;
			break;
		case 10:
			res = this->s_pose.y;
			break;
		case 11:
			res = this->s_pose.z;
			break;
		};
		return res;
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
		double res;
		switch(_number){
			case 0:
				res = this->proection.x; 
				break;
			case 1:
				res = this->proection.y;
				break;
		};
		return res;
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
};