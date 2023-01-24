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
	// сет и гет
	Measurement_vector() {};
};

class Trail_sequence {
public:

	int start;
	int end;
	double time_discret;

	vector<double> timestamps;
	vector<State_vector> model_state_vector;
	vector<Measurement_vector> model_measurement_vector;

	Trail_sequence() {};
	// инициализация
	// дальше работать, чтобы оно создавало трейлы
};