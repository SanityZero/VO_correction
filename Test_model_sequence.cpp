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
// State_vector
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