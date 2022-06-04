#pragma once

#define M_PI (double)3.14159265358979323846
#define zeroPoint3d cv::Point3d(0,0,0)

double min(double a, double b, double c);
double max(double a, double b, double c);
cv::Point2d rotate2d(cv::Point2d vec, double angle);
double length(cv::Point2d vec);
cv::Point2d get_point_vect(cv::Point2d end, cv::Point2d start = cv::Point2d(0, 0));
cv::Point2d get_norm_vect(cv::Point2d end, cv::Point2d start = cv::Point2d(0, 0));
cv::Point2d get_arc_end_point(cv::Point2d cent, cv::Point2d start, double angle);
cv::Point3d toAngle3d(cv::Point3d vec);
cv::Point3d normalize(cv::Point3d vec);

class State_type {
public:
    cv::Point3d vel;
    cv::Point3d accel;
    cv::Point3d orient;
    cv::Point3d anqular_vel;
    cv::Point3d anqular_accel;

    State_type(
        cv::Point3d _vel = zeroPoint3d,
        cv::Point3d _accel = zeroPoint3d,
        cv::Point3d _orient = zeroPoint3d,
        cv::Point3d _anqular_vel = zeroPoint3d,
        cv::Point3d _anqular_accel = zeroPoint3d
    ) : vel(_vel), accel(_accel), orient(_orient), anqular_vel(_anqular_vel), anqular_accel(_anqular_accel) {};

    State_type(cv::Point3d _arg, int _arg_num);

    void change(cv::Point3d _arg, int _arg_num);
    void change_vel(cv::Point3d _arg) { this->change(_arg, 0); };
    void change_accel(cv::Point3d _arg) { this->change(_arg, 1); };
    void change_orient(cv::Point3d _arg) { this->change(_arg, 2); };
    void change_anqular_vel(cv::Point3d _arg) { this->change(_arg, 3); };
    void change_anqular_accel(cv::Point3d _arg) { this->change(_arg, 4); };
};

class Line_track_type {
public:
    cv::Point2d start;
    cv::Point2d end;
    double time;
    Line_track_type() {};
    Line_track_type(cv::Point2d s, cv::Point2d e, double _time) : start(s), end(e), time(_time) {};

    double len();
    cv::Point2d part(double dist = 0);
    State_type orientation(double dist);
};

class Corner_type {
public:
    cv::Point2d start;
    cv::Point2d start_vec;
    cv::Point2d end;
    cv::Point2d center;
    double angle;
    double time;

    Corner_type() {};
    Corner_type(cv::Point2d s, cv::Point2d e, cv::Point2d c, double angl, double _time, cv::Point2d _start_vec);

    double len();
    cv::Point2d part(double dist);
    State_type orientation(double dist);
};

class Track_part_type {
public:
    Line_track_type line;
    Corner_type turn;
    cv::Point2d exit_vec;
    cv::Point2d end;

    Track_part_type(
        cv::Point2d start,
        cv::Point2d start_p_vec,
        double mean_line_length,
        double stddev_line,
        double mean_corner_radius,
        double stddev_radius,
        double min_corner_angle,
        double max_corner_angle,
        double average_vel,
        double stddev_vel
    );

    double len();
    cv::Point2d part(double dist);
    State_type orientation(double dist);
};

//inline Point3d integrator(Point3d x_cur, Point3d x_prev, Point3d y_prev, double T) {
//    return -(T / 2) * (x_cur + x_prev) + y_prev;
//};
