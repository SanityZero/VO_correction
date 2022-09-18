#pragma once

#define M_PI (double)3.14159265358979323846
#define zeroPoint3d cv::Point3d(0,0,0)

#define State_type_HEADER(sep) string("\"velx\"") + sep + "\"vely\"" + sep + "\"velz\"" + sep + "\"accelx\"" + sep + "\"accely\"" + sep + "\"accelz\"" + sep + "\"orientx\"" + sep + "\"orienty\"" + sep + "\"orientz\"" + sep + "\"anqular_velx\"" + sep + "\"anqular_vely\"" + sep + "\"anqular_velz\"" + sep + "\"anqular_accelx\"" + sep + "\"anqular_accely\"" + sep + "\"anqular_accelz\""

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

    void read_csv(std::string line, std::string sep = ",");
    std::string get_csv_data(std::string sep = ",");
};

class Line_track_type {
public:
    cv::Point2d start;
    cv::Point2d end;
    double time;
    Line_track_type() {};
    Line_track_type(cv::Point2d s, cv::Point2d e, double _time) : start(s), end(e), time(_time) {};
    Line_track_type(std::vector<double> double_vec) {
        if (double_vec.size() != 5) {
            std::cout << "Line_track_type init wrong size" << std::endl;
            return;
        };
        this->start = cv::Point2d(double_vec[0], double_vec[1]);
        this->end = cv::Point2d(double_vec[2], double_vec[3]);
        this->time = double_vec[4];
    };

    double len();
    cv::Point2d part(double dist = 0);
    State_type orientation(double dist);

    std::string get_csv_data(std::string sep = ",") {
        std::string result = std::to_string(start.x) + sep + std::to_string(start.y) + sep;
        result += std::to_string(end.x) + sep + std::to_string(end.y) + sep;
        result += std::to_string(time);
        return result;
    };
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
    Corner_type(std::vector<double> double_vec) {
        if (double_vec.size() != 10) {
            std::cout << "Corner_type init wrong size" << std::endl;
            return;
        };
        this->start = cv::Point2d(double_vec[0], double_vec[1]);
        this->start_vec = cv::Point2d(double_vec[2], double_vec[3]);
        this->end = cv::Point2d(double_vec[4], double_vec[5]);
        this->center = cv::Point2d(double_vec[6], double_vec[7]);
        this->angle = double_vec[8];
        this->time = double_vec[9];
    };

    double len();
    cv::Point2d part(double dist);
    State_type orientation(double dist);

    std::string get_csv_data(std::string sep = ",") {
        std::string result = std::to_string(start.x) + sep + std::to_string(start.y) + sep;
        result += std::to_string(start_vec.x) + sep + std::to_string(start_vec.y) + sep;
        result += std::to_string(end.x) + sep + std::to_string(end.y) + sep;
        result += std::to_string(center.x) + sep + std::to_string(center.y) + sep;
        result += std::to_string(angle) + sep + std::to_string(time);
        return result;
    };
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

    Track_part_type(std::vector<double> double_vec) {
        std::vector<double>::const_iterator line_start = double_vec.begin();
        std::vector<double>::const_iterator turn_start = line_start + 5;
        std::vector<double>::const_iterator turn_end = turn_start + 10;
        std::vector<double>::const_iterator vec_end = double_vec.end();

        std::vector<double> line_ini_vec(line_start, turn_start);
        std::vector<double> turn_ini_vec(turn_start, turn_end);
        std::vector<double> self_ini_vec(turn_end, vec_end);

        //for (double item : line_ini_vec)
        //    std::cout << "l" << std::to_string(item) << std::endl;

        //for (double item : turn_ini_vec)
        //    std::cout << "t" << std::to_string(item) << std::endl;

        //for (double item : self_ini_vec)
        //    std::cout << "s" << std::to_string(item) << std::endl;

        this->line = Line_track_type(line_ini_vec);
        this->turn = Corner_type(turn_ini_vec);
        this->exit_vec = cv::Point2d(self_ini_vec[0], self_ini_vec[1]);
        this->end = cv::Point2d(self_ini_vec[2], self_ini_vec[3]);
    };

    double len();
    cv::Point2d part(double dist);
    State_type orientation(double dist);

    std::string get_csv_data(std::string sep = ";") {
        std::string result = "";
        result += line.get_csv_data(sep) + sep;
        result += turn.get_csv_data(sep) + sep;
        result += std::to_string(exit_vec.x) + sep + std::to_string(exit_vec.y) + sep;
        result += std::to_string(end.x) + sep + std::to_string(end.y);
        return result;
    };
};

//inline Point3d integrator(Point3d x_cur, Point3d x_prev, Point3d y_prev, double T) {
//    return -(T / 2) * (x_cur + x_prev) + y_prev;
//};
