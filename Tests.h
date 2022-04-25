#pragma once
#define M_PI (double)3.14159265358979323846
#define zeroPoint3d Point3d(0,0,0)

double min(double a, double b, double c);
double max(double a, double b, double c);
Point2d rotate2d(Point2d vec, double angle);
double length(Point2d vec);
Point2d get_point_vect(Point2d end, Point2d start = Point2d(0, 0));
Point2d get_norm_vect(Point2d end, Point2d start = Point2d(0, 0));
Point2d get_arc_end_point(Point2d cent, Point2d start, double angle);
Point3d toAngle3d(Point3d vec);
Point3d normalize(Point3d vec);

class State_type{
public:
    Point3d vel;
    Point3d accel;
    Point3d orient;
    Point3d anqular_vel;
    Point3d anqular_accel;

    State_type(
        Point3d _vel = zeroPoint3d,
        Point3d _accel = zeroPoint3d,
        Point3d _orient = zeroPoint3d,
        Point3d _anqular_vel = zeroPoint3d,
        Point3d _anqular_accel = zeroPoint3d
    ) :
        vel(_vel), accel(_accel), orient(_orient), anqular_vel(_anqular_vel), anqular_accel(_anqular_accel) {};

    State_type(
        Point3d _arg,
        int _arg_num
    ) {
        switch (_arg_num) {
        case 0:
            this->vel = _arg;
            this->accel = zeroPoint3d;
            this->orient = zeroPoint3d;
            this->anqular_vel = zeroPoint3d;
            this->anqular_accel = zeroPoint3d;
            break;
        case 1:
            this->vel = zeroPoint3d;
            this->accel = _arg;
            this->orient = zeroPoint3d;
            this->anqular_vel = zeroPoint3d;
            this->anqular_accel = zeroPoint3d;
            break;
        case 2:
            this->vel = zeroPoint3d;
            this->accel = zeroPoint3d;
            this->orient = _arg;
            this->anqular_vel = zeroPoint3d;
            this->anqular_accel = zeroPoint3d;
            break;
        case 3:
            this->vel = zeroPoint3d;
            this->accel = zeroPoint3d;
            this->orient = zeroPoint3d;
            this->anqular_vel = _arg;
            this->anqular_accel = zeroPoint3d;
            break;
        case 4:
            this->vel = zeroPoint3d;
            this->accel = zeroPoint3d;
            this->orient = zeroPoint3d;
            this->anqular_vel = zeroPoint3d;
            this->anqular_accel = _arg;
            break;
        };
    };

    void change(Point3d _arg, int _arg_num) {
        switch (_arg_num) {
        case 0:
            this->vel = _arg;
            break;
        case 1:
            this->accel = _arg;
            break;
        case 2:
            this->orient = _arg;
            break;
        case 3:
            this->anqular_vel = _arg;
            break;
        case 4:
            this->anqular_accel = _arg;
            break;
        };
    };

    void change_vel(Point3d _arg) { this->change(_arg, 0); };
    void change_accel(Point3d _arg) { this->change(_arg, 1); };
    void change_orient(Point3d _arg) { this->change(_arg, 2); };
    void change_anqular_vel(Point3d _arg) { this->change(_arg, 3); };
    void change_anqular_accel(Point3d _arg) { this->change(_arg, 4); };
       
};


class Line_track_type{
public:
    Point2d start;
    Point2d end;
    double time;
    Line_track_type() {};
    Line_track_type(Point2d s, Point2d e, double _time) : start(s), end(e), time(_time) {};

    double len() {
        return length(start - end);
    };

    Point2d part(double dist=0) {
        double ratio = dist / this->len();
        return start + ((end - start) * ratio);
    };

    State_type orientation(double dist=0);
};

class Corner_type{
public:
    Point2d start;
    Point2d start_vec;
    Point2d end;
    Point2d center;
    double angle;
    double time;

    Corner_type() {};
    Corner_type(
        Point2d s, 
        Point2d e, 
        Point2d c, 
        double angl, 
        double _time, 
        Point2d _start_vec
    ) : start(s), end(e), center(c), angle(angl), time(_time), start_vec(_start_vec) {};

    double len() {
        return length(this->center - this->start) * this->angle;
    };

    Point2d part(double dist) {
        return this->center + get_arc_end_point(this->start, this->center,  this->angle * (dist / this->len()));
    };

    State_type orientation(double dist=0);
};

class Track_part_type {
public:
    Line_track_type line;
    Corner_type turn;
    Point2d exit_vec;
    Point2d end;

    Track_part_type(
        Point2d start,
        Point2d start_p_vec,
        double mean_line_length, 
        double stddev_line, 
        double mean_corner_radius, 
        double stddev_radius,
        double min_corner_angle,
        double max_corner_angle,
        double average_vel
    );

    double len() {
        return line.len() + turn.len();
    };

    Point2d part(double dist) {
        dist -= line.len();
        if (dist == 0) return this->line.end;
        else if (dist < 0) {
            return this->line.part(dist + line.len());
        }
        else if (dist > 0) {
            return this->turn.part(dist);
        };
    };

    State_type orientation(double dist = 0) {
        dist -= line.len();
        if (dist == 0) return Point3d(this->line.end.x, this->line.end.y, 0);
        else if (dist < 0) {
            return this->line.orientation(dist + line.len());
        }
        else if (dist > 0) {
            return this->turn.orientation(dist);
        };
    };
};


class Test_model {
private:
    vector<Track_part_type> track;
    vector<double> track_length;
    vector<Pose_type> gt_point;
    vector<State_type> states;

    double total_length;

    Point2d part(double dist) {
        int i = 0;
        while (true) {
            dist -= this->track[i].len();
            if (dist == 0) return this->track[i].end;
            if (dist < 0) {
                return this->track[i].part(dist + this->track[i].len());
            };
            i++;
            if (i == track.size()) return Point2d(0, 0);
        };
    };

    State_type orientation(double dist = 0) {
        int i = 0;
        while (true) {
            dist -= this->track[i].len();
            if (dist == 0) this->track[i].orientation(dist + this->track[i].len());
            if (dist < 0) {
                return this->track[i].orientation(dist + this->track[i].len());
            };
            i++;
            if (i == track.size()) return State_type();
        };
    };

    void generate_gt_points(double delta_m, int point_num = 0);

    void generate_states(double delta_m, int point_num = 0);

    void generate_mesured_points(double mean_angle, double stddev_angle, double mean_pose, double pose_stddev) {
        default_random_engine generator;
        normal_distribution<double> distribution_angle(mean_angle, stddev_angle);
        normal_distribution<double> distribution_pose(mean_pose, pose_stddev);
    };

public:
    void generate_test_model(
        int max_track_parts, 
        double dicret,
        double mean_line_length,
        double stddev_line,
        double mean_corner_radius,
        double stddev_radius,
        double mean_corner_angle,
        double stddev_angle,
        double average_vel
    ) {
        track.push_back(Track_part_type(
            Point2d(0,0),
            Point2d(-1,0),
            mean_line_length,
            stddev_line,
            mean_corner_radius,
            stddev_radius,
            mean_corner_angle,
            stddev_angle,
            average_vel
        ));
        track_length.push_back(track[0].len());
        this->total_length = track_length[0];
        for (int i = 1; i < max_track_parts; i++) {
            track.push_back(Track_part_type(
                this->track[i - 1].end,
                this->track[i - 1].exit_vec,
                mean_line_length,
                stddev_line,
                mean_corner_radius,
                stddev_radius,
                mean_corner_angle,
                stddev_angle,
                average_vel
            ));
            track_length.push_back(track[i].len());
            this->total_length += track_length[i];
        };// сгенерировать трак в соответствии с ограничениями
        
        // сгенерировать гт данные по ограничениям т.е. набор значений 
        generate_states(dicret);
        generate_gt_points(dicret);
        // сгенерировать бинс данные по ограничениям, т.е. набор значений
        // расставить точки
        // сгенерировать изображения в соответствии с точками

    };

    void show_gt(string mode = "screen", bool pause_enable = false);
    void show_gt_measures(bool pause_enable = false);

    void print_states(string filename) {
        ofstream out;          // поток для записи
        out.open(filename); // окрываем файл для записи
        if (out.is_open())
        {
            //Point3d vel;
            //Point3d accel;
            //Point3d orient;
            //Point3d anqular_vel;
            //Point3d anqular_accel;
            for (int i = 0; i < this->states.size(); i++) {
                out << format(
                    "%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", 
                    this->states[i].vel.x, this->states[i].vel.y, this->states[i].vel.z,
                    this->states[i].accel.x, this->states[i].accel.y, this->states[i].accel.z,
                    this->states[i].orient.x, this->states[i].orient.y, this->states[i].orient.z,
                    this->states[i].anqular_vel.x, this->states[i].anqular_vel.y, this->states[i].anqular_vel.z,
                    this->states[i].anqular_accel.x, this->states[i].anqular_accel.y, this->states[i].anqular_accel.z
                );
            };   
        }
        out.close();
    };
};

DataSeq_model_Type generate_model(
    double mean_1,
    const int size,
    double stddev,
    Point3d w,
    Point3d ang_0,
    double deltatime,
    double accel_stddev,
    Point3d accel,
    Point3d vel_0
    );/*double mean_1,
    const int size,
    double stddev,
    Point3d w,
    Point3d ang_0,
    Point3d ang_0,
    double deltatime,
    double accel_stddev,
    Point3d accel = Point3d(0, 0, 0),
    Point3d vel_0 = Point3d(0, 0, 0)*/


void motion_Test(double accel_std = 1, double sko = 0.2, double delta = 0.004, double duration = 10);
void angle_Test(double w_std = 0.0001 * M_PI/180, Point3d vel_0 = Point3d(0,0,0), double sko = 0.000001 * M_PI/180, double delta = 0.004, double duration = 60);
