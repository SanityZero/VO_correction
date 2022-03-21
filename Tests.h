#pragma once
#define M_PI (double)3.14159265358979323846

Point2d rotate(Point2d vec, double angle) {
    double rot_arr[2][2] = {
        {cos(angle), -sin(angle)},
        {sin(angle), cos(angle)}
    };
    Mat rot = Mat(2, 2, CV_64F, rot_arr);
    double radius_vec_arr[2][1] = {
        {vec.x},
        {vec.y}
    };
    Mat radius_vec = Mat(2, 1, CV_64F, radius_vec_arr);
    Mat end_vec = rot * radius_vec;
    return Point2d(end_vec.at<double>(0, 0), end_vec.at<double>(1, 0));
};

double length(Point2d vec) {
    return sqrt(vec.x * vec.x + vec.y * vec.y);
};

Point2d get_point_vect(Point2d end, Point2d start = Point2d(0, 0)) {
    return Point2d((end.x - start.x) / length(end - start), (end.y - start.y) / length(end - start));
};

Point2d get_norm_vect(Point2d end, Point2d start = Point2d(0, 0)) {
    return rotate(end - start, M_PI / 2)/length(end - start);
};

Point2d get_arc_end_point(Point2d cent, Point2d start, double angle) {

    return rotate(cent-start, angle);
};

Point3d get_angle_from_vec(Point3d vec) {
    Point3d res(0, 0, 0);

    return res;
};

class Line_track_type{
public:
    Point2d start;
    Point2d end;
    Line_track_type() {};
    Line_track_type(Point2d s, Point2d e) : start(s), end(e) {};

    double len() {
        return length(start - end);
    };

    Point2d part(double dist=0) {
        double ratio = dist / this->len();
        return start + ((end - start) * ratio);
    };

    Point2d orientation(double dist=0) {
        return get_point_vect(this->start, this->end);
    };
};

class Corner_type{
public:
    Point2d start;
    Point2d end;
    Point2d center;
    double angle;

    Corner_type() {};
    Corner_type(Point2d s, Point2d e, Point2d c, double angl) : start(s), end(e), center(c), angle(angl){};

    double len() {
        return length(this->center - this->start) * this->angle;
    };

    Point2d part(double dist) {
        return this->center + get_arc_end_point(this->center, this->start, -this->angle * (dist / this->len()));
    };

    Point2d orientation(double dist=0) {
        Point2d pt = this->part(dist);
        return get_norm_vect(this->center, pt);
    };
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
        double mean_corner_angle,
        double stddev_angle
    ) {
        default_random_engine generator;
        normal_distribution<double> distribution_line(mean_line_length, stddev_line);
        normal_distribution<double> distribution_radius(mean_corner_radius, stddev_radius);
        normal_distribution<double> distribution_angle(mean_corner_angle, stddev_angle);

        double line_len = distribution_line(generator);
        this->line = Line_track_type(start, Point2d(start.x + start_p_vec.x * line_len, start.y + start_p_vec.y * line_len));

        double radius = distribution_radius(generator);
        double angle = distribution_angle(generator);
        Point2d center = this->line.end + get_norm_vect(this->line.end, this->line.start) * radius;
        Point2d end = center + get_arc_end_point( this->line.end, center, angle);// not sure

        this->turn = Corner_type(this->line.end, end, center, angle);
        this->exit_vec = get_norm_vect(this->turn.end, this->turn.center);
        this->end = this->turn.end;
    };

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

    Point2d orientation(double dist = 0) {
        dist -= line.len();
        if (dist == 0) return this->line.end;
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

    Point2d orientation(double dist = 0) {
        int i = 0;
        while (true) {
            dist -= this->track[i].len();
            if (dist == 0) return this->track[i].end;
            if (dist < 0) {
                return this->track[i].orientation(dist);
            };
            i++;
            if (i == track.size()) return Point2d(0, 0);
        };
    };

    void generate_gt_points(double delta_m, int point_num = 0) {
        //нужно расставить точки в соответствии с заданной скоростью
        point_num = point_num == 0 ? this->total_length / delta_m : point_num;
        Point2d zero(1, 0);

        Pose_type pose1_tmp;
        pose1_tmp.lat = 0;
        pose1_tmp.lon = 0;
        pose1_tmp.alt = 0;

        pose1_tmp.roll = 0;
        pose1_tmp.pitch = 0;
        pose1_tmp.yaw = acos(zero.x * this->track[0].line.orientation().x);

        gt_point.push_back(pose1_tmp);

        for (int i = 1; i < point_num; i++) {
            Point2d pose_delta = this->part(i * delta_m);
            
            Pose_type pose_tmp;
            pose_tmp.lat = gt_point[i - 1].lat + pose_delta.x;
            pose_tmp.lon = gt_point[i - 1].lon + pose_delta.y;
            pose_tmp.alt = gt_point[i - 1].alt;

            pose_tmp.roll = 0;
            pose_tmp.pitch = 0;
            pose_tmp.yaw = acos(zero.x * this->orientation(i * delta_m).x);
            gt_point.push_back(pose_tmp);
        };
        //и выписать ориентации в каждой точке
    };

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
        double stddev_angle
    ) {
        track.push_back(Track_part_type(
            Point2d(0,0),
            Point2d(1,0),
            mean_line_length,
            stddev_line,
            mean_corner_radius,
            stddev_radius,
            mean_corner_angle,
            stddev_angle
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
                stddev_angle
            ));
            track_length.push_back(track[i].len());
            this->total_length += track_length[i];
        };// сгенерировать трак в соответствии с ограничениями
        
        // сгенерировать гт данные по ограничениям т.е. набор значений 
        generate_gt_points(dicret);
        // сгенерировать бинс данные по ограничениям, т.е. набор значений
        // расставить точки
        // сгенерировать изображения в соответствии с точками

    };

    void show_gt(string mode = "screen", bool pause_enable = true) {
        static Point2i img_size = Point2i(500, 500);
        Mat img(img_size.x, img_size.y, CV_8UC3, Scalar(255, 255, 255));
        int border = 50;
        double max_x = this->gt_point[0].lon;
        double max_y = this->gt_point[0].lat;
        double min_x = this->gt_point[0].lon;
        double min_y = this->gt_point[0].lat;

        for (int i = 0; i < this->gt_point.size(); i++)
        {
            max_x = this->gt_point[i].lon > max_x ? this->gt_point[i].lon : max_x;
            max_y = this->gt_point[i].lat > max_y ? this->gt_point[i].lat : max_y;

            min_x = this->gt_point[i].lon < min_x ? this->gt_point[i].lon : min_x;
            min_y = this->gt_point[i].lat < min_y ? this->gt_point[i].lat : min_y;
        };

        Point2d scale = Point2d(max_x - min_x, max_y - min_y);
        double im_scale = scale.x > scale.y ? (img_size.x - 2 * border) / scale.x : (img_size.y - 2 * border) / scale.y;
        Point2i zero_offset = -Point2i(min_x * im_scale, min_y * im_scale);

        for (int i = 1; i < this->gt_point.size(); i++)
        {
            Point2d next = Point2d(this->gt_point[i].lon, this->gt_point[i].lat);
            Point2d prev = Point2d(this->gt_point[i - 1].lon, this->gt_point[i - 1].lat);

            Point2i beg = Point2i(border + im_scale * (prev.x - min_x), border + im_scale * (prev.y - min_y));
            Point2i end = Point2i(border + im_scale * (next.x - min_x), border + im_scale * (next.y - min_y));
            line(img, beg, end, Scalar(0, 0, 0), 3);
            //circle(img, beg, 6, Scalar(0, 0, 0));
            if (i == 1) circle(img, beg, 6, Scalar(0, 0, 0));
        };

        for (int i = 0; i < this->track.size(); i++) {
            Point2i points_arr[5] = {
                Point2i(border + 1 * (this->track[i].line.start.x - min_x),     border + 1 * (this->track[i].line.start.y - min_y)),
                Point2i(border + 1 * (this->track[i].line.end.x - min_x),       border + 1 * (this->track[i].line.end.y - min_y)),
                Point2i(border + 1 * (this->track[i].turn.start.x - min_x),     border + 1 * (this->track[i].turn.start.y - min_y)),
                Point2i(border + 1 * (this->track[i].turn.center.x - min_x),    border + 1 * (this->track[i].turn.center.y - min_y)),
                Point2i(border + 1 * (this->track[i].turn.end.x - min_x),       border + 1 * (this->track[i].turn.end.y - min_y)),
            };
            for (int p_i = 0; p_i < 5; p_i++) circle(img, points_arr[p_i], 4, Scalar(0, 0, 0));
        };
        

        if (mode.find("screen") != std::string::npos)
        {
            putText(img,
                "TEST",
                Point(img.cols - 200, 30),
                0,
                0.5,
                CV_RGB(0, 0, 0),
                0.5
            );
            namedWindow("Traect");
            imshow("Traect", img);
            if (pause_enable) waitKey(0);
        };
        if (mode.find("save") != std::string::npos) imwrite("Traect_TEST.jpg", img);
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
    Point3d accel = Point3d(0, 0, 0),
    Point3d vel_0 = Point3d(0, 0, 0)
    )
{
    DataSeq_model_Type res;

    default_random_engine generator_angle;
    normal_distribution<double> distribution_angle(mean_1, stddev);
    res.angle.push_back(ang_0);

    default_random_engine generator_accel;
    normal_distribution<double> distribution_accel(mean_1, accel_stddev);
    res.pose.push_back(Point3d(0, 0, 0));
    vector<Point3d> vel;
    vel.push_back(vel_0);

    for (int i = 1; i < size + 1; i++) {
        res.angle.push_back(Point3d(res.angle[i - 1].x + w.x * deltatime, res.angle[i - 1].y + w.y * deltatime, res.angle[i - 1].z + w.z * deltatime));
        res.w.push_back(Point3d(w.x + distribution_angle(generator_angle), w.y + distribution_angle(generator_angle), w.z + distribution_angle(generator_angle)));
        res.timestamp.push_back(deltatime * i);

        res.accel.push_back(Point3d(accel.x + distribution_accel(generator_accel), accel.y + distribution_accel(generator_accel), accel.z + distribution_accel(generator_accel)));
        vel.push_back(Point3d(vel[i - 1].x + accel.x * deltatime, vel[i - 1].y + accel.y * deltatime, vel[i - 1].z + (accel.z + GCONST) * deltatime));
        res.pose.push_back(Point3d(res.pose[i - 1].x + vel[i].x * deltatime, res.pose[i - 1].y + vel[i].y * deltatime, res.pose[i - 1].z + vel[i].z * deltatime));
    }

    return res;
};/*double mean_1,
    const int size,
    double stddev,
    Point3d w,
    Point3d ang_0,
    Point3d ang_0,
    double deltatime,
    double accel_stddev,
    Point3d accel = Point3d(0, 0, 0),
    Point3d vel_0 = Point3d(0, 0, 0)*/

void motion_Test(double accel_std = 1, double sko = 0.2, double delta = 0.004, double duration = 10) {

    cout << endl;
    cout << endl;
    cout << "-------------<<<" << "Testing motion models" << ">>>-------------" << endl;
    cout << ">>>>accel_std:\t" << accel_std << "m/sec^2" << endl;
    cout << ">>>>sko:\t" << sko << endl;
    cout << ">>>>delta:\t" << delta << " sec" << endl;
    cout << ">>>>duration:\t" << duration << " sec" << endl;
    cout << endl;

    Data_seq test_1("Test1 stand still"); {
        vector<Pose_type> pose_err_vec;
        test_1.load_model(generate_model(
            0.0, 
            (int)(duration / delta), 
            sko, 
            Point3d(0, 0, 0), 
            Point3d(M_PI / 2, 0, 0), 
            delta, 
            sko, 
            Point3d(0, 0, 0),
            Point3d(0, 0, 0)));
        for (int i = 0; i < test_1.limit-1; i++) {
            Pose_type GT = test_1.dataline[i];
            pose_err_vec.push_back(calcDisplasment(GT, test_1.pose[i], test_1.pose[0], "Disp_test1", 3));
        };

        Pose_type GT = test_1.dataline[test_1.limit-1];
        pose_err_vec.push_back(calcDisplasment(GT, test_1.pose[test_1.limit-1], test_1.pose[0], "Disp_test1"));

        Pose_type av_pose_err;
        av_pose_err.lat = 0.0;
        av_pose_err.lon = 0.0;
        av_pose_err.alt = 0.0;
        for (int i = 0; i < pose_err_vec.size(); i++)
        {
            av_pose_err.lat += abs(pose_err_vec[i].lat) / pose_err_vec.size();
            av_pose_err.lon += abs(pose_err_vec[i].lon) / pose_err_vec.size();
            av_pose_err.alt += abs(pose_err_vec[i].alt) / pose_err_vec.size();
        }
        //ds.print_traect(num_sources);
        
        cout << "Disp_test1" << "\ttime: " << test_1.timestamps[test_1.limit - 1] - test_1.timestamps[0] << "sec" << endl;
        cout << ">>>>" << "Average pose  error: lat  " << av_pose_err.lat << " \tlon " << av_pose_err.lon << " \t alt " << av_pose_err.alt << endl;
        cout << endl;
        test_1.print_traect(1, "screen save", true);
    };

    Data_seq test_2("Test2 ecvi_dist"); {
        vector<Pose_type> pose_err_vec;
        test_2.load_model(generate_model(
            0.0, 
            (int)(duration / delta), 
            sko, 
            Point3d(0, 0, 0), 
            Point3d(M_PI / 2, 0, 0), 
            delta, 
            sko,
            Point3d(0, 0, 0),
            Point3d(1, 0, 0))
        );
        for (int i = 0; i < test_2.limit; i++) {
            Pose_type GT = test_2.dataline[i];
            pose_err_vec.push_back(calcDisplasment(GT, test_2.pose[i], test_2.pose[0], "Disp_test1", 3));
        };

        Pose_type GT = test_2.dataline[test_2.limit-1];
        pose_err_vec.push_back(calcDisplasment(GT, test_2.pose[test_2.limit-1], test_2.pose[0], "Disp_test1"));

        Pose_type av_pose_err;
        av_pose_err.lat = 0.0;
        av_pose_err.lon = 0.0;
        av_pose_err.alt = 0.0;
        for (int i = 0; i < pose_err_vec.size(); i++)
        {
            av_pose_err.lat += abs(pose_err_vec[i].lat) / pose_err_vec.size();
            av_pose_err.lon += abs(pose_err_vec[i].lon) / pose_err_vec.size();
            av_pose_err.alt += abs(pose_err_vec[i].alt) / pose_err_vec.size();
        }
        //ds.print_traect(num_sources);
        
        cout << "Disp_test2" << "\ttime: " << test_2.timestamps[test_2.limit - 1] - test_2.timestamps[0] << "sec" << endl;
        cout << ">>>>" << "Average pose  error: lat  " << av_pose_err.lat << " \tlon " << av_pose_err.lon << " \t alt " << av_pose_err.alt << endl;
        cout << endl;
        test_2.print_traect(2, "screen save", true);
    };

    Data_seq test_3("Test3 accel"); {
        vector<Pose_type> pose_err_vec;
        test_3.load_model(generate_model(
            0.0,
            (int)(duration / delta),
            sko,
            Point3d(0, 0, 0),
            Point3d(M_PI / 2, 0, 0),
            delta,
            sko,
            Point3d(accel_std, 0, 0),
            Point3d(0, 0, 0))
        );
        for (int i = 0; i < test_3.limit; i++) {
            Pose_type GT = test_3.dataline[i];
            pose_err_vec.push_back(calcDisplasment(GT, test_3.pose[i], test_3.pose[0], test_3.dirname, 3));
        };

        Pose_type GT = test_3.dataline[test_3.limit-1];
        pose_err_vec.push_back(calcDisplasment(GT, test_3.pose[test_3.limit-1], test_3.pose[0], test_3.dirname));

        Pose_type av_pose_err;
        av_pose_err.lat = 0.0;
        av_pose_err.lon = 0.0;
        av_pose_err.alt = 0.0;
        for (int i = 0; i < pose_err_vec.size(); i++)
        {
            av_pose_err.lat += abs(pose_err_vec[i].lat) / pose_err_vec.size();
            av_pose_err.lon += abs(pose_err_vec[i].lon) / pose_err_vec.size();
            av_pose_err.alt += abs(pose_err_vec[i].alt) / pose_err_vec.size();
        }
        //ds.print_traect(num_sources);
        
        cout << test_3.dirname << "\ttime: " << test_3.timestamps[test_3.limit - 1] - test_3.timestamps[0] << "sec" << endl;
        cout << ">>>>" << "Average pose  error: lat  " << av_pose_err.lat << " \tlon " << av_pose_err.lon << " \t alt " << av_pose_err.alt << endl;
        cout << endl;
        test_3.print_traect(3, "screen save", true);
    };
};

void angle_Test(double w_std = 0.0001 * M_PI/180, Point3d vel_0 = Point3d(0,0,0), double sko = 0.000001 * M_PI/180, double delta = 0.004, double duration = 60) {
    cout << endl; 
    cout << endl;
    cout << "-------------<<<" << "Testing angle models" << ">>>-------------" << endl;
    cout << ">>>>w_std:\t" << w_std*180/ M_PI << "deg/sec" << endl;
    cout << ">>>>sko:\t" << sko << endl;
    cout << ">>>>delta:\t" << delta << " sec" << endl;
    cout << ">>>>duration:\t" << duration << " sec" << endl;
    cout << endl;

    Data_seq test_0("Test0 stand still");
    vector<Point3d> test_0_ang_err_vec;
    test_0.load_model(generate_model(0.0, (int)(duration / delta), sko, Point3d(0, 0, 0), Point3d(M_PI / 2, 0, 0), delta, sko));
    for (int i = 0; i < test_0.limit; i++)
        test_0_ang_err_vec.push_back(calcDisplasment_ang(test_0.rot_ang_GT[i], test_0.rot_ang[i], 1));
    Point3d test_0_av_ang_err = Point3d(0, 0, 0);
    Point3d test_0_max_ang_err = Point3d(0, 0, 0);
    for (int i = 0; i < test_0_ang_err_vec.size(); i++)
    {
        test_0_max_ang_err.x = abs(test_0_ang_err_vec[i].x) > test_0_max_ang_err.x ? abs(test_0_ang_err_vec[i].x) : test_0_max_ang_err.x;
        test_0_max_ang_err.y = abs(test_0_ang_err_vec[i].y) > test_0_max_ang_err.y ? abs(test_0_ang_err_vec[i].y) : test_0_max_ang_err.y;
        test_0_max_ang_err.z = abs(test_0_ang_err_vec[i].z) > test_0_max_ang_err.z ? abs(test_0_ang_err_vec[i].z) : test_0_max_ang_err.z;

        test_0_av_ang_err.x += abs(test_0_ang_err_vec[i].x) / test_0_ang_err_vec.size();
        test_0_av_ang_err.y += abs(test_0_ang_err_vec[i].y) / test_0_ang_err_vec.size();
        test_0_av_ang_err.z += abs(test_0_ang_err_vec[i].z) / test_0_ang_err_vec.size();
    };
    cout << test_0.dirname << "\ttime: " << test_0.timestamps[test_0.limit - 1] - test_0.timestamps[0] << "sec" << endl;
    cout << ">>>>" << "Average angle error: roll " << test_0_av_ang_err.x << " deg\tpitch " << test_0_av_ang_err.y << " deg\tyaw " << test_0_av_ang_err.z << " deg" << endl;
    cout << ">>>>" << "Extrime angle error: roll " << test_0_max_ang_err.x << " deg\tpitch " << test_0_max_ang_err.y << " deg\tyaw " << test_0_max_ang_err.z << " deg" << endl;
    cout << endl;

    Data_seq test_1("Test1 yaw rotation");
    vector<Point3d> test_1_ang_err_vec;
    test_1.load_model(generate_model(0.0, (int)(duration / delta), sko, Point3d(w_std, 0, 0), Point3d(M_PI / 2, 0, 0), delta, sko));
    for (int i = 0; i < test_1.limit; i++)
        test_1_ang_err_vec.push_back(calcDisplasment_ang(test_1.rot_ang_GT[i], test_1.rot_ang[i], 1));
    Point3d test_1_av_ang_err = Point3d(0, 0, 0);
    Point3d test_1_max_ang_err = Point3d(0, 0, 0);
    for (int i = 0; i < test_1_ang_err_vec.size(); i++)
    {
        test_1_max_ang_err.x = abs(test_1_ang_err_vec[i].x) > test_1_max_ang_err.x ? abs(test_1_ang_err_vec[i].x) : test_1_max_ang_err.x;
        test_1_max_ang_err.y = abs(test_1_ang_err_vec[i].y) > test_1_max_ang_err.y ? abs(test_1_ang_err_vec[i].y) : test_1_max_ang_err.y;
        test_1_max_ang_err.z = abs(test_1_ang_err_vec[i].z) > test_1_max_ang_err.z ? abs(test_1_ang_err_vec[i].z) : test_1_max_ang_err.z;

        test_1_av_ang_err.x += abs(test_1_ang_err_vec[i].x) / test_1_ang_err_vec.size();
        test_1_av_ang_err.y += abs(test_1_ang_err_vec[i].y) / test_1_ang_err_vec.size();
        test_1_av_ang_err.z += abs(test_1_ang_err_vec[i].z) / test_1_ang_err_vec.size();
    };
    cout << test_1.dirname << "\ttime: " << test_1.timestamps[test_1.limit - 1] - test_1.timestamps[0] << "sec" << endl;
    cout << ">>>>" << "Average angle error: roll " << test_1_av_ang_err.x * 180 / M_PI << " deg\tpitch " << test_1_av_ang_err.y * 180 / M_PI << " deg\tyaw " << test_1_av_ang_err.z * 180 / M_PI << " deg" << endl;
    cout << ">>>>" << "Extrime angle error: roll " << test_1_max_ang_err.x * 180 / M_PI << " deg\tpitch " << test_1_max_ang_err.y * 180 / M_PI << " deg\tyaw " << test_1_max_ang_err.z * 180 / M_PI << " deg" << endl;
    cout << endl;

    Data_seq test_3("Test3 pitch rotation");
    vector<Point3d> test_3_ang_err_vec;
    test_3.load_model(generate_model(0.0, (int)(duration / delta), sko, Point3d(0, 0, w_std), Point3d(M_PI / 2, 0, 0), delta, sko));
    for (int i = 0; i < test_1.limit; i++)
        test_3_ang_err_vec.push_back(calcDisplasment_ang(test_3.rot_ang_GT[i], test_3.rot_ang[i], 1));
    Point3d test_3_av_ang_err = Point3d(0, 0, 0);
    Point3d test_3_max_ang_err = Point3d(0, 0, 0);
    for (int i = 0; i < test_1_ang_err_vec.size(); i++)
    {
        test_3_max_ang_err.x = abs(test_3_ang_err_vec[i].x) > test_3_max_ang_err.x ? abs(test_3_ang_err_vec[i].x) : test_3_max_ang_err.x;
        test_3_max_ang_err.y = abs(test_3_ang_err_vec[i].y) > test_3_max_ang_err.y ? abs(test_3_ang_err_vec[i].y) : test_3_max_ang_err.y;
        test_3_max_ang_err.z = abs(test_3_ang_err_vec[i].z) > test_3_max_ang_err.z ? abs(test_3_ang_err_vec[i].z) : test_3_max_ang_err.z;

        test_3_av_ang_err.x += abs(test_3_ang_err_vec[i].x) / test_3_ang_err_vec.size();
        test_3_av_ang_err.y += abs(test_3_ang_err_vec[i].y) / test_3_ang_err_vec.size();
        test_3_av_ang_err.z += abs(test_3_ang_err_vec[i].z) / test_3_ang_err_vec.size();
    };
    cout << test_3.dirname << "\ttime: " << test_3.timestamps[test_1.limit - 1] - test_3.timestamps[0] << "sec" << endl;
    cout << ">>>>" << "Average angle error: roll " << test_3_av_ang_err.x << " deg\tpitch " << test_3_av_ang_err.y << " deg\tyaw " << test_3_av_ang_err.z << " deg" << endl;
    cout << ">>>>" << "Extrime angle error: roll " << test_3_max_ang_err.x << " deg\tpitch " << test_3_max_ang_err.y << " deg\tyaw " << test_3_max_ang_err.z << " deg" << endl;
    cout << endl;
};