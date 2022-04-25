#pragma once
#define GCONST  9.80665

class Data_seq {
public:
    vector<Pose_type> dataline;
    vector<Pose_type> pose;
    vector<Pose_type> pose_VOcr;
    vector<Pose_type> pose_GT;
    vector<Point3d> rot_ang_GT;//rad
    vector<Point3d> rot_ang;//rad
    vector<Point3d> vel;
    vector<double> timestamps;
    vector<double> deltatime;
    string dirname;
    string angle_type;
    string calib_filename;
    int limit;

    int this_ds_i;

    Data_seq(
        string dirname = "C:/Kitti/2011_09_26/2011_09_26_drive_0001_sync/oxts/", 
        string calib_filenam= "C:/Kitti/2011_09_26/calib_cam_to_cam.txt") : dirname(dirname), limit(0), calib_filename(calib_filenam)
    {};
    ~Data_seq() {};

    void load_model(DataSeq_model_Type model, string angle_type = "EC");
    void loadBINS(string, string);
    bool calculate_next_point(string integr_method = "E");
    Point3d interpolatedAxel(double, Point3d);

    Pose_type get_last(int shift = 0) {
        return this->pose[this->this_ds_i + shift];
    };

    void print_traect(int num, string mode, bool pause_enable);
    void correct_last(Pose_type corrected) {
        this->pose[this->this_ds_i] = corrected;
    }
};


