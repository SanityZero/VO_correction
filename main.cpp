#include <iostream>
#include <locale>
#include <iomanip> 
#include <fstream>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
#include <algorithm>
#include <random>
using namespace cv;
using namespace std;

#include "Types.h"
#include "ErEstVO.h"
#include "DataSequence.h"
#include "VOCorrection.h"
#include "Tests.h"

//#define TEST_ANGLE true
#define TEST_ANGLE false

#define TEST_MOTION true
//#define TEST_MOTION false

//#define PROCESS_DATASET true
#define PROCESS_DATASET false

//#define USE_VO true
#define USE_VO false

#define REGENERATE_POINTS_AFTER 4

Mat getSourceMat(string, int);

//////////////////////////////////////////////////  MAIN
int main(int argc, char** argv) {

    //cout << angle2V(Point3d(0,0,1), Point3d(M_PI/2, 0, 1));

    if (TEST_ANGLE) {
        old_angle_Test();
        waitKey(0);
    };

    if (TEST_MOTION) {
        Test_model tm1("test1", "C:\\ProgStaff\\NIRS_models\\test1\\");

        //tm1.read_restriction_file();


        vector<bool> options;

        // настройки загрузки/сохранения
        options.push_back(true); // track_model
        options.push_back(true); // motion_model
        options.push_back(false); // s_points
        options.push_back(true); // bins_model


        tm1.generate_test_model(options);
        //tm1.print_states("C:/ProgStaff/NIRS_models/test1/states.txt");
        //tm1.print_bins_gts("C:/ProgStaff/NIRS_models/test1/bins_gt_points.txt");
        ////system("del /f /s /q C:\\ProgStaff\\test_generated_images");
        tm1.print_camera_proections();
        tm1.show_bins_gt();
        waitKey(0);
        //tm1.show_gt_measures();
        //system("C:\\ProgStaff\\NIRS_precentation\\dist\\main.exe");
        
    };

    vector<string> sources;
    sources.push_back("C:/Kitti/2011_09_26/2011_09_26_drive_0001_sync/");
    sources.push_back("C:/Kitti/2011_09_26/2011_09_26_drive_0002_sync/");
    sources.push_back("C:/Kitti/2011_09_26/2011_09_26_drive_0005_sync/");
    sources.push_back("C:/Kitti/2011_09_26/2011_09_26_drive_0009_sync/");
    sources.push_back("C:/Kitti/2011_09_26/2011_09_26_drive_0011_sync/");
    sources.push_back("C:/Kitti/2011_09_26/2011_09_26_drive_0013_sync/");
    sources.push_back("C:/Kitti/2011_09_26/2011_09_26_drive_0014_sync/");
    sources.push_back("C:/Kitti/2011_09_26/2011_09_26_drive_0017_sync/");
    sources.push_back("C:/Kitti/2011_09_26/2011_09_26_drive_0018_sync/");
    sources.push_back("C:/Kitti/2011_09_26/2011_09_26_drive_0048_sync/");
    sources.push_back("C:/Kitti/2011_09_26/2011_09_26_drive_0051_sync/");
    sources.push_back("C:/Kitti/2011_09_26/2011_09_26_drive_0056_sync/");
    sources.push_back("C:/Kitti/2011_09_26/2011_09_26_drive_0057_sync/");
    sources.push_back("C:/Kitti/2011_09_26/2011_09_26_drive_0059_sync/");
    sources.push_back("C:/Kitti/2011_09_26/2011_09_26_drive_0060_sync/");
    sources.push_back("C:/Kitti/2011_09_26/2011_09_26_drive_0096_sync/");

    vector<string> calib_files;
    calib_files.push_back("C:/Kitti/2011_09_26/calib_cam_to_cam.txt");
    calib_files.push_back("C:/Kitti/2011_09_26/calib_cam_to_cam.txt");
    calib_files.push_back("C:/Kitti/2011_09_26/calib_cam_to_cam.txt");
    calib_files.push_back("C:/Kitti/2011_09_26/calib_cam_to_cam.txt");
    calib_files.push_back("C:/Kitti/2011_09_26/calib_cam_to_cam.txt");
    calib_files.push_back("C:/Kitti/2011_09_26/calib_cam_to_cam.txt");
    calib_files.push_back("C:/Kitti/2011_09_26/calib_cam_to_cam.txt");
    calib_files.push_back("C:/Kitti/2011_09_26/calib_cam_to_cam.txt");
    calib_files.push_back("C:/Kitti/2011_09_26/calib_cam_to_cam.txt");
    calib_files.push_back("C:/Kitti/2011_09_26/calib_cam_to_cam.txt");
    calib_files.push_back("C:/Kitti/2011_09_26/calib_cam_to_cam.txt");
    calib_files.push_back("C:/Kitti/2011_09_26/calib_cam_to_cam.txt");
    calib_files.push_back("C:/Kitti/2011_09_26/calib_cam_to_cam.txt");
    calib_files.push_back("C:/Kitti/2011_09_26/calib_cam_to_cam.txt");
    calib_files.push_back("C:/Kitti/2011_09_26/calib_cam_to_cam.txt");
    calib_files.push_back("C:/Kitti/2011_09_26/calib_cam_to_cam.txt");

    for (int num_sources = 0; (num_sources < sources.size()) && PROCESS_DATASET; num_sources++)
    {
        cout << endl;
        cout << endl;
        cout << "-------------<<<" << sources[num_sources] << ">>>-------------" << endl;
        cout << endl;
        int i_main = 0;

        Data_seq ds(sources[num_sources] + "oxts/", calib_files[num_sources]);

        //ds.loadBINS("GT","E");
        //ds.loadBINS("EC","E");
        ds.loadBINS("EC2","E");

        vector<Point3d> ang_err_vec;
        vector<Point2f> p0, p1, p2, p3;
        vector<uchar> status0, status1, status2, status3;
        vector<Pose_type> pose_err_vec;

        namedWindow("Orientation", WINDOW_NORMAL);
        bool first_step_VO = true;

        Mat frame_old;
        Mat frame, frame_raw;

        while (i_main < ds.limit - 1) {
            ds.calculate_next_point();

            {
                if (USE_VO) {
                    //////////////////////////////////////////////////
                    // часть про визуальную одометрию.
                    frame = getSourceMat(sources[num_sources], i_main);
                    if (frame.empty()) break;

                    if (first_step_VO) {
                        frame.copyTo(frame_old);
                        first_step_VO = false;
                        i_main++;

                        goodFeaturesToTrack(
                            frame_old,
                            p1,
                            100,
                            0.3,
                            7,
                            Mat(),
                            7,
                            false,
                            0.04
                        );
                        continue;
                    };

                    if (i_main % REGENERATE_POINTS_AFTER == 0)
                    {
                        eraseLostFeatures(p0, p1, p2, p3, status0, status1, status2, status3);
                        Pose_type last = ds.pose[ds.pose.size() - 1];
                        Pose_type prelast = ds.pose[ds.pose.size() - 2];
                        Point3d true_pose_delta = Point3d(prelast.lat - last.lat, prelast.lon - last.lon, prelast.alt - last.alt);
                        Point3d true_angle_delta = Point3d(prelast.roll - last.roll, prelast.pitch - last.pitch, prelast.yaw - last.yaw);

                        vector<Point3d> VOC_res = VOCorrect(ds, p3, p2, p1, true_pose_delta, true_angle_delta);
                        
                        Point3d VOC_res_deltapose = Point3d(VOC_res[1].x, VOC_res[1].y, VOC_res[1].z);
                        Point3d VOC_res_angles = Point3d(VOC_res[0].x, VOC_res[0].y, VOC_res[0].z);
                        
                        cout << "GT\t" << true_pose_delta<< endl;
                        VOC_res_deltapose = rotateP3d(VOC_res_deltapose, Point3d(last.roll, last.pitch, last.yaw));
                        //VOC_res_deltapose = rotateP3d(VOC_res_deltapose, Point3d(VOC_res_angles.x, VOC_res_angles.y, VOC_res_angles.z));
                        
                        //Pose_type last = ds.dataline.back();
                        Pose_type corrected = Pose_type(Point3d(
                            VOC_res_deltapose.x + last.lat,
                            VOC_res_deltapose.y + last.lon,
                            VOC_res_deltapose.z + last.alt),

                            //last.lat,
                            //last.lon,
                            //last.alt,

                            //VOC_res_deltapose.x,
                            //VOC_res_deltapose.y,
                            //VOC_res_deltapose.z,

                            //VOC_res_angles.x + last.roll,
                            //VOC_res_angles.y + last.pitch,
                            //VOC_res_angles.z + last.yaw,

                            //VOC_res_angles.x,
                            //VOC_res_angles.y,
                            //VOC_res_angles.z,

                            Point3d(last.roll,
                            last.pitch,
                            last.yaw),

                            Point3d(0,0,0),
                            Point3d(0,0,0)
                        );
                        //ds.pose.push_back(last);
                        ds.pose_VOcr.push_back(corrected);
                        ds.vel.pop_back();
                        ds.vel.push_back(VOC_res[1] / ds.deltatime.back());
                        
                        ds.pose.pop_back();
                        ds.pose.pop_back();
                        ds.pose.push_back(prelast);
                        ds.pose.push_back(corrected);

                        goodFeaturesToTrack(frame_old, p1, 100, 0.3, 7, Mat(), 7, false, 0.04);
                        p2.clear();
                        p3.clear();
                    };



                    // calculate optical flow
                    //во времени от самого старого р3 -> p2 -> p1 -> p0
                    //т.е. на регенирации точек мы выбираем новые p1, а потом получаем р0 из функции
                    //оптического потока
                    vector<float> err;
                    TermCriteria criteria = TermCriteria((TermCriteria::COUNT)+(TermCriteria::EPS), 10, 0.03);
                    //calcOpticalFlowPyrLK(frame_old, frame, p0, p1, status, err, Size(15, 15), 2, criteria, 0, 0.001);
                    calcOpticalFlowPyrLK(frame_old, frame, p1, p0, status0, err, Size(15, 15), 2, criteria, 0, 0.001);
                    //удалять надо из р0
                    p3.assign(p2.begin(), p2.end());
                    p2.assign(p1.begin(), p1.end());
                    p1.assign(p0.begin(), p0.end());


                    status3.assign(status2.begin(), status2.end());
                    status2.assign(status1.begin(), status1.end());
                    status1.assign(status0.begin(), status0.end());



                    //vector<Point2f> good_new;
                    //for (uint i = 0; i < p1.size(); i++) {
                    //    // Select good points
                    //    if (status0[i] == 1) {
                    //        good_new.push_back(p1[i]);
                    //    }
                    //};

                    frame_old = frame.clone();
                    //add(frame, mask, img);
                    imshow("Frame", frame);
                };
                ds.print_traect(num_sources, "screen save", false);
            };
            i_main++;
            Pose_type GT = getGTData(sources[num_sources] + "oxts/data/", i_main);
            imshow("Orientation", printOrientation(GT, ds.pose[i_main]));
            //waitKey(0);
            //cout << "GT" << scientific << setprecision(6) << ":\t" << GT.lat - ds.pose[0].lat << "\t" << GT.lon - ds.pose[0].lon << "\t" << GT.alt - ds.pose[0].alt << "\t";
            //cout << GT.pitch - ds.pose[0].pitch << "\t" << GT.roll - ds.pose[0].roll << "\t" << GT.yaw - ds.pose[0].yaw << endl;
            //cout << "BINS" << ":\t" << ds.pose[i_main].lat - ds.pose[0].lat << "\t" << ds.pose[i_main].lon - ds.pose[0].lon << "\t" << ds.pose[i_main].alt - ds.pose[0].alt << "\t";
            //cout << ds.pose[i_main].pitch - ds.pose[0].pitch << "\t" << ds.pose[i_main].roll - ds.pose[0].roll << "\t" << ds.pose[i_main].yaw - ds.pose[0].yaw << endl;
            ang_err_vec.push_back(calcDisplasment_ang(ds.rot_ang_GT[i_main], ds.rot_ang[i_main], 1));
            pose_err_vec.push_back(calcDisplasment(GT, ds.pose[i_main], ds.pose[0], "Disp" + to_string(num_sources)));
        };
        cout << endl;
        Point3d av_ang_err;
        Point3d max_ang_err = Point3d(0, 0, 0);
        for (int i = 0; i < ang_err_vec.size(); i++)
        {
            max_ang_err.x = abs(ang_err_vec[i].x) > max_ang_err.x ? abs(ang_err_vec[i].x) : max_ang_err.x;
            max_ang_err.y = abs(ang_err_vec[i].y) > max_ang_err.y ? abs(ang_err_vec[i].y) : max_ang_err.y;
            max_ang_err.z = abs(ang_err_vec[i].z) > max_ang_err.z ? abs(ang_err_vec[i].z) : max_ang_err.z;

            av_ang_err.x += abs(ang_err_vec[i].x) / ang_err_vec.size();
            av_ang_err.y += abs(ang_err_vec[i].y) / ang_err_vec.size();
            av_ang_err.z += abs(ang_err_vec[i].z) / ang_err_vec.size();
        };
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
        cout << sources[num_sources] << "\ttime: " << ds.timestamps[ds.limit - 1] - ds.timestamps[0] << "sec" << endl;
        cout << ">>>>" << "Average angle error: roll " << av_ang_err.x * 180/M_PI << " deg\tpitch " << av_ang_err.y * 180 / M_PI << " deg\tyaw " << av_ang_err.z * 180 / M_PI << " deg" << endl;
        cout << ">>>>" << "Extrime angle error: roll " << max_ang_err.x * 180 / M_PI << " deg\tpitch " << max_ang_err.y * 180 / M_PI << " deg\tyaw " << max_ang_err.z * 180 / M_PI << " deg" << endl;
        cout << ">>>>" << "Average pose  error: lat  " << av_pose_err.lat << " \tlon " << av_pose_err.lon << " \t alt " << av_pose_err.alt << endl;
        cout << endl;
        ofstream fout("Traect" + to_string(num_sources) + ".txt", ios_base::app); // создаём объект класса ofstream для записи и связываем его с файлом cppstudio.txt
        fout <<">>>>" << "Average pose  error: lat  " << av_pose_err.lat << " \tlon " << av_pose_err.lon << " \t alt " << av_pose_err.alt << endl;
        waitKey(0);
        fout.close(); // закрываем файл
    }
}

Mat getSourceMat(string source_dir, int number) {
    static string source = "";
    static int max = 0;
    if (source != source_dir) {
        ifstream timestamp_file;
        source = source_dir;
        string filename = source_dir +"image_00/timestamps.txt";
        timestamp_file.open(filename);
        
        if (!timestamp_file) cout << "Timestamps file in not open\n";

        max = 0;
        do {
            string tmp;
            getline(timestamp_file, tmp, '\n');
            if (tmp == "") break;
            max++;
        } while(!timestamp_file.eof());
        timestamp_file.close();
    };
    if (number <= max && number >= 0) {
        //00 0000 0000.png
        string zeros = "000000";
        if (number < 1000) zeros += "0";
        if (number < 100) zeros += "0";
        if (number < 10) zeros += "0";

        string num_part = zeros + to_string(number);

        string mat_filename = source_dir + "image_00/data/" + num_part + ".png";
        Mat img = imread(mat_filename);
        Mat gray = img.clone();
        cvtColor(img, gray, COLOR_BGR2GRAY);
        return gray;
    };
    return Mat(3, 3, CV_64F);
    //оно должно считать количество изображений по timestemps.txt
};
//////////////////////////////////////////////////  MAIN