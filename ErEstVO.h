#pragma once
using namespace cv;
#include "Test_math.h"
/////////////////////////////////////////////////////
//Считывание данных
Pose_type getGTData(string, int);
Mat loadCameraModel(string, int);

/////////////////////////////////////////////////////
//Расчёт ошибки
inline Pose_type calcDisplasment(Pose_type, Pose_type, Pose_type, string, int);
inline Point3d calcDisplasment_ang(Point3d, Point3d, int);

/////////////////////////////////////////////////////
inline Mat printOrientation(Pose_type gtrue, Pose_type estimated) {
    Point2i img_size = Point2i(220, 220);
    Mat img0(img_size.x, img_size.y, CV_8UC3, Scalar(255, 255, 255));
    Mat img1(img_size.x, img_size.y, CV_8UC3, Scalar(255, 255, 255));
    Mat img2(img_size.x, img_size.y, CV_8UC3, Scalar(255, 255, 255));

    Point center(img_size.x / 2, img_size.x / 2);

    //Point3d W_t = rotate(rotate(rotate(Point3d(0, 1, 0), Point3d(gtrue.yaw,0,0)), Point3d(gtrue.pitch, 0, 0)), Point3d(gtrue.roll, 0, 0));
    //Point3d W_e = rotate(Point3d(scale, scale, scale), Point3d(angles.x, angles.y, angles.z));

    int scale = 60;
    int ang_scale = 1;
    Point3d angles(ang_scale * (gtrue.roll - estimated.roll), ang_scale * (gtrue.pitch - estimated.pitch), ang_scale * (gtrue.yaw - estimated.yaw));

    Point3d W_t = Point3d(scale, scale, scale);
    Point3d W_e = rotateP3d(Point3d(scale, scale, scale), Point3d(angles.x, angles.y, angles.z));

    line(img0, center, center + Point(W_t.y, W_t.z), Scalar(0, 0, 0), 2);
    line(img0, center, center + Point(W_e.y, W_e.z), Scalar(0, 0, 255), 1);
    putText(img0,
        "roll",
        Point(0, 20),
        0,
        0.5,
        CV_RGB(0, 0, 0),
        0.5
    );

    line(img1, center, center + Point(W_t.x, W_t.z), Scalar(0, 0, 0), 2);
    line(img1, center, center + Point(W_e.x, W_e.z), Scalar(0, 0, 255), 1);
    putText(img1,
        "pitch",
        Point(0, 20), 0,
        0.5,
        CV_RGB(0, 0, 0),
        0.5
    );

    line(img2, center, center + Point(W_t.x, W_t.y), Scalar(0, 0, 0), 2);
    line(img2, center, center + Point(W_e.x, W_e.y), Scalar(0, 0, 255), 1);
    putText(img2,
        "yaw",
        Point(0, 20), 0,
        0.5,
        CV_RGB(0, 0, 0),
        0.5
    );

    Mat res(img_size.x, img_size.y * 3, CV_8UC3, Scalar(255, 255, 255));

    img0.copyTo(res(Rect(0, 0, img0.cols, img0.rows)));
    img1.copyTo(res(Rect(img0.cols, 0, img1.cols, img1.rows)));
    img2.copyTo(res(Rect(img0.cols + img1.cols, 0, img2.cols, img2.rows)));
    /*   static int i = 0;
       putText(res,
           to_string(i++),
           Point(res.cols - 60, 40), 0,
           0.5,
           CV_RGB(0, 0, 0),
           0.5
       );*/
    return res;
};

/////////////////////////////////////////////////////
//Реализации
inline Mat loadCameraModel(string calibration_filename = "", int cam_number = 0) {
    //static Point2d a = Point2d(984.2439, 0.0);
    //static Point3d T = Point3d(0.0, 0.0, 0.0);
    //static double u0 = 690;
    //static double v0 = 0.0;
    //static double gamma = 0.0;

    //if (calibration_filename != "") {
    //    ifstream in(calibration_filename);
    //    if (!in) cout << "File in not open\n";

    //    string K_str = "K_0" + to_string(cam_number);
    //    string T_str = "T_0" + to_string(cam_number);
    //    int flag = 0;

    //    do
    //    {
    //        string line;
    //        getline(in, line);
    //        if (line.find(K_str) != std::string::npos) {
    //            line = line.substr(line.find(' ') + 1, line.length());
    //            a.x = stod(line.substr(0, line.find(' ') + 1));
    //            line = line.substr(line.find(' ') + 1, line.length());
    //            gamma = stod(line.substr(0, line.find(' ') + 1));
    //            line = line.substr(line.find(' ') + 1, line.length());
    //            u0 = stod(line.substr(0, line.find(' ') + 1));
    //            line = line.substr(line.find(' ') + 1, line.length());
    //            line = line.substr(line.find(' ') + 1, line.length());
    //            a.y = stod(line.substr(0, line.find(' ') + 1));
    //            line = line.substr(line.find(' ') + 1, line.length());
    //            v0 = stod(line.substr(0, line.find(' ') + 1));
    //            flag++;
    //        }
    //        else if (line.find(T_str) != std::string::npos) {
    //            line = line.substr(line.find(' ') + 1, line.length());
    //            T.x = stod(line.substr(0, line.find(' ') + 1));
    //            line = line.substr(line.find(' ') + 1, line.length());
    //            T.y = stod(line.substr(0, line.find(' ') + 1));
    //            line = line.substr(line.find(' ') + 1, line.length());
    //            T.z = stod(line.substr(0, line.find(' ') + 1));
    //            flag++;
    //        };
    //    } while (!in.eof() && flag != 2);
    //};

    double IternalCalib_ar[3][3] = {
           {984.2439, 0, 690},
           {0, 980.8141, 233.1966},
           {0, 0, 1}
    };
    Mat IC = Mat(3, 3, CV_64F, IternalCalib_ar);

    /* double z = 1 - T.z;
     double y = (p.x - v0 * z - T.y) / a.y;
     double x = (p.y - T.x - gamma * y - u0 * z) / a.x;

     double theta = atan(sqrt(x*x+y*y)/z);
     double phi = atan(y / x);
     Point2d res = Point2d(theta, phi);*/

    return IC;
    // a = 2arctg (d/2f),
    //  6.40мм
    // 4.80мм
    //где:
    // нужно вытаскивать из файла калибровки камеры фокусное расстояние, оно в матрице К, на вики это матрица А
    //    a – угол обзора видеокамеры, в метрических градусах;
    //    arctg - тригонометрическая функция(арктангенс);
    //    d – ширина матрицы в миллиметрах;
    //    f – эффективное фокусное расстояние объектива в миллиметрах;
};


/////////////////////////////////////////////////////
inline Pose_type getGTData(string source_dir, int num) {//rad
    Pose_type res;
    static Pose_type zero;
    string filename, num_part;
    num_part.append(10 - to_string(num).length(), '0') += to_string(num); //000 000 0000.png 000 000 0000
    filename = source_dir + num_part + ".txt";

    string pose_line;
    ifstream in_pos(filename);
    if (!in_pos) cout << "File in_pos not open \"" + filename + "\"" + "\n";
    getline(in_pos, pose_line);

    double pose_data[32];
    for (int i = 0; i < 25; i++) {
        pose_data[i] = stod(pose_line.substr(0, pose_line.find(' ')));
        pose_line = pose_line.substr(pose_line.find(' ') + 1, pose_line.length());
    };

    res.setPose(Point3d(
        cos(M_PI * pose_data[0] / 180.0) * 111134.861111 * pose_data[1],
        111134.861111 * pose_data[0],
        pose_data[2]
    ));

    //res.lon = cos(M_PI * pose_data[0] / 180.0) * 40075000 / 360;
    //res.lat = (111132.954 - 559.822 * cos(2 * pose_data[0]) + 1.175 * cos(4 * pose_data[0]))* pose_data[0];
    //res.lon = 111132.954 * cos(pose_data[0]);

    res.setOrient(Point3d(pose_data[3], pose_data[4], M_PI / 2 + pose_data[5]));
    res.setAccel(Point3d(pose_data[11], -pose_data[12], pose_data[13]));
    res.setW(Point3d(pose_data[17], pose_data[18], pose_data[19]));
 
    in_pos.close();
    return res;
};

/////////////////////////////////////////////////////
inline Pose_type calcDisplasment(Pose_type gt, Pose_type pose, Pose_type zero, string name = "", int format = 0) {
    Pose_type res;
    Pose_type delta;
    ofstream out;
    out.open(name + ".txt", ios::app);

    res.setPose(pose.getPose() - gt.getPose());
    res.setOrient(pose.getOrient() - gt.getOrient());

    delta.setPose(zero.getPose() - gt.getPose());
    delta.setOrient(zero.getOrient() - gt.getOrient());

    double per_r = 100.0 * sqrt(res.lat * res.lat + res.lon * res.lon + res.alt * res.alt);
    per_r /= sqrt(delta.lat * delta.lat + delta.lon * delta.lon + delta.alt * delta.alt);

    double per_a = 100.0 * sqrt(res.roll * res.roll + res.pitch * res.pitch + res.yaw * res.yaw);

    switch (format) {
    case 0:
        cout << name << ":\t" << resetiosflags(ios_base::floatfield) << sqrt(delta.lat * delta.lat + delta.lon * delta.lon + delta.alt * delta.alt) << "m\t" << sqrt(res.lat * res.lat + res.lon * res.lon + res.alt * res.alt) << "m\t" << per_r << "%";
        cout << endl;
        out << sqrt(delta.lat * delta.lat + delta.lon * delta.lon + delta.alt * delta.alt) << "\t" << sqrt(res.lat * res.lat + res.lon * res.lon + res.alt * res.alt) << "\t" << per_r << endl;
        break;
    case 1:
        cout << name << ":\t" << res.lat << "\t" << res.lon << "\t" << res.alt << "\t";
        cout << res.pitch << "\t" << res.roll << "\t" << res.yaw;
        break;
    case 2:
        cout << name << ":\t" << resetiosflags(ios_base::floatfield) << sqrt(delta.lat * delta.lat + delta.lon * delta.lon + delta.alt * delta.alt) << "m\t" << sqrt(res.lat * res.lat + res.lon * res.lon + res.alt * res.alt) << "m\t" << per_r << "%";
        break;
    case 3:
        break;
    };
    out.close();
    return res;
};

/////////////////////////////////////////////////////
inline Point3d calcDisplasment_ang(Point3d gt, Point3d pose, int mode = 0) {
    if (mode == 1) return Point3d(gt.x - pose.x, gt.y - pose.y, gt.z - pose.z);

    cout << "AnG_disp" << ":\t" << resetiosflags(ios_base::floatfield) << gt.x - pose.x << "del\t" << gt.y - pose.y << "\t" << gt.z - pose.z << endl;
    cout << "AnG_disp" << ":\t" << gt.x << "\t" << gt.y << "\t" << gt.z << endl;
    cout << "AnG_disp" << ":\t" << pose.x << "\t" << pose.y << "\t" << pose.z << endl;
    return Point3d(gt.x - pose.x, gt.y - pose.y, gt.z - pose.z);
};

