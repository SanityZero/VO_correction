
/*


typedef struct {
    float trancl[3] = { 0 };
    float rot[3] = { 0 };
} Pose_type;

Pose_type QtoAngl() {

};

Pose_type getGTData(string gt_file, double time) {
    static double timestamp = 0;
    Pose_type res;

    ifstream in(gt_file);
    static ios::pos_type pos = ios::beg;
    in.seekg(pos);
    double min_deltatime;
    do //while (!in.eof());
    {
        double ts_tmp;
        float t_tmp[3];
        float q_tmp[4];

        string data_line;
        string old_data_line;
        getline(in, data_line);
        if (data_line.find("#") != -1) continue;//skip comments

        if (data_line[0] == '5') data_line = "130" + data_line;//костыль, т.к. в первой строке он зажовывает первые 3 символа



        //преобразование строки в набор чисел
        ts_tmp = stod(data_line.substr(0, data_line.find(' ')));

        if (min_deltatime > time - ts_tmp) {//ищется место, в котором разница начинает увеличиваться и тогда
            min_deltatime = time - ts_tmp;
            old_data_line = data_line;
            continue;
        }
        old_data_line = old_data_line.substr(0, old_data_line.find(' '));

        for (int i = 0; i < 3; i++) {
            t_tmp[i] = stof(old_data_line.substr(0, old_data_line.find(' ')));
            old_data_line = old_data_line.substr(0, old_data_line.find(' '));
        };

        for (int i = 0; i < 3; i++) {
            q_tmp[i] = stof(old_data_line.substr(0, old_data_line.find(' ')));
            old_data_line = old_data_line.substr(0, old_data_line.find(' '));
        };
        q_tmp[3] = stof(old_data_line);

        //преобразование в

        in.seekg(-1, ios::cur);
        pos = in.tellg();//сохранить указатель, чтобы потом перейти в нужное место
        break;
    } while (!in.eof());

    return res;
};

typedef struct {
    Mat mat;
    double timestamp;

} Source_Type;

Source_Type GetSourceMat(string data_dir) {
    Source_Type res;

    ifstream in(data_dir+"rgb.txt");
    static ios::pos_type pos = ios::beg;
    in.seekg(pos);
    do //while (!in.eof());
    {
        string data_line;
        getline(in, data_line);
        if (data_line.find("#") != -1) continue;//skip comments

        if (data_line[0] == '5') data_line = "130" + data_line;//костыль, т.к. в первой строке он зажовывает первые 3 символа

        res.timestamp = stod(data_line.substr(0, data_line.find(' ')));
        res.mat = imread(data_dir + data_line.substr(data_line.find(' ')+1, data_line.length()));

        in.seekg(-1, ios::cur);
        pos = in.tellg();//сохранить указатель, чтобы потом перейти в нужное место
        break;
    } while (!in.eof());

    in.close();
    return res;
}



int main(int argc, char** argv){
    // Create some random colors
    vector<Scalar> colors;
    RNG rng;
    for (int i = 0; i < 100; i++)
    {
        int r = rng.uniform(0, 256);
        int g = rng.uniform(0, 256);
        int b = rng.uniform(0, 256);
        colors.push_back(Scalar(r, g, b));
    }
    Mat old_frame, old_gray;
    vector<Point2f> p0, p1;
    // Take first frame and find corners in it
    old_frame = GetSourceMat("E:/ProgStaff/rgbd_dataset_freiburg1_xyz/rgbd_dataset_freiburg1_xyz/").mat;
    cvtColor(old_frame, old_gray, COLOR_BGR2GRAY);
    //goodFeaturesToTrack(old_gray, p0, 100, 0.3, 7, Mat(), 7, false, 0.04);
    // Create a mask image for drawing purposes
    Mat mask = Mat::zeros(old_frame.size(), old_frame.type());
    int i = 0;
    while (true) {
        if (i % 3 == 0)
        {
            goodFeaturesToTrack(old_gray, p0, 100, 0.3, 7, Mat(), 7, false, 0.04);
        }
        Mat frame, frame_gray;
        frame = GetSourceMat("E:/ProgStaff/rgbd_dataset_freiburg1_xyz/rgbd_dataset_freiburg1_xyz/").mat;
        if (frame.empty()) break;
        cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
        // calculate optical flow
        vector<uchar> status;
        vector<float> err;
        TermCriteria criteria = TermCriteria((TermCriteria::COUNT)+(TermCriteria::EPS), 10, 0.03);
        calcOpticalFlowPyrLK(old_gray, frame_gray, p0, p1, status, err, Size(15, 15), 2, criteria);
        vector<Point2f> good_new;
        for (uint i = 0; i < p0.size(); i++) {
            // Select good points
            if (status[i] == 1) {
                good_new.push_back(p1[i]);
                // draw the tracks
                line(mask, p1[i], p0[i], colors[i], 2);
                circle(frame, p1[i], 5, colors[i], -1);
            }
        }
        Mat img;
        add(frame, mask, img);
        imshow("Frame", img);
        int keyboard = waitKey(30);
        if (keyboard == 'q' || keyboard == 27) break;
        // Now update the previous frame and previous points
        old_gray = frame_gray.clone();
        p0 = good_new;
    }
}



*/




//// OpenCV-test.cpp : Этот файл содержит функцию "main". Здесь начинается и заканчивается выполнение программы.
////
//#define _CRT_SECURE_NO_WARNINGS
//#include "opencv2/imgproc.hpp"
//#include "opencv2/highgui.hpp"
//#include <opencv2/video/tracking.hpp>
//#include <math.h>
//#include <iostream>
//#include <string>
//#include <algorithm>
//
//using namespace std;
//using namespace cv;
//
//
//Mat GetSourceMat()
//{
//    const int SOURCE_NUM = 2;
//    static int source_cnt;
//    static int num = 0;
//
//    int source_limits[] = {
//        107,
//        107,
//    };
//    string source_files[] = {
//        "C:/Kitti/2011_09_26/2011_09_26_drive_0001_sync/image_00/data/",
//        "C:/Kitti/2011_09_26/2011_09_26_drive_0001_sync/image_01/data/"
//    };
//
//    string filename, num_part;
//    num_part.append(10 - to_string(num).length(), '0') += to_string(num); //0000000000.png
//    filename = "C:/Kitti/2011_09_26/2011_09_26_drive_0001_sync/image_00/data/" + num_part + ".png";
//
//    Mat res = imread(filename);
//    Mat res_true = res;
//    if (num < source_limits[source_cnt])
//    {
//        num++;
//    }
//    else
//    {
//        source_cnt < SOURCE_NUM ? source_cnt++ : 0;
//        num = 0;
//    }
//    cvtColor(res, res_true, COLOR_BGR2GRAY);
//    return res_true;
//}
//
//typedef enum {
//    HARRIS,
//} Detect_type;
//
//typedef struct {
//    int blockSize;
//    int apertureSize;
//    double k;
//} DetecorParam;
//
//typedef struct {
//    Detect_type dt;
//    DetecorParam dp;
//} Detector;
//
//Mat DetectorProcess(Mat src, Detector detector, bool status = true)
//{
//    static int count = 0;
//    Mat res = Mat::zeros(src.size(), CV_32FC1);
//    Mat res_norm, res_norm_scaled;
//    switch (detector.dt) {
//    case HARRIS:
//        /// Detector parameters
//        int blockSize = 2;
//        int apertureSize = 3;
//        double k = 0.04;
//        /// Detecting corners
//        cornerHarris(src, res, blockSize, apertureSize, k);
//        break;
//    };
//
//
//    /// Normalizing
//    normalize(res, res_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat());
//    convertScaleAbs(res_norm, res_norm_scaled);
//
//    if (status)
//    {
//        string filename = "C:/ProgStaff/img/res/" + to_string(count++) + ".png";
//        imwrite(filename, res_norm_scaled);
//    };
//    return res_norm_scaled;
//};
//
//string corners_window = "Corners detected";
////string aaaaaaaa = "AAaaaAaAaAAAaa";
//string source_window = "Source image";
//
//typedef enum
//{
//    NONE,
//    RES,
//    SOURCE,
//    COMMAND,
//} App_Status;
//
//typedef struct Target_s
//{
//    int first_time;
//    int last_time;
//    vector<Point> states;
//    vector<int> states_value;
//    Target_s(int ft, Point st, int st_v)
//    {
//        this->first_time = ft;
//        this->states.push_back(st);
//        this->states_value.push_back(st_v);
//    };
//} Target;
//
/////////////////////////////////////////////////////////////////
//#define WEIGHT_THRESH 5
//
/////////////////////////////////////////////////////////////////
//// функции весов
//#define TIME_WEIGHT(x) 10*exp(-x-1) //вес от разности во времени
//#define RAD_WEIGHT(x) 6*exp(-x/2) - 6  // вес от расстояния
//#define VALUE_WEIGHT(x) exp(-abs(2*x)) * 10 // вес от разности значения
//
//int main(int argc, char** argv)
//{
//    //App_Status status = NONE;
//    vector <Target> track_list;
//    int thresh = 80;
//    int max_thresh = 255;
//    int key = 0;
//    int time = 0;
//
//    vector<Scalar> colors;
//    RNG rng;
//    for (int i = 0; i < 100; i++)
//    {
//        int r = rng.uniform(0, 256);
//        int g = rng.uniform(0, 256);
//        int b = rng.uniform(0, 256);
//        colors.push_back(Scalar(r, g, b));
//    }
//    Mat old_frame, old_gray;
//    vector<Point2f> p0, p1;
//    // Take first frame and find corners in it
//    old_frame = GetSourceMat();
//    goodFeaturesToTrack(old_frame, p0, 100, 0.3, 7, Mat(), 7, false, 0.04);
//    // Create a mask image for drawing purposes
//    Mat mask = Mat::zeros(old_frame.size(), old_frame.type());
//
//    while (key != 27) // цикл frame
//    {
//        Mat frame;
//        frame = GetSourceMat();
//        if (frame.empty())
//            break;
//        // calculate optical flow
//        vector<uchar> status;
//        vector<float> err;
//        TermCriteria criteria = TermCriteria((TermCriteria::COUNT)+(TermCriteria::EPS), 10, 0.03);
//        calcOpticalFlowPyrLK(old_gray, frame, p0, p1, status, err, Size(15, 15), 2, criteria);
//        vector<Point2f> good_new;
//        for (uint i = 0; i < p0.size(); i++)
//        {
//            // Select good points
//            if (status[i] == 1) {
//                good_new.push_back(p1[i]);
//                // draw the tracks
//                line(mask, p1[i], p0[i], colors[i], 2);
//                circle(frame, p1[i], 5, colors[i], -1);
//            }
//        }
//        Mat img;
//        add(frame, mask, img);
//        imshow("Frame", img);
//        int keyboard = waitKey(30);
//        if (keyboard == 'q' || keyboard == 27)
//            break;
//        // Now update the previous frame and previous points
//        old_gray = frame.clone();
//        p0 = good_new;
//
//
//        //Detector detector;
//        //detector.dt = HARRIS;
//        //detector.dp.blockSize = 2;
//        //detector.dp.apertureSize = 3;
//        //detector.dp.k = 0.04;
//        //Mat src, res;
//        //vector<Point> cur_trg;
//        //
//        //namedWindow(source_window, WINDOW_AUTOSIZE);
//        //src = GetSourceMat();
//        //res = DetectorProcess(src, detector);
//        //time++;
//        //
//        //for (int j = 1; j < res.rows - 1; j++)
//        //{
//        //    for (int i = 1; i < res.cols - 1; i++)
//        //    {
//        //        if (res.at<char>(j, i) > thresh)
//        //        {
//        //            cur_trg.push_back(Point(i,j));
//        //            // тут нужно понять, как они будут отслеживаться.
//        //            circle(src, Point(i, j), 5, Scalar(0), 2, 8, 0);
//        //        }
//        //    }
//        //};
//        //
//        //// использовать только одиночные точки
//        //
//        //
//        //if(time == 1) 
//        //    for (vector<Point>::iterator it = cur_trg.begin(); it != cur_trg.end(); ++it) 
//        //    {
//        //        track_list.push_back(Target(time, *it, res.at<char>(*it)));
//        //    }
//        //else
//        //    for (vector<Point>::iterator it_out = cur_trg.begin(); it_out != cur_trg.end(); ++it_out)
//        //    {
//        //        vector<float> weight;
//        //        for (vector<Target>::iterator it_in = track_list.begin(); it_in != track_list.end(); ++it_in)
//        //        {
//        //            float w = 0.0;
//        //            float r = sqrt(pow(it_in->states.back().x - it_out->x, 2) + pow(it_in->states.back().y - it_out->y, 2));
//        //            w = TIME_WEIGHT(time - it_in->last_time) + RAD_WEIGHT(r) + VALUE_WEIGHT(it_in->states_value.back() - res.at<char>(*it_out));
//        //            weight.push_back(w);
//        //        };
//        //
//        //        float max = *max_element(weight.begin(), weight.end());
//        //        if (max > WEIGHT_THRESH)
//        //        {
//        //            // привязать к существующей цепочке
//        //            for (int i = 0; i < weight.size(); i++)
//        //            {
//        //                if (weight[i] == max)
//        //                {
//        //                    track_list[i].states.push_back(*it_out);
//        //                    track_list[i].last_time = time;
//        //                    track_list[i].states_value.push_back(res.at<char>(*it_out));
//        //                    break;
//        //                };
//        //            }
//        //        }
//        //        else
//        //        {
//        //           // начать новую
//        //            track_list.push_back(Target(time, *it_out, res.at<char>(*it_out)));
//        //        };
//        //
//        //
//        //        
//        //    };
//        //
//        //// тут нужно рисовать линии по цепочкам, я так думаю.
//        //for (vector<Target>::iterator it_out = track_list.begin(); it_out != track_list.end(); ++it_out)
//        //{
//        //    for (int i = 1; i < it_out->states.size() - 1; i++)
//        //    {
//        //        line(src, it_out->states[i - 1], it_out->states[i], Scalar(0, 0, 0), 2, LINE_8);
//        //    };
//        //};
//        //
//        //imshow(corners_window, res);
//        //imshow(source_window, src);
//
//
//        //switch (key)
//        //{
//        //case 115://s
//        //    status = SOURCE;
//        //    break;
//        //case 114://r
//        //    status = RES;
//        //    break;
//        //case 'c'://c
//        //    status = COMMAND;
//        //    break;
//        //};
//        //
//        //string mess;
//        //
//        //switch (status) 
//        //{
//        //case RES:
//        //    imshow(source_window, res);
//        //    break;
//        //case SOURCE:
//        //    imshow(source_window, src);
//        //    break;
//        //case COMMAND:
//        //    //cout << "command: ";
//            //getline(cin, mess);
//            //cout << mess << endl;
//            //cout << "key:\t" << key << endl;
//            //system("cls");
//            //status = NONE; 
//        //    break;
//        //case NONE:
//        //    break;
//        //}
//        //
//          //Mat dst, dst_norm, dst_norm_scaled;
//            //dst = Mat::zeros(src.size(), CV_32FC1);
//            //
//            ///// Detector parameters
//            //int blockSize = 2;
//            //int apertureSize = 3;
//            //double k = 0.04;
//            ///// Detecting corners
//            //cornerHarris(src_gray, dst, blockSize, apertureSize, k);
//            //
//            ///// Normalizing
//            //normalize(dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat());
//            //convertScaleAbs(dst_norm, dst_norm_scaled);
//            //
//            ///// Drawing a circle around corners
//            //for (int j = 0; j < dst_norm.rows; j++)
//            //{
//            //    for (int i = 0; i < dst_norm.cols; i++)
//            //    {
//            //        if ((int)dst_norm.at<float>(j, i) > thresh)
//            //        {
//            //            circle(dst_norm_scaled, Point(i, j), 5, Scalar(0), 2, 8, 0);
//            //        }
//            //    }
//            //}
//            //Mat res = src.clone();
//            //
//            ///// Showing the result
//            //namedWindow(corners_window, WINDOW_AUTOSIZE);
//            //namedWindow(aaaaaaaa, WINDOW_AUTOSIZE);
//            //imshow(corners_window, dst_norm_scaled);
//            // you can do some image processing here
//        key = waitKey(10); // you can change wait time
//        //cout << "key:\t" << key << '\t' << status << endl;
//        //114 r
//    } //while (key != 27) // цикл frame
//
//
//}
//
//
////#include "opencv2/imgproc.hpp"
////#include "opencv2/highgui.hpp"
////#include <Windows.h>
////#include <iostream>
////#include <string>
////
////using namespace std;
////using namespace cv;
////
////Mat hwnd2mat(HWND hwnd)
////{
////    HDC hwindowDC, hwindowCompatibleDC;
////
////    int height, width, srcheight, srcwidth;
////    HBITMAP hbwindow;
////    Mat src;
////    BITMAPINFOHEADER  bi;
////
////    hwindowDC = GetDC(hwnd);
////    hwindowCompatibleDC = CreateCompatibleDC(hwindowDC);
////    SetStretchBltMode(hwindowCompatibleDC, COLORONCOLOR);
////
////    RECT windowsize;    // get the height and width of the screen
////    GetClientRect(hwnd, &windowsize);
////
////    srcheight = windowsize.bottom;
////    srcwidth = windowsize.right;
////    height = windowsize.bottom / 1;  //change this to whatever size you want to resize to
////    width = windowsize.right / 1;
////
////    src.create(height, width, CV_8UC4);
////
////    // create a bitmap
////    hbwindow = CreateCompatibleBitmap(hwindowDC, width, height);
////    bi.biSize = sizeof(BITMAPINFOHEADER);    //http://msdn.microsoft.com/en-us/library/windows/window/dd183402%28v=vs.85%29.aspx
////    bi.biWidth = width;
////    bi.biHeight = -height;  //this is the line that makes it draw upside down or not
////    bi.biPlanes = 1;
////    bi.biBitCount = 32;
////    bi.biCompression = BI_RGB;
////    bi.biSizeImage = 0;
////    bi.biXPelsPerMeter = 0;
////    bi.biYPelsPerMeter = 0;
////    bi.biClrUsed = 0;
////    bi.biClrImportant = 0;
////
////    // use the previously created device context with the bitmap
////    SelectObject(hwindowCompatibleDC, hbwindow);
////    // copy from the window device context to the bitmap device context
////    StretchBlt(hwindowCompatibleDC, 0, 0, width, height, hwindowDC, 0, 0, srcwidth, srcheight, SRCCOPY); //change SRCCOPY to NOTSRCCOPY for wacky colors !
////    GetDIBits(hwindowCompatibleDC, hbwindow, 0, height, src.data, (BITMAPINFO*)&bi, DIB_RGB_COLORS);  //copy from hwindowCompatibleDC to hbwindow
////
////    // avoid memory leak
////    DeleteObject(hbwindow);
////    DeleteDC(hwindowCompatibleDC);
////    ReleaseDC(hwnd, hwindowDC);
////
////    return src;
////}
////
////string source_window = "Source image";
////string corners_window = "Corners detected";
////string aaaaaaaa = "AAaaaAaAaAAAaa";
////
////int main(int argc, char** argv)
////{
////    int thresh = 180;
////    int max_thresh = 255;
////    HWND hwndDesktop = GetDesktopWindow();
////    int key = 0;
////
////    while (key != 27)
////    {
////        Mat src_graya, src_gray;
////        Mat src = hwnd2mat(hwndDesktop);
////        cvtColor(src, src_gray, COLOR_BGR2GRAY);
////
////
////        /// Create a window and a trackbar
////        namedWindow(source_window, WINDOW_AUTOSIZE);
////        imshow(source_window, src);
////
////        Mat dst, dst_norm, dst_norm_scaled;
////        dst = Mat::zeros(src.size(), CV_32FC1);
////
////        /// Detector parameters
////        int blockSize = 2;
////        int apertureSize = 3;
////        double k = 0.04;
////        /// Detecting corners
////        cornerHarris(src_gray, dst, blockSize, apertureSize, k);
////
////        /// Normalizing
////        normalize(dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat());
////        convertScaleAbs(dst_norm, dst_norm_scaled);
////
////        /// Drawing a circle around corners
////        for (int j = 0; j < dst_norm.rows; j++)
////        {
////            for (int i = 0; i < dst_norm.cols; i++)
////            {
////                if ((int)dst_norm.at<float>(j, i) > thresh)
////                {
////                    circle(dst_norm_scaled, Point(i, j), 5, Scalar(0), 2, 8, 0);
////                }
////            }
////        }
////        Mat res = src.clone();
////
////        /// Showing the result
////        namedWindow(corners_window, WINDOW_AUTOSIZE);
//        namedWindow(aaaaaaaa, WINDOW_AUTOSIZE);
//        imshow(corners_window, dst_norm_scaled);
//        // you can do some image processing here
//        key = waitKey(60); // you can change wait time
//    }
//
//}
//
