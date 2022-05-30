#pragma once
////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Утилитарные функции

Mat rotationMat(Point3d);

Mat transMat(Point3d);

Mat ExternalCalibMat(Mat, Mat, Mat);

Point3d calculateX(Point, Pose_type, Pose_type, Mat);

Mat calculateB(Point2f, Mat);

void print_prv(vector<Point3d> data, Point3d true_data, string name, int plot_cols = 160, bool clear=false) {
    Point2i img_size = Point2i(320, 420);
    static vector<Point3d> delta_data;

    if (clear) delta_data.clear();

    namedWindow("PRV window", WINDOW_AUTOSIZE);

    double max = data[0].x - true_data.x;
    double min = data[0].x - true_data.x;
    double step;
    Point2d scale;

     
    for (int i = 0; i < data.size(); i++) {
        delta_data.push_back(- data[i] + true_data);
        max = max < delta_data[i].x ? delta_data[i].x : max;
        max = max < delta_data[i].y ? delta_data[i].y : max;
        max = max < delta_data[i].z ? delta_data[i].z : max;

        min = min > delta_data[i].x ? delta_data[i].x : min;
        min = min > delta_data[i].y ? delta_data[i].y : min;
        min = min > delta_data[i].z ? delta_data[i].z : min;
    };
    double range = abs(max) > abs(min) ? abs(max) : abs(min);
    step = range * 2 / plot_cols;
    scale.y = img_size.y*0.5 / range;
    
    vector<double> ranges;
    vector<int> col_value_x;
    vector<int> col_value_y;
    vector<int> col_value_z;
    for (int i = 0; i < plot_cols + 1; i++) {
        ranges.push_back(-range + step * i);
    };

    for (int i = 0; i < plot_cols; i++) {
        col_value_x.push_back(0);
        col_value_y.push_back(0);
        col_value_z.push_back(0);
        for (int data_i = 0; data_i < data.size(); data_i++) {

            col_value_x[col_value_x.size() - 1] += ((data[data_i].x >= ranges[i]) && (data[data_i].x < ranges[i + 1])) ? 1 : 0;
            col_value_y[col_value_y.size() - 1] += ((data[data_i].y >= ranges[i]) && (data[data_i].y < ranges[i + 1])) ? 1 : 0;
            col_value_z[col_value_x.size() - 1] += ((data[data_i].z >= ranges[i]) && (data[data_i].z < ranges[i + 1])) ? 1 : 0;
        };
    };

    max = 0;

    for (int i = 0; i < plot_cols; i++) {
        max = max < col_value_x[i] ? col_value_x[i] : max;
        max = max < col_value_y[i] ? col_value_y[i] : max;
        max = max < col_value_z[i] ? col_value_z[i] : max;
    };

    scale.x = img_size.x * 0.5 / (max);

    Mat img0(img_size.x, img_size.y, CV_8UC3, Scalar(255, 255, 255));
    Mat img1(img_size.x, img_size.y, CV_8UC3, Scalar(255, 255, 255));
    Mat img2(img_size.x, img_size.y, CV_8UC3, Scalar(255, 255, 255));

    Point center(img_size.x / 2 + 50, img_size.y / 2);
    Point zero(center.x, center.y + 100);
    Point start = zero - Point(100,0);

    ///X
    for (int i = 0; i < plot_cols; i++) {
        rectangle(
            img0,
            //center + Point(0, scale.y * (-plot_cols / 2 + plot_cols)),
            //center + Point(scale.x * col_value_z[i], scale.y * (-plot_cols / 2 + plot_cols + 1)),
            zero - Point(scale.y * ranges[i], 0),
            zero - Point(scale.y * ranges[i + 1], scale.x * col_value_x[i]),
            Scalar(255, 0, 0), 2, 8, 0
        );
    };

    line(img0, center, center + Point(0, 100), Scalar(0, 0, 0), 2);
    putText(img0,
        name+" x",
        Point(0, 20),
        0,
        0.5,
        CV_RGB(0, 0, 0),
        0.5
    );

    ///Y
    for (int i = 0; i < plot_cols; i++) {
        rectangle(
            img1,
            //center + Point(0, scale.y * (-plot_cols / 2 + plot_cols)),
            //center + Point(scale.x * col_value_z[i], scale.y * (-plot_cols / 2 + plot_cols + 1)),
            zero - Point(scale.y * ranges[i], 0),
            zero - Point(scale.y * ranges[i + 1], scale.x * col_value_y[i]),
            Scalar(255, 0, 0), 2, 8, 0
        );
    };

    line(img1, center, center + Point(0, 100), Scalar(0, 0, 0), 2);
    putText(img1,
        name + " y",
        Point(0, 20),
        0,
        0.5,
        CV_RGB(0, 0, 0),
        0.5
    );

    ///Z
    for (int i = 0; i < plot_cols; i++) {
        rectangle(
            img2,
            //center + Point(0, scale.y * (-plot_cols / 2 + plot_cols)),
            //center + Point(scale.x * col_value_z[i], scale.y * (-plot_cols / 2 + plot_cols + 1)),
            zero - Point(scale.y * ranges[i], 0),
            zero - Point(scale.y * ranges[i+1], scale.x * col_value_z[i]),
            Scalar(255, 0, 0), 2, 8, 0
        );
    };

    line(img2, center, center + Point(0, 100), Scalar(0, 0, 0), 2);
    putText(img2,
        name + " z",
        Point(0, 20),
        0,
        0.5,
        CV_RGB(0, 0, 0),
        0.5
    );

    Mat res(img_size.x*3 , img_size.y, CV_8UC3, Scalar(255, 255, 255));

    img0.copyTo(res(Rect(0, 0, img0.cols, img0.rows)));
    img1.copyTo(res(Rect(0, img0.rows, img1.cols, img1.rows)));
    img2.copyTo(res(Rect(0, img0.rows + img1.rows, img2.cols, img2.rows)));


    waitKey(0);
    imshow("PRV window", res);
};

void eraseLostFeatures(vector<Point2f>& p0, vector<Point2f>& p1, vector<Point2f>& p2, vector<Point2f>& p3, 
                       vector<uchar>& status0, vector<uchar>& status1, vector<uchar>& status2, vector<uchar>& status3) {
    vector<uchar>::iterator iters0 = status0.begin();
    vector<uchar>::iterator iters1 = status1.begin();
    vector<uchar>::iterator iters2 = status2.begin();
    vector<uchar>::iterator iters3 = status3.begin();

    vector<Point2f>::iterator iter0 = p0.begin();
    vector<Point2f>::iterator iter1 = p1.begin();
    vector<Point2f>::iterator iter2 = p2.begin();
    vector<Point2f>::iterator iter3 = p3.begin();
    int i = 0;
    while ((*(status0.end() - 1) == 0) || (*(status1.end() - 1) == 0) || (*(status2.end() - 1) == 0) || (*(status3.end() - 1) == 0)) {
        p0.pop_back();
        p1.pop_back();
        p2.pop_back();
        p3.pop_back();
        status0.pop_back();
        status1.pop_back();
        status2.pop_back();
        status3.pop_back();
    };

    for (; (iter0 != p0.end() - 1) && (iter1 != p1.end() - 1) && (iter2 != p2.end() - 1) && (iter3 != p3.end() - 1)
        && (iters0 != status0.end() - 1) && (iters1 != status1.end() - 1) && (iters2 != status2.end() - 1) && (iters3 != status3.end() - 1); i++) {
        if ((*iters0 == 0)|| (*iters1 == 0) || (*iters2 == 0) || (*iters3 == 0)) {
            iter0 = p0.erase(iter0);
            iter1 = p1.erase(iter1);
            iter2 = p2.erase(iter2);
            iter3 = p3.erase(iter3);
            iters0 = status0.erase(iters0);
            iters1 = status1.erase(iters1);
            iters2 = status2.erase(iters2);
            iters3 = status3.erase(iters3);
            continue;
        };
        iter0++;
        iter1++;
        iter2++;
        iter3++;
        iters0++;
        iters1++;
        iters2++;
        iters3++;
    };
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Вычисление оценки движения по пикселям и истинным значениям четырёх точек

vector<Point3d> pose4est(Mat, Point2f, Point2f, Point2f, Point2f, Point3d, Point3d, Point3d, Point3d);

vector<Point3d> finalFilter(vector<Point3d> angl_vec, vector<Point3d> tran_vec, Point3d true_pos_delta, Point3d true_angle, string mode = "average", bool clear_prv = false) {
    vector<Point3d> res;
    vector<Point3d> array_pos;
    vector<Point3d> array_ang;

    //if (clear_prv) {
    //    array_pos.clear();
    //    array_ang.clear();
    //};

    if (mode == "average") {
        Point3d tmp_angl = Point3d(0,0,0), tmp_tran = Point3d(0, 0, 0);
        for (int i = 0; i < angl_vec.size(); i++) {
            //cout << "\t" << tran_vec[i];
            tmp_angl += angl_vec[i];
            tmp_tran += tran_vec[i];
            array_pos.push_back(tran_vec[i]);
            array_ang.push_back(angl_vec[i]);
        };

        //cout << endl;

        tmp_angl = Point3d(
            tmp_angl.x / angl_vec.size(),
            tmp_angl.y / angl_vec.size(),
            tmp_angl.z / angl_vec.size()
        );

        tmp_tran = Point3d(
            tmp_tran.x / tran_vec.size(),
            tmp_tran.y / tran_vec.size(),
            tmp_tran.z / tran_vec.size()
        );
        cout << "VO:\t" << tmp_tran << endl;
        res.push_back(tmp_angl);
        res.push_back(tmp_tran);


        ///////////////////////////////////
        //тут нужно выводить прв
        print_prv(array_pos, true_pos_delta, "pose", 160 ,clear_prv);
        waitKey(0);
        print_prv(array_ang, true_angle, "angl", 160, clear_prv);
        waitKey(0);

    }
    else if (mode == "") {
        res.push_back(Point3d(0, 0, 0));
        res.push_back(Point3d(0, 0, 0));
    };

    return res;
};

vector<Point3d> VOCorrect(
    //Pose_type pose0, Pose_type pose1, Pose_type pose2, 
    Data_seq ds,
    vector<Point2f> p0_vec, vector<Point2f> p1_vec, vector<Point2f> p2_vec,
    Point3d true_pos_delta, Point3d true_angle
) {//возвращает вектор, первый элемент трансляция, второй - ротация 0- старший
    Pose_type pose0 = ds.pose[ds.pose.size() - 3];
    Pose_type pose1 = ds.pose[ds.pose.size() - 2];
    Pose_type pose2 = ds.pose[ds.pose.size() - 1];
    double IternalCalib_ar[3][3] = {
           {984.2439, 0, 690},
           {0, 980.8141, 233.1966},
           {0, 0, 1}
    };
    Mat IC = Mat(3, 3, CV_64F, IternalCalib_ar);
    //Mat IC = cameraModel(ds.calib_filename);

    //тут надо вытащить из позиций значения поворотов и трансляции
    Mat EC = ExternalCalibMat(IC, rotationMat(angDelta(pose1, pose0)), transMat(poseDelta(pose1, pose0)));


    double A_ar[3][3] = {
        {EC.at<double>(0, 0), EC.at<double>(0, 1), EC.at<double>(0, 2)},
        {EC.at<double>(1, 0), EC.at<double>(1, 1), EC.at<double>(1, 2)},
        {EC.at<double>(2, 0), EC.at<double>(2, 1), EC.at<double>(2, 2)}
    };
    Mat A = Mat(3, 3, CV_64F, A_ar);

    vector<Point3d> X;

    //for (int i = 0; i < p2_vec.size(); i++) {

    //    double B_ar[3][1] = {
    //    {1 - EC.at<double>(2, 3)},
    //    {p1_vec[i].x - EC.at<double>(1, 3)},
    //    {p1_vec[i].y - EC.at<double>(0, 3)}
    //    };
    //    Mat B = Mat(3, 1, CV_64F, B_ar);

    //    Mat X_mat = A.inv() * B;
    //    X.push_back(Point3d(X_mat.at<double>(0, 0), X_mat.at<double>(1, 0), X_mat.at<double>(2, 0)));
    //};

    for (int i = 0; i < p2_vec.size(); i++) {
        X.push_back(calculateX(p0_vec[i], pose0, pose1, IC));
    };

    //calculateX

    vector<Point3d> B;

    for (int i = 0; i < p2_vec.size(); i++) {

        double tmp_ar[3][1] = {
        {p2_vec[i].x},
        {p2_vec[i].y},
        {1}
        };
        Mat tmp = Mat(3, 1, CV_64F, tmp_ar);

        Mat B_mat = IC.inv() * tmp;
        B.push_back(Point3d(B_mat.at<double>(0, 0), B_mat.at<double>(1, 0), B_mat.at<double>(2, 0)));
    };


    vector<Point3d> VO_raw_data_rotat;
    vector<Point3d> VO_raw_data_trans;
    for (int i = 0; i < p2_vec.size()/4; i++) {
        vector<Point3d> tmp;
        tmp = pose4est(
            IC,
            p2_vec[i * 4], 
            p2_vec[(i * 4) + 1], 
            p2_vec[(i * 4) + 2], 
            p2_vec[(i * 4) + 3],
            X[i * 4], 
            X[(i * 4) + 1], 
            X[(i * 4) + 2], 
            X[(i * 4) + 3]
            );
        VO_raw_data_trans.push_back(tmp[0]);
        VO_raw_data_rotat.push_back(tmp[1]);
    };

    //нужно ещё сделать вывод, чтобы нормально отображалось всё на диаграммах.

    return finalFilter(VO_raw_data_rotat, VO_raw_data_trans, true_pos_delta, true_angle);
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////
Mat rotationMat(Point3d angles) {
    double a = angles.x, b = angles.y, c = angles.z;
    double rotMat_ar[3][3] = {
        {cos(c) * cos(b), -cos(b) * sin(b), sin(b)},
        {cos(a) * sin(b) + cos(c) * sin(b) * sin(a), cos(c) * cos(a) - sin(a) * sin(b) * sin(b), -cos(b) * sin(a)},
        {sin(b) * sin(a) - cos(c) * cos(a) * sin(b), cos(a) * sin(b) * sin(b) + cos(c) * sin(a), cos(b) * cos(a)}
    };
    Mat res = Mat(3, 3, CV_64F, rotMat_ar);
    return res;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////
Mat transMat(Point3d delta) {
    double transMat_ar[3][1] = {
        {delta.x},
        {delta.y},
        {delta.z}
    };
    Mat res = Mat(3, 1, CV_64F, transMat_ar);
    return res;
};

Mat ExternalCalibMat(Mat A, Mat R, Mat t) {
    double E_ar[3][4] = {
        {1, 0, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 1, 0}
    };
    Mat E = Mat(3, 4, CV_64F, E_ar);

    Mat R_t = R.t();

    Mat tmp = -R_t * t;

    double M_ar[4][4] = {
         {R_t.at<double>(0, 0), R_t.at<double>(0, 1), R_t.at<double>(0, 2), tmp.at<double>(0, 0)},
         {R_t.at<double>(1, 0), R_t.at<double>(1, 1), R_t.at<double>(1, 2), tmp.at<double>(1, 0)},
         {R_t.at<double>(2, 0), R_t.at<double>(2, 1), R_t.at<double>(2, 2), tmp.at<double>(2, 0)},
         {0, 0, 0, 1}
    };
    Mat M = Mat(4, 4, CV_64F, M_ar);

    tmp = A * E * M;
    return tmp;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////
Point3d calculateX(Point p, Pose_type pose0, Pose_type pose1, Mat IC) {

    double E_ar[3][4] = {
        {1, 0, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 1, 0}
    };
    Mat E = Mat(3, 4, CV_64F, E_ar);

    Mat tmp = IC * E;

    double A_ar[3][3] = {
        {tmp.at<double>(0, 0), tmp.at<double>(0, 1), tmp.at<double>(0, 2)},
        {tmp.at<double>(1, 0), tmp.at<double>(1, 1), tmp.at<double>(1, 2)},
        {tmp.at<double>(2, 0), tmp.at<double>(2, 1), tmp.at<double>(2, 2)}
    };
    Mat A = Mat(3, 3, CV_64F, A_ar);

    double B_ar[3][1] = {
        {1 - tmp.at<double>(2, 3)},
        {p.x - tmp.at<double>(1, 3)},
        {p.y - tmp.at<double>(0, 3)}
    };
    Mat B = Mat(3, 1, CV_64F, B_ar);
    Mat tmp2 = A.inv() * B;
    Point3d X = Point3d(tmp2.at<double>(0,0), tmp2.at<double>(1, 0), tmp2.at<double>(2, 0));
    return X;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////
Mat calculateB(Point2f p, Mat IternalnalCalib) {
    double tmp_ar[3][1] = {
        {1 - IternalnalCalib.at<double>(2, 2)},
        {p.x - IternalnalCalib.at<double>(1, 2)},
        {p.y - IternalnalCalib.at<double>(0, 2)}
    };
    Mat tmp = Mat(3, 1, CV_64F, tmp_ar);
    Mat B = IternalnalCalib.inv() * tmp;
    return B;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////
Mat calculateOdd(Point2f p0, Point2f p1, Point2f p2, Point2f p3, Mat IternalnalCalib, int number) {
    double tmp_ar[4] = { 0,0,0,0 };

    Mat I_inv = IternalnalCalib.inv();

    double m_p0_arr[3][1] = { {p0.x},{p0.y},{1} };
    Mat m_p0 = Mat(3, 1, CV_64F, m_p0_arr);
    Mat b_p0 = I_inv * m_p0;

    double m_p1_arr[3][1] = { {p1.x},{p1.y},{1} };
    Mat m_p1 = Mat(3, 1, CV_64F, m_p1_arr);
    Mat b_p1 = I_inv * m_p1;

    double m_p2_arr[3][1] = { {p2.x},{p2.y},{1} };
    Mat m_p2 = Mat(3, 1, CV_64F, m_p2_arr);
    Mat b_p2 = I_inv * m_p2;

    double m_p3_arr[3][1] = { {p3.x},{p3.y},{1} };
    Mat m_p3 = Mat(3, 1, CV_64F, m_p3_arr);
    Mat b_p3 = I_inv * m_p3;

    switch (number) {
    case 0:
        tmp_ar[0] = b_p0.at<double>(0, 0);
        tmp_ar[1] = b_p1.at<double>(0, 0);
        tmp_ar[2] = b_p2.at<double>(0, 0);
        tmp_ar[3] = b_p3.at<double>(0, 0);
        break;
    case 1:
        tmp_ar[0] = b_p0.at<double>(1, 0);
        tmp_ar[1] = b_p1.at<double>(1, 0);
        tmp_ar[2] = b_p2.at<double>(1, 0);
        tmp_ar[3] = b_p3.at<double>(1, 0);
        break;
    case 3:
        tmp_ar[0] = b_p0.at<double>(2, 0);
        tmp_ar[1] = b_p1.at<double>(2, 0);
        tmp_ar[2] = b_p2.at<double>(2, 0);
        tmp_ar[3] = b_p3.at<double>(2, 0);
        break;
    };
    double tmp_ar2[4][1] = {
                {tmp_ar[0]},
                {tmp_ar[1]},
                {tmp_ar[2]},
                {tmp_ar[3]}
    };
    
    Mat tmp = Mat(4, 1, CV_64F, tmp_ar);
    return tmp;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////
vector<Point3d> pose4est(Mat IC, Point2f p0, Point2f p1, Point2f p2, Point2f p3, Point3d X0, Point3d X1, Point3d X2, Point3d X3) {

    //вычислить матрицы В
    Mat odd_0 = calculateOdd(p0, p1, p2, p3, IC, 0);
    Mat odd_1 = calculateOdd(p0, p1, p2, p3, IC, 1);
    Mat odd_2 = calculateOdd(p0, p1, p2, p3, IC, 2);

    //составить 4 СЛАУ
    double A_ar[4][4] = {
        {X0.x, X0.y, X0.z, 1},
        {X1.x, X1.y, X1.z, 1},
        {X2.x, X2.y, X2.z, 1},
        {X3.x, X3.y, X3.z, 1}
    };
    Mat A = Mat(4, 4, CV_64F, A_ar);
    Mat A_inv = A.inv();

    //записать их решения
    Mat res0 = A_inv * odd_0;
    Mat res1 = A_inv * odd_1;
    Mat res2 = A_inv * odd_2;

    //из полученных решений составить матрицу М
    double R_t_ar[3][3] = {
        {res0.at<double>(0,0), res0.at<double>(1,0), res0.at<double>(2,0)},
        {res1.at<double>(0,0), res1.at<double>(1,0), res1.at<double>(2,0)},
        {res2.at<double>(0,0), res2.at<double>(1,0), res2.at<double>(2,0)}
    };
    Mat R_t = Mat(3, 3, CV_64F, R_t_ar);
    Mat R = R_t.t();

    double V_ar[3][1] = {
        {res0.at<double>(3,0)},
        {res1.at<double>(3,0)},
        {res2.at<double>(3,0)}
    };
    Mat V = Mat(3, 1, CV_64F, V_ar);

    //по матрице М получить оценку перемещения и вращения 
    Point3d angles = Point3d(
        atan2(R.at<double>(2, 1), R.at<double>(2, 2)),
        atan2(-R.at<double>(2, 0), sqrt(R.at<double>(2, 1) * R.at<double>(2, 1) + R.at<double>(2, 2) * R.at<double>(2, 2))),
        atan2(R.at<double>(1, 0), R.at<double>(0, 0))
    );

    Mat T_mat = R_t.inv() * (-V);
    Point3d T = Point3d(T_mat.at<double>(0, 0), T_mat.at<double>(1, 0), T_mat.at<double>(2, 0));

    //...
    vector<Point3d> res;
    res.push_back(angles);
    res.push_back(T);

    //Profit!
    return res;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////