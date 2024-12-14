#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>
#include <sstream>

#include <opencv2/core/core.hpp>

#include "ImuTypes.h"
#include <System.h>

using namespace std;

void LoadIMU(
    const string &strImuPath,
    vector<double> &vTimeStamps,
    vector<cv::Point3f> &vAcc,
    vector<cv::Point3f> &vGyro);

int main(int argc, char **argv) {
    if (argc != 5) {
        cerr << endl
             << "Usage: ./multi_inertial_euroc path_to_vocabulary path_to_settings "
                "path_to_sequence_folder path_to_times_file"
             << endl;
        return 1;
    }

    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<string> vstrImageSideLeft;
    vector<string> vstrImageSideRight;

    vector<string> vstrImageLeft2;
    vector<string> vstrImageRight2;
    vector<string> vstrImageSideLeft2;
    vector<string> vstrImageSideRight2;

    vector<string> vstrDepthLeft2;
    vector<string> vstrDepthRight2;
    vector<string> vstrDepthSideLeft2;
    vector<string> vstrDepthSideRight2;

    vector<double> vTimestampsCam;
    vector<cv::Point3f> vAcc, vGyro;
    vector<double> vTimestampsImu;
    int nImages;
    int nImu;
    int first_imu = 0;

    string pathSeq(argv[3]);
    string pathTimeStamps(argv[4]);

    string pathCam0 = pathSeq + "/cam1/Zero_DCE_PLUS"; // Left Camera
    string pathCam1 = pathSeq + "/cam0/Zero_DCE_PLUS"; // Right Camera
    string pathCam2 = pathSeq + "/cam4/Zero_DCE_PLUS"; // SideLeft Camera
    string pathCam3 = pathSeq + "/cam3/Zero_DCE_PLUS"; // Sideright Camera

    string pathCam4 = pathSeq + "/cam1/Zero_DCE_PLUS_undistorted"; // Left Camera
    string pathCam5 = pathSeq + "/cam0/Zero_DCE_PLUS_undistorted"; // Right Camera
    string pathCam6 = pathSeq + "/cam4/Zero_DCE_PLUS_undistorted"; // SideLeft Camera
    string pathCam7 = pathSeq + "/cam3/Zero_DCE_PLUS_undistorted"; // Sideright Camera

    string pathDepth4 = pathSeq + "/cam1/Abs_Depth_undistorted"; // Left Camera
    string pathDepth5 = pathSeq + "/cam0/Abs_Depth_undistorted"; // Right Camera
    string pathDepth6 = pathSeq + "/cam4/Abs_Depth_undistorted"; // SideLeft Camera
    string pathDepth7 = pathSeq + "/cam3/Abs_Depth_undistorted"; // Sideright Camera

    string pathImu = pathSeq + "/imu0/data.csv";

    // LoadImages
    {
        ifstream fTimes;
        fTimes.open(pathTimeStamps.c_str());

        while (!fTimes.eof()) {
            string s;
            getline(fTimes, s);
            if (!s.empty()) {
                stringstream ss;
                ss << s;
                vstrImageLeft.push_back(pathCam0 + "/" + ss.str() + ".png");
                vstrImageRight.push_back(pathCam1 + "/" + ss.str() + ".png");
                vstrImageSideLeft.push_back(pathCam2 + "/" + ss.str() + ".png");
                vstrImageSideRight.push_back(pathCam3 + "/" + ss.str() + ".png");

                vstrImageLeft2.push_back(pathCam4 + "/" + ss.str() + ".png");
                vstrImageRight2.push_back(pathCam5 + "/" + ss.str() + ".png");
                vstrImageSideLeft2.push_back(pathCam6 + "/" + ss.str() + ".png");
                vstrImageSideRight2.push_back(pathCam7 + "/" + ss.str() + ".png");

                vstrDepthLeft2.push_back(pathDepth4 + "/" + ss.str() + ".png");
                vstrDepthRight2.push_back(pathDepth5 + "/" + ss.str() + ".png");
                vstrDepthSideLeft2.push_back(pathDepth6 + "/" + ss.str() + ".png");
                vstrDepthSideRight2.push_back(pathDepth7 + "/" + ss.str() + ".png");

                double t;
                ss >> t;
                vTimestampsCam.push_back(t / 1e9);
            }
        }
    }

    cout << "LOADED!" << endl;

    cout << "Loading IMU ...";
    LoadIMU(pathImu, vTimestampsImu, vAcc, vGyro);
    cout << "LOADED!" << endl;

    nImages = vstrImageLeft.size();
    nImu = vTimestampsImu.size();

    if ((nImages <= 0) || (nImu <= 0)) {
        cerr << "ERROR: Failed to load images or IMU" << endl;
        return 1;
    }

    // Find first imu to be considered, supposing imu measurements start first
    while (vTimestampsImu[first_imu] <= vTimestampsCam[0]) {
        first_imu++;
    }
    first_imu--; // first imu measurement to be considered

    cout << endl << "-------" << endl;
    cout.precision(17);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_MULTI, true);
    ORB_SLAM3::Verbose::SetTh(ORB_SLAM3::Verbose::VERBOSITY_DEBUG);

    for (int ni = 0; ni < nImages; ni++) {

        cv::Mat imLeft = cv::imread(vstrImageLeft[ni], cv::IMREAD_GRAYSCALE);
        cv::Mat imRight = cv::imread(vstrImageRight[ni], cv::IMREAD_GRAYSCALE);
        cv::Mat imSideLeft = cv::imread(vstrImageSideLeft[ni], cv::IMREAD_GRAYSCALE);
        cv::Mat imSideRight = cv::imread(vstrImageSideRight[ni], cv::IMREAD_GRAYSCALE);

        cv::Mat imLeft2 = cv::imread(vstrImageLeft2[ni], cv::IMREAD_GRAYSCALE);
        cv::Mat imRight2 = cv::imread(vstrImageRight2[ni], cv::IMREAD_GRAYSCALE);
        cv::Mat imSideLeft2 = cv::imread(vstrImageSideLeft2[ni], cv::IMREAD_GRAYSCALE);
        cv::Mat imSideRight2 = cv::imread(vstrImageSideRight2[ni], cv::IMREAD_GRAYSCALE);

        if (imLeft.empty() || imRight.empty() || imSideLeft.empty() || imSideRight.empty()) {
            cerr << endl << "Failed to load image at: " << string(vstrImageLeft[ni]) << endl;
            return 1;
        }
        if (imLeft2.empty() || imRight2.empty() || imSideLeft2.empty() || imSideRight2.empty()) {
            cerr << endl << "Failed to load image at: " << string(vstrImageLeft[ni]) << endl;
            return 1;
        }

        cv::Mat depthLeft2 = cv::imread(vstrDepthLeft2[ni], cv::IMREAD_UNCHANGED);
        cv::Mat depthRight2 = cv::imread(vstrDepthRight2[ni], cv::IMREAD_UNCHANGED);
        cv::Mat depthSideLeft2 = cv::imread(vstrDepthSideLeft2[ni], cv::IMREAD_UNCHANGED);
        cv::Mat depthSideRight2 = cv::imread(vstrDepthSideRight2[ni], cv::IMREAD_UNCHANGED);

        if (depthLeft2.empty() && depthRight2.empty() && depthSideLeft2.empty() &&
            depthSideRight2.empty()) {

            depthLeft2 = cv::Mat::zeros(imLeft2.size(), CV_32F);
            depthRight2 = cv::Mat::zeros(imRight2.size(), CV_32F);
            depthSideLeft2 = cv::Mat::zeros(imSideLeft2.size(), CV_32F);
            depthSideRight2 = cv::Mat::zeros(imSideRight2.size(), CV_32F);
        } else if (
            !depthLeft2.empty() && !depthRight2.empty() && !depthSideLeft2.empty() &&
            !depthSideRight2.empty()) {

            depthLeft2.convertTo(depthLeft2, CV_32F, 1.0 / 256.0);
            depthRight2.convertTo(depthRight2, CV_32F, 1.0 / 256.0);
            depthSideLeft2.convertTo(depthSideLeft2, CV_32F, 1.0 / 256.0);
            depthSideRight2.convertTo(depthSideRight2, CV_32F, 1.0 / 256.0);
        } else {
            cerr << endl << "Depth Read Error" << endl;
            return 1;
        }

        double tframe = vTimestampsCam[ni];

        // Load imu measurements from previous frame
        vector<ORB_SLAM3::IMU::Point> vImuMeas;
        if (ni > 0) {
            while (vTimestampsImu[first_imu] <= vTimestampsCam[ni]) {
                vImuMeas.push_back(
                    ORB_SLAM3::IMU::Point(
                        vAcc[first_imu].x,
                        vAcc[first_imu].y,
                        vAcc[first_imu].z,
                        vGyro[first_imu].x,
                        vGyro[first_imu].y,
                        vGyro[first_imu].z,
                        vTimestampsImu[first_imu]));
                first_imu++;
            }
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // Pass the images to the SLAM system
        SLAM.TrackMulti(
            imLeft,
            imRight,
            imSideLeft,
            imSideRight,
            imLeft2,
            imRight2,
            imSideLeft2,
            imSideRight2,
            depthLeft2,
            depthRight2,
            depthSideLeft2,
            depthSideRight2,
            tframe,
            vImuMeas);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();

        // Wait to load the next frame
        double T = 0;
        if (ni < nImages - 1) {
            T = vTimestampsCam[ni + 1] - tframe;
        } else if (ni > 0) {
            T = tframe - vTimestampsCam[ni - 1];
        }

        if (ttrack < T) {
            usleep((T - ttrack) * 1e6);
        }
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");

    return 0;
}

void LoadIMU(
    const string &strImuPath,
    vector<double> &vTimeStamps,
    vector<cv::Point3f> &vAcc,
    vector<cv::Point3f> &vGyro) {
    ifstream fImu;
    fImu.open(strImuPath.c_str());
    vTimeStamps.reserve(5000);
    vAcc.reserve(5000);
    vGyro.reserve(5000);

    while (!fImu.eof()) {
        string s;
        getline(fImu, s);
        if (s[0] == '#')
            continue;

        if (!s.empty()) {
            string item;
            size_t pos = 0;
            double data[7];
            int count = 0;
            while ((pos = s.find(',')) != string::npos) {
                item = s.substr(0, pos);
                data[count++] = stod(item);
                s.erase(0, pos + 1);
            }
            item = s.substr(0, pos);
            data[6] = stod(item);

            vTimeStamps.push_back(data[0] / 1e9);
            vAcc.push_back(cv::Point3f(data[4], data[5], data[6]));
            vGyro.push_back(cv::Point3f(data[1], data[2], data[3]));
        }
    }
}
