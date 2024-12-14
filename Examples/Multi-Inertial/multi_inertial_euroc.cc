#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>
#include <sstream>

#include <opencv2/core/core.hpp>

#include "ImuTypes.h"
#include <System.h>

using namespace std;

void LoadImages(
    const string &strPathLeft,
    const string &strPathRight,
    const string &strPathSideLeft,
    const string &strPathSideRight,
    const string &strPathTimes,
    vector<string> &vstrImageLeft,
    vector<string> &vstrImageRight,
    vector<string> &vstrImageSideLeft,
    vector<string> &vstrImageSideRight,
    vector<double> &vTimeStamps);

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
    string pathImu = pathSeq + "/imu0/data.csv";
    LoadImages(
        pathCam0,
        pathCam1,
        pathCam2,
        pathCam3,
        pathTimeStamps,
        vstrImageLeft,
        vstrImageRight,
        vstrImageSideLeft,
        vstrImageSideRight,
        vTimestampsCam);
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

    cv::Mat imLeft, imRight, imSideLeft, imSideRight;
    vector<ORB_SLAM3::IMU::Point> vImuMeas;

    for (int ni = 0; ni < nImages; ni++) {

        imLeft = cv::imread(vstrImageLeft[ni], cv::IMREAD_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni], cv::IMREAD_UNCHANGED);
        imSideLeft = cv::imread(vstrImageSideLeft[ni], cv::IMREAD_UNCHANGED);
        imSideRight = cv::imread(vstrImageSideRight[ni], cv::IMREAD_UNCHANGED);

        if (imLeft.empty()) {
            cerr << endl << "Failed to load image at: " << string(vstrImageLeft[ni]) << endl;
            return 1;
        }

        if (imRight.empty()) {
            cerr << endl << "Failed to load image at: " << string(vstrImageRight[ni]) << endl;
            return 1;
        }

        if (imSideLeft.empty()) {
            cerr << endl << "Failed to load image at: " << string(vstrImageSideLeft[ni]) << endl;
            return 1;
        }

        if (imSideRight.empty()) {
            cerr << endl << "Failed to load image at: " << string(vstrImageSideRight[ni]) << endl;
            return 1;
        }

        double tframe = vTimestampsCam[ni];

        // Load imu measurements from previous frame
        vImuMeas.clear();
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
        SLAM.TrackMulti(imLeft, imRight, imSideLeft, imSideRight, tframe, vImuMeas);

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

void LoadImages(
    const string &strPathLeft,
    const string &strPathRight,
    const string &strPathSideLeft,
    const string &strPathSideRight,
    const string &strPathTimes,
    vector<string> &vstrImageLeft,
    vector<string> &vstrImageRight,
    vector<string> &vstrImageSideLeft,
    vector<string> &vstrImageSideRight,
    vector<double> &vTimeStamps) {

    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());

    while (!fTimes.eof()) {
        string s;
        getline(fTimes, s);
        if (!s.empty()) {
            stringstream ss;
            ss << s;
            vstrImageLeft.push_back(strPathLeft + "/" + ss.str() + ".png");
            vstrImageRight.push_back(strPathRight + "/" + ss.str() + ".png");
            vstrImageSideLeft.push_back(strPathSideLeft + "/" + ss.str() + ".png");
            vstrImageSideRight.push_back(strPathSideRight + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t / 1e9);
        }
    }
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
