/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc != 6) {
        cerr << endl << "Usage: ./liveZed [path_to_vocabulary] [path_to_settings] [zedCameraIndex] [resolution]" << endl;
        return 1;
    }

    // Init camera
    int width = atoi(argv[4]);
    int height = -1;
    if(width == 1280){
        height = 480;
    }else if(width == 2560){
        height = 720;
    }else{
        std::cout << "Not allowed resolution, set it to 1280 or 2560" << std::endl;
        return -1;
    }

    cv::VideoCapture zedCamera(atoi(argv[3]));
    zedCamera.set(CV_CAP_PROP_FRAME_WIDTH, width);
    zedCamera.set(CV_CAP_PROP_FRAME_HEIGHT, height);
    if(zedCamera.get(CV_CAP_PROP_FRAME_WIDTH) !=  width){
       std::cout << "Couldn't set camera resolution "<< width << ". Current resolution is " << zedCamera.get(CV_CAP_PROP_FRAME_WIDTH) << std::endl;
        return -1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO, atoi(argv[5]) == 1?true:false);
    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    cout << endl << "-------" << endl;
    cout << "Start recording..." << endl;

    // Main loop -----------------------------------------------
    cv::Mat imLeft, imRight;
    auto rightFrame = cv::Rect(width/2, 0, width/2, height);
    auto leftFrame = cv::Rect(0, 0, width/2, height);

    std::chrono::steady_clock::time_point initTime = std::chrono::steady_clock::now();
    for(;;){
        // Read left and right images from file
        std::chrono::steady_clock::time_point currentTime = std::chrono::steady_clock::now();
        zedCamera >> imLeft;
        if(imLeft.rows == 0){
            std::cout << "Error getting images!" << std::endl;
            return -1;
        }

        imRight = imLeft(rightFrame).clone();
        imLeft = imLeft(leftFrame);


        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        // Pass the images to the SLAM system
        double tFrame = std::chrono::duration_cast<std::chrono::duration<double> >(currentTime - initTime).count();
        SLAM.TrackStereo(imLeft,imRight,tFrame);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        vTimesTrack.push_back(ttrack);
        std::cout << "time step: " << ttrack << std::endl;
        cv::Mat location, rotation;
        SLAM.lastPose(location, rotation);
        std::cout << rotation << std::endl;
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(unsigned ni=0; ni<vTimesTrack.size(); ni++) {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[vTimesTrack.size()/2] << endl;
    cout << "mean tracking time: " << totaltime/vTimesTrack.size() << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");

    return 0;
}
