#include <ctime>
#include <cstdlib>
#include <chrono>
#include <opencv2/opencv.hpp> 
//#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/aruco.hpp> 

#include <fstream>



int main(int argc, char * argv[]) 
{
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams = new cv::aruco::DetectorParameters;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
//    cv::aruco::ArucoDetector detector(dictionary, detectorParams);

    cv::Mat cameraMatrix = cv::Mat::eye(3,3,CV_32FC1);
    cameraMatrix.at<float>(0,0) = 607.019;
    cameraMatrix.at<float>(1,1) = 607.019;
    cameraMatrix.at<float>(0,2) = 315.875;
    cameraMatrix.at<float>(1,2) = 248.763;
    cv::Mat distCoeffs(5,1,CV_32FC1); // [0.182874,-0.56215,-0.000804634,-0.00028305,0.502297]
    distCoeffs.at<float>(0) = 0.182874;
    distCoeffs.at<float>(1) = -0.56215;
    distCoeffs.at<float>(2) = -0.000804634;
    distCoeffs.at<float>(3) = -0.00028305;
    distCoeffs.at<float>(4) = 0.502297;
    cv::Mat objPoints(4, 1, CV_32FC3);
    float markerLength = 0.02;
    int origin_id = 0;
    objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength / 2.f, markerLength / 2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength / 2.f, markerLength / 2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength / 2.f, -markerLength / 2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength / 2.f, -markerLength / 2.f, 0);

    cv::Mat image = cv::imread("image.png");
    cv::cvtColor(image,image,cv::COLOR_RGB2BGR);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners,rejectedCandidates;
    cv::aruco::detectMarkers(image, 
    dictionary, 
    corners, 
    ids, detectorParams, rejectedCandidates); 
   // detector.detectMarkers(image, corners, ids);
    std::ofstream fs("calibration_tag_pos.txt");
    if (ids.size() > 0) {
        cv::aruco::drawDetectedMarkers(image, corners, ids);
        int nMarkers = corners.size();
        std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);
        std::vector<cv::Vec3d> r_relative_vecs(nMarkers), t_relative_vecs(nMarkers);

        // Calculate pose for each marker
        for (int i = 0; i < nMarkers; i++) {
            solvePnP(objPoints, corners.at(i), cameraMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
        }
        std::vector<int>::iterator origin_id_iter = std::find(ids.begin(),ids.end(),origin_id);
        int origin_id = -1;
        if(origin_id_iter != ids.end()) {
            origin_id = std::distance(ids.begin(),origin_id_iter);
        } else {
            std::cout << "can not find id 0 tag!\n";
            exit(0);
        }
        printf("origin id:%d\n",origin_id);
        cv::Mat r_mat_origion(3,3,CV_32FC1);
        cv::Mat t_mat_origion(tvecs[origin_id]);
        cv::Rodrigues( rvecs[origin_id],r_mat_origion);

        for (int i = 0; i < nMarkers; i++) {
            if (ids[i] == 0) {
                continue;
            }
            printf("marker id %d\n",ids[i]);
            // solvePnP(objPoints, corners.at(i), cameraMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
            // std::cout << rvecs[i] << std::endl;
            // std::cout << tvecs[i] << std::endl;
            // std::cout << std::endl;
            cv::Mat r_mat(3,3,CV_32FC1);
            cv::Rodrigues( rvecs[i],r_mat);
            cv::Mat delta_r = r_mat_origion.t() * r_mat;
            cv::Mat delta_t = r_mat_origion.t() * (tvecs[i] - t_mat_origion);
            std::cout << delta_r << std::endl;
            std::cout << delta_t << std::endl;
            std::cout << std::endl;
            cv::Mat delta_r_vec;
            cv::Rodrigues( delta_r,delta_r_vec);

            fs << ids[i] << " " << delta_r_vec.at<double>(0) 
                         << " " << delta_r_vec.at<double>(1) 
                         << " " << delta_r_vec.at<double>(2)
                         << " " << delta_t.at<double>(0)
                         << " " << delta_t.at<double>(1)
                         << " " << delta_t.at<double>(2) << '\n';
        }


        // Draw axis for each marker
        for(unsigned int i = 0; i < ids.size(); i++) {
            cv::aruco::drawAxis(image, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.03);
        }
    }

    // Update the window with new data
    cv::imshow("window", image);
    cv::waitKey();
    return 0;
}
