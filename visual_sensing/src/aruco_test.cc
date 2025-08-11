#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp> 

int main() {
    cv::Mat markerImage;
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    // cv::aruco::generateImageMarker(dictionary, 23, 200, markerImage, 1);
    // cv::imwrite("marker23.png", markerImage);

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners,rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams = new cv::aruco::DetectorParameters;

    cv::aruco::detectMarkers(img_, dictionary, corners, ids,
        detectorParams_,rejectedCandidates);
    return 0;
}