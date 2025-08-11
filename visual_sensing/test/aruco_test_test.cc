#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

class ArucoTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Load a test image with markers
        testImage = cv::imread("./test_images/marker_test.png");
        dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        detectorParams = cv::aruco::DetectorParameters::create();
    }

    cv::Mat testImage;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams;
};

TEST_F(ArucoTest, DetectMarkersSuccess) {
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners, rejectedCandidates;

    cv::aruco::detectMarkers(testImage, dictionary, corners, ids, detectorParams, rejectedCandidates);

    // Verify that markers are detected
    EXPECT_FALSE(ids.empty());
    EXPECT_FALSE(corners.empty());
}

TEST_F(ArucoTest, DetectMarkersEmptyImage) {
    cv::Mat emptyImage;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners, rejectedCandidates;

    cv::aruco::detectMarkers(emptyImage, dictionary, corners, ids, detectorParams, rejectedCandidates);

    // Verify no markers are detected in an empty image
    EXPECT_TRUE(ids.empty());
    EXPECT_TRUE(corners.empty());
}

TEST_F(ArucoTest, DetectMarkersInvalidDictionary) {
    cv::Ptr<cv::aruco::Dictionary> invalidDictionary;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners, rejectedCandidates;

    cv::aruco::detectMarkers(testImage, invalidDictionary, corners, ids, detectorParams, rejectedCandidates);

    // Verify no markers are detected with an invalid dictionary
    EXPECT_TRUE(ids.empty());
    EXPECT_TRUE(corners.empty());
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}