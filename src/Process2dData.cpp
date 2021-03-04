#include "Process2dData.h"

Process2dData::Process2dData()
{
    ;
}

cv::Mat Process2dData::removeNoneRoi(const cv::Mat& original_img, std::vector<int> roi_vect)
{
    int rows = original_img.rows;
    int cols = original_img.cols;

    std::cout << rows << std::endl;
    std::cout << cols << std::endl;

    cv::Mat roi_img = cv::Mat::zeros(cv::Size(cols,rows),CV_8UC3);
    

    for(int y = 0; y < rows; ++y)
    {
        for(int x = 0; x < cols; ++x)
        {
            if(x >= roi_vect[0] && x <= roi_vect[2] && y >= roi_vect[1] && y <= roi_vect[3])
            {
                roi_img.at<cv::Vec3b>(y,x)[0] = original_img.at<cv::Vec3b>(y,x)[0];
                roi_img.at<cv::Vec3b>(y,x)[1] = original_img.at<cv::Vec3b>(y,x)[1];
                roi_img.at<cv::Vec3b>(y,x)[2] = original_img.at<cv::Vec3b>(y,x)[2];
            }
        }
    }

    return(roi_img);
}

cv::Mat Process2dData::edgeDetection(const cv::Mat& img, std::vector<int> roi_vect)
{
    int rows = img.rows;
    int cols = img.cols;

    cv::Mat res_img = cv::Mat::zeros(cv::Size(cols,rows),CV_8UC1);
    cv::Mat bil_img, gray_img, canny_img;

    // Blur img
    cv::bilateralFilter(img, bil_img, 6, 30, 30);

    // Convert to grayscale
    cv::cvtColor(bil_img, gray_img, cv::COLOR_BGR2GRAY);

    // Canny edge detection
    cv::Canny(gray_img,canny_img,35,90);

    // Remove contours of background
    for(int y = 0; y < rows; ++y)
    {
        for(int x = 0; x < cols; ++x)
        {
            if(x < roi_vect[0] + 5 || x > roi_vect[2] - 5 || y < roi_vect[1] + 5 || y > roi_vect[3] - 5)
            {
                canny_img.at<uchar>(y,x) = 0;
            }
        }

    }

    // Get contours
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours( canny_img, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );

    // Get convex hull
    std::vector<std::vector<cv::Point>> hull( contours.size() );
    for( size_t i = 0; i < contours.size(); i++ )
    {
        cv::convexHull( contours[i], hull[i] );
    }

    // Fill poly
    cv::fillPoly(res_img, hull, cv::Scalar(255,255,255), 8, 0);

    return( res_img );
}

cv::Mat Process2dData::getBinaryImg(const cv::Mat& original_img, std::vector<int> roi_vect)
{
    std::cout << "Get Binary img" << std::endl;

    // "Cut" out roi of img
    cv::Mat roi_img = removeNoneRoi(original_img, roi_vect);

    // Get binary image of object
    cv::Mat bin_obj_img = edgeDetection(roi_img, roi_vect);

    return(bin_obj_img);
}

