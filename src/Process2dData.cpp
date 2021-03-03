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

void Process2dData::edgeDetection(const cv::Mat& img)
{
    cv::Mat bil_img, gray_img, canny_img;

    // cv::bilateralFilter(img, bil_img, 3, 5, 3);
    cv::bilateralFilter(img, bil_img, 6, 30, 30);

    cv::cvtColor(bil_img, gray_img, cv::COLOR_BGR2GRAY);

    cv::Canny(gray_img,canny_img,35,90);

    // ------------NOTE---------------
    // Check how contours work with their anarchy. find outer contour of canny's results and remove contour of ROI.
    // Then use fillPoly to fill the object plane

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours( canny_img, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );

    cv::namedWindow( "imag", cv::WINDOW_AUTOSIZE );
    cv::namedWindow( "bil image", cv::WINDOW_AUTOSIZE );
    cv::namedWindow( "Gray image", cv::WINDOW_AUTOSIZE );
    cv::namedWindow( "Canny image", cv::WINDOW_AUTOSIZE );


    cv::imshow( "imag", img );
    cv::imshow( "bil image", bil_img );
    cv::imshow( "Gray image", gray_img );
    cv::imshow( "Canny image", canny_img );

    cv::waitKey(0);
}

cv::Mat Process2dData::getBinaryImg(const cv::Mat& original_img, std::vector<int> roi_vect)
{
    std::cout << "Get Binary img" << std::endl;

    // Create black img
    cv::Mat bin_img(480, 640, CV_8UC3, cv::Scalar(0,0,0));
    
    cv::Mat roi_img = removeNoneRoi(original_img, roi_vect);

    edgeDetection(roi_img);

    return(roi_img);
}

