#include "GetRoi.h"

using namespace std;

GetRoi::GetRoi()
{
    cout << "GetRoi created" << endl;
}

vector<int> GetRoi::Yolo(int argc, char **argv, Mat img, bool debug)
{
    vector<int> roi_vect;
    Mat rgb_img = img;

    ros::init(argc, argv, "get_roi_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<vision_3d_test::GetRoi>("get_roi");
    vision_3d_test::GetRoi srv;
    
    //https://stackoverflow.com/questions/27080085/how-to-convert-a-cvmat-into-a-sensor-msgs-in-ros

    sensor_msgs::Image img_msg; //message to be sent 
    cv_bridge::CvImage img_bridge;

    std_msgs::Header header; // empty header
    header.seq = 1; //user defined counter
    header.stamp = ros::Time::now(); // time
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, img);
    img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
    srv.request.input = img_msg;

    if (client.call(srv))
    {

        int64 list_length = srv.response.output.size();
        if(debug)
            cout << "list length is " << list_length << endl;
        for(int i = 0; i < list_length; i++)
        {
            roi_vect.push_back(srv.response.output[i]);
            if(debug)
                ROS_INFO("Sum: %ld", (long int)srv.response.output[i]);
        }
    }
    else
    {
        ROS_ERROR("Failed to call service get_roi");
    }
    return(roi_vect);
}
