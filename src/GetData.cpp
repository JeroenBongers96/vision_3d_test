#include "GetData.h"

/**
 * Constructor to set local variables
 */
GetData::GetData(bool debug, bool create_data, bool save_data)
{
    this->debug = debug;
    this->create_data = create_data;
    this->save_data = save_data;
}

// ----------------------------------------------------------------------------------------------------

/**
 * Get RGB texture
 */
std::tuple<int, int, int>  GetData::rgbTexture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY)
{
    int NT1 = 0;
    int NT2 = 0;
    int NT3 = 0;

    // Get Width and Height coordinates of texture
    int width  = texture.get_width();  // Frame width in pixels
    int height = texture.get_height(); // Frame height in pixels

    // Normals to Texture Coordinates conversion
    int x_value = min(max(int(Texture_XY.u * width  + .5f), 0), width - 1);
    int y_value = min(max(int(Texture_XY.v * height + .5f), 0), height - 1);
    //std::cout << x_value << "\n";
    

    int bytes = x_value * texture.get_bytes_per_pixel();   // Get # of bytes per pixel
    int strides = y_value * texture.get_stride_in_bytes(); // Get line width in bytes
    int Text_Index =  (bytes + strides);

    const auto New_Texture = reinterpret_cast<const uint8_t*>(texture.get_data());

    // RGB components to save in tuple
    NT1 = New_Texture[Text_Index];
    NT2 = New_Texture[Text_Index + 1];
    NT3 = New_Texture[Text_Index + 2];
    
    return std::tuple<int, int, int>(NT1, NT2, NT3);
}

// ----------------------------------------------------------------------------------------------------

/**
 * Convert data to point cloud
 */
void GetData::pclConversion(const rs2::points& points, const rs2::video_frame& color, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &original_cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // Declare Tuple for RGB value Storage (<t0>, <t1>, <t2>)
    std::tuple<uint8_t, uint8_t, uint8_t> rgb_color;
    //================================
    // PCL Cloud Object Configuration
    //================================
    // Convert data captured from Realsense camera to Point Cloud
    auto sp = points.get_profile().as<rs2::video_stream_profile>();

    cloud->width  = static_cast<uint32_t>( sp.width()  );   
    cloud->height = static_cast<uint32_t>( sp.height() );
    cloud->is_dense = false;
    cloud->points.resize( points.size() );

    auto Texture_Coord = points.get_texture_coordinates();
    auto Vertex = points.get_vertices();

    // Iterating through all points and setting XYZ coordinates
    // and RGB values
    for (int i = 0; i < points.size(); i++)
    {   
        //===================================
        // Mapping Depth Coordinates
        // - Depth data stored as XYZ values
        //===================================
        cloud->points[i].x = Vertex[i].x;
        cloud->points[i].y = Vertex[i].y;
        cloud->points[i].z = Vertex[i].z;

        // Obtain color texture for specific point
        rgb_color = rgbTexture(color, Texture_Coord[i]);

        // Mapping Color (BGR due to Camera Model)
        cloud->points[i].r = get<2>(rgb_color); // Reference tuple<2>
        cloud->points[i].g = get<1>(rgb_color); // Reference tuple<1>
        cloud->points[i].b = get<0>(rgb_color); // Reference tuple<0>
    }

    original_cloud = cloud;
}


// ----------------------------------------------------------------------------------------------------

/**
 * Creates data from Realsense cameras
 * NOTE: that the software takes 30 frames to let the camera auto tune. 
 * THIS COSTS EXTR PROCESSING TIME!
 */
void GetData::createData(ImageData &my_data)
{
    //Contruct a pipeline which abstracts the device
    rs2::pipeline pipe;

    //Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    rs2::pointcloud pc;
    rs2::points points;

    //Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

    //Instruct pipeline to start streaming with the requested configuration
    pipe.start(cfg);

    // Camera warmup - dropping several first frames to let auto-exposure stabilize
    rs2::frameset frames;
    for(int i = 0; i < 30; i++)
    {
        //Wait for all configured streams to produce a frame
        auto frames = pipe.wait_for_frames();
    }

    frames = pipe.wait_for_frames();
    auto depth = frames.get_depth_frame();
	auto color = frames.get_color_frame();

    //Create colored pc
    pc.map_to(color);
	points = pc.calculate(depth);
    pclConversion(points, color, my_data.original_cloud);

    //Create cv image from point cloud
    my_data.cv_img = cvFromPcl(my_data.original_cloud);
}

// ----------------------------------------------------------------------------------------------------

/**
 * Load data from file
 */
void GetData::loadData(ImageData &my_data)
{
    string folder_name = "/g20x20_6";
    // cout << "Please enter the name of desired folder: ";
    // cin >> folder_name;

    string loading_name = my_data.folder_path + folder_name;

    string file_name = "/cv_img.png";    
    string input = loading_name + file_name;
    // string test_img = "/home/jeroen/cv_img.png";
    // my_data.cv_img = cv::imread(test_img, cv::IMREAD_UNCHANGED);

    // Temp image for D435 test
    loading_name = "/home/robohub/workspaces/vision_images/D435/both/1_1";

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    file_name = "/point_cloud.pcd";
    input = loading_name + file_name;
    pcl::io::loadPCDFile<pcl::PointXYZRGB> (input, *cloud);
    my_data.original_cloud = cloud;

    my_data.cv_img = cvFromPcl(cloud);

    cout << "Finished loading data from: " << my_data.folder_path + folder_name << endl;
}

// ----------------------------------------------------------------------------------------------------

/**
 * Create CV image from PCL colored point cloud
 */
cv::Mat GetData::cvFromPcl(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{    
    cv::Mat cv_img = cv::Mat::zeros(cv::Size(cloud->width,cloud->height),CV_8UC3);

    for(int y = 0; y < cloud->height; ++y)
    {
        for(int x = 0; x < cloud->width; ++x)
        {
            pcl::PointXYZRGB point = cloud->at(x, y);

            cv_img.at<cv::Vec3b>(y,x)[0] = (int)point.b;
            cv_img.at<cv::Vec3b>(y,x)[1] = (int)point.g;
            cv_img.at<cv::Vec3b>(y,x)[2] = (int)point.r;
        }
    }

    return cv_img;
}

// ----------------------------------------------------------------------------------------------------

/**
 * Create or load data
 */
void GetData::getData(ImageData &my_data)
{   
    //Create or load data
    if(create_data == true)
    {
        cout << "Create data from realsense camera ...." << endl;
        createData(my_data);

        if(save_data == true)
        {
            SaveData save_data(my_data);
            save_data.saveData(my_data);
        }
    }
    else
    {
        my_data.folder_path = "/home/robohub/colcon_ws/src/vision_3d_test/test_data";
        loadData(my_data);
    }
}