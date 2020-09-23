#include "SaveData.h"

// ----------------------------------------------------------------------------------------------------

/**
 * Constructor to set saving file name
 */
SaveData::SaveData(ImageData &my_data)
{
    my_data.folder_path = "/home/jeroen/workspace/vision_imgs/";
}

// ----------------------------------------------------------------------------------------------------

/**
 * Function to save data
 */
void SaveData::saveData(ImageData &my_data)
{
    int nr = 1;

    cout << "Type your desired name to which files and folder will be named: " << endl;
    cin >> my_data.save_name;

    string saving_folder_path = my_data.folder_path + my_data.save_name + "_" + to_string(nr);

    while(1)
    {
        boost::filesystem::path p (saving_folder_path);        
        if(boost::filesystem::exists(p))
        {
            nr++;
            saving_folder_path = my_data.folder_path + my_data.save_name + "_" + to_string(nr);
        }
        else
        {
            string make_folder = ("mkdir -p " + saving_folder_path);
            const int dir_err = system(make_folder.c_str()); //Function needs a c-style string .c_str()
            if (-1 == dir_err)
            {
                printf("Error creating directory!n");
                exit(1);
            }
            else
            {
                cout << "Created folder data: " << saving_folder_path << endl;
                
                // string save_name = saving_folder_path + "/cv_color_show.png";
                // cv::imwrite(save_name, data.cv_color_show);

                string save_name = saving_folder_path + "/point_cloud.pcd";
                pcl::io::savePCDFileASCII (save_name, *my_data.original_cloud);

                cout << "Saved data to: " << saving_folder_path << endl;
            }
            break;
        }
    }  
}