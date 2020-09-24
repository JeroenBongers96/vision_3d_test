#include "SaveData.h"

// ----------------------------------------------------------------------------------------------------

/**
 * Constructor to set saving file name
 */
SaveData::SaveData(ImageData &my_data)
{
    my_data.folder_path = "/home/jeroen/workspace/vision_imgs/";
}

//! Write cv::Mat as binary
/*!
\param[out] ofs output file stream
\param[in] out_mat mat to save
\https://github.com/takmin/BinaryCvMat
*/
bool SaveData::writeMatBinary(std::ofstream& ofs, const cv::Mat& out_mat)
{
	if(!ofs.is_open()){
		return false;
	}
	if(out_mat.empty()){
		int s = 0;
		ofs.write((const char*)(&s), sizeof(int));
		return true;
	}
	int type = out_mat.type();
	ofs.write((const char*)(&out_mat.rows), sizeof(int));
	ofs.write((const char*)(&out_mat.cols), sizeof(int));
	ofs.write((const char*)(&type), sizeof(int));
	ofs.write((const char*)(out_mat.data), out_mat.elemSize() * out_mat.total());

	return true;
}


//! Save cv::Mat as binary
/*!
\param[in] filename filaname to save
\param[in] output cvmat to save
\https://github.com/takmin/BinaryCvMat
*/
bool SaveData::saveMatBinary(const std::string& filename, const cv::Mat& output)
{
	std::ofstream ofs(filename, std::ios::binary);
	return writeMatBinary(ofs, output);
}

// ----------------------------------------------------------------------------------------------------

/**
 * Function to save data
 */
void SaveData::saveData(ImageData &my_data)
{
    int nr = 1;

    cout << "Type your desired name in which files and folder will be saved: " << endl;
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

                cout << "OpenCV version : " << CV_VERSION << endl;
                // cout << "Major version : " << cv::MAJOR_VERSION << endl;
                // cout << "Minor version : " << cv::MINOR_VERSION << endl;
                // cout << "Subminor version : " << cv::SUBMINOR_VERSION << endl;
                // cout << "Created folder data: " << saving_folder_path << endl;
                
                vector<int> compression_params;
                compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
                compression_params.push_back(9);

                string save_name = saving_folder_path + "/cv_img";
                cv::imwrite(save_name, my_data.cv_img, compression_params);
                // write Mat to file
                //bool check = saveMatBinary(save_name, my_data.cv_img);

                save_name = saving_folder_path + "/point_cloud.pcd";
                pcl::io::savePCDFileASCII (save_name, *my_data.original_cloud);

                cout << "Saved data to: " << saving_folder_path << endl;
            }
            break;
        }
    }  
}