#include<librealsense2/rs.hpp>
#include<opencv2/opencv.hpp>
#include<iostream>  //Terminal IO
#include <fstream>              // File IO
#include <sstream>              // Stringstreams

// 3rd party header for writing png files
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

using namespace cv;
using namespace std;

bool save_frame_raw_data(const std::string& filename, rs2::frame frame); //This function save a depth image to a binary file. I found it here:https://github.com/IntelRealSense/librealsense/issues/1485
//here's how to view .bin file as image: https://superuser.com/questions/294270/how-to-view-raw-binary-data-as-an-image-with-given-width-and-height

//even better : here's how to save color and depth images as csv files :
//https://github.com/dorodnic/librealsense/blob/text_sample/examples/capture/rs-capture.cpp

void metadata_to_csv(const rs2::frame& frm, const std::string& filename);
float get_depth_scale(rs2::device dev);

int main(){

  rs2::config cfg;
  // cfg.enable_stream(RS2_STREAM_INFRARED, 640, 480 , RS2_FORMAT_Y8, 30);
  cfg.enable_stream(RS2_STREAM_COLOR, 640, 480 , RS2_FORMAT_BGR8, 30);//this format is compatible with opencv
  cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

  rs2::pipeline pipe;
  rs2::pipeline_profile profile = pipe.start(cfg);
  rs2::colorizer color_map;

  // save intrincsics to .txt file (and the depth scale too)
  ofstream depth_intrinsics_f;
  depth_intrinsics_f.open ("data/depth_intrinsics.txt");

  auto const depth_intrinsics = pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
  // auto const extrinsics = pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_extrinsics();
  depth_intrinsics_f<<depth_intrinsics.width<<endl;
  depth_intrinsics_f<<depth_intrinsics.height<<endl;
  depth_intrinsics_f<<depth_intrinsics.ppx<<endl;
  depth_intrinsics_f<<depth_intrinsics.ppy<<endl;
  depth_intrinsics_f<<depth_intrinsics.fx<<endl;
  depth_intrinsics_f<<depth_intrinsics.fy<<endl;
  depth_intrinsics_f<<depth_intrinsics.model<<endl;
  for(int k=0;k<5; k++)
    depth_intrinsics_f<<depth_intrinsics.coeffs[k]<<"\t";

  float depth_scale = get_depth_scale(profile.get_device());
  depth_intrinsics_f<<"\n\n";
  depth_intrinsics_f<<depth_scale<<endl;
  depth_intrinsics_f.close();

  ofstream color_intrinsics_f;
  color_intrinsics_f.open ("data/color_intrinsics.txt");
  auto const color_intrinsics = pipe.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
  // auto const extrinsics = pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_extrinsics();
  color_intrinsics_f<<color_intrinsics.width<<endl;
  color_intrinsics_f<<color_intrinsics.height<<endl;
  color_intrinsics_f<<color_intrinsics.ppx<<endl;
  color_intrinsics_f<<color_intrinsics.ppy<<endl;
  color_intrinsics_f<<color_intrinsics.fx<<endl;
  color_intrinsics_f<<color_intrinsics.fy<<endl;
  color_intrinsics_f<<color_intrinsics.model<<endl;
  for(int k=0;k<5; k++)
    color_intrinsics_f<<color_intrinsics.coeffs[k]<<"\t";
  color_intrinsics_f.close();

  //wait for the first few frames for the camera to settle
  for(int i=0; i<30; i++)
    pipe.wait_for_frames();

  int counter = 0; //helps to name the frames correctly
  while((cv::waitKey() & 0xEFFFFF) != 27) //while esc wasn't pressed
  {
    //extract frames form buffer
    rs2::frameset data = pipe.wait_for_frames(); //becaue there are 3 streams, this gives 3 frames
    rs2::frame depth_frame = data.get_depth_frame();
    rs2::frame color_frame = data.get_color_frame();
    if(!depth_frame)
      cout<<"!depth_frame"<<endl;
    if(!color_frame)
      cout<<"!color_frame"<<endl;

    Mat depth(Size(640, 480), CV_16UC1, (void*)depth_frame.get_data(), Mat::AUTO_STEP);
    Mat color(Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);

    std::ostringstream frame_number;
    frame_number<< counter;

    imwrite( "data/Depth-"+frame_number.str()+".png", depth );
    imwrite( "data/Color-"+frame_number.str()+".png", color );

    metadata_to_csv(depth_frame, "data/Depth-"+frame_number.str()+"-metadata.csv");
    metadata_to_csv(color_frame, "data/Color-"+frame_number.str()+"-metadata.csv");

    counter++;
    cout<<counter<<endl;
  }
}

void metadata_to_csv(const rs2::frame& frm, const std::string& filename)
{
    std::ofstream csv;
    csv.open(filename);
    //    std::cout << "Writing metadata to " << filename << endl;
    csv << "Stream," << rs2_stream_to_string(frm.get_profile().stream_type()) << "\nMetadata Attribute,Value\n";

    // Record all the available metadata attributes
    for (size_t i = 0; i < RS2_FRAME_METADATA_COUNT; i++)
    {
        if ( frm.supports_frame_metadata( (rs2_frame_metadata_value)i ) )
        {
            csv << rs2_frame_metadata_to_string((rs2_frame_metadata_value)i) << ","
                << frm.get_frame_metadata((rs2_frame_metadata_value)i) << "\n";
        }
    }
    csv.close();
}
float get_depth_scale(rs2::device dev)
{
    // Go over the device's sensors
    for (rs2::sensor& sensor : dev.query_sensors())
    {
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
        {
            return dpt.get_depth_scale();
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}

bool save_frame_raw_data(const std::string& filename, rs2::frame frame)
{
    bool ret = false;
    auto image = frame.as<rs2::video_frame>();
    if (image)
    {
        std::ofstream outfile(filename.data(), std::ofstream::binary);
        outfile.write(static_cast<const char*>(image.get_data()), image.get_height()*image.get_stride_in_bytes());

        outfile.close();
        ret = true;
    }

    return ret;
}
