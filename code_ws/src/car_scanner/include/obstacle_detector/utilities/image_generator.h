#include <stdlib.h>
#include <opencv2/core/core.hpp>  
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>  
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp> 
#include "opencv2/imgproc/imgproc.hpp"
#include "laser_converter.h"
#include "point_set.h"
using namespace std;
using namespace obstacle_detector;

class ImageGenerator
{
public: 
      ImageGenerator(){}
      ~ImageGenerator(){}

      void generate(PointSet& tmp1, PointSet& tmp2, string image_path_, char *first_input_, char *second_input_)
      {
      cv::Mat image1 = cv::Mat::zeros(cv::Size(50,50), CV_32FC1);
      cv::Mat image2 = cv::Mat::zeros(cv::Size(50,50), CV_32FC1);
      Converter converter1;
      Converter converter2;
      converter1.get_data(tmp1.group, tmp1.feature_point);
      converter2.get_data(tmp2.group, tmp2.feature_point);
      converter1.convert(image1);
      converter2.convert(image2);

      std::ostringstream dir1, dir2;
      dir1 << image_path_ << first_input_ << ".png";
      dir2 << image_path_ << second_input_ << ".png";
      cv::imwrite(dir1.str(), image1);
      cv::imwrite(dir2.str(), image2);
      cout << "already written" << endl;
      }
};