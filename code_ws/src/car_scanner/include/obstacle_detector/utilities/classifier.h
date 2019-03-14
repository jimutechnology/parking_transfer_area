// the file is used to classify whether a point set is a wheel
#include <stdlib.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "laser_converter.h"

using namespace std;
using namespace obstacle_detector;
using namespace cv;
using namespace cv::ml;
class Classifier
{
 public:
    Classifier(const string model_path)
    {
        svm = Algorithm::load<SVM>(model_path);
    }
    ~Classifier(){}

    void predict(PointSet& ps)
    {
        ps.get_feature_points();
        cv::Mat image = cv::Mat::zeros(cv::Size(50,50), CV_32FC1);
        //cout << "get data" << endl;
        converter.get_data(ps.group, ps.feature_point);
        //cout << "initialize the converter" << endl;
        converter.convert(image);
        //cout << "converted" << endl;
        cv::Mat image_r = image.clone().reshape(1, 1);
        image_r.convertTo(image_r, CV_32FC1);;
        ps.class_result = svm->predict(image_r);
        double confidence; 
        confidence = svm->predict(image_r, noArray(), ml::StatModel::RAW_OUTPUT); // get the confidence score
        ps.confidence = std::abs(confidence);
 	// ps.confidence = 1.0 / (1.0 + exp(-confidence));
        //cout << "predict finished" << endl;
    }

 private:
        Ptr<SVM> svm;
        Converter converter;

};
