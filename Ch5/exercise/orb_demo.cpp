#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <chrono>
using namespace std;
using namespace cv;

int main(int argc, char** argv) {
    std::cout << "Hello, world!\n";
    if(argc !=3){
        cout<<"usage: feature_extraction img1 img2"<<endl;
        return 1;
    }
    Mat img_1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    Mat img_2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);
    assert(img_1.data != nullptr && img_2.data!=nullptr); //check is two address is illegal

    //initialize
    vector<KeyPoint> keypoint_1, keypoint_2;
    Mat descriptors_1, descriptors_2;
    //Ptr smart pointer of type <T>once the pointing object is dead, it also dead. 
    //create() function will return a object of Feature2D (typedef Feature2D FeatureDetector);
    Ptr<FeatureDetector> detector = ORB::create(); 
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

    //step one find the ORB point
    chrono::steady_clock::time_point p1 = chrono::steady_clock::now();
    detector->detect(img_1,keypoint_1);
    detector->detect(img_2,keypoint_2);
    cout<<"keypoint size "<<keypoint_1.size()<<endl;
    cout<<"keypoint size "<<keypoint_2.size()<<endl;
    //step two calc the BRIEF descriptor
    descriptor->compute(img_1, keypoint_1, descriptors_1);
    descriptor->compute(img_2, keypoint_2, descriptors_2);
    cout<<"descriptor_1 size "<<descriptors_1.size()<<endl;
    cout<<"descriptor_2 size "<<descriptors_2.size()<<endl;
    chrono::steady_clock::time_point p2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(p2-p1);
    cout<<"time used "<<time_used.count()<<endl;

    Mat outimg1;
    drawKeypoints(img_1, keypoint_1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    imshow("ORB features", outimg1);

    //step three matching features with hamming distance
    vector<DMatch> matches; //DMatch the class for matching keypoint descriptor
    p1 = chrono::steady_clock::now();
    matcher->match(descriptors_1,descriptors_2, matches);
    p2 = chrono::steady_clock::now();
    time_used = chrono::duration_cast<chrono::duration<double>>(p2-p1);
    cout<<"meatches size "<<matches.size()<<endl;
    cout<<"matching used time "<<time_used.count()<<endl;

    //step four

    //lambda expression : 
    //[]:The function can visit global variabls 
    //(const DMatch &m1, const DMatch &m2) : parameters
    //{return m1.distance < m2.distance;} : function body
    auto min_max = minmax_element(matches.begin(), matches.end(), [](const DMatch &m1, const DMatch &m2){return m1.distance < m2.distance;});
    double min_dist = min_max.first->distance;
    double max_dist = min_max.second->distance;
    cout<<"max dist "<<max_dist<<" min_dist "<<min_dist<<endl;

    std::vector<DMatch> good_matches;

    for (int i = 0; i < descriptors_1.rows; i++)
    {
        if (matches[i].distance <= max(2 * min_dist, 30.0))
        {
            good_matches.push_back(matches[i]);
        }
    }

    //step 5 draw
    Mat img_match;
    Mat img_goodmatch;
    drawMatches(img_1, keypoint_1, img_2, keypoint_2, matches, img_match);
    drawMatches(img_1, keypoint_1, img_2, keypoint_2, good_matches, img_goodmatch);
    imshow("all match", img_match);
    imshow("good match", img_goodmatch);
    waitKey(0);


    return 0;
}
