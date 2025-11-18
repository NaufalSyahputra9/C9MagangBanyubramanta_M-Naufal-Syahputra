#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include<opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

void callback(int pos, void* userdata){
    cout << pos << endl;
    return;
}



int main(void)
{
    VideoCapture cap("second.mp4");

    // red
    int h_min1 = 0, h_max1 = 10;
    int h_min2 = 160, h_max2 = 180;

    // green
    //int h_min1 = 35, h_max1 = 85;
    //int h_min2 = 0, h_max2 = 0;

    int s_min = 100, v_min = 100;
    int s_max = 255, v_max = 255;

    namedWindow("Original", WINDOW_AUTOSIZE); 
    namedWindow("Track", WINDOW_AUTOSIZE);
    
    createTrackbar("Hue min 1", "Track", NULL, 180, callback);
    createTrackbar("Hue max 1", "Track", NULL, 180, callback);
    createTrackbar("Hue min 2", "Track", NULL, 180, callback);
    createTrackbar("Hue max 2", "Track", NULL, 180, callback);
    createTrackbar("Sat Min", "Track",NULL, 255, callback);
    createTrackbar("Sat Max", "Track",NULL, 255, callback);
    createTrackbar("Val Min", "Track",NULL, 255, callback);
    createTrackbar("Val Max", "Track",NULL, 255, callback);

    setTrackbarPos("Hue min 1", "Track", h_min1);
    setTrackbarPos("Hue max 1", "Track", h_max1);
    setTrackbarPos("Hue min 2", "Track", h_min2);
    setTrackbarPos("Hue max 2", "Track", h_max2);
    setTrackbarPos("Sat Min", "Track", s_min);
    setTrackbarPos("Sat Max", "Track", s_max);
    setTrackbarPos("Val Min", "Track", v_min);
    setTrackbarPos("Val Max", "Track", v_max);


    while (cap.isOpened()){
        Mat frame;
        if (!cap.read(frame)) break;

        h_min1 = getTrackbarPos("Hue min 1", "Track");
        h_max1 = getTrackbarPos("Hue max 1", "Track");
        h_min2 = getTrackbarPos("Hue min 2", "Track");
        h_max2 = getTrackbarPos("Hue max 2", "Track");
        s_min = getTrackbarPos("Sat Min", "Track");
        s_max = getTrackbarPos("Sat Max", "Track");
        v_min = getTrackbarPos("Val Min", "Track");
        v_max = getTrackbarPos("Val Max", "Track");

        Mat frameHSV, upper_mask, lower_mask, mask, resultHSV;
        cvtColor(frame, frameHSV, COLOR_BGR2HSV);

        Scalar minHSV1 = Scalar(h_min1, s_min, v_min);
        Scalar maxHSV1 = Scalar(h_max1, s_max, v_max);

        Scalar minHSV2 = Scalar(h_min2, s_min, v_min);
        Scalar maxHSV2 = Scalar(h_max2, s_max, v_max);
        
        inRange(frameHSV, minHSV1, maxHSV1, lower_mask);
        inRange(frameHSV, minHSV2, maxHSV2, upper_mask);
        
        bitwise_or(upper_mask, lower_mask, mask);

        GaussianBlur(mask, mask, Size(5, 5), 0);

        bitwise_and(frame, frame, resultHSV, mask);

        imshow("Original", frame);
        imshow("Mask", mask);
        if (waitKey(30) == 'q')
            break;
    }
    cap.release(); 
    destroyAllWindows(); 
    return 0;
}