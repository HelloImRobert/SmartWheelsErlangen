#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <vector>


#include "swe_cboundary.h"
#include "filter.h"

using namespace std;


int main()
{
    /*
    cv::Point2d o1(0,2), p1(3,-1);
    cv::Point2d Pis;

    SWE_cBoundary b1;
    b1.boundary.push_back(o1);
    b1.boundary.push_back(p1);

    cv::Point2d refPoint(0,0);

    bool isInter = b1.getIntersection(CV_PI/4*0,0,Pis);


    cout << isInter << endl << Pis.x << endl << Pis.y << endl;

    double distance;
    int direction;

    bool hasDist = b1.getPerpenticDist(refPoint, distance, direction);

    cout << endl << hasDist << endl << direction << endl << distance << endl;

    vector<int> dudl(4,200);
    vector<int> dudl1111(4,100);

    dudl[3] = 1;


    Filter filter;
    SWE_cBoundary b2;
    b2.boundary.push_back(refPoint);
    b2.boundary.push_back(p1);
    //filter.boundaries.push_back(b1);
    //filter.boundaries.push_back(b2);
    vector<SWE_cBoundary> bounds;
    bounds.push_back(b1);
    bounds.push_back(b2);

    cout << bounds[0].boundary[0].x << endl << bounds[0].boundary[0].y << endl;
    cout << bounds[0].boundary[1].x << endl << bounds[0].boundary[1].y << endl;
    cout << bounds[1].boundary[0].x << endl << bounds[1].boundary[0].y << endl;
    cout << bounds[1].boundary[1].x << endl << bounds[1].boundary[1].y << endl;

    filter.setBoundaries(bounds);

    std::vector<cv::Point2d> intersecPoints;
    int isChosen = 0;
    int hasTwoIntersecs = 0;

    filter.intersecPointCalc(intersecPoints, isChosen, hasTwoIntersecs);
*/

    Filter filter;
    filter.doSth();

    return 0;
}
