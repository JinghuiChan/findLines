#include <iostream>
#include <cmath>
#include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
using namespace std;
using namespace cv;
using namespace cv::ximgproc;

#define CX 156.712112
#define CY 130.649704
#define PI 3.1415926
#define distBoarder 10
string IMAGEPATH = "/home/chan/Documents/dataset/findDoors/undistor/img/Wall_Line_DEBUG_undist07.bin.png";
int width = 0;
int height = 0;

static int temp = 0;

typedef struct
{
    float x;
    float y;
}AM_xy;
typedef struct
{
    float angle;
    float radius;
}normal_line_t;

inline double pi_to_pi(double angle)
{
    angle = fmod(angle, 2 * M_PI);
    if (angle >= M_PI)
        angle -= 2 * M_PI;
    return angle;
}

normal_line_t calculate_normal_line(AM_xy p1,AM_xy p2)
{
    normal_line_t line;
    double slope;
    if (fabs(p2.x - p1.x) > 1e-9)
    {
        slope = (p2.y - p1.y) / (p2.x - p1.x);
        line.angle = pi_to_pi(atan(slope) + M_PI/2);
    }
    else
    {
        line.angle = 0.0;
    }

    line.radius = p1.x * cos(line.angle) + p1.y * sin(line.angle);
    if (line.radius < 0)
    {
        line.radius = -line.radius;
        line.angle = pi_to_pi(line.angle + M_PI);
    }
    return line;
}

float dist_point_to_line(normal_line_t line,AM_xy point)
{
    float p_rad = sqrt(pow(point.x, 2) + pow(point.y, 2));
    float p_ang = atan2(point.y, point.x);
    return fabs(p_rad * cos(p_ang - line.angle) - line.radius);
}

int showTempResult(vector<Vec4f> lines){
    Mat image = imread(IMAGEPATH, IMREAD_GRAYSCALE);
    string windowNamee = "image" + to_string(temp++);

    for(int i = 0;i < lines.size(); i++){
        printf("%s[(%f,%f),(%f,%f)]\n", windowNamee.c_str(), lines.at(i)[0], lines.at(i)[1], lines.at(i)[2], lines.at(i)[3]);
        line( image, Point(lines.at(i)[0], lines.at(i)[1]), Point(lines.at(i)[2], lines.at(i)[3]), Scalar(0,0,255), 3, LINE_AA);
    }
    imshow(windowNamee, image);
}

bool hasSamePoint(Vec4f one, Vec4f two){
    int gap = 400;
    double dist1 = (one[0] - two[0]) * (one[0] - two[0]) + (one[1] - two[1]) * (one[1] - two[1]);
    double dist2 = (one[0] - two[2]) * (one[0] - two[2]) + (one[1] - two[3]) * (one[1] - two[3]);
    double dist3 = (one[2] - two[0]) * (one[2] - two[0]) + (one[3] - two[1]) * (one[3] - two[1]);
    double dist4 = (one[2] - two[2]) * (one[2] - two[2]) + (one[3] - two[3]) * (one[3] - two[3]);

    if(dist1 <= gap || dist2 <= gap || dist3 <= gap || dist4 <= gap){

        return 1;
    }
    return 0;
}

bool isVertical(Vec4f twoPoints){
    Point2f pt1(twoPoints[0], twoPoints[1]);
    Point2f pt2(twoPoints[2], twoPoints[3]);
    Point2f pt3(CX, CY);
    float lineP1P2 = sqrt((pt1.x - pt2.x) * (pt1.x - pt2.x) + (pt1.y - pt2.y) * (pt1.y - pt2.y));
    float lineP1P3 = sqrt((pt1.x - pt3.x) * (pt1.x - pt3.x) + (pt1.y - pt3.y) * (pt1.y - pt3.y));
    float lineP2P3 = sqrt((pt2.x - pt3.x) * (pt2.x - pt3.x) + (pt2.y - pt3.y) * (pt2.y - pt3.y));
    double anglep1p3 = acos((lineP1P2 * lineP1P2 + lineP1P3 * lineP1P3 - lineP2P3 * lineP2P3) / (2 * lineP1P2 * lineP1P3)) * 180.0 / PI;

    printf("angle  p1p3 = %f \n", anglep1p3);
    if(anglep1p3 <= 10){
        return 1;
    }
    return 0;
}

bool isVerticalLines(Vec4f twoPoints){
    float distt = 10;
    AM_xy pp1, pp2, ppc;
    pp1.x = twoPoints[0];
    pp1.y = twoPoints[1];
    pp2.x = twoPoints[2];
    pp2.y = twoPoints[3];
    ppc.x = CX;
    ppc.y = CY;

    normal_line_t lineee = calculate_normal_line(pp1, pp2);
    float dist = dist_point_to_line(lineee, ppc);

    if(dist <= distt){
        return 1;
    }

    return 0;
    /*
    int angle = 10;
    Point2f pt1(twoPoints[0], twoPoints[1]);
    Point2f pt2(twoPoints[2], twoPoints[3]);
    Point2f pt3(CX, CY);
    float lineP1P2 = sqrt((pt1.x - pt2.x) * (pt1.x - pt2.x) + (pt1.y - pt2.y) * (pt1.y - pt2.y));
    float lineP1P3 = sqrt((pt1.x - pt3.x) * (pt1.x - pt3.x) + (pt1.y - pt3.y) * (pt1.y - pt3.y));
    float lineP2P3 = sqrt((pt2.x - pt3.x) * (pt2.x - pt3.x) + (pt2.y - pt3.y) * (pt2.y - pt3.y));
    double anglep1p3 = acos((lineP1P2 * lineP1P2 + lineP1P3 * lineP1P3 - lineP2P3 * lineP2P3) / (2 * lineP1P2 * lineP1P3)) * 180.0 / PI;

    //printf("angle  p1p3 = %f \n", anglep1p3);
    if(anglep1p3 <= angle){
        return 1;
    }
    return 0;*/

}

bool isNearBoard(Vec4f twoPoints){
    if(twoPoints[0] <= distBoarder || twoPoints[0] >= (width - distBoarder) ||
    twoPoints[1] <= distBoarder || twoPoints[1] >= (height - distBoarder) ||
    twoPoints[2] <= distBoarder || twoPoints[2] >= (width - distBoarder)   ||
    twoPoints[3] <= distBoarder || twoPoints[3] >= (height - distBoarder)){
        return 1;
    }
    return 0;
}

int FillterVerticalLines(vector<Vec4f> lines, vector<Vec4f> *v_lines, vector<Vec4f> *h_lines){
    for (int i = 0; i < lines.size(); ++i) {
        if(isVerticalLines(lines.at(i))){
            v_lines->push_back(lines.at(i));
        }else{
            h_lines->push_back(lines.at(i));
        }
    }

    printf("linesP:%d, v_lines:%d, h_lines:%d \n", lines.size(), v_lines->size(), h_lines->size());
    return 1;
}

vector<Vec4i> FillterConnectedLines(vector<Vec4i> *lines){

    vector<Vec4i> fillterLines;
    for (vector<Vec4i>::iterator it=lines->begin(); it!=lines->end(); ++it) {
        if(isVerticalLines(*it)){
            fillterLines.push_back(*it);
            lines->erase(it);
        }
    }

    return fillterLines;
}

vector<vector<Vec4f>> fillterDoors(vector<Vec4f> v_lines, vector<Vec4f> h_lines){
    vector<Vec4f> result_lines;
    vector<vector<Vec4f>> doors;
    int doors_index = 0;
    vector<int> temp;

    for (int i = 0; i < h_lines.size(); ++i) {
        vector<Vec4f> temp_door;
        for (int j = 0; j < v_lines.size(); ++j) {
            if(hasSamePoint(h_lines.at(i), v_lines.at(j))){
                //if(temp_door.size()>=2)
                    //break;
                temp_door.push_back(v_lines.at(j));
            }
        }

        if(temp_door.size() >= 2){
            printf("temp door : %d \n", temp_door.size());
            result_lines.push_back(h_lines.at(i));
            for (int j = 0; j < temp_door.size(); ++j) {
                result_lines.push_back(temp_door.at(j));
            }
            showTempResult(result_lines);
            doors.push_back(result_lines);

            doors_index++;
        }
        result_lines.clear();
        temp_door.clear();
    }

    return doors;
}

vector<Vec4f> fillterVerticalLines(vector<Vec4f> lines){
    vector<Vec4f> fillterLines;
    for (int i = 0; i < lines.size(); ++i) {
        if(isVertical(lines.at(i))){
            fillterLines.push_back(lines.at(i));
        }
    }

    return fillterLines;
}

vector<Vec4f> fillterOverBoard(vector<Vec4f> lines){
    vector<Vec4f> fillterLines;
    for (int i = 0; i < lines.size(); ++i) {
        if(isNearBoard(lines.at(i))){
            fillterLines.push_back(lines.at(i));
        }
    }
    return fillterLines;
}

void DrawLines(string windowName, vector<Vec4f> lines, Mat img){
    if(lines.empty()){
        printf("window:%s  empty \n", windowName.c_str());
        waitKey(-1);
    } else{
        // Draw the lines
        for( size_t i = 0; i < lines.size(); i++ )
        {
            Vec4i l = lines[i];
            //line( cdstP, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, LINE_AA);
            line( img, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, LINE_AA);
        }
        imshow(windowName, img);
    }
}

void DrawDoors(string windowName, vector<vector<Vec4f>> doors, Mat img){
    if(doors.empty()){
        printf("windows:%s empty\n", windowName.c_str());
        waitKey(-1);
    } else{
        // Draw the lines
        for( size_t i = 0; i < doors.size(); i++ )
        {
            vector<Vec4f> l = doors[i];
            //vector<Vec4f> l = doors[0];
            if(l.size()>3){
                //continue;
            }
            printf("l:%d\n", l.size());
            cv::RNG rng(time(0));

            for (int j = 0; j < l.size(); ++j) {
                Vec4f ll = l[j];
                //line( img, Point(ll[0], ll[1]), Point(ll[2], ll[3]), cv::Scalar(rng.uniform(0,255)/255,rng.uniform(0,255)/255,rng.uniform(0,255)/255), 3, LINE_AA);
                line( img, Point(ll[0], ll[1]), Point(ll[2], ll[3]), cv::Scalar(1, 0, 0), 3, 0);
            }

        }
        imshow(windowName, img);
    }
}


int main(int argc, char** argv)
{

    Mat image = imread(IMAGEPATH, IMREAD_GRAYSCALE);
    if( image.empty() )
    {
        printf("image load error! \n");
        return -1;
    }

    width = image.cols;
    height = image.rows;

    //--------------------------
    // Declare the output variables
    Mat image_copy = image.clone();
    Mat image_copy2 = image.clone();
    Mat image_copy_p = image.clone();
    Mat image_all_lines = image.clone();

    Mat dst;

    // Edge detection
    //Canny(image, dst, 50, 200, 3);
    Canny(image, dst, 50, 800, 5);

    imshow("canny", dst);

    // Probabilistic Line Transform
    vector<Vec4f> linesP; // will hold the results of the detection
    HoughLinesP(dst, linesP, 1, CV_PI/180, 50, 50, 10 ); // runs the actual detection
    DrawLines("all lines", linesP, image_all_lines);
    printf("linesP size = %d \n", linesP.size());

    //fillter Vertical lines
    vector<Vec4f> v_lines, h_lines;
    FillterVerticalLines(linesP, &v_lines, &h_lines);
    DrawLines("vertical lines", v_lines, image_copy_p);

    //fillter doors
    vector<vector<Vec4f>> doors = fillterDoors(v_lines, h_lines);
    DrawDoors("doors", doors, image_copy_p);

    waitKey(-1);

    return 0;
}
