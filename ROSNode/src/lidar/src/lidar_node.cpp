// ROS Point Cloud DEM Generation
// MacCallister Higgins

#include <cmath>
#include <vector>
#include <float.h>
#include <stdio.h>
#include <math.h>
#include <sstream>
#include <map>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <lidar/lidar_pose.h>

#define IMAGE_HEIGHT	701
#define IMAGE_WIDTH	801
#define BIN		0.100

// choices for min/max point height (z) to consider
#define MIN_Z -2.0    // previous -1.9
#define MAX_Z  0.5    // previous 0.7

// capture car dimensions (for removing it from point cloud)
#define CAPTURE_CAR_FRONT_X 2.0
#define CAPTURE_CAR_REAR_X -1.5
#define CAPTURE_CAR_LEFT_Y 1.0
#define CAPTURE_CAR_RIGHT_Y -1.0

using namespace cv;
using namespace std;

// Global Publishers/Subscribers
ros::Subscriber subPointCloud;
//ros::Publisher pubPointCloud;
ros::Publisher pubImgWithPose;
ros::Subscriber subObjRTK;
ros::Subscriber subCapFRTK;
ros::Subscriber subCapRRTK;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_grid (new pcl::PointCloud<pcl::PointXYZ>);
//sensor_msgs::PointCloud2 output;
lidar::lidar_pose output;
nav_msgs::Odometry objRTK;
nav_msgs::Odometry capFRTK;
nav_msgs::Odometry capRRTK;

double heightArray[IMAGE_HEIGHT][IMAGE_WIDTH];

cv::Mat *heightmap, *cluster_img;
cv_bridge::CvImage cluster_img_bridge;
std::vector<int> compression_params;

int fnameCounter;
double lowest;

ofstream out;

map<long long, bool> cache;

vector<int> q;
vector<int> id(IMAGE_HEIGHT*IMAGE_WIDTH, 0);
vector<vector<int> > comp;

const bool PUBLISH_DATA = true;

// map meters to 0->255
int map_m2i(double z){
  return int(round((z + 3.0)/6.0 * 255));
}

// map meters to index
// returns 0 if not in range, 1 if in range and row/column are set
int map_pc2rc(double x, double y, int *row, int *column){
  // Find x -> row mapping
  *row = (int)round(floor(((((IMAGE_HEIGHT*BIN)/2.0) - x)/(IMAGE_HEIGHT*BIN)) * IMAGE_HEIGHT));
  // obviously can be simplified, but leaving for debug
  // Find y -> column mapping
  *column = (int)round(floor(((((IMAGE_WIDTH*BIN)/2.0) - y)/(IMAGE_WIDTH*BIN)) * IMAGE_WIDTH));
  // Return success
  return 1;
}

// map index to meters
// returns 0 if not in range, 1 if in range and x/y are set
int map_rc2pc(double *x, double *y, int row, int column){
  // Check if falls within range
  if(row >= 0 && row < IMAGE_HEIGHT && column >= 0 && column < IMAGE_WIDTH){
    // Find row -> x mapping
    *x = (double)(BIN*-1.0 * (row - (IMAGE_HEIGHT/2.0)));	// this one is simplified
    // column -> y mapping
    *y = (double)(BIN*-1.0 * (column - (IMAGE_WIDTH/2.0)));
    // Return success
    return 1;
  }
  return 0;
}


// main generation function
void DEM(const sensor_msgs::PointCloud2ConstPtr& pointCloudMsg)
{
    ROS_INFO("Point Cloud Received");

    long long t = (long long)pointCloudMsg->header.stamp.sec*1000000000 + (long long)pointCloudMsg->header.stamp.nsec;

    if (cache.find(t) != cache.end()) {
        //return;
    }

    cache[t] = true;

    ROS_INFO("Point Cloud Computing");
    cout<<t<<endl;

    // Convert from ROS message to PCL point cloud
    pcl::fromROSMsg(*pointCloudMsg, *cloud);

    // get width and height of 2D point cloud data
    int n = cloud->width;

    //cout<<pointCloudMsg->width<<" "<<pointCloudMsg->height<<endl;

    int row, column;

    for (int i=0; i<IMAGE_HEIGHT; i++)
    {
        for (int j=0; j<IMAGE_WIDTH; j++)
        {
            heightArray[i][j] = 0;
        }
    }

    for (int i=0; i<n; i++) {

        map_pc2rc((*cloud)[i].x, (*cloud)[i].y, &row, &column);

        float height = (*cloud)[i].z;

        if (row >=0 && row < IMAGE_HEIGHT && column >= 0 && column < IMAGE_WIDTH) {

            heightArray[row][column] = map_m2i(height);
            //cout<<row<<" "<<column<<" "<<heightArray[row][column]<<endl;

            if (heightArray[row][column] < 0)
            {
                heightArray[row][column] = 0;
            }
        }

    }

    for (int i=330; i<=375; i++) {
        for (int j=388; j<=415; j++) {
            heightArray[i][j] = 0;
        }
    }


    int delta = 45;

    for (int x=0; x<delta; x++) {
        for (int y=0; y<delta; y++) {

            int x1 = (IMAGE_HEIGHT/delta)*x;
            int x2 = (IMAGE_HEIGHT/delta)*(x+1);
            if (x+1 == delta)
                x2 = IMAGE_HEIGHT;

            int y1 = (IMAGE_WIDTH/delta)*y;
            int y2 = (IMAGE_WIDTH/delta)*(y+1);
            if (y+1 == delta)
                y2 = IMAGE_WIDTH;

            int h = 255;
            int H = 0;
            for (int i=x1; i<x2; i++) {
                for (int j=y1; j<y2; j++) {
                    if (heightArray[i][j] > 50) {
                        h = min((int)heightArray[i][j],  h);
                        H = max((int)heightArray[i][j],  H);
                    }
                }
            }

            if (H - h < 4)
            {
                for (int i=x1; i<x2; i++) {
                    for (int j=y1; j<y2; j++) {
                        heightArray[i][j] = 0;
                    }
                }
            }
        }
    }

    delta = 4;

    for (int x=0; x<delta; x++) {
        for (int y=0; y<delta; y++) {

            int x1 = (IMAGE_HEIGHT/delta)*x;
            int x2 = (IMAGE_HEIGHT/delta)*(x+1);
            if (x+1 == delta)
                x2 = IMAGE_HEIGHT;

            int y1 = (IMAGE_WIDTH/delta)*y;
            int y2 = (IMAGE_WIDTH/delta)*(y+1);
            if (y+1 == delta)
                y2 = IMAGE_WIDTH;

            int h = 255;
            int H = 0;
            for (int i=x1; i<x2; i++) {
                for (int j=y1; j<y2; j++) {
                    if (heightArray[i][j] > 50) {
                        h = min((int)heightArray[i][j],  h);
                        H = max((int)heightArray[i][j],  H);
                    }
                }
            }

            //cout<<x1<<" "<<x2<<" "<<y1<<" "<<y2<<" "<<h<<endl;

            for (int i=x1; i<x2; i++) {
                for (int j=y1; j<y2; j++) {
                    if (heightArray[i][j] - h < 2 || heightArray[i][j]>120 || heightArray[i][j]<70)
                    {
                        heightArray[i][j] = 0;
                    }
                }
            }
        }
    }


    q.clear();
    for (int i=0; i<IMAGE_HEIGHT*IMAGE_WIDTH; i++) {

        int x = i/IMAGE_WIDTH;
        int y = i%IMAGE_WIDTH;

        if (heightArray[x][y] != 0) q.push_back(i);
    }
    for (int i=0; i<IMAGE_HEIGHT*IMAGE_WIDTH; i++) id[i] = 0;

    int dx[] = {1, 0, -1, 0, 1, 1, -1, -1};
    int dy[] = {0, 1, 0, -1, 1, -1, 1, -1};
    int d = 20;

    int cnt = 0;
    while (!q.empty()) {

        //cout<<q.size()<<endl;

        int p = q.back();
        q.pop_back();

        if (id[p] == 0) {
            id[p] = ++cnt;
        }

        int x = p / IMAGE_WIDTH;
        int y = p%IMAGE_WIDTH;

        //cout<<x<<" "<<y<<" "<<q.size()<<endl;

        for (int k=0; k<8; k++) {
            for (int t=1; t<=d; t++) {

                int xx = x + dx[k]*t;
                int yy = y + dy[k]*t;

                if (abs(dx[k]*t) + abs(dy[k]*t) > d || xx<0 ||
                        xx>=IMAGE_HEIGHT || yy<0 || yy>=IMAGE_WIDTH) {
                    break;
                }

                int pp = xx*IMAGE_WIDTH + yy;

                if (id[pp] != 0)
                    break;

                if (id[pp] == 0 && heightArray[xx][yy] != 0) {
                    id[pp] = id[p];
                    q.push_back(pp);
                }
            }
        }

    }

    comp.clear();
    comp.resize(cnt+1);

    for (int i=0; i<IMAGE_HEIGHT*IMAGE_WIDTH; i++) {
        if (id[i] > 0)
            comp[id[i]-1].push_back(i);
    }

    out << t << endl;
    vector<pair<pair<int,int>, pair<int,int> > > res;

    for (int i=0; i<cnt; i++) {

        int lx = 100000;
        int hx = 0;
        int ly = 100000;
        int hy = 0;

        for (int j=0; j<comp[i].size(); j++) {

            int x = comp[i][j]/IMAGE_WIDTH;
            int y = comp[i][j]%IMAGE_WIDTH;

            lx = min(lx, x);
            hx = max(hx, x);
            ly = min(ly, y);
            hy = max(hy, y);

        }

        if ((hx-lx > 200 || hy-ly > 100) || (hx-lx < 10 || hy-ly < 10)) {
            for (int j=0; j<comp[i].size(); j++) {

                int x = comp[i][j]/IMAGE_WIDTH;
                int y = comp[i][j]%IMAGE_WIDTH;

                heightArray[x][y] = 0;
            }
        }
        else {
            res.push_back(make_pair(make_pair(lx, hx), make_pair(ly, hy)));
            //out<<lx<<" "<<hx<<" "<<ly<<" "<<hy<<endl;
        }

    }

    out<<res.size()<<endl;

    output.xl.clear();
    output.xh.clear();
    output.yl.clear();
    output.yh.clear();
    output.timestamp = t;
    output.header.stamp = ros::Time::now();

    for (int i=0; i<res.size(); i++) {
        out<<res[i].first.first<<" "<<res[i].first.second<<" "
                <<res[i].second.first<<" "<<res[i].second.second<<endl;

        if (PUBLISH_DATA)
        {
            output.xl.push_back(res[i].first.first);
            output.xh.push_back(res[i].first.second);
            output.yl.push_back(res[i].second.first);
            output.yh.push_back(res[i].second.second);

            //cout<<i<<" "<<output.xl<<" "<<output.xh<<" "<<output.yl<<" "<<output.yh<<endl;

            //pubImgWithPose.publish(output);
        }
    }

    pubImgWithPose.publish(output);

    out.flush();
}

void createAlphaMat(Mat &mat)
{
    CV_Assert(mat.channels() == 4);
    for (int i = 0; i < mat.rows; ++i) {
        for (int j = 0; j < mat.cols; ++j) {
            Vec4b& bgra = mat.at<Vec4b>(i, j);
            bgra[0] = UCHAR_MAX; // Blue
            bgra[1] = saturate_cast<uchar>((float (mat.cols - j)) / ((float)mat.cols) * UCHAR_MAX); // Green
            bgra[2] = saturate_cast<uchar>((float (mat.rows - i)) / ((float)mat.rows) * UCHAR_MAX); // Red
            bgra[3] = saturate_cast<uchar>(0.5 * (bgra[1] + bgra[2])); // Alpha
        }
    }
}

int main(int argc, char** argv)
{
  ROS_INFO("Starting LIDAR Node");
  ros::init(argc, argv, "lidar_node");
  ros::NodeHandle nh;

  // Setup output cloud
  /*
  cloud_grid->width  = IMAGE_WIDTH;
  cloud_grid->height = IMAGE_HEIGHT;
  cloud_grid->points.resize (cloud_grid->width * cloud_grid->height);
  */

  // Setup images
  cv::Mat map(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));
  heightmap = &map;

  cv::Mat map2(64, 64, CV_8UC1, cv::Scalar(0));
  cluster_img = &map2;

  // init heightmap image display
  /*
  cvNamedWindow("Height Map", CV_WINDOW_AUTOSIZE);
  cvStartWindowThread();
  cv::imshow("Height Map", *heightmap);
  */

  // Setup Image Output Parameters
  fnameCounter = 0;
  lowest = FLT_MAX;
  compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
  compression_params.push_back(9);

  // Setup indicies in point clouds
/*
  int index = 0;
  ROS_INFO("x");
  double x, y;
  for(int i = 0; i < IMAGE_HEIGHT; ++i){
    for(int j = 0; j < IMAGE_WIDTH; ++j){
      index = i * j;
      (void)map_rc2pc(&x, &y, i, j);
      cloud_grid->points[index].x = x;
      cloud_grid->points[index].y = y;
      cloud_grid->points[index].z = (-FLT_MAX);
      // Temp storage
      heightArray[i][j] = (-FLT_MAX);
      }
    }
*/

	out.open("/home/alex/lidar_ford02_v4.txt");

    ROS_INFO("Starting callback");
    subPointCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 2, DEM);

    if (PUBLISH_DATA)
        pubImgWithPose = nh.advertise<lidar::lidar_pose> ("/lidar/lidar_pose", 1);
  //subObjRTK = nh.subscribe<nav_msgs::Odometry>("/objects/obs1/rear/gps/rtkfix", 2, ObjRTKRecd);
  //subCapFRTK = nh.subscribe<nav_msgs::Odometry>("/objects/capture_vehicle/front/gps/rtkfix", 2, CapFRTKRecd);
  //subCapRRTK = nh.subscribe<nav_msgs::Odometry>("/objects/capture_vehicle/rear/gps/rtkfix", 2, CapRRTKRecd);

  ros::spin();

  return 0;
}
