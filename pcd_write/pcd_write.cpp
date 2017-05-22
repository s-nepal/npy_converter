#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>


#include <string>
#include <fstream>
#include <vector>

using namespace std;

int user_data;

int main (int argc, char** argv)
{
  //pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  
  // Fill in the cloud data
  //cloud.width    = 5;
  
  cloud->width = 42000;
  const size_t len = 42000;
  cloud->height   = 1;
  cloud->is_dense = false;
  cloud->points.resize (cloud->width * cloud->height);

  /*string a = "-1.5";
  cout << stod(a) + 1 << endl;
*/

  ifstream file_x("C:\\Users\\shitoshna.nepal\\Desktop\\x.txt");
  string str_x;
  size_t ctr_x = 0;
  double x[len];
  while(getline(file_x,str_x))
  {
	  x[ctr_x] = stod(str_x);
	  //cout << x[ctr_x] << endl;
	  ctr_x++;
  }


  ifstream file_y("C:\\Users\\shitoshna.nepal\\Desktop\\y.txt");
  string str_y;
  size_t ctr_y = 0;
  double y[len];
  while(getline(file_y,str_y))
  {
	  y[ctr_y] = stod(str_y);
	  //cout << y[ctr_y] << endl;
	  ctr_y++;
  }


  ifstream file_z("C:\\Users\\shitoshna.nepal\\Desktop\\z.txt");
  string str_z;
  size_t ctr_z = 0;
  double z[len];
  while(getline(file_z,str_z))
  {
	  z[ctr_z] = stod(str_z);
	  //cout << z[ctr_z] << endl;
	  ctr_z++;
  }

	
  //for (size_t i = 0; i < cloud.points.size (); ++i)
  for (size_t i = 0; i < len; ++i)
  {
	  cloud->points[i].x = x[i];
	  cloud->points[i].y = y[i];
	  cloud->points[i].z = z[i];
  }

  
  cout << "Reached Here" << endl;
  pcl::io::savePCDFileASCII ("C:\\Users\\shitoshna.nepal\\Desktop\\my_point_cloud.pcd", *cloud);
  std::cerr << "Saved " << cloud->points.size () << " data points to test_pcd.pcd." << std::endl;

 

   return (0);
}