#include "pcd_io_utils.h"

#include <stdio.h>

#include <fstream>
#include <iostream>
#include <string>

#include "pcl/io/pcd_io.h"    // pcl::io::savePCDFile
#include "pcl/point_types.h"  // pcl::PointXYZ

bool pcd_write(std::string filepath, std::string output_file) {
  // @return true if no error and false otherwise
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // Read data from .xyz file
  std::ifstream fin(filepath, std::ios::in);
  if (!fin.is_open()) return false;
  std::size_t n_lines = 0;
  std::string buffer;
  // ! char is specified with single quotes whilst string is specified with
  // ! double quotes
  while (std::getline(fin, buffer, '\n')) {
    std::istringstream iss(buffer);
    double x, y, z;
    if (iss >> x >> y >> z) {
      // Fill in the cloud data
      cloud.points.emplace_back(x, y, z);
      n_lines++;
    }
  }
  cloud.width = n_lines;
  cloud.height = 1;
  // ? Specific to vscode?
  // * '\n' and "\n" will also additionally flush the buffer
  pcl::io::savePCDFile("data/3dv_tutorial/pcd/" + output_file + ".pcd", cloud);
  std::cerr << "Saved " << cloud.size() << " points in " << output_file
            << ".pcd" << '\n';

  // * Zero is interpreted as false and anything non-zero is interpreted as true
  return true;
}

bool pcd_load(std::string filepath) {
  // * Use pointer when handling with files by convention
  // * Ptr and ConstPtr are alias of the respective shared_ptrs
  // * Here, we cannot use ConstPtr, because we will convert the cloud_blob to
  // * cloud later on.
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PCLPointCloud2 cloud_blob;
  // * template-free loadPCDFile version for binary file loading.
  if (pcl::io::loadPCDFile(filepath, cloud_blob) == -1) {
    // * Macros cannot be placed within namespaces
    // * Hence no need to denote it with scope resolution operator ::
    PCL_ERROR("Couldn't read the pcd file\n");
    return false;
  }
  pcl::fromPCLPointCloud2(cloud_blob, *cloud);
  for (std::size_t i = 0; i < cloud->points.size(); ++i) {
    // std::cout << cloud->points[i].x << " " << cloud->points[i].y << " "
    //           << cloud->points[i].z << '\n';
    printf("%6.1f %6.1f %6.1f \n", cloud->points[i].x, cloud->points[i].y,
           cloud->points[i].z);
  }
  std::cout << "Loaded " << cloud->points.size() << " points" << '\n';
  return true;
}

void test() {
  // pcd_write test
  std::string filepath("data/3dv_tutorial/box.xyz");
  std::cerr << std::boolalpha << pcd_write(filepath, "box") << '\n';

  // pcd_load test
  std::cerr << std::boolalpha << pcd_load("data/3dv_tutorial/pcd/box.pcd")
            << '\n';
}

int main(int argc, char** argv) {
  // test();
  return 0;
}