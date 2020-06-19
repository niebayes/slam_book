#ifndef PCL_PCD_IO_UTILS_H_
#define PCL_PCD_IO_UTILS_H_

#include <string>

bool pcd_write_from_xyz(std::string filepath, std::string output_file);

bool pcd_load(std::string filepath);

#endif  // PCL_PCD_IO_UTILS_H_