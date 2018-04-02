#include <cstdint>
#include <cmath>
#include <limits>

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

#include <boost/shared_ptr.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pyrosmsg/converters.hpp>

#include "typedefs.h"

namespace pypcl_ros {

namespace py = pybind11;


PCXYZ::Ptr pc2_to_pcxyz(const sensor_msgs::PointCloud2& msg) {
  using Pc2CItr = sensor_msgs::PointCloud2ConstIterator<float>;
  Pc2CItr itr_x(msg, "x"), itr_y(msg, "y"), itr_z(msg, "z");
  size_t n_pts = msg.width * msg.height;
  PCXYZ::Ptr pc(new PCXYZ());
  pc->reserve(n_pts);
  for (size_t i=0; i < n_pts; ++i) {
    pcl::PointXYZ p;
    p.x = *itr_x;
    p.y = *itr_y;
    p.z = *itr_z;
    pc->push_back(p);
    ++itr_x;
    ++itr_y;
    ++itr_z;
  }
  return pc;
}

ndarray2f pc2_to_xyz_ndarray(const sensor_msgs::PointCloud2& pc2,
                             bool skip_nan) {
  int valid_pts = 0;
  if (skip_nan) {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(pc2, "x");
    while (iter_x != iter_x.end()) {
      float x = iter_x[0];
      float y = iter_x[1];
      float z = iter_x[2];
      if (std::isnan(x)||std::isnan(y)||std::isnan(z)) {
        // pass
      } else {
        ++valid_pts;
      }
      ++iter_x;
    }
  } else {
    // all points are valid
    valid_pts = pc2.width * pc2.height;
  }

  ndarray2f xyz({valid_pts, 3});
  auto xyz_buf = xyz.mutable_unchecked();
  int out_ix = 0;
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(pc2, "x");
  for (int n=0; n < valid_pts; ++n) {
    float x = iter_x[0];
    float y = iter_x[1];
    float z = iter_x[2];
    if (skip_nan && (std::isnan(x)||std::isnan(y)||std::isnan(z))) {
      ++iter_x;
      continue;
    }
    xyz_buf(out_ix, 0) = x;
    xyz_buf(out_ix, 1) = y;
    xyz_buf(out_ix, 2) = z;
    ++out_ix;
    ++iter_x;
  }
  return xyz;
}

sensor_msgs::PointCloud2 ndarray_to_pc2(const ndarray2f& arr,
                                        const std::string& frame_id) {

  if (arr.shape(1) != 3) {
    throw std::runtime_error("only Nx3 arrays supported for now.");
  }
  size_t n_pts = arr.shape(0);
  sensor_msgs::PointCloud2 msg;
  msg.height = 1;
  msg.width = n_pts;
  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2Fields(3,
                                "x", 1, sensor_msgs::PointField::FLOAT32,
                                "y", 1, sensor_msgs::PointField::FLOAT32,
                                "z", 1, sensor_msgs::PointField::FLOAT32);
  modifier.reserve(n_pts);
  //modifier.resize(n_pts);

  using Pc2Itr = sensor_msgs::PointCloud2Iterator<float>;
  Pc2Itr itr_x(msg, "x"), itr_y(msg, "y"), itr_z(msg, "z");
  auto arr_buf = arr.unchecked();
  for (size_t i=0; i < n_pts; ++i) {
    *itr_x = arr_buf(i, 0);
    *itr_y = arr_buf(i, 1);
    *itr_z = arr_buf(i, 2);
    ++itr_x;
    ++itr_y;
    ++itr_z;
  }
  msg.header.frame_id = frame_id;
  return msg;
}

sensor_msgs::PointCloud2 xyz_img_to_pc2(const py::array_t<float, 3>& xyz_img,
                                        bool skip_nan,
                                        const std::string& frame_id) {

  PCXYZ pc;
  auto xyz_img_buf = xyz_img.unchecked();
  for (size_t v = 0; v < xyz_img.shape(0); ++v) {
    for (size_t u = 0; u < xyz_img.shape(1); ++u) {
      float x = xyz_img_buf(v, u, 0);
      float y = xyz_img_buf(v, u, 1);
      float z = xyz_img_buf(v, u, 2);
      if (skip_nan && (std::isnan(x)||std::isnan(y)||std::isnan(z))) {
        continue;
      }
      pcl::PointXYZ p;
      p.x = x;
      p.y = y;
      p.z = z;
      pc.push_back(p);
    }
  }
  pc.width = pc.size();
  pc.height = 1;

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(pc, msg);
  msg.header.frame_id = frame_id;
  return msg;
}

sensor_msgs::PointCloud2 xyz_img_to_organized_pc2(const ndarray3f& xyz_img,
                                                  const std::string& frame_id) {

  PCXYZ pc;
  auto xyz_img_buf = xyz_img.unchecked();
  for (size_t v = 0; v < xyz_img.shape(0); ++v) {
    for (size_t u = 0; u < xyz_img.shape(1); ++u) {
      pcl::PointXYZ p;
      p.x = xyz_img_buf(v, u, 0);
      p.y = xyz_img_buf(v, u, 1);
      p.z = xyz_img_buf(v, u, 2);
      pc.push_back(p);
    }
  }
  pc.width = xyz_img.shape(1);
  pc.height = xyz_img.shape(0);

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(pc, msg);
  msg.header.frame_id = frame_id;
  return msg;
}

sensor_msgs::PointCloud2 depth_img_to_pc2(py::array_t<float, 2> depth_img,
                                          float cx,
                                          float cy,
                                          float fx,
                                          float fy,
                                          float max_valid_depth,
                                          const std::string& frame_id) {
  auto depth_buf = depth_img.unchecked();
  PCXYZ cloud;
  for (size_t v=0; v < depth_img.shape(0); ++v) {
    for (size_t u=0; u < depth_img.shape(1); ++u) {
      float d = depth_buf(v, u);
      if (d == 0.0 || std::isnan(d) || d > max_valid_depth) {
        continue;
      }
      pcl::PointXYZ p;
      p.x = (u-cx)*d/fx;
      p.y = (v-cy)*d/fy;
      p.z = d;
      cloud.push_back(p);
    }
  }
  cloud.width = cloud.size();
  cloud.height = 1;

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = frame_id;
  return msg;
}

sensor_msgs::PointCloud2 pclpc2_to_pc2(const PCLPC2::Ptr pclpc2,
                                       const std::string& frame_id) {
  // sending to python will trigger copy anyway, so move should be safe
  sensor_msgs::PointCloud2 pc2;
  // or could do toROSMsg?
  pcl_conversions::moveFromPCL(*pclpc2, pc2);
  pc2.header.frame_id = frame_id;
  return pc2;
}


template<class PointCloudT>
sensor_msgs::PointCloud2 pclpc_to_pc2(const typename PointCloudT::Ptr pc,
                                      const std::string& frame_id) {
  // there's no move for pointcloudT
  sensor_msgs::PointCloud2 pc2;
  pcl::toROSMsg(*pc, pc2);
  pc2.header.frame_id = frame_id;
  return pc2;
}



void export_converters(py::module& m) {
  m.def("xyz_img_to_pc2",
        &xyz_img_to_pc2,
        py::arg("xyz_img"),
        py::arg("skip_nan") = true,
        py::arg("frame_id") = "");
  m.def("xyz_img_to_organized_pc2",
        &xyz_img_to_organized_pc2,
        py::arg("xyz_img"),
        py::arg("frame_id") = "");
  m.def("pc2_to_pcxyz",
        &pc2_to_pcxyz,
        py::arg("msg"));
  m.def("ndarray_to_pc2",
        &ndarray_to_pc2,
        py::arg("arr"),
        py::arg("frame_id")="");
  m.def("pcxyz_to_pc2",
        &pclpc_to_pc2<PCXYZ>,
        py::arg("pc"),
        py::arg("frame_id")="");
  m.def("pclpc2_to_pc2",
        &pclpc2_to_pc2,
        py::arg("pclpc2"),
        py::arg("frame_id")="");
}

}
