#include <cmath>
#include <cstdint>
#include <limits>

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>

#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

#include <boost/shared_ptr.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <pyrosmsg/converters.hpp>

#include "typedefs.h"

namespace pypcl_ros {

namespace py = pybind11;

/**
 * Extracts float xyz to a PC2 message. xyz are not necessarily
 * contiguous. Will ignore any other pointfields, if present.
 */
PCXYZ::Ptr pc2_to_pcxyz(const sensor_msgs::PointCloud2& msg) {
  // TODO assuming float
  using Pc2CItr = sensor_msgs::PointCloud2ConstIterator<float>;
  Pc2CItr itr_x(msg, "x"), itr_y(msg, "y"), itr_z(msg, "z");
  size_t n_pts = msg.width * msg.height;
  PCXYZ::Ptr pc(new PCXYZ());
  pc->reserve(n_pts);
  for (size_t i = 0; i < n_pts; ++i) {
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

/**
 * Extracts float xyz and packed float RGB to a PC2 message. xyzrgb are not necessarily
 * contiguous. Will ignore any other pointfields, if present.
 */
PCXYZRGB::Ptr pc2_to_pcxyzrgb(const sensor_msgs::PointCloud2& msg) {
  // TODO assuming float
  using Pc2CItr = sensor_msgs::PointCloud2ConstIterator<float>;
  Pc2CItr itr_x(msg, "x"), itr_y(msg, "y"), itr_z(msg, "z"),
      itr_rgb(msg, "rgb");
  size_t n_pts = msg.width * msg.height;
  PCXYZRGB::Ptr pc(new PCXYZRGB());
  pc->reserve(n_pts);
  for (size_t i = 0; i < n_pts; ++i) {
    pcl::PointXYZRGB p;
    p.x = *itr_x;
    p.y = *itr_y;
    p.z = *itr_z;
    p.rgb = *itr_rgb;
    pc->push_back(p);
    ++itr_x;
    ++itr_y;
    ++itr_z;
    ++itr_rgb;
  }
  return pc;
}

/**
 * Create float Nx3 ndarray from PC2. Optionally skip NaN points.
 * Assumes xyz fields are contiguous.
 */
ndarray2f pc2_to_xyz_ndarray(const sensor_msgs::PointCloud2& pc2,
                             bool skip_nan) {
  // TODO optimize this
  size_t valid_pts = 0;
  if (skip_nan) {
    sensor_msgs::PointCloud2ConstIterator<float> iter_xyz(pc2, "x");
    while (iter_xyz != iter_xyz.end()) {
      float x = iter_xyz[0];
      float y = iter_xyz[1];
      float z = iter_xyz[2];
      if (std::isnan(x) || std::isnan(y) || std::isnan(z)) {
        // pass
      } else {
        ++valid_pts;
      }
      ++iter_xyz;
    }
  } else {
    // all points are valid
    valid_pts = pc2.width * pc2.height;
  }

  ndarray2f xyz({valid_pts, (size_t) 3});
  auto xyz_buf = xyz.mutable_unchecked();
  size_t out_ix = 0;
  sensor_msgs::PointCloud2ConstIterator<float> iter_xyz(pc2, "x");
  for (size_t n = 0; n < valid_pts; ++n) {
    float x = iter_xyz[0];
    float y = iter_xyz[1];
    float z = iter_xyz[2];
    if (skip_nan && (std::isnan(x) || std::isnan(y) || std::isnan(z))) {
      ++iter_xyz;
      continue;
    }
    xyz_buf(out_ix, 0) = x;
    xyz_buf(out_ix, 1) = y;
    xyz_buf(out_ix, 2) = z;
    ++out_ix;
    ++iter_xyz;
  }
  return xyz;
}

/**
 * Create PC2 from float Nx3 ndarray.
 */
sensor_msgs::PointCloud2 xyz_to_pc2(const ndarray2f& arr,
                                    const std::string& frame_id) {
  if (arr.shape(1) != 3) {
    throw std::runtime_error("only Nx3 arrays supported");
  }
  size_t n_pts = arr.shape(0);
  sensor_msgs::PointCloud2 msg;
  msg.height = 1;
  msg.width = n_pts;
  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2Fields(3,
                                "x",
                                1,
                                sensor_msgs::PointField::FLOAT32,
                                "y",
                                1,
                                sensor_msgs::PointField::FLOAT32,
                                "z",
                                1,
                                sensor_msgs::PointField::FLOAT32);
  modifier.reserve(n_pts);
  // modifier.resize(n_pts);

  using Pc2Itr = sensor_msgs::PointCloud2Iterator<float>;
  Pc2Itr itr_x(msg, "x");
  auto arr_buf = arr.unchecked();
  for (size_t i = 0; i < n_pts; ++i) {
    itr_x[0] = arr_buf(i, 0);
    itr_x[1] = arr_buf(i, 1);
    itr_x[2] = arr_buf(i, 2);
    ++itr_x;
  }
  msg.header.frame_id = frame_id;
  return msg;
}

/**
 * Create PC2 (packed RGB format) from Nx6 float (x,y,z,r,g,b) ndarray.
 * if scale_to_255:  r, g, b in the ndarray are assumed to be in (0, 1.) and scaled by 255.
 * if not:  r, g, b in the ndarray are assumed to be in (0, 255).
 */
sensor_msgs::PointCloud2 xyzrgb_to_pc2(const ndarray2f& arr,
                                       bool scale_to_255,
                                       const std::string& frame_id) {
  if (arr.shape(1) != 6) {
    throw std::runtime_error("only Nx6 arrays supported for now.");
  }
  size_t n_pts = arr.shape(0);
  sensor_msgs::PointCloud2 msg;
  msg.height = 1;
  msg.width = n_pts;
  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.reserve(n_pts);

  using Pc2Itr = sensor_msgs::PointCloud2Iterator<float>;
  using Pc2Uint8Itr = sensor_msgs::PointCloud2Iterator<uint8_t>;
  Pc2Itr itr_xyz(msg, "xyz");
  Pc2Uint8Itr itr_rgb(msg, "rgb");

  float scaler = scale_to_255 ? 255.0f : 1.0f;
  auto arr_buf = arr.unchecked();
  for (size_t i = 0; i < n_pts; ++i) {
    itr_xyz[0] = arr_buf(i, 0);
    itr_xyz[1] = arr_buf(i, 1);
    itr_xyz[2] = arr_buf(i, 2);
    uint8_t r = std::min(std::max((arr_buf(i, 3) * scaler), 0.f), 255.f);
    uint8_t g = std::min(std::max((arr_buf(i, 4) * scaler), 0.f), 255.f);
    uint8_t b = std::min(std::max((arr_buf(i, 5) * scaler), 0.f), 255.f);
    itr_rgb[0] = r;
    itr_rgb[1] = g;
    itr_rgb[2] = b;

    ++itr_xyz;
    ++itr_rgb;
  }
  msg.header.frame_id = frame_id;
  return msg;
}

/**
 * Convert HxWx3 float ndarray to unorganized pc2.
 */
sensor_msgs::PointCloud2 xyz_img_to_pc2(const ndarray3f& xyz_img,
                                        bool skip_nan,
                                        const std::string& frame_id) {
  if (xyz_img.shape(2) != 3) {
    throw std::runtime_error("xyz_img must have 3 channels");
  }
  PCXYZ pc;
  auto xyz_img_buf = xyz_img.unchecked();
  for (size_t v = 0; v < xyz_img.shape(0); ++v) {
    for (size_t u = 0; u < xyz_img.shape(1); ++u) {
      float x = xyz_img_buf(v, u, 0);
      float y = xyz_img_buf(v, u, 1);
      float z = xyz_img_buf(v, u, 2);
      if (skip_nan && (std::isnan(x) || std::isnan(y) || std::isnan(z))) {
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

/**
 * Convert HxWx3 float ndarray to organized pc2.
 */
sensor_msgs::PointCloud2 xyz_img_to_organized_pc2(const ndarray3f& xyz_img,
                                                  const std::string& frame_id) {
  if (xyz_img.shape(2) != 3) {
    throw std::runtime_error("xyz_img must have 3 channels");
  }
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

/**
 * Convert HxW float depth img ndarray to organized pc2.
 * Units are meters.
 */
sensor_msgs::PointCloud2 depth_img_to_pc2(py::array_t<float, 2> depth_img,
                                          float cx,
                                          float cy,
                                          float fx,
                                          float fy,
                                          float max_valid_depth,
                                          const std::string& frame_id) {
  auto depth_buf = depth_img.unchecked();
  PCXYZ cloud;
  for (size_t v = 0; v < depth_img.shape(0); ++v) {
    for (size_t u = 0; u < depth_img.shape(1); ++u) {
      float d = depth_buf(v, u);
      if (d == 0.0 || std::isnan(d) || d > max_valid_depth) {
        continue;
      }
      pcl::PointXYZ p;
      p.x = (u - cx) * d / fx;
      p.y = (v - cy) * d / fy;
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

/**
 * Convert PCLPointCloud2 to ROS PointCloud2.
 */
sensor_msgs::PointCloud2 pclpc2_to_pc2(const PCLPC2::Ptr pclpc2,
                                       const std::string& frame_id) {
  // sending to python will trigger copy anyway, so move should be safe

  sensor_msgs::PointCloud2 pc2;
  // TODO provide move option
  // pcl_conversions::moveFromPCL(*pclpc2, pc2);
  pcl_conversions::fromPCL(*pclpc2, pc2);
  pc2.header.frame_id = frame_id;
  return pc2;
}

/**
 * Convert ROS PointCloud2 to PCL PointCloud2.
 */
PCLPC2::Ptr pc2_to_pclpc2(sensor_msgs::PointCloud2 pc2) {
  // sending to python will trigger copy anyway, so move should be safe
  PCLPC2::Ptr pclpc2(new PCLPC2);
  pcl_conversions::moveToPCL(pc2, *pclpc2);
  return pclpc2;
}

/**
 * Convert PCL PointCloud<T> to ROS PointCloud2.
 */
template <class PointCloudT>
sensor_msgs::PointCloud2 pclpc_to_pc2(const typename PointCloudT::Ptr pc,
                                      const std::string& frame_id) {
  // there's no move for pointcloudT
  sensor_msgs::PointCloud2 pc2;
  pcl::toROSMsg(*pc, pc2);
  pc2.header.frame_id = frame_id;
  return pc2;
}

void export_converters(py::module& m) {
  //using namespace pybind11::literals;

  m.def("xyz_img_to_pc2",
        &xyz_img_to_pc2,
        py::arg("xyz_img"),
        py::arg("skip_nan") = true,
        py::arg("frame_id") = "");
  m.def("xyz_img_to_organized_pc2",
        &xyz_img_to_organized_pc2,
        py::arg("xyz_img"),
        py::arg("frame_id") = "");
  m.def("pc2_to_pcxyz", &pc2_to_pcxyz, py::arg("msg"));
  m.def("pc2_to_pcxyzrgb", &pc2_to_pcxyzrgb, py::arg("msg"));
  m.def("xyz_to_pc2", &xyz_to_pc2, py::arg("arr"), py::arg("frame_id") = "");
  m.def("xyzrgb_to_pc2",
        &xyzrgb_to_pc2,
        py::arg("arr"),
        py::arg("scale_to_255") = false,
        py::arg("frame_id") = "");
  m.def("pcxyz_to_pc2",
        &pclpc_to_pc2<PCXYZ>,
        py::arg("pc"),
        py::arg("frame_id") = "");
  m.def("pcxyzrgb_to_pc2",
        &pclpc_to_pc2<PCXYZRGB>,
        py::arg("pc"),
        py::arg("frame_id") = "");
  m.def("pclpc2_to_pc2",
        &pclpc2_to_pc2,
        py::arg("pclpc2"),
        py::arg("frame_id") = "");
  m.def("pc2_to_pclpc2", &pc2_to_pclpc2, py::arg("pc2"));
  m.def("depth_img_to_pc2",
        &depth_img_to_pc2,
        py::arg("depth_img"),
        py::arg("cx"),
        py::arg("cy"),
        py::arg("fx"),
        py::arg("fy"),
        py::arg("max_valid_depth"),
        py::arg("frame_id"));
  m.def("pc2_to_xyz_ndarray",
        &pc2_to_xyz_ndarray,
        py::arg("pc2"),
        py::arg("skip_nan"));
}
}
