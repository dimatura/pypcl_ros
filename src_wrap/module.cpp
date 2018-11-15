#include <pybind11/pybind11.h>

#include <pybind11/eigen.h>
#include <Eigen/Core>
#include <cmath>

namespace py = pybind11;

namespace pypcl_ros {

void export_converters(py::module& m);

}  // pypcl_ros

PYBIND11_MODULE(libpypcl_ros, m) {
  m.doc() = "libpypcl_ros";
  pypcl_ros::export_converters(m);
}
