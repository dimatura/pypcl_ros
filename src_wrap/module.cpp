#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace pypcl_ros {
void export_converters(py::module& m);
}

PYBIND11_MODULE(libpypcl_ros, m) {
  m.doc() = "libpypcl_ros";
  pypcl_ros::export_converters(m);
}
