#include <RBDyn/MultiBodyConfig.h>

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/bind_vector.h>

namespace nb = nanobind;

void bind_MultiBodyConfig(nb::module_ & rbdyn)
{
  using MBC = rbd::MultiBodyConfig;


  // Make std::vector<std::vector<double>> availble as a read-write vector
  // nb::bind_vector<std::vector<double>>(rbdyn, "GeneralizedCoordinate");
  // nb::class_<std::vector<std::vector<double>>>(rbdyn, "GeneralizedCoordinatesVector")
  //     .def(nb::init<>())
  //     .def("resize", [](std::vector<std::vector<double>>& self, size_t size) {
  //         self.resize(size);
  //     })
  //     .def("append", [](std::vector<std::vector<double>>& self, std::vector<double> vec) {
  //         self.push_back(vec);
  //     })
  //     .def("__getitem__", [](std::vector<std::vector<double>>& self, size_t index) -> std::vector<double>& {
  //         return self.at(index);
  //     }, nb::rv_policy::reference)
  //     .def("__setitem__", [](std::vector<std::vector<double>>& self, size_t index, std::vector<double> vec) {
  //         self.at(index) = vec;
  //     });
  //
  // auto mbc = nb::class_<rbd::MultiBodyConfig>(rbdyn, "MultiBodyConfig");
  // mbc.def(nb::init());
  //
  // // TODO:
  // // void zero(const MultiBody & mb);
  //
  // mbc.def_rw("q", &MBC::q, "Generalized position variable")
  //   .def_rw("alpha", &MBC::alpha, "Generalized speed variable")
  //   .def_rw("alphaD", &MBC::alphaD, "Generalized acceleration variable");

}
