#include <nanobind/nanobind.h>

namespace nb = nanobind;

void bind_MultiBodyConfig(nb::module_ &);

NB_MODULE(_rbdyn, m)
{
  // register functions and classes here
  m.doc() = "Python bindings for the RBDyn library";
  bind_MultiBodyConfig(m);
}
