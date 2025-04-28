#include <nanobind/nanobind.h>

namespace nb = nanobind;

void bind_MultiBodyConfig(nb::module_ &);

NB_MODULE(_rbdyn, unused_m)
{
  /**
   * We use a trick to load the symbols in the _rbdyn namespace within the parent rbdyn module's namespace.
   * This is done so that we expose the rbdyn types as rbdyn.Type instead of rbdyn._rbdyn.Type.
   *
   * In particular this makes types exported in stub files cleaner and more user-friendly, and makes sphinx autoapi
   * cross-references work accross projects.
   *
   * See https://github.com/wjakob/nanobind/issues/420#issuecomment-1950233151 for further details
   */
  (void) unused_m;
  nb::module_ m = nb::module_::import_("rbdyn");
  // register functions and classes here
  m.doc() = "Python bindings for the RBDyn library";
  bind_MultiBodyConfig(m);
}
