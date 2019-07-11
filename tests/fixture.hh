/*
 * Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <boost/filesystem/path.hpp>
#include <boost/make_shared.hpp>
#include <boost/test/output_test_stream.hpp>
#include <boost/test/test_case_template.hpp>
#include <boost/test/unit_test.hpp>

#include <iostream>

#ifdef TESTS_DATA_DIR
// See: http://stackoverflow.com/q/26299144/1043187
#  ifdef __NVCC__
#    include <boost/preprocessor/stringize.hpp>
const static boost::filesystem::path tests_data_dir(BOOST_PP_STRINGIZE(TESTS_DATA_DIR));
#  else
const static boost::filesystem::path tests_data_dir(TESTS_DATA_DIR);
#  endif //! __NVCC__
#else
const static boost::filesystem::path tests_data_dir;
#endif //! TESTS_DATA_DIR

namespace rbd
{
struct TestSuiteConfiguration
{
  TestSuiteConfiguration() {}

  ~TestSuiteConfiguration() {}
};

boost::shared_ptr<boost::test_tools::output_test_stream> retrievePattern(const std::string & testName)
{
  std::string patternFilename = TESTS_DATA_DIR;
  patternFilename += "/";
  patternFilename += testName;
  patternFilename += ".stdout";

  boost::shared_ptr<boost::test_tools::output_test_stream> output =
      boost::make_shared<boost::test_tools::output_test_stream>(patternFilename, true);
  return output;
}
} // end of namespace rbd
