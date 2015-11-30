// Copyright (C) 2015 by Benjamin Chrétien, CNRS-LIRMM.
//
// This file is part of the rsdf library.
//
// rsdf is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// rsdf is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with rsdf.  If not, see <http://www.gnu.org/licenses/>.

#pragma once

# include <iostream>

# include <boost/make_shared.hpp>
# include <boost/filesystem/path.hpp>

# include <boost/test/output_test_stream.hpp>
# include <boost/test/test_case_template.hpp>
# include <boost/test/unit_test.hpp>

# ifdef TESTS_DATA_DIR
// See: http://stackoverflow.com/q/26299144/1043187
#  ifdef __NVCC__
#   include <boost/preprocessor/stringize.hpp>
const static boost::filesystem::path
  tests_data_dir (BOOST_PP_STRINGIZE (TESTS_DATA_DIR));
#  else
const static boost::filesystem::path
  tests_data_dir (TESTS_DATA_DIR);
#  endif //! __NVCC__
# else
const static boost::filesystem::path tests_data_dir;
# endif //! TESTS_DATA_DIR

namespace rbd
{
  struct TestSuiteConfiguration
  {
    TestSuiteConfiguration ()
    {
    }

    ~TestSuiteConfiguration ()
    {
    }
  };

  boost::shared_ptr<boost::test_tools::output_test_stream>
  retrievePattern (const std::string& testName)
  {
    std::string patternFilename = TESTS_DATA_DIR;
    patternFilename += "/";
    patternFilename += testName;
    patternFilename += ".stdout";

    boost::shared_ptr<boost::test_tools::output_test_stream>
      output = boost::make_shared<boost::test_tools::output_test_stream>
      (patternFilename, true);
    return output;
  }
} // end of namespace rbd
