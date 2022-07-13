# This script is meant to be sourced from other scripts

# Check for clang-format, prefer 10 if available
if [[ -x "$(command -v clang-format-10)" ]]; then
  clang_format=clang-format-10
elif [[ -x "$(command -v clang-format)" ]]; then
  clang_format=clang-format
else
  echo "clang-format or clang-format-10 must be installed"
  exit 1
fi

# Find all source files in the project minus those that are auto-generated or we do not maintain
src_files=`find src tests -type f \( -name '*.cpp' -or -name '*.h*' \)|grep -v tests/benchmark`

