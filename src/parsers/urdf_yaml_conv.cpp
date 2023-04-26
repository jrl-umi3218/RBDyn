/*
 * Copyright 2012-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <RBDyn/parsers/urdf.h>
#include <RBDyn/parsers/yaml.h>

#include <fstream>
#include <iostream>

std::string getContent(std::istream & in)
{
  // Without sync_with_stdio = false, in_avail always returns zero even if data is available
  std::ios::sync_with_stdio(false);
  const auto available_char = in.rdbuf()->in_avail();
  std::ios::sync_with_stdio(true);
  if(available_char == 0)
  {
    return {};
  }

  std::string content;
  std::string line;
  while(std::getline(in, line))
  {
    content += line + '\n';
  }
  return content;
}

int main(int argc, const char * argv[])
{
  auto print_usage = [&]()
  {
    std::cout << "Converts between URDF and YAML robot model formats. The input format is detected based on the given "
                 "content.\n";
    std::cout << "Usage:\n";
    std::cout << "\t" << argv[0] << ": reads from the standard input and prints to the standard output\n";
    std::cout << "\t" << argv[0]
              << " input_file: reads input_file and prints to the standard output (selected if the standard input is "
                 "empty)\n";
    std::cout
        << "\t" << argv[0]
        << " output_file: reads from the standard input and writes to output_file (selected if the standard input "
           "has data)\n";
    std::cout << "\t" << argv[0] << " input_file output_file: reads input_file and writes to output_file\n";
  };
  if(argc == 2 && (std::strcmp(argv[1], "-h") == 0 || std::strcmp(argv[1], "--help") == 0))
  {
    print_usage();
    std::exit(0);
  }

  if(argc > 3)
  {
    print_usage();
    std::exit(-1);
  }

  const auto stdin_content = getContent(std::cin);
  const auto file_input = stdin_content.size() == 0;
  const auto file_output = (file_input && argc == 3) || (!file_input && argc == 2);

  if(file_input && argc == 1)
  {
    std::cerr << "No file given and nothing to read from the standard input" << std::endl;
    print_usage();
    std::exit(-1);
  }

  const auto content = [&]() -> std::string
  {
    if(file_input)
    {
      std::ifstream file(argv[1]);
      if(file.is_open())
      {
        return getContent(file);
      }
      else
      {
        std::cerr << "Failed to read the content of " << argv[1] << std::endl;
        std::exit(-1);
      }
    }
    else
    {
      return stdin_content;
    }
  }();

  const auto robot_pos = content.find("robot");
  if(robot_pos == std::string::npos)
  {
    std::cerr << "The description doesn't contain a robot node" << std::endl;
    std::exit(-1);
  }

  bool is_urdf{true};
  const auto parser_result = [&]()
  {
    if((robot_pos > 0) && (content[robot_pos - 1] == '<'))
    {
      return rbd::parsers::from_urdf(content);
    }
    else if((content.size() > robot_pos + 5) && (content[robot_pos + 5] == ':'))
    {
      is_urdf = false;
      return rbd::parsers::from_yaml(content);
    }
    else
    {
      std::cerr << "The description is neither a valid URDF or YAML one, no '<' before or ':' after 'robot'"
                << std::endl;
      std::exit(-1);
    }
  }();

  const auto conversion_result = [&]()
  {
    if(is_urdf)
    {
      return rbd::parsers::to_yaml(parser_result);
    }
    else
    {
      return rbd::parsers::to_urdf(parser_result);
    }
  }();

  if(file_output)
  {
    const auto file_path = argv[argc - 1];
    std::ofstream file(file_path, std::ios::out);
    if(file.is_open())
    {
      file << conversion_result;
    }
    else
    {
      std::cerr << "Failed to open " << file_path << " for writing" << std::endl;
      std::exit(-1);
    }
  }
  else
  {
    std::cout << conversion_result << std::endl;
  }

  return 0;
}
