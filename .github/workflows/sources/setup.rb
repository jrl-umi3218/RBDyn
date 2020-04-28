#!/usr/bin/env ruby

require "yaml"
require "json"

data = File.open(File.expand_path("package.yml", __dir__)).read().gsub(/^on:$/, "\"on\":");
yaml = YAML.load(data)

out = YAML.load(yaml.to_json).to_yaml(line_width: 1024).gsub(/^'on':$/, "on:");
File.write(File.expand_path("../package.yml", __dir__), out)
