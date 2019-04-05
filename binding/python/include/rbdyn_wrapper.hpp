#pragma once

/*
 * Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */


#include <RBDyn/Body.h>
#include <RBDyn/Joint.h>
#include <RBDyn/MultiBodyConfig.h>
#include <RBDyn/MultiBodyGraph.h>

namespace rbd
{

std::string BodyToString(const Body & b)
{
  std::stringstream ss;
  ss << b;
  return ss.str();
}

std::string JointToString(const Joint & j)
{
  std::stringstream ss;
  ss << j;
  return ss.str();
}

std::vector<double> ZeroParam(Joint::Type t)
{
  return Joint::ZeroParam(t);
}

std::vector<double> ZeroDof(Joint::Type t)
{
  return Joint::ZeroDof(t);
}

ConfigConverter* ConfigConverterConstructor(const MultiBody& mb1, const MultiBody& mb2)
{
  return rbd::ConfigConverter::sConstructor(mb1, mb2);
}

MultiBody & const_cast_mb(const MultiBody & mb)
{
  return const_cast<MultiBody&>(mb);
}

MultiBodyConfig& const_cast_mbc(const MultiBodyConfig& mbc)
{
  return const_cast<MultiBodyConfig&>(mbc);
}

MultiBodyGraph & const_cast_mbg(const MultiBodyGraph & mbg)
{
  return const_cast<MultiBodyGraph&>(mbg);
}

void dv_set_item(std::vector<double> & v, unsigned int idx, double value)
{
  v[idx] = value;
}

void dvv_set_item(std::vector<std::vector<double>> & v, unsigned int idx, const std::vector<double> & value)
{
  v[idx] = value;
}

void mbcv_set_item(std::vector<MultiBodyConfig> & v, unsigned int idx, const MultiBodyConfig & mbc)
{
  v[idx] = mbc;
}

}
