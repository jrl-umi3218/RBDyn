#pragma once

/* Copyright 2012-2017 CNRS-UM LIRMM, CNRS-AIST JRL
 *
 * This file is part of RBDyn.
 *
 * RBDyn is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RBDyn is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with RBDyn.  If not, see <http://www.gnu.org/licenses/>.
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
