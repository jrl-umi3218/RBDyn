/*
 * Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <rbdyn/config.hh>

namespace rbd
{
class MultiBody;
struct MultiBodyConfig;

/**
 * Compute the forward velocity of a MultiBody.
 * @param mb MultiBody used has model.
 * @param mbc Use alpha generalized velocity vector, bodyPosW, jointConfig
 * and parentToSon.
 * Fill jointVelocity, bodyVelW and bodyVelB.
 */
RBDYN_DLLAPI void forwardVelocity(const MultiBody & mb, MultiBodyConfig & mbc);

/**
 * Safe version.
 * @see forwardVelocity.
 * @throw std::domain_error If there is a mismatch between mb and mbc.
 */
RBDYN_DLLAPI void sForwardVelocity(const MultiBody & mb, MultiBodyConfig & mbc);

} // namespace rbd
