/*
 * Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

// associated header
#include "RBDyn/Jacobian.h"

// includes
// std
#include <algorithm>
#include <numeric>
#include <stdexcept>

// RBDyn
#include "RBDyn/MultiBodyConfig.h"

namespace rbd
{

Jacobian::Jacobian() {}

Jacobian::Jacobian(const MultiBody & mb, const std::string & bodyName, const Eigen::Vector3d & point)
: jointsPath_(), point_(point), jac_(), jacDot_()
{
  bodyIndex_ = mb.sBodyIndexByName(bodyName);
  refBodyIndex_ = 0;

  int index = bodyIndex_;

  int dof = 0;
  while(index != -1)
  {
    jointsPath_.insert(jointsPath_.begin(), index);
    dof += mb.joint(index).dof();
    jointsSign_.insert(jointsSign_.begin(), 1);

    index = mb.parent(index);
  }

  jac_.resize(6, dof);
  jacDot_.resize(6, dof);
}

Jacobian::Jacobian(const MultiBody & mb,
                   const std::string & bodyName,
                   const std::string & refBodyName,
                   const Eigen::Vector3d & point)
: jointsPath_(), point_(point), jac_(), jacDot_()
{
  bodyIndex_ = mb.sBodyIndexByName(bodyName);
  refBodyIndex_ = mb.sBodyIndexByName(refBodyName);

  int index = bodyIndex_;

  int dof = 0;
  while(index != -1)
  {
    if(index == refBodyIndex_)
    {
      break;
    }

    jointsPath_.insert(jointsPath_.begin(), index);
    dof += mb.joint(index).dof();
    jointsSign_.insert(jointsSign_.begin(), 1);

    index = mb.parent(index);
  }

  // The two bodies don't belong to the same branch -> start from ref body to the root of the tree
  if(index != refBodyIndex_)
  {
    index = refBodyIndex_;
    int count = 0;
    // Add joints until reaching the common node
    do
    {
      jointsPath_.insert(jointsPath_.begin() + count, index);
      dof += mb.joint(index).dof();
      jointsSign_.insert(jointsSign_.begin(), -1);

      index = mb.parent(index);
      count++;
    } while(std::find(jointsPath_.begin() + count, jointsPath_.end(), index) == jointsPath_.end());

    if(index > 0)
    {
      // Delete joints between the common joint and the root
      int commonIdx = count;
      while(jointsPath_[static_cast<size_t>(++commonIdx)] != index)
      {
        // Get to the common node
      }
      dof -= std::accumulate(jointsPath_.begin() + count, jointsPath_.begin() + commonIdx + 1, 0,
                             [&](int dofC, int idx) { return dofC + mb.joint(idx).dof(); });
      jointsPath_.erase(jointsPath_.begin() + count, jointsPath_.begin() + commonIdx + 1);
      jointsSign_.erase(jointsSign_.begin() + count, jointsSign_.begin() + commonIdx + 1);
    }
  }

  jac_.resize(6, dof);
  jacDot_.resize(6, dof);
}

MultiBody Jacobian::subMultiBody(const MultiBody & mb) const
{
  std::vector<Body> bodies;
  std::vector<Joint> joints;

  std::vector<int> pred;
  std::vector<int> succ;
  std::vector<int> parent;
  std::vector<sva::PTransformd> Xt;

  for(int index = 0; index < static_cast<int>(jointsPath_.size()); ++index)
  {
    const auto uindex = static_cast<size_t>(index);
    int i = jointsPath_[uindex];

    // body info
    bodies.push_back(mb.body(i));
    parent.push_back(index - 1);

    // joint info
    succ.push_back(index);
    pred.push_back(index - 1);

    if(index == 0)
    {
      if(jointsSign_[uindex] == -1)
        Xt.push_back(sva::PTransformd(Eigen::Vector3d(0., 0., 0.)));
      else
        Xt.push_back(mb.transform(i));
    }
    else
    {
      if(jointsSign_[uindex - 1] == -1)
      {
        if(jointsSign_[uindex] == -1)
          Xt.push_back(mb.transform(jointsPath_[uindex - 1]).inv());
        else
          Xt.push_back(mb.transform(jointsPath_[uindex - 1]).inv() * mb.transform(jointsPath_[uindex]));
      }
      else
      {
        Xt.push_back(mb.transform(i));
      }
    }
    auto joint = mb.joint(i);
    if(jointsSign_[uindex] == -1)
    {
      auto fwd = joint.forward() ? false : true;
      joint.forward(fwd);
    }
    joints.push_back(joint);
  }
  return MultiBody(std::move(bodies), std::move(joints), std::move(pred), std::move(succ), std::move(parent),
                   std::move(Xt));
}

/// private implementation of the Jacobian computation
/// We use the Transform template allow Eigen3 to
/// remove the Matrix3d from the computation.
template<typename Transform>
static inline const Eigen::MatrixXd & jacobian_(const MultiBody & mb,
                                                const MultiBodyConfig & mbc,
                                                const Transform & Trans_0_p,
                                                const std::vector<int> & jointsPath,
                                                const std::vector<double> & jointsSign,
                                                Eigen::MatrixXd & jac)
{
  const std::vector<Joint> & joints = mb.joints();
  int curJ = 0;

  sva::PTransformd X_0_p(Trans_0_p);

  for(std::size_t index = 0; index < jointsPath.size(); ++index)
  {
    const auto i = static_cast<size_t>(jointsPath[index]);

    sva::PTransformd X_i_N = X_0_p * mbc.bodyPosW[i].inv();

    // If the joint motion is seen from child body to parent body, we have to invert its effect
    for(int dof = 0; dof < joints[i].dof(); ++dof)
    {
      jac.col(curJ + dof).noalias() =
          (X_i_N * (sva::MotionVecd(jointsSign[index] * mbc.motionSubspace[i].col(dof)))).vector();
    }

    curJ += joints[i].dof();
  }
  return jac;
}

const Eigen::MatrixXd & Jacobian::jacobian(const MultiBody & mb,
                                           const MultiBodyConfig & mbc,
                                           const sva::PTransformd & X_0_p)
{
  jacobian_(mb, mbc, X_0_p, jointsPath_, jointsSign_, jac_);
  // Change Jacobian base : root of the tree --> reference body
  if(refBodyIndex_ > 0)
  {
    auto dof_count = jac_.cols();
    jac_.block(0, 0, 3, dof_count) =
        mbc.bodyPosW[static_cast<size_t>(refBodyIndex_)].rotation() * jac_.block(0, 0, 3, dof_count);
    jac_.block(3, 0, 3, dof_count) =
        mbc.bodyPosW[static_cast<size_t>(refBodyIndex_)].rotation() * jac_.block(3, 0, 3, dof_count);
  }
  return jac_;
}

const Eigen::MatrixXd & Jacobian::jacobian(const MultiBody & mb, const MultiBodyConfig & mbc)
{
  auto N = static_cast<size_t>(jointsPath_.back());

  // the transformation must be read {}^0E_p {}^pT_N {}^NX_0
  Eigen::Vector3d T_0_Np((point_ * mbc.bodyPosW[N]).translation());
  jacobian_(mb, mbc, T_0_Np, jointsPath_, jointsSign_, jac_);
  // Change Jacobian base : root of the tree --> reference body
  if(refBodyIndex_ > 0)
  {
    auto dof_count = jac_.cols();
    jac_.block(0, 0, 3, dof_count) =
        mbc.bodyPosW[static_cast<size_t>(refBodyIndex_)].rotation() * jac_.block(0, 0, 3, dof_count);
    jac_.block(3, 0, 3, dof_count) =
        mbc.bodyPosW[static_cast<size_t>(refBodyIndex_)].rotation() * jac_.block(3, 0, 3, dof_count);
  }
  return jac_;
}

const Eigen::MatrixXd & Jacobian::bodyJacobian(const MultiBody & mb, const MultiBodyConfig & mbc)
{
  auto N = static_cast<size_t>(jointsPath_.back());

  sva::PTransformd X_0_Np = point_ * mbc.bodyPosW[N];
  return jacobian_(mb, mbc, X_0_Np, jointsPath_, jointsSign_, jac_);
}

const Eigen::MatrixXd & Jacobian::vectorJacobian(const MultiBody & mb,
                                                 const MultiBodyConfig & mbc,
                                                 const Eigen::Vector3d & vector)
{
  const std::vector<Joint> & joints = mb.joints();

  int curJ = 0;
  const auto N = static_cast<size_t>(jointsPath_.back());

  Eigen::Matrix3d E_N_0(mbc.bodyPosW[N].rotation().transpose());
  sva::PTransformd X_Np = point_ * mbc.bodyPosW[N];
  sva::PTransformd vec = sva::PTransformd(vector);
  for(std::size_t index = 0; index < jointsPath_.size(); ++index)
  {
    const auto i = static_cast<size_t>(jointsPath_[index]);

    sva::PTransformd X_i_N = X_Np * mbc.bodyPosW[i].inv();
    Eigen::Matrix3d E_i_0(E_N_0 * X_i_N.rotation());
    sva::PTransformd X_i_Nv = vec * X_i_N;
    Eigen::Vector3d diff(X_i_N.translation() - X_i_Nv.translation());

    // Compute translation component of : Jac_{Nv} - Jac_{N}
    // Iteration : {}^{Nv}X_i S_i - {}^NX_i S_i
    //             ({}^{Nv}T_i - {}^NT_i) S_i
    //             {}^0E_i(T) (({}^{N}T_i - {}^{Nv}T_i) \times W_i)
    for(int dof = 0; dof < joints[i].dof(); ++dof)
    {
      jac_.col(curJ + dof).tail<3>().noalias() = E_i_0 * (diff.cross(mbc.motionSubspace[i].col(dof).head<3>()));
    }

    curJ += joints[i].dof();
  }

  return jac_;
}

const Eigen::MatrixXd & Jacobian::vectorBodyJacobian(const MultiBody & mb,
                                                     const MultiBodyConfig & mbc,
                                                     const Eigen::Vector3d & vector)
{
  const std::vector<Joint> & joints = mb.joints();

  int curJ = 0;
  const auto N = static_cast<size_t>(jointsPath_.back());

  sva::PTransformd X_Np = point_ * mbc.bodyPosW[N];
  sva::PTransformd vec = sva::PTransformd(vector);
  for(std::size_t index = 0; index < jointsPath_.size(); ++index)
  {
    const auto i = static_cast<size_t>(jointsPath_[index]);

    sva::PTransformd X_i_N = X_Np * mbc.bodyPosW[i].inv();
    sva::PTransformd X_i_Nv = vec * X_i_N;
    Eigen::Vector3d diff(X_i_N.translation() - X_i_Nv.translation());

    // Compute translation component of : Jac_{Nv} - Jac_{N}
    // Iteration : {}^{Nv}X_i S_i - {}^NX_i S_i
    //             ({}^{Nv}T_i - {}^NT_i) S_i
    //             {}^NE_i(T) (({}^{N}T_i - {}^{Nv}T_i) \times W_i)
    for(int dof = 0; dof < joints[i].dof(); ++dof)
    {
      jac_.col(curJ + dof).tail<3>().noalias() =
          X_i_N.rotation() * (diff.cross(mbc.motionSubspace[i].col(dof).head<3>()));
    }

    curJ += joints[i].dof();
  }

  return jac_;
}

const Eigen::MatrixXd & Jacobian::jacobianDot(const MultiBody & mb, const MultiBodyConfig & mbc)
{
  const std::vector<Joint> & joints = mb.joints();

  int curJ = 0;
  const auto N = static_cast<size_t>(jointsPath_.back());

  sva::PTransformd X_0_Np = point_ * mbc.bodyPosW[N];
  // speed of point in body N
  sva::MotionVecd X_VNp = point_ * mbc.bodyVelB[N];

  sva::PTransformd E_N_0(Eigen::Matrix3d(mbc.bodyPosW[N].rotation().transpose()));
  // angular velocity of rotation N to O
  sva::MotionVecd E_VN(mbc.bodyVelW[N].angular(), Eigen::Vector3d::Zero());

  for(std::size_t index = 0; index < jointsPath_.size(); ++index)
  {
    const auto i = static_cast<size_t>(jointsPath_[index]);

    sva::PTransformd X_i_Np = X_0_Np * mbc.bodyPosW[i].inv();
    // speed of X_i_N in Np coordinate
    sva::MotionVecd X_VNp_i_Np = X_i_Np * mbc.bodyVelB[i] - X_VNp;

    for(int j = 0; j < joints[i].dof(); ++j)
    {
      sva::MotionVecd S_ij(mbc.motionSubspace[i].col(j));

      // JD_i = E_N_0_d*X_i_N*S_i + E_N_0*X_i_N_d*S_i
      // E_N_0_d = (ANG_VN)_0 x E_N_0
      // X_i_N_d = (Vi - VN)_N x X_i_N

      jacDot_.col(curJ).noalias() =
          (E_VN.cross(E_N_0 * X_i_Np * S_ij) + E_N_0 * X_VNp_i_Np.cross(X_i_Np * S_ij)).vector();
      ++curJ;
    }
  }

  return jacDot_;
}

const Eigen::MatrixXd & Jacobian::bodyJacobianDot(const MultiBody & mb, const MultiBodyConfig & mbc)
{
  const std::vector<Joint> & joints = mb.joints();

  int curJ = 0;
  const auto N = static_cast<size_t>(jointsPath_.back());

  sva::PTransformd X_0_Np = point_ * mbc.bodyPosW[N];
  // speed of point in body N
  sva::MotionVecd X_VNp = point_ * mbc.bodyVelB[N];

  for(std::size_t index = 0; index < jointsPath_.size(); ++index)
  {
    const auto i = static_cast<size_t>(jointsPath_[index]);

    sva::PTransformd X_i_Np = X_0_Np * mbc.bodyPosW[i].inv();
    // speed of X_i_N in Np coordinate
    sva::MotionVecd X_VNp_i_Np = X_i_Np * mbc.bodyVelB[i] - X_VNp;

    for(int j = 0; j < joints[i].dof(); ++j)
    {
      sva::MotionVecd S_ij(mbc.motionSubspace[i].col(j));

      // JD_i = X_i_N_d*S_i
      // X_i_N_d = (Vi - VN)_N x X_i_N

      jacDot_.col(curJ).noalias() = (X_VNp_i_Np.cross(X_i_Np * S_ij)).vector();
      ++curJ;
    }
  }

  return jacDot_;
}

sva::MotionVecd Jacobian::velocity(const MultiBody & /* mb */,
                                   const MultiBodyConfig & mbc,
                                   const sva::PTransformd & X_b_p) const
{
  const auto N = static_cast<size_t>(jointsPath_.back());
  return X_b_p * mbc.bodyVelB[N];
}

sva::MotionVecd Jacobian::velocity(const MultiBody & /* mb */, const MultiBodyConfig & mbc) const
{
  const auto N = static_cast<size_t>(jointsPath_.back());
  // equivalent of E_N_0*X_N_Np
  sva::PTransformd X_Np_w(mbc.bodyPosW[N].rotation().transpose(), point_.translation());
  return X_Np_w * mbc.bodyVelB[N];
}

sva::MotionVecd Jacobian::bodyVelocity(const MultiBody & /* mb */, const MultiBodyConfig & mbc) const
{
  const auto N = static_cast<size_t>(jointsPath_.back());
  return point_ * mbc.bodyVelB[N];
}

static inline sva::MotionVecd normalAccB_(const rbd::MultiBodyConfig & mbc, const std::vector<int> & jointsPath)
{
  sva::MotionVecd accel(Eigen::Vector6d::Zero());
  for(std::size_t index = 0; index < jointsPath.size(); ++index)
  {
    const auto i = static_cast<size_t>(jointsPath[index]);
    const sva::PTransformd & X_p_i = mbc.parentToSon[i];
    const sva::MotionVecd & vj_i = mbc.jointVelocity[i];
    const sva::MotionVecd & vb_i = mbc.bodyVelB[i];

    accel = X_p_i * accel + vb_i.cross(vj_i);
  }
  return accel;
}

sva::MotionVecd Jacobian::normalAcceleration(const MultiBody & /* mb */,
                                             const MultiBodyConfig & mbc,
                                             const sva::PTransformd & X_b_p,
                                             const sva::MotionVecd & V_b_p) const
{
  return normalAcceleration(mbc, normalAccB_(mbc, jointsPath_), X_b_p, V_b_p);
}

sva::MotionVecd Jacobian::normalAcceleration(const MultiBody & /* mb */, const MultiBodyConfig & mbc) const
{
  return normalAcceleration(mbc, normalAccB_(mbc, jointsPath_));
}

sva::MotionVecd Jacobian::bodyNormalAcceleration(const MultiBody & /* mb */, const MultiBodyConfig & mbc) const
{
  return bodyNormalAcceleration(mbc, normalAccB_(mbc, jointsPath_));
}

sva::MotionVecd Jacobian::normalAcceleration(const MultiBody & /* mb */,
                                             const MultiBodyConfig & mbc,
                                             const std::vector<sva::MotionVecd> & normalAccB,
                                             const sva::PTransformd & X_b_p,
                                             const sva::MotionVecd & V_b_p) const
{
  return normalAcceleration(mbc, normalAccB[static_cast<size_t>(jointsPath_.back())], X_b_p, V_b_p);
}

sva::MotionVecd Jacobian::normalAcceleration(const MultiBody & /* mb */,
                                             const MultiBodyConfig & mbc,
                                             const std::vector<sva::MotionVecd> & normalAccB) const
{
  return normalAcceleration(mbc, normalAccB[static_cast<size_t>(jointsPath_.back())]);
}

sva::MotionVecd Jacobian::bodyNormalAcceleration(const MultiBody & /* mb */,
                                                 const MultiBodyConfig & mbc,
                                                 const std::vector<sva::MotionVecd> & normalAccB) const
{
  return bodyNormalAcceleration(mbc, normalAccB[static_cast<size_t>(jointsPath_.back())]);
}

void Jacobian::translateJacobian(const Eigen::Ref<const Eigen::MatrixXd> & jac,
                                 const MultiBodyConfig & mbc,
                                 const Eigen::Vector3d & point,
                                 Eigen::MatrixXd & res)
{
  const auto N = static_cast<size_t>(jointsPath_.back());
  sva::PTransformd E_N_0(Eigen::Matrix3d(mbc.bodyPosW[N].rotation().transpose()));
  sva::PTransformd t(point);

  t = E_N_0 * t * E_N_0.inv();

  for(int i = 0; i < jac.cols(); ++i)
  {
    sva::MotionVecd mv(jac.col(i));
    res.col(i) = (t * mv).vector();
  }
}

void Jacobian::translateBodyJacobian(const Eigen::Ref<const Eigen::MatrixXd> & jac,
                                     const MultiBodyConfig & /* mbc */,
                                     const Eigen::Vector3d & point,
                                     Eigen::MatrixXd & res)
{
  sva::PTransformd t(point);

  for(int i = 0; i < jac.cols(); ++i)
  {
    sva::MotionVecd mv(jac.col(i));
    res.col(i) = (t * mv).vector();
  }
}

void Jacobian::fullJacobian(const MultiBody & mb,
                            const Eigen::Ref<const Eigen::MatrixXd> & jac,
                            Eigen::MatrixXd & res) const
{
  res.block(0, 0, jac.rows(), mb.nrDof()).setZero();
  addFullJacobian(mb, jac, res);
}

void Jacobian::addFullJacobian(const MultiBody & mb,
                               const Eigen::Ref<const Eigen::MatrixXd> & jac,
                               Eigen::MatrixXd & res) const
{
  int jacPos = 0;
  for(std::size_t index = 0; index < jointsPath_.size(); ++index)
  {
    int i = jointsPath_[index];
    int dof = mb.joint(i).dof();
    res.block(0, mb.jointPosInDof(i), jac.rows(), dof) += jac.block(0, jacPos, jac.rows(), dof);
    jacPos += dof;
  }
}

void Jacobian::addFullJacobian(const Blocks & compactPath,
                               const Eigen::Ref<const Eigen::MatrixXd> & jac,
                               Eigen::MatrixXd & res) const
{
  for(const auto & b : compactPath)
  {
    res.block(0, b.startDof, jac.rows(), b.length) += jac.block(0, b.startJac, jac.rows(), b.length);
  }
}

Eigen::MatrixXd Jacobian::expand(const MultiBody & mb, const Eigen::Ref<const Eigen::MatrixXd> & jac) const
{
  Eigen::MatrixXd res = Eigen::MatrixXd::Zero(mb.nrDof(), mb.nrDof());
  expandAdd(mb, jac, res);
  return res;
}

void Jacobian::expandAdd(const MultiBody & mb,
                         const Eigen::Ref<const Eigen::MatrixXd> & jac,
                         Eigen::MatrixXd & res) const
{
  assert(res.cols() == mb.nrDof() && res.rows() == mb.nrDof());
  int rowJac = 0;
  int colJac = 0;
  for(int i : jointsPath_)
  {
    colJac = 0;
    for(int j : jointsPath_)
    {
      res.block(mb.jointPosInDof(i), mb.jointPosInDof(j), mb.joint(i).dof(), mb.joint(j).dof()).noalias() +=
          jac.block(rowJac, colJac, mb.joint(i).dof(), mb.joint(j).dof());
      colJac += mb.joint(j).dof();
    }
    rowJac += mb.joint(i).dof();
  }
}

Blocks Jacobian::compactPath(const rbd::MultiBody & mb) const
{
  Blocks res;

  int start_block = mb.jointPosInDof(jointsPath_[0]);
  int len_block = mb.joint(jointsPath_[0]).dof();

  int startJac = 0;

  for(std::size_t j = 1; j < jointsPath_.size(); ++j)
  {
    int i = jointsPath_[j];
    int start = mb.jointPosInDof(i);

    if(start != start_block + len_block)
    {
      res.emplace_back(start_block, startJac, len_block);
      start_block = start;
      startJac += len_block;
      len_block = 0;
    }
    len_block += mb.joint(i).dof();
  }
  res.emplace_back(start_block, startJac, len_block);
  return res;
}

void Jacobian::expandAdd(const Blocks & compactPath,
                         const Eigen::Ref<const Eigen::MatrixXd> & jacMat,
                         Eigen::MatrixXd & res) const
{
  for(const auto & b1 : compactPath)
  {
    for(const auto & b2 : compactPath)
    {
      res.block(b1.startDof, b2.startDof, b1.length, b2.length).noalias() +=
          jacMat.block(b1.startJac, b2.startJac, b1.length, b2.length);
    }
  }
}

const Eigen::MatrixXd & Jacobian::sJacobian(const MultiBody & mb,
                                            const MultiBodyConfig & mbc,
                                            const sva::PTransformd & X_0_p)
{
  checkMatchBodyPos(mb, mbc);
  checkMatchMotionSubspace(mb, mbc);

  int m = *std::max_element(jointsPath_.begin(), jointsPath_.end());
  if(m >= static_cast<int>(mb.nrJoints()))
  {
    throw std::domain_error("jointsPath mismatch MultiBody");
  }

  return jacobian(mb, mbc, X_0_p);
}

const Eigen::MatrixXd & Jacobian::sJacobian(const MultiBody & mb, const MultiBodyConfig & mbc)
{
  checkMatchBodyPos(mb, mbc);
  checkMatchMotionSubspace(mb, mbc);

  int m = *std::max_element(jointsPath_.begin(), jointsPath_.end());
  if(m >= static_cast<int>(mb.nrJoints()))
  {
    throw std::domain_error("jointsPath mismatch MultiBody");
  }

  return jacobian(mb, mbc);
}

const Eigen::MatrixXd & Jacobian::sBodyJacobian(const MultiBody & mb, const MultiBodyConfig & mbc)
{
  checkMatchBodyPos(mb, mbc);
  checkMatchMotionSubspace(mb, mbc);

  int m = *std::max_element(jointsPath_.begin(), jointsPath_.end());
  if(m >= static_cast<int>(mb.nrJoints()))
  {
    throw std::domain_error("jointsPath mismatch MultiBody");
  }

  return bodyJacobian(mb, mbc);
}

const Eigen::MatrixXd & Jacobian::sVectorJacobian(const MultiBody & mb,
                                                  const MultiBodyConfig & mbc,
                                                  const Eigen::Vector3d & vec)
{
  checkMatchBodyPos(mb, mbc);
  checkMatchMotionSubspace(mb, mbc);

  int m = *std::max_element(jointsPath_.begin(), jointsPath_.end());
  if(m >= static_cast<int>(mb.nrJoints()))
  {
    throw std::domain_error("jointsPath mismatch MultiBody");
  }

  return vectorJacobian(mb, mbc, vec);
}

const Eigen::MatrixXd & Jacobian::sVectorBodyJacobian(const MultiBody & mb,
                                                      const MultiBodyConfig & mbc,
                                                      const Eigen::Vector3d & vec)
{
  checkMatchBodyPos(mb, mbc);
  checkMatchMotionSubspace(mb, mbc);

  int m = *std::max_element(jointsPath_.begin(), jointsPath_.end());
  if(m >= static_cast<int>(mb.nrJoints()))
  {
    throw std::domain_error("jointsPath mismatch MultiBody");
  }

  return vectorBodyJacobian(mb, mbc, vec);
}

MultiBody Jacobian::sSubMultiBody(const MultiBody & mb) const
{
  int m = *std::max_element(jointsPath_.begin(), jointsPath_.end());
  if(m >= static_cast<int>(mb.nrJoints()))
  {
    throw std::domain_error("jointsPath mismatch MultiBody");
  }

  return subMultiBody(mb);
}

const Eigen::MatrixXd & Jacobian::sJacobianDot(const MultiBody & mb, const MultiBodyConfig & mbc)
{
  checkMatchBodyPos(mb, mbc);
  checkMatchBodyVel(mb, mbc);
  checkMatchMotionSubspace(mb, mbc);

  int m = *std::max_element(jointsPath_.begin(), jointsPath_.end());
  if(m >= static_cast<int>(mb.nrJoints()))
  {
    throw std::domain_error("jointsPath mismatch MultiBody");
  }

  return jacobianDot(mb, mbc);
}

const Eigen::MatrixXd & Jacobian::sBodyJacobianDot(const MultiBody & mb, const MultiBodyConfig & mbc)
{
  checkMatchBodyPos(mb, mbc);
  checkMatchBodyVel(mb, mbc);
  checkMatchMotionSubspace(mb, mbc);

  int m = *std::max_element(jointsPath_.begin(), jointsPath_.end());
  if(m >= static_cast<int>(mb.nrJoints()))
  {
    throw std::domain_error("jointsPath mismatch MultiBody");
  }

  return bodyJacobianDot(mb, mbc);
}

void Jacobian::sTranslateJacobian(const Eigen::MatrixXd & jac,
                                  const MultiBodyConfig & mbc,
                                  const Eigen::Vector3d & point,
                                  Eigen::MatrixXd & res)
{
  if(jointsPath_.back() >= static_cast<int>(mbc.bodyPosW.size()))
  {
    throw std::domain_error("jointsPath mismatch MultiBodyConfig");
  }

  if(jac.cols() != jac_.cols() || jac.rows() != jac_.rows())
  {
    std::ostringstream str;
    str << "jac matrix size mismatch: expected size (" << jac_.rows() << " x " << jac_.cols() << ")"
        << " gived (" << jac.rows() << " x " << jac.cols() << ")";
    throw std::domain_error(str.str());
  }

  if(res.cols() != jac_.cols() || res.rows() != jac_.rows())
  {
    std::ostringstream str;
    str << "res matrix size mismatch: expected size (" << jac_.rows() << " x " << jac_.cols() << ")"
        << " gived (" << res.rows() << " x " << res.cols() << ")";
    throw std::domain_error(str.str());
  }

  translateJacobian(jac, mbc, point, res);
}

void Jacobian::sFullJacobian(const MultiBody & mb, const Eigen::MatrixXd & jac, Eigen::MatrixXd & res) const
{
  int m = *std::max_element(jointsPath_.begin(), jointsPath_.end());
  if(m >= static_cast<int>(mb.nrJoints()))
  {
    throw std::domain_error("jointsPath mismatch MultiBody");
  }

  if(jac.cols() != jac_.cols() || jac.rows() != jac_.rows())
  {
    std::ostringstream str;
    str << "jac matrix size mismatch: expected size (" << jac_.rows() << " x " << jac_.cols() << ")"
        << " gived (" << jac.rows() << " x " << jac.cols() << ")";
    throw std::domain_error(str.str());
  }

  if(res.cols() != mb.nrDof() || res.rows() != 6)
  {
    std::ostringstream str;
    str << "res matrix size mismatch: expected size (" << mb.nrDof() << " x "
        << "6 )"
        << " gived (" << res.rows() << " x " << res.cols() << ")";
    throw std::domain_error(str.str());
  }

  fullJacobian(mb, jac, res);
}

sva::MotionVecd Jacobian::sVelocity(const MultiBody & mb,
                                    const MultiBodyConfig & mbc,
                                    const sva::PTransformd & X_b_p) const
{
  checkMatchBodyPos(mb, mbc);
  checkMatchBodyVel(mb, mbc);

  return velocity(mb, mbc, X_b_p);
}

sva::MotionVecd Jacobian::sVelocity(const MultiBody & mb, const MultiBodyConfig & mbc) const
{
  checkMatchBodyPos(mb, mbc);
  checkMatchBodyVel(mb, mbc);

  return velocity(mb, mbc);
}

sva::MotionVecd Jacobian::sBodyVelocity(const MultiBody & mb, const MultiBodyConfig & mbc) const
{
  checkMatchBodyVel(mb, mbc);

  return bodyVelocity(mb, mbc);
}

sva::MotionVecd Jacobian::sNormalAcceleration(const MultiBody & mb,
                                              const MultiBodyConfig & mbc,
                                              const sva::PTransformd & X_b_p,
                                              const sva::MotionVecd & V_b_p) const
{
  checkMatchBodyPos(mb, mbc);
  checkMatchBodyVel(mb, mbc);
  checkMatchJointConf(mb, mbc);
  checkMatchParentToSon(mb, mbc);

  return normalAcceleration(mb, mbc, X_b_p, V_b_p);
}

sva::MotionVecd Jacobian::sNormalAcceleration(const MultiBody & mb, const MultiBodyConfig & mbc) const
{
  checkMatchBodyPos(mb, mbc);
  checkMatchBodyVel(mb, mbc);
  checkMatchJointConf(mb, mbc);
  checkMatchParentToSon(mb, mbc);

  return normalAcceleration(mb, mbc);
}

sva::MotionVecd Jacobian::sNormalAcceleration(const MultiBody & mb,
                                              const MultiBodyConfig & mbc,
                                              const std::vector<sva::MotionVecd> & normalAccB,
                                              const sva::PTransformd & X_b_p,
                                              const sva::MotionVecd & V_b_p) const
{
  checkMatchBodyPos(mb, mbc);
  checkMatchBodyVel(mb, mbc);
  checkMatchBodiesVector(mb, normalAccB, "normalAccB");

  return normalAcceleration(mb, mbc, normalAccB, X_b_p, V_b_p);
}

sva::MotionVecd Jacobian::sBodyNormalAcceleration(const MultiBody & mb, const MultiBodyConfig & mbc) const
{
  checkMatchJointConf(mb, mbc);
  checkMatchParentToSon(mb, mbc);

  return bodyNormalAcceleration(mb, mbc);
}

sva::MotionVecd Jacobian::sNormalAcceleration(const MultiBody & mb,
                                              const MultiBodyConfig & mbc,
                                              const std::vector<sva::MotionVecd> & normalAccB) const
{
  checkMatchBodyPos(mb, mbc);
  checkMatchBodyVel(mb, mbc);
  checkMatchBodiesVector(mb, normalAccB, "normalAccB");

  return normalAcceleration(mb, mbc, normalAccB);
}

sva::MotionVecd Jacobian::sBodyNormalAcceleration(const MultiBody & mb,
                                                  const MultiBodyConfig & mbc,
                                                  const std::vector<sva::MotionVecd> & normalAccB) const
{
  checkMatchBodiesVector(mb, normalAccB, "normalAccB");

  return bodyNormalAcceleration(mb, mbc, normalAccB);
}

sva::MotionVecd Jacobian::normalAcceleration(const MultiBodyConfig & mbc,
                                             const sva::MotionVecd & bodyNNormalAcc,
                                             const sva::PTransformd & X_b_p,
                                             const sva::MotionVecd & V_b_p) const
{
  const auto N = static_cast<size_t>(jointsPath_.back());
  return X_b_p * bodyNNormalAcc + V_b_p.cross(X_b_p * mbc.bodyVelB[N]);
}

sva::MotionVecd Jacobian::normalAcceleration(const MultiBodyConfig & mbc, const sva::MotionVecd & bodyNNormalAcc) const
{
  const auto N = static_cast<size_t>(jointsPath_.back());
  // equivalent of E_N_0*X_N_Np
  sva::PTransformd X_Np_w(mbc.bodyPosW[N].rotation().transpose(), point_.translation());
  sva::MotionVecd E_VN(mbc.bodyVelW[N].angular(), Eigen::Vector3d::Zero());
  return X_Np_w * bodyNNormalAcc + E_VN.cross(X_Np_w * mbc.bodyVelB[N]);
}

sva::MotionVecd Jacobian::bodyNormalAcceleration(const MultiBodyConfig & /* mbc */,
                                                 const sva::MotionVecd & bodyNNormalAcc) const
{
  return point_ * bodyNNormalAcc;
}

} // namespace rbd
