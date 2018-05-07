#ifndef CORIOLIS_H_KED6CAHK
#define CORIOLIS_H_KED6CAHK

#include <RBDyn/MultiBodyConfig.h>
#include <RBDyn/CoM.h>
#include <rbdyn/config.hh>

namespace rbd
{

using Block = std::array<int, 3>;
using Blocks = std::vector<Block>;

RBDYN_DLLAPI Eigen::MatrixXd expand(const rbd::Jacobian& jac,
			const rbd::MultiBody& mb,
			const Eigen::MatrixXd& jacMat);

RBDYN_DLLAPI void expandAdd(const rbd::Jacobian& jac,
		const rbd::MultiBody& mb,
		const Eigen::MatrixXd& jacMat,
		Eigen::MatrixXd& res);

RBDYN_DLLAPI Blocks compactPath(const rbd::Jacobian& jac,
		const rbd::MultiBody& mb);

RBDYN_DLLAPI void compactExpandAdd(const Blocks& compactPath,
			const Eigen::MatrixXd& jacMat,
			Eigen::MatrixXd& res);

class RBDYN_DLLAPI Coriolis
{
	public:
		Coriolis(const rbd::MultiBody& mb);

		Eigen::MatrixXd coriolis(const rbd::MultiBody& mb, const rbd::MultiBodyConfig& mbc);

	private:
		std::vector<rbd::Jacobian> jacs_;
		std::vector<Eigen::MatrixXd> jacMats_;
		std::vector<Eigen::MatrixXd> jacDotMats_;
		std::vector<Blocks> compactPaths_;
};

} // ns coriolis

#endif /* end of include guard: CORIOLIS_H_KED6CAHK */
