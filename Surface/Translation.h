// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 - 2026, the Anboto author and contributors
#include <Core/Core.h>
#include "Surface.h"

namespace Upp {

template <typename TT>
Eigen::Matrix<TT, 6, 6> MakeTransferMatrix(const Eigen::Matrix<TT, 3, 1>& from, const Eigen::Matrix<TT, 3, 1>& to) {
	Eigen::Matrix<TT, 3, 1> r = from - to;
	
    Eigen::Matrix<TT, 3, 3> R;		// Skew-symmetric matrix
    R <<     0.,  r(2), -r(1),
          -r(2),    0.,  r(0),
           r(1), -r(0),    0.;

    Eigen::Matrix<TT, 6, 6> T = Eigen::Matrix<TT, 6, 6>::Identity();
    T.template block<3, 3>(3, 0) = R;   			// lower-left 3x3 block

    return T;
}

// Experimental translation algorithm for stiffness and linear damping matrices
template <typename TT, typename Derived>
Eigen::Matrix<TT, 6, 6> TranslateMatrix6(const Eigen::MatrixBase<Derived>& M_A, const Eigen::Matrix<TT, 3, 1>& from, const Eigen::Matrix<TT, 3, 1>& to) {
    ASSERT_(Derived::RowsAtCompileTime == 6 || Derived::RowsAtCompileTime == Eigen::Dynamic, "M_A must be 6x6");
    ASSERT_(Derived::ColsAtCompileTime == 6 || Derived::ColsAtCompileTime == Eigen::Dynamic, "M_A must be 6x6");
    ASSERT(M_A.rows() == 6 && M_A.cols() == 6);

    const Eigen::Matrix<TT, 6, 6> T = MakeTransferMatrix<TT>(from, to);
    return T.transpose() * M_A.template cast<TT>() * T;
}
	
}