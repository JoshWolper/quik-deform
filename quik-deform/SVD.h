//
// SVD Routine written for an MPM simulation project (Fall 2017) by Yaoyi Bai, Ziyin Qu, and Josh Wolper
//

#pragma once
#ifndef MPM_SVD_H
#define MPM_SVD_H

#include "Eigen/Eigen"

#include "global.h"

using namespace Eigen;

SVDResult SingularValueDecomposition3D(Matrix3f F);

SVDResultDouble SingularValueDecomposition3DDouble(Matrix3d F);

#endif //MPM_SVD_H
