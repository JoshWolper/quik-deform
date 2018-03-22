//
// SVD Routine written for an MPM simulation project (Fall 2017) by Yaoyi Bai, Ziyin Qu, and Josh Wolper
//

#pragma once
#ifndef QUIKDEFORM_SVD_H
#define QUIKDEFORM_SVD_H

#include "Eigen/Eigen"

#include "global.h"

using namespace Eigen;

SVDResult SingularValueDecomposition3D(Matrix3f F);

SVDResultDouble SingularValueDecomposition3DDouble(Matrix3d F);

#endif //QUIKDEFORM_SVD_H
