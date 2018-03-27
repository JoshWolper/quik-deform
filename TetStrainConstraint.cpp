//
// Created by Josh Wolper on 3/22/18.
//

#include "TetStrainConstraint.h"

using namespace Eigen;
using namespace std;

// local step. solve for the set of project current points that will satisfy this constraint
void TetStrainConstraint::projectConstraint(Eigen::VectorXd qN_1){

    //cout << "Solving tet strain constraint! " << endl;

    //Want Bp = UV.transpose, so need to compute F and then take the SVD

    VectorXd pTemp = VectorXd(9);

    //Compute Ds
    MatrixXd Ds = MatrixXd(3,3).setZero();
    std::vector<int> currTet = getIndeces();

    int id0 = 3 * (currTet[0] - 1); //might have to subtract 1 if NOT 0-indexed
    int id1 = 3 * (currTet[1] - 1);
    int id2 = 3 * (currTet[2] - 1);
    int id3 = 3 * (currTet[3] - 1);

    Ds(0,0) = qN_1(id1 + 0) - qN_1(id0 + 0); //top left = X1.x - X0.x
    Ds(1,0) = qN_1(id1 + 1) - qN_1(id0 + 1); //middle left = X1.y - X0.y
    Ds(2,0) = qN_1(id1 + 2) - qN_1(id0 + 2); //bottom left = X1.z - X0.z

    Ds(0,1) = qN_1(id2 + 0) - qN_1(id0 + 0); //top middle = X2.x - X0.x
    Ds(1,1) = qN_1(id2 + 1) - qN_1(id0 + 1); //middle middle = X2.y - X0.y
    Ds(2,1) = qN_1(id2 + 2) - qN_1(id0 + 2); //bottom middle = X2.z - X0.z

    Ds(0,2) = qN_1(id3 + 0) - qN_1(id0 + 0); //top right = X3.x - X0.x
    Ds(1,2) = qN_1(id3 + 1) - qN_1(id0 + 1); //middle right = X3.y - X0.y
    Ds(2,2) = qN_1(id3 + 2) - qN_1(id0 + 2); //bottom right = X3.z - X0.z

    setDs(Ds);

    //Get DmInverse
    MatrixXd Dminv = getDmInv();

    //Compute F
    Matrix3d F = Ds * Dminv;
    setDefGrad(F); //set the def grad

    //Compute SVD of F
    SVDResultDouble svd = SingularValueDecomposition3DDouble(F);
    Matrix3d U = svd.U;
    Matrix3d V = svd.V;
    Matrix3d UVt = U * V.transpose();

    //Set entries of pTemp = entries of UV.transpose but as a column vector
    pTemp(0) = UVt(0,0);
    pTemp(1) = UVt(0,1);
    pTemp(2) = UVt(0,2);
    pTemp(3) = UVt(1,0);
    pTemp(4) = UVt(1,1);
    pTemp(5) = UVt(1,2);
    pTemp(6) = UVt(2,0);
    pTemp(7) = UVt(2,1);
    pTemp(8) = UVt(2,2);

    setP(pTemp); //set the p

    return;
}
