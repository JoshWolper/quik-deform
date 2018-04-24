//
// Created by Josh Wolper on 3/22/18.
//

#include "TriangleStrainConstraint.h"

using namespace Eigen;
using namespace std;

// local step. solve for the set of project current points that will satisfy this constraint
void TriangleStrainConstraint::projectConstraint(Eigen::VectorXd qN_1){

    VectorXd pTemp = VectorXd(6).setZero();

    //Compute Ds so we can compute F_current
    MatrixXd Ds = MatrixXd(3,2).setZero();
    std::vector<int> currTriangle = getIndeces();

    int id0 = 3 * (currTriangle[0]); //might have to subtract 1 if NOT 0-indexed
    int id1 = 3 * (currTriangle[1]);
    int id2 = 3 * (currTriangle[2]);

    Ds(0,0) = qN_1(id1 + 0) - qN_1(id0 + 0); //top left = X1.x - X0.x
    Ds(1,0) = qN_1(id1 + 1) - qN_1(id0 + 1); //middle left = X1.y - X0.y
    Ds(2,0) = qN_1(id1 + 2) - qN_1(id0 + 2); //bottom left = X1.z - X0.z

    Ds(0,1) = qN_1(id2 + 0) - qN_1(id0 + 0); //top right = X2.x - X0.x
    Ds(1,1) = qN_1(id2 + 1) - qN_1(id0 + 1); //middle right = X2.y - X0.y
    Ds(2,1) = qN_1(id2 + 2) - qN_1(id0 + 2); //bottom right = X2.z - X0.z

    setDs(Ds);

    //Get DmInverse
    MatrixXd Dminv = getDmInv();

    //Compute F
    MatrixXd F = MatrixXd(3,2).setZero();
    F = Ds * Dminv;

    //Now that we have computed F, take the SVD of it!
    JacobiSVD<MatrixXd> SVD( F, ComputeFullV | ComputeFullU );
    MatrixXd U = SVD.matrixU();
    MatrixXd V = SVD.matrixV();

    //Check if U is rotation or not!
    if(U.determinant() < 0){
        U.col(1) *= -1;
    }

    //Check if V is rotation or not!
    if(V.determinant() < 0){
        V.col(1) *= -1;
    }

    MatrixXd S = MatrixXd(3,2).setZero();
    S(0,0) = 1;
    S(1,1) = 1;

    MatrixXd Vt = V.transpose();

    /*/-------------------------------------//
    //CHECK THAT SVD IS CORRECT!!
    Eigen::MatrixXd I2 = Eigen::MatrixXd(2, 2).setIdentity();
    MatrixXd temp22 = U * U.transpose() - I2;
    if(temp22(0,0) >= 1e-5){ cout << "Rhat SVD ERROR: U * Ut \n" << endl; cout << "U*Ut: \n" << U*U.transpose() << endl;}
    if(temp22(0,1) >= 1e-5){ cout << "Rhat SVD ERROR: U * Ut \n" << endl; cout << "U*Ut: \n" << U*U.transpose() << endl;}
    if(temp22(1,0) >= 1e-5){ cout << "Rhat SVD ERROR: U * Ut \n" << endl; cout << "U*Ut: \n" << U*U.transpose() << endl;}
    if(temp22(1,1) >= 1e-5){ cout << "Rhat SVD ERROR: U * Ut \n" << endl; cout << "U*Ut: \n" << U*U.transpose() << endl;}
    temp22 = V * V.transpose() - I2;
    if(temp22(0,0) >= 1e-5){ cout << "Rhat SVD ERROR: V * Vt \n" << endl; cout << "V*Vt: \n" << V*V.transpose() << endl;}
    if(temp22(0,1) >= 1e-5){ cout << "Rhat SVD ERROR: V * Vt \n" << endl; cout << "V*Vt: \n" << V*V.transpose() << endl;}
    if(temp22(1,0) >= 1e-5){ cout << "Rhat SVD ERROR: V * Vt \n" << endl; cout << "V*Vt: \n" << V*V.transpose() << endl;}
    if(temp22(1,1) >= 1e-5){ cout << "Rhat SVD ERROR: V * Vt \n" << endl; cout << "V*Vt: \n" << V*V.transpose() << endl;}
    temp22 = U * S * V.transpose() - Rhat;
    if(temp22(0,0) >= 1e-5){ cout << "Rhat SVD ERROR: USVt - Rhat \n" << endl; cout << "USVt: \n" << U*S*V.transpose() << endl; cout << "Rhat: \n" << Rhat << endl;}
    if(temp22(0,1) >= 1e-5){ cout << "Rhat SVD ERROR: USVt - Rhat \n" << endl; cout << "USVt: \n" << U*S*V.transpose() << endl; cout << "Rhat: \n" << Rhat << endl;}
    if(temp22(1,0) >= 1e-5){ cout << "Rhat SVD ERROR: USVt - Rhat \n" << endl; cout << "USVt: \n" << U*S*V.transpose() << endl; cout << "Rhat: \n" << Rhat << endl;}
    if(temp22(1,1) >= 1e-5){ cout << "Rhat SVD ERROR: USVt - Rhat \n" << endl; cout << "USVt: \n" << U*S*V.transpose() << endl; cout << "Rhat: \n" << Rhat << endl;}
    if(singVals(0) < abs(singVals(1))){
        cout << "Rhat SVD ERROR: sigma1 > |sigma2| \n" << endl;
        cout << "Sigma1: \n" << singVals(0) << endl;
        cout << "|Sigma2|: \n" << abs(singVals(1)) << endl;
    }
    if(U.determinant() - 1 >= 1e-5 || U.determinant() == -1){
        cout << "Rhat SVD ERROR: det(U) \n" << endl;
    }
    if(V.determinant() - 1 >= 1e-5 || V.determinant() == -1){
        cout << "Rhat SVD ERROR: det(V) \n" << endl;
    }
    //-------------------------------------/*/

    MatrixXd BpMat = MatrixXd(3,2).setZero();
    BpMat = U * S * Vt;

    //cout << "Bp = UVt: \n" << BpMat << endl;

    //Set Bp to be the elements of Q * newR
    pTemp(0) = BpMat(0,0); //top left
    pTemp(1) = BpMat(0,1); //top right
    pTemp(2) = BpMat(1,0); //middle left
    pTemp(3) = BpMat(1,1); //middle right
    pTemp(4) = BpMat(2,0); //bottom left
    pTemp(5) = BpMat(2,1); //bottom right

    setP(pTemp); //set the p

    return;
}
