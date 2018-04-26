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
    VectorXd singVals = SVD.singularValues();

    MatrixXd S = MatrixXd(3,2).setZero();
    S(0,0) = singVals(0);
    S(1,1) = singVals(1);

    //Determinant correction so that we ALWAYS have either BOTH determinants negative or BOTh positive
    if(U.determinant() * V.determinant() == -1){
        U.col(2) *= -1;
    }

    //TODO: in the future, make these variables passed into the constructor!!!
    double strainMin = 0.97;
    double strainMax = 1.03;

    //Clamp singular values to be between strainMin and strainMax
    if(singVals(0) > strainMax){
        singVals(0) = strainMax;
    }
    if(singVals(0) < strainMin){
        singVals(0) = strainMin;
    }
    if(singVals(1) < strainMin){
        singVals(1) = strainMin;
    }
    if(singVals(1) > strainMax){
        singVals(1) = strainMax;
    }

    //reset S to have the new singular values
    S(0,0) = singVals(0);
    S(1,1) = singVals(1);

    MatrixXd Vt = V.transpose();

    /*/-------------------------------------//
    //CHECK THAT SVD IS CORRECT!!
    Eigen::MatrixXd I3 = Eigen::MatrixXd(3, 3).setIdentity();
    Eigen::MatrixXd I2 = Eigen::MatrixXd(2, 2).setIdentity();
    MatrixXd temp22 = V * V.transpose() - I2;
    MatrixXd temp33 = U * U.transpose() - I3;
    MatrixXd temp32 = MatrixXd(3,2).setZero();
    if(temp33(0,0) >= 1e-5){ cout << "F SVD ERROR: U * Ut \n" << endl; cout << "U*Ut: \n" << U*U.transpose() << endl;}
    if(temp33(0,1) >= 1e-5){ cout << "F SVD ERROR: U * Ut \n" << endl; cout << "U*Ut: \n" << U*U.transpose() << endl;}
    if(temp33(0,2) >= 1e-5){ cout << "F SVD ERROR: U * Ut \n" << endl; cout << "U*Ut: \n" << U*U.transpose() << endl;}
    if(temp33(1,0) >= 1e-5){ cout << "F SVD ERROR: U * Ut \n" << endl; cout << "U*Ut: \n" << U*U.transpose() << endl;}
    if(temp33(1,1) >= 1e-5){ cout << "F SVD ERROR: U * Ut \n" << endl; cout << "U*Ut: \n" << U*U.transpose() << endl;}
    if(temp33(1,2) >= 1e-5){ cout << "F SVD ERROR: U * Ut \n" << endl; cout << "U*Ut: \n" << U*U.transpose() << endl;}
    if(temp33(2,0) >= 1e-5){ cout << "F SVD ERROR: U * Ut \n" << endl; cout << "U*Ut: \n" << U*U.transpose() << endl;}
    if(temp33(2,1) >= 1e-5){ cout << "F SVD ERROR: U * Ut \n" << endl; cout << "U*Ut: \n" << U*U.transpose() << endl;}
    if(temp33(2,2) >= 1e-5){ cout << "F SVD ERROR: U * Ut \n" << endl; cout << "U*Ut: \n" << U*U.transpose() << endl;}

    if(temp22(0,0) >= 1e-5){ cout << "F SVD ERROR: V * Vt \n" << endl; cout << "V*Vt: \n" << V*V.transpose() << endl;}
    if(temp22(0,1) >= 1e-5){ cout << "F SVD ERROR: V * Vt \n" << endl; cout << "V*Vt: \n" << V*V.transpose() << endl;}
    if(temp22(1,0) >= 1e-5){ cout << "F SVD ERROR: V * Vt \n" << endl; cout << "V*Vt: \n" << V*V.transpose() << endl;}
    if(temp22(1,1) >= 1e-5){ cout << "F SVD ERROR: V * Vt \n" << endl; cout << "V*Vt: \n" << V*V.transpose() << endl;}

    temp32 = U * S * V.transpose() - F;
    if(temp32(0,0) >= 1e-5){ cout << "F SVD ERROR: USVt - F \n" << endl; cout << "USVt: \n" << U*S*V.transpose() << endl; cout << "F: \n" << F << endl;}
    if(temp32(0,1) >= 1e-5){ cout << "F SVD ERROR: USVt - F \n" << endl; cout << "USVt: \n" << U*S*V.transpose() << endl; cout << "F: \n" << F << endl;}
    if(temp32(1,0) >= 1e-5){ cout << "F SVD ERROR: USVt - F \n" << endl; cout << "USVt: \n" << U*S*V.transpose() << endl; cout << "F: \n" << F << endl;}
    if(temp32(1,1) >= 1e-5){ cout << "F SVD ERROR: USVt - F \n" << endl; cout << "USVt: \n" << U*S*V.transpose() << endl; cout << "F: \n" << F << endl;}
    if(temp32(2,0) >= 1e-5){ cout << "F SVD ERROR: USVt - F \n" << endl; cout << "USVt: \n" << U*S*V.transpose() << endl; cout << "F: \n" << F << endl;}
    if(temp32(2,1) >= 1e-5){ cout << "F SVD ERROR: USVt - F \n" << endl; cout << "USVt: \n" << U*S*V.transpose() << endl; cout << "F: \n" << F << endl;}

    if(singVals(0) < abs(singVals(1))){
        cout << "F SVD ERROR: sigma1 > |sigma2| \n" << endl;
        cout << "Sigma1: \n" << singVals(0) << endl;
        cout << "|Sigma2|: \n" << abs(singVals(1)) << endl;
    }
    if(abs(U.determinant() - 1) >= 1e-5){
        cout << "F SVD ERROR: det(U) \n" << endl;
    }
    if(abs(V.determinant() - 1) >= 1e-5){
        cout << "F SVD ERROR: det(V) \n" << endl;
    }
    //-------------------------------------/*/

    MatrixXd BpMat = MatrixXd(3,2).setZero();
    BpMat = U * S * Vt;

    //cout << "S(after): \n" << S << endl;
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
