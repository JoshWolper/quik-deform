//Projective Dynamics solver main function
//Standalone app for QuikDeform Maya plugin
//Developed by Josh Wolper and Yichen Shou

#include "Eigen/Eigen"
#include <iostream>

using namespace Eigen;
using namespace std;

int main(){

    //Define our variables
    double h = 1e-4; //timestep
    double mass = 1.0;

    Vector3d gravity = Vector3d(0.0, -9.8, 0.0);

    int numPoints = 5; //eventually dynamically fill this

    //Set the diagonal mass matrix
    MatrixXd iden(numPoints, numPoints);
    iden = MatrixXd::Identity(numPoints, numPoints);
    MatrixXd M = iden * mass; //underlined as an error but this actually works lol

    cout << "Our mass matrix M is: " << endl << M << endl;



    //Cholesky example (take matrix A, decompose into L * L.transpose, we need L later)
    Matrix3d A = Matrix3d::Zero();
    A << 4, -1, 2, -1, 6, 0, 2, 0, 5; //must make sure the matrix A is positive definite to ensure Cholesky is stable

    cout << "The matrix A is" << endl << A << endl;

    LLT<Matrix3d, Lower> lltOfA(A); // compute the Cholesky decomposition of A
    Matrix3d L = lltOfA.matrixL(); // retrieve factor L  in the decomposition
    // The previous two lines can also be written as "L = A.llt().matrixL()"

    cout << "The Cholesky factor L is" << endl << L << endl;
    cout << "To check this, let us compute L * L.transpose()" << endl;
    cout << L * L.transpose() << endl;
    cout << "This should equal the matrix A" << endl;

    return 0;
}