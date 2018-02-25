//Projective Dynamics solver main function
//Standalone app for QuikDeform Maya plugin
//Developed by Josh Wolper and Yichen Shou

#include "Eigen/Eigen"
#include "global.h"
#include <iostream>
#include "QuikDeformer.h"

using namespace Eigen;
using namespace std;

int main(){

    //Define our variables
    double h = 1e-4; //timestep
    int iter = 5; // solverIterations
    double mass = 1.0; //mass of each point
    int fr = 24; // frame rate
    Vector3d gravity = Vector3d(0.0, -9.8, 0.0); //gravity vector for external force later
    string objectFile = "../Models/tetrahedron.obj";

    QuikDeformer quikDeformer(objectFile, h, iter, fr, mass);
    quikDeformer.addConstraint("strain");
    quikDeformer.printMatrices();

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