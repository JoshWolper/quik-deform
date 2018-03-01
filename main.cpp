//Projective Dynamics solver main function
//Standalone app for QuikDeform Maya plugin
//Developed by Josh Wolper and Yichen Shou

#include "Eigen/Eigen"
#include "global.h"
#include <iostream>
//#include <omp.h>
#include "QuikDeformer.h"
#include <chrono> // for testing function speed

using namespace Eigen;
using namespace std;

long long testFunc(int start, int end){
    long long sum = 0;
    for(int i = start; i < end; i++){
        i * i * i / i;
        sum += i;
    }
    return sum;
}


void ompTest(){
    // no parallel
    long top = 100000000000;
    long top1 = top / 4;
    long top2 = top / 2;
    long top3 = top / 4 * 3;
    chrono::high_resolution_clock::time_point t1 = chrono::high_resolution_clock::now();
    long long result = testFunc(0, top);
    chrono::high_resolution_clock::time_point t2 = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::microseconds>( t2 - t1 ).count();
    cout << "no parallel duration: " << duration << ", result: " << result << endl;

    // with parallel
    chrono::high_resolution_clock::time_point t3 = chrono::high_resolution_clock::now();
    long long result2 = 0;
#pragma omp parallel sections // starts a new team
    {
        { result2 += testFunc(0, top1); }
#pragma omp section
        { result2 += testFunc(top1, top2); }
#pragma omp section
        { result2 += testFunc(top2, top3); }
#pragma omp section
        { result2 += testFunc(top3, top); }
    }
    chrono::high_resolution_clock::time_point t4 = chrono::high_resolution_clock::now();
    auto duration2 = chrono::duration_cast<chrono::microseconds>( t4 - t3 ).count();
    cout << "with parallel duration: " << duration2 << ", result: " << result << endl;
}


int main(){

    //Define our variables
    double h = 1e-4; //timestep
    int iter = 5; // solverIterations
    double mass = 1; //mass of each point
    int fr = 60; // frame rate
    double vx = 0;
    double vy = 0; //init initial velocity direction and magnitude
    double vz = 0;
    string objectFile = "../Models/tetrahedron.obj";

    double seconds = 3;
    string outputFilepath = "../Output/";

    QuikDeformer quikDeformer(objectFile, h, iter, fr, mass, vx, vy, vz);
    quikDeformer.runSimulation(seconds, outputFilepath);

    //quikDeformer.addConstraint("strain");
    //quikDeformer.printMatrices();

    // openMP test:
    //ompTest();

    return 0;
}