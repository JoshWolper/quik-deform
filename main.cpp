//Projective Dynamics solver main function
//Standalone app for QuikDeform Maya plugin
//Developed by Josh Wolper and Yichen Shou

#include "Eigen/Eigen"
#include "global.h"
#include <iostream>
#include "QuikDeformer.h"
#include <chrono> // for testing function speed
#include "OBJGenerator.h"

using namespace Eigen;
using namespace std;

int main(){

    //SIM PARAMETERS
    double h = 1e-4; //timestep
    int iter = 5; // solverIterations
    double mass = 1; //mass of each point
    int fr = 60; // frame rate
    double vx = 0;
    double vy = 0; //init initial velocity direction and magnitude
    double vz = 0;
    bool gravityOn = true;
    bool printsOn = false;
    bool volumetric = true; //whether this is a thin shell or volumetric
    double seconds = 4; //how long to run sim
    string outputFilepath = "../Output/";

    //WIND PARAMETERS
    bool windOn = false;
    double wx = 1; //wind direction
    double wy = 0;
    double wz = 0;
    double windMag = 1.5; //wind magnitude
    bool windOsc = false; //whether the wind oscillates or is constant

    //Weight PARAMETERS
    double E = 500000;
    double nu = 0.3;
    double lame_lambda = E * nu / (((double)1 + nu) * ((double)1 - (double)2 * nu));
    double lame_mu = E / ((double)2 * ((double)1 + nu));

    double tetStrainWeight = 2 * lame_mu;
    double volumeWeight = 3 * lame_lambda;


    cout << "Strain Weight is: " << tetStrainWeight << endl;

    string objectFile = "../Models/tetrahedron.obj";
    //string objectFile = "../Models/cube.obj";

    /* //Uncomment this chunk if we want to use the cloth generator to make our input object file!
    string objectFile = "../Models/cloth.obj";
    double vertexDist = 0.5;
    int width = 5;
    int height = 5;
    OBJGeneratorMode mode = OBJGeneratorMode::vertical;
    double startHeight = 3;
    OBJGenerator::generateClothOBJ(objectFile, vertexDist, width, height, mode, startHeight); //make the cloth to be used
    */

    /* Use these files for testing high poly stuff! 600 tets
    string nodeFile = "../tetgen1.5.1-beta1/example.1.node";
    string eleFile = "../tetgen1.5.1-beta1/example.1.ele";
    string faceFile = "../tetgen1.5.1-beta1/example.1.face";
    */

    //Tet with 1 tet
    string nodeFile = "../Models/tet.1.node";
    string eleFile = "../Models/tet.1.ele";
    string faceFile = "../Models/tet.1.face";

    //Cube with 6 tets
    //string nodeFile = "../Models/cube.1.node";
    //string eleFile = "../Models/cube.1.ele";
    //string faceFile = "../Models/cube.1.face";

    //QuikDeformer quikDeformer(objectFile, h, iter, fr, mass, vx, vy, vz, gravityOn, volumetric);
    QuikDeformer quikDeformer(nodeFile, eleFile, faceFile, h, iter, fr, mass, vx, vy, vz, gravityOn, volumetric);

    quikDeformer.printMatrices();

    //------ADD POSITION CONSTRAINTS------//
    //int posConstraintIndex = 0;
    //double posConstraintW = 100000;
    //quikDeformer.addPositionConstraint(posConstraintW, posConstraintIndex);

    //posConstraintIndex = 5;
    //posConstraintW = 100000;
    //quikDeformer.addPositionConstraint(posConstraintW, posConstraintIndex);


    //------ADD STRAIN CONSTRAINTS--------//
    if(volumetric) { quikDeformer.add3DStrainConstraints(tetStrainWeight); }//go through mesh and find all tets, add a constraint for each one!


    //------ADD WIND EFFECTS--------------//
    if(windOn){
        quikDeformer.addWind(wx, wy, wz, windMag, windOsc);
    }


    //-------RANDOMIZE VERTICES TEST----------//
    //quikDeformer.randomizeVertices();


    //---------RUN SIMULATION--------------//
    //Run the simulation!
    quikDeformer.runSimulation(seconds, outputFilepath, printsOn);

    return 0;
}