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
    double h = 1e-2; //timestep
    int iter = 1; // solverIterations
    double mass = 1; //mass of each point
    int fr = 24; // frame rate
    double vx = 0;
    double vy = 0; //init initial velocity direction and magnitude
    double vz = 0;
    bool printsOn = false;
    double seconds = 10; //how long to run sim
    string outputFilepath = "../Output/";

    //WIND PARAMETERS
    double wx = 1; //wind direction
    double wy = 0;
    double wz = 0;
    double windMag = 10; //wind magnitude
    double windAmp = 1;
    double windPeriod = 0.5;
    bool windOsc = true; //whether the wind oscillates or is constant

    //Weight PARAMETERS
    double E = 700;
    double nu = 0.3;
    double lame_lambda = E * nu / (((double)1 + nu) * ((double)1 - (double)2 * nu));
    double lame_mu = E / ((double)2 * ((double)1 + nu));

    double tetStrainWeight = 2 * lame_mu;
    double triangleStrainWeight = tetStrainWeight;
    double volumeWeight = 3 * lame_lambda;

    cout << "Strain Weight is: " << tetStrainWeight << endl;

    string objectFile;
    string nodeFile;
    string eleFile;
    string faceFile;
    int indexBase;

    //CHANGE THESE TWO PARAMS WHEN CHANGING MODELS!!!
    int whichObject = 5;
    bool gravityOn = true;
    bool volumetric = false; //whether this is a thin shell or volumetric
    bool windOn = false;

    if(whichObject == 0){
        //Tet with 1 tet --> INDEX BASE IS 1
        cout << "Processing volumetric tetrahedron model:" << endl;
        indexBase = 1;
        nodeFile = "../Models/tet.1.node";
        eleFile = "../Models/tet.1.ele";
        faceFile = "../Models/tet.1.face";
    } else if (whichObject == 1){
        //Cube with 6 tets --> INDEX BASE IS 1
        cout << "Processing volumetric cube model:" << endl;
        indexBase = 1;
        nodeFile = "../Models/cube.1.node";
        eleFile = "../Models/cube.1.ele";
        faceFile = "../Models/cube.1.face";
    } else if (whichObject == 2){
        //Icosahedron with 16 tets --> INDEX BASE IS 0
        cout << "Processing volumetric icosahedron model:" << endl;
        indexBase = 0;
        nodeFile = "../Models/icosahedron.1.node";
        eleFile = "../Models/icosahedron.1.ele";
        faceFile = "../Models/icosahedron.1.face";
    } else if (whichObject == 3){
        //Tetgen weird box shape 600 tets --> INDEX BASE IS 1
        cout << "Processing volumetric box with hole model:" << endl;
        indexBase = 1;
        nodeFile = "../tetgen1.5.1-beta1/example.1.node";
        eleFile = "../tetgen1.5.1-beta1/example.1.ele";
        faceFile = "../tetgen1.5.1-beta1/example.1.face";
    } else if (whichObject == 4){
        //Sphere with 2064 tets --> INDEX BASE IS 0
        cout << "Processing volumetric sphere model:" << endl;
        indexBase = 0;
        nodeFile = "../Models/sphere.1.node";
        eleFile = "../Models/sphere.1.ele";
        faceFile = "../Models/sphere.1.face";
    } else if(whichObject == 5) {
        cout << "Processing cloth model:" << endl;
        indexBase = 1;
        objectFile = "../Models/cloth.obj";
        double vertexDist = 0.5;
        int width = 10;
        int height = 10;
        OBJGeneratorMode mode = OBJGeneratorMode::vertical;
        double startHeight = 10;
        OBJGenerator::generateClothOBJ(objectFile, vertexDist, width, height, mode, startHeight); //make the cloth to be used
    } else if(whichObject == 6){
        cout << "Processing single triangle model:" << endl;
        indexBase = 1;
        objectFile = "../Models/triangle.obj";
    } else if(whichObject == 7){
        cout << "Processing square model (2 tris):" << endl;
        indexBase = 1;
        objectFile = "../Models/square.obj";
    } else {
        cout << "Failed to choose a case so basic tetrahedron selected" << endl;
        //Tet with 1 tet --> INDEX BASE IS 1
        indexBase = 1;
        nodeFile = "../Models/tet.1.node";
        eleFile = "../Models/tet.1.ele";
        faceFile = "../Models/tet.1.face";
    }

    //---------DEFINE COLLISION PLANES------------//

    //Define some collision planes
    vector<Vector3d> pCenters;
    vector<double> pLengths;
    vector<double> pWidths;
    vector<Vector3d> pNormals;

    //Plane centers
    pCenters.push_back(Vector3d(0,0,0)); //ground plane
    //pCenters.push_back(Vector3d(5,0,0)); //right wall plane (90 deg)

    //Plane normals
    pNormals.push_back(Vector3d(0,1,0));
    //pNormals.push_back(Vector3d(-1,0,0));

    //---------CONSTRUCT AND RUN SIMULATOR-----------//

    if(volumetric){

        //---------CONSTRUCTOR--------//
        QuikDeformer quikDeformer(nodeFile, eleFile, faceFile, pCenters, pLengths, pWidths, pNormals, h, iter, fr, mass, vx, vy, vz, gravityOn, volumetric, indexBase);

        //------PRINT MATRICES-------//
        //quikDeformer.printMatrices();

        //-----ADD STRAIN CONSTRAINTS-------//
        quikDeformer.add3DStrainConstraints(tetStrainWeight);

        //------ADD POSITION CONSTRAINTS------//
        //int posConstraintIndex = 0;
        //double posConstraintW = 100000;
        //quikDeformer.addPositionConstraint(posConstraintW, posConstraintIndex);

        //------ADD WIND EFFECTS--------------//
        if(windOn){
            quikDeformer.addWind(wx, wy, wz, windMag, windOsc, windAmp, windPeriod);
        }

        //-------RANDOMIZE VERTICES TEST----------//
        //quikDeformer.randomizeVertices();

        //---------RUN SIMULATION--------------//
        quikDeformer.runSimulation(seconds, outputFilepath, printsOn);

    }
    else if(!volumetric){

        //---------CONSTRUCTOR--------//
        QuikDeformer quikDeformer(objectFile, pCenters, pLengths, pWidths, pNormals, h, iter, fr, mass, vx, vy, vz, gravityOn, volumetric, indexBase);

        //------PRINT MATRICES-------//
        //quikDeformer.printMatrices();

        //-----ADD STRAIN CONSTRAINTS-------//
        quikDeformer.add2DStrainConstraints(triangleStrainWeight);

        //------ADD POSITION CONSTRAINTS------//
        double posConstraintW = 100000;
        quikDeformer.addPositionConstraint(posConstraintW, 0);
        quikDeformer.addPositionConstraint(posConstraintW, 11);
        quikDeformer.addPositionConstraint(posConstraintW, 22);
        quikDeformer.addPositionConstraint(posConstraintW, 33);
        quikDeformer.addPositionConstraint(posConstraintW, 44);
        quikDeformer.addPositionConstraint(posConstraintW, 55);
        quikDeformer.addPositionConstraint(posConstraintW, 66);
        quikDeformer.addPositionConstraint(posConstraintW, 77);
        quikDeformer.addPositionConstraint(posConstraintW, 88);
        quikDeformer.addPositionConstraint(posConstraintW, 99);
        quikDeformer.addPositionConstraint(posConstraintW, 110);

        //------ADD WIND EFFECTS--------------//
        if(windOn){
            quikDeformer.addWind(wx, wy, wz, windMag, windOsc, windAmp, windPeriod);
        }

        //-------RANDOMIZE VERTICES TEST----------//
        //quikDeformer.randomizeVertices();

        //---------RUN SIMULATION--------------//
        quikDeformer.runSimulation(seconds, outputFilepath, printsOn);
    }

    return 0;
}