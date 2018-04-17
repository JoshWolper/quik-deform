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
    int iter = 5; // solverIterations
    double mass = 1; //mass of each point
    int fr = 24; // frame rate
    double vx = 0;
    double vy = 0; //init initial velocity direction and magnitude
    double vz = 0;
    bool gravityOn = true;
    bool printsOn = false;
    bool volumetric = false; //whether this is a thin shell or volumetric
    double seconds = 10; //how long to run sim
    string outputFilepath = "../Output/";

    //WIND PARAMETERS
    bool windOn = false;
    double wx = 1; //wind direction
    double wy = 0;
    double wz = 0;
    double windMag = 4; //wind magnitude
    bool windOsc = false; //whether the wind oscillates or is constant

    //Weight PARAMETERS
    double E = 50000;
    double nu = 0.3;
    double lame_lambda = E * nu / (((double)1 + nu) * ((double)1 - (double)2 * nu));
    double lame_mu = E / ((double)2 * ((double)1 + nu));

    double tetStrainWeight = 2 * lame_mu;
    double triangleStrainWeight = tetStrainWeight;
    double volumeWeight = 3 * lame_lambda;

    cout << "Strain Weight is: " << tetStrainWeight << endl;

    //string objectFile = "../Models/tetrahedron.obj";
    //string objectFile = "../Models/cube.obj";

    //Uncomment this chunk if we want to use the cloth generator to make our input object file!
    string objectFile = "../Models/cloth.obj";
    double vertexDist = 0.5;
    int width = 5;
    int height = 5;
    OBJGeneratorMode mode = OBJGeneratorMode::vertical;
    double startHeight = 3;
    OBJGenerator::generateClothOBJ(objectFile, vertexDist, width, height, mode, startHeight); //make the cloth to be used

    string nodeFile;
    string eleFile;
    string faceFile;
    int indexBase;

    int whichObject = -1;

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
    } else{
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
    pCenters.push_back(Vector3d(5,0,0)); //right wall plane (90 deg)

    //Plane normals
    pNormals.push_back(Vector3d(0,1,0));
    pNormals.push_back(Vector3d(-1,0,0));

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
            quikDeformer.addWind(wx, wy, wz, windMag, windOsc);
        }

        //-------RANDOMIZE VERTICES TEST----------//
        //quikDeformer.randomizeVertices();

        //---------RUN SIMULATION--------------//
        quikDeformer.runSimulation(seconds, outputFilepath, printsOn);

    }
    else if(!volumetric){

        //---------CONSTRUCTOR--------//
        QuikDeformer quikDeformer(objectFile, pCenters, pLengths, pWidths, pNormals, h, iter, fr, mass, vx, vy, vz, gravityOn, volumetric);

        //------PRINT MATRICES-------//
        //quikDeformer.printMatrices();

        //-----ADD STRAIN CONSTRAINTS-------//
        quikDeformer.add2DStrainConstraints(triangleStrainWeight);

        //------ADD POSITION CONSTRAINTS------//
        //int posConstraintIndex = 0;
        //double posConstraintW = 100000;
        //quikDeformer.addPositionConstraint(posConstraintW, posConstraintIndex);

        //------ADD WIND EFFECTS--------------//
        if(windOn){
            quikDeformer.addWind(wx, wy, wz, windMag, windOsc);
        }

        //-------RANDOMIZE VERTICES TEST----------//
        //quikDeformer.randomizeVertices();

        //---------RUN SIMULATION--------------//
        quikDeformer.runSimulation(seconds, outputFilepath, printsOn);
    }

    return 0;
}