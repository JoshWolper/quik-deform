// Main class for QuikDeformer
// Each object of this class contains simulated objected

#ifndef QUIKDEFORM_QUIKDEFORMER_H
#define QUIKDEFORM_QUIKDEFORMER_H

#include "Eigen/Eigen"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <vector>
#include <string>
#include "Constraint.h"
#include "PositionConstraint.h"
#include "TetStrainConstraint.h"
#include "TriangleStrainConstraint.h"
#include "SVD.h"
#include "/usr/local/Cellar/gcc/7.3.0_1/lib/gcc/7/gcc/x86_64-apple-darwin15.6.0/7.3.0/include/omp.h"

class QuikDeformer {
public:
    // Constructor for object files
    QuikDeformer(const std::string& objFilePath,
                 double timeStep,
                 int solverIterations,
                 int frameRate,
                 double mass,
                 double initVelX,
                 double initVelY,
                 double initVelZ,
                 bool gravOn,
                 bool volumetricOn,
                 int indexBase):
                    timeStep(timeStep),
                    solverIterations(solverIterations),
                    frameRate(frameRate),
                    numVertices(0),
                    gravityOn(gravOn),
                    volumetric(volumetricOn)
                    {
                        readObj(objFilePath, indexBase); //setup particles and fragments
                        setupMatrices(mass, initVelX, initVelY, initVelZ);
                    };

    //Constructor for volumetric models (use node, ele, and face files)
    QuikDeformer(const std::string& nodeFilePath,
                 const std::string& eleFilePath,
                 const std::string& faceFilePath,
                 double timeStep,
                 int solverIterations,
                 int frameRate,
                 double mass,
                 double initVelX,
                 double initVelY,
                 double initVelZ,
                 bool gravOn,
                 bool volumetricOn,
                 int indexBase,
                 bool scaleModel):
                    timeStep(timeStep),
                    solverIterations(solverIterations),
                    frameRate(frameRate),
                    numVertices(0),
                    gravityOn(gravOn),
                    volumetric(volumetricOn)
                    {
                        readVolumetric(nodeFilePath, eleFilePath, faceFilePath, indexBase, scaleModel); //setup particles, tets, and fragments
                        setupMatrices(mass, initVelX, initVelY, initVelZ);
                    };

    //Constructor for volumetric models that directly takes in the geometry!!
    QuikDeformer(std::vector<Eigen::Vector3d> particles,
                 std::vector<Eigen::Vector3i> faces,
                 std::vector<std::vector<int>> tets,
                 double timeStep,
                 int solverIterations,
                 int frameRate,
                 double mass,
                 double initVelX,
                 double initVelY,
                 double initVelZ,
                 bool gravOn,
                 bool volumetricOn):
                    timeStep(timeStep),
                    solverIterations(solverIterations),
                    frameRate(frameRate),
                    numVertices(0),
                    gravityOn(gravOn),
                    volumetric(volumetricOn)
                    {
                        vertices.assign(particles.begin(), particles.end());
                        fragments.assign(faces.begin(), faces.end());
                        tetrahedrons.assign(tets.begin(), tets.end());
                        numVertices = vertices.size();
                        setupMatrices(mass, initVelX, initVelY, initVelZ);
                    };

    ~QuikDeformer();

    int size () const { return numVertices; }
    void runSimulation(double seconds, const std::string& outputFilePath, bool printsOn);
    void runSimulation(double seconds, bool printsOn, std::vector<Eigen::VectorXd>& frames); //second func for Maya version

    Eigen::Vector3d planeCollision(Eigen::Vector3d p); //return true if point is above plane
    void randomizeVertices();

    void addPositionConstraint(double weight,  int posConstraintIndex);
    void add2DStrainConstraints(double strain2DWeight);
    void add3DStrainConstraints(double strain3DWeight);

    void printMatrices() const;

    void setPrintsOn(bool printStatus){ printsOn = printStatus; };
    bool getPrintsOn(){ return printsOn; };
    void setStartTime(std::clock_t start){ startTime = start; };
    void setElapsedTime(double diff){ elapsedTime = diff; };

    std::clock_t getStartTime(){return startTime; };
    double getElapsedTime(){return elapsedTime; };

    Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> m_solver; //create a public variable for our solver!

    //Plane Collision variables
    void addCollisionPlanes(const std::vector<Eigen::Vector3d> pCenters, const std::vector<double> pLengths, const std::vector<double> pWidths, const std::vector<Eigen::Vector3d> pNormals, double frictionCoeff);
    std::vector<Eigen::Vector3d> planeCenters;
    std::vector<double> planeWidths;
    std::vector<double> planeLengths;
    std::vector<Eigen::Vector3d> planeNormals;
    double frictionCoefficient;

    //Wind stuff
    void addWind(double wx, double wy, double wz, double windMag, bool windOsc, double oscAmp, double period);
    void updateWind();
    bool windOscillates = false;
    double windX = 0;
    double windY = 0;
    double windZ = 0;
    double windMagnitude = 0;
    double oscillationAmplitude = 0;
    double windPeriod = 0;

private:

    //Simulation Variables
    double timeStep;
    int solverIterations;
    int frameRate;
    int numVertices;
    bool printsOn = false;
    bool gravityOn = true;
    bool volumetric = false;
    std::vector<Eigen::Vector3d> vertices;
    std::vector<Eigen::Vector3i> fragments;
    std::vector<std::vector<int>> tetrahedrons; //vector of int vectors
    std::vector<Constraint*> constraints;
    Eigen::VectorXd* qMatrix;
    Eigen::VectorXd* vMatrix;
    Eigen::VectorXd* fExtMatrix;
    Eigen::MatrixXd* mMatrix;
    Eigen::MatrixXd* invMassMatrix;

    //Timers
    clock_t startTime;
    double elapsedTime = 0;

    void readObj(const std::string& fileName, int indexBase); //setup particles and frags from object file
    void readVolumetric(const std::string& nodePath, const std::string& elePath, const std::string& facePath, int indexBase, bool scaleModel); //setup particles, tets, and fragments
    void writeObj(const std::string& fileName, Eigen::VectorXd qMat) const;
    void writeBgeo(const std::string& fileName) const;
    void setupMatrices(double mass, double vx, double vy, double vz);
    void buildTriangleStrainA(MatrixXd& A_matrix, MatrixXd& G);
    void buildTetStrainA(Eigen::MatrixXd& A_matrix, Eigen::MatrixXd& G);
    Eigen::MatrixXd solveLinearSystem(Eigen::MatrixXd sn, Eigen::MatrixXd L, Eigen::MatrixXd Ltranspose); //SLOW VERSION
    Eigen::MatrixXd solveLinearSystem(Eigen::MatrixXd sn); //FAST VERSION
    double randDouble(double min, double max);
};


#endif //QUIKDEFORM_QUIKDEFORMER_H
