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
#include "StrainConstraint.h"

class QuikDeformer {
public:
    // the constructor takes in all of the needed variables for the simulation
    QuikDeformer(const std::string& objFilePath,
                 double timeStep,
                 int solverIterations,
                 int frameRate,
                 double mass,
                 double initVelX,
                 double initVelY,
                 double initVelZ):
                    timeStep(timeStep),
                    solverIterations(solverIterations),
                    frameRate(frameRate),
                    numVertices(0)
                    {
                        readObj(objFilePath);
                        setupMatrices(mass, initVelX, initVelY, initVelZ);
                    };
    ~QuikDeformer();

    int size () const { return numVertices; }
    void runSimulation(double seconds, const std::string& outputFilePath);
    void addConstraint(const std::string& type); // TODO: maybe use enums to add constraints instead?
    void printMatrices() const;

    // TODO: getters and setters for private variables?


private:
    double timeStep;
    int solverIterations;
    int frameRate;
    int numVertices;
    std::vector<Eigen::Vector3d> vertices;
    std::vector<Eigen::Vector3i> fragments;
    std::vector<Constraint*> constraints;
    Eigen::MatrixXd* qMatrix;
    Eigen::MatrixXd* vMatrix;
    Eigen::MatrixXd* fExtMatrix;
    Eigen::MatrixXd* mMatrix;
    Eigen::MatrixXd* invMassMatrix;

    void readObj(const std::string& fileName);
    void writeObj(const std::string& fileName, Eigen::MatrixXd qMat) const;
    void writeBgeo(const std::string& fileName) const;
    void setupMatrices(double mass, double vx, double vy, double vz);
    Eigen::MatrixXd solveLinearSystem(Eigen::MatrixXd sn);
};


#endif //QUIKDEFORM_QUIKDEFORMER_H
