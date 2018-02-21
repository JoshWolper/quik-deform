//
// Created by Josh Wolper on 2/21/18.
//

#ifndef QUIKDEFORM_INITIALIZE_H
#define QUIKDEFORM_INITIALIZE_H


#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include "Eigen/Eigen"

using namespace Eigen;

//Function to read a given object file, fill a matrix q that is m X 3, and return the number of points total
void readObj(std::string filename, std::vector<Vector3d>& vertices, std::vector<Vector3i>& fragments){

    std::ifstream inputfile;
    inputfile.open(filename);
    if(!inputfile){
        std::cerr << "Unable to open " << filename << "!" << std::endl;
        exit(1);
    }
    std::string line;
    while(std::getline(inputfile, line)){
        std::stringstream ss(line);
        if (line[0] == 'v'){
            Vector3f currPoint;
            ss.ignore();
            for (int i = 0; i < 3; i++){
                ss >> currPoint(i);
            }
            vertices.push_back(currPoint);
        }
        else if(line[0] == 'f'){
            Vector3i currFragment;
            ss.ignore();
            for (int i = 0; i < 3; i++){
                ss >> currFragment(i);
            }
            fragments.push_back(currFragment);
        }

    }

    return;
}

MatrixXd initializeQ(std::string filename){

    std::vector<Vector3d> points;
    std::vector<Vector3i> fragments;
    readObj(filename, points, fragments); //call our object file reader

    int numPoints = points.size();

    MatrixXd q(numPoints, 3); //init our M by 3 matrix, q

    for(int i = 0; i < numPoints; i++){



    }


    return q;
}

#endif //QUIKDEFORM_INITIALIZE_H

