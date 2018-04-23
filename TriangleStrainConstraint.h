//
// Created by Josh Wolper on 3/22/18.
//

#ifndef QUIKDEFORM_TRIANGLESTRAINCONSTRAINT_H
#define QUIKDEFORM_TRIANGLESTRAINCONSTRAINT_H

#include "Eigen/Eigen"
#include "Constraint.h"
#include "SVD.h"
#include <vector>
#include <iostream>

class TriangleStrainConstraint : public Constraint{
public:
    TriangleStrainConstraint(double weight,
                        Eigen::MatrixXd sMatrix,
                        Eigen::VectorXd p,
                        Eigen::MatrixXd aMatrix,
                        Eigen::MatrixXd bMatrix,
                        std::vector<int> indeces,
                        double area,
                        Eigen::MatrixXd dminv)
    {
        //Set our variables
        setW(weight * area); //now when we multiply by weight we are actually multiplying by weight * area!! Which is correct I think
        setS(sMatrix);
        setP(p);
        setArea(area); //store area
        setIndeces(indeces);
        setA(aMatrix);
        setB(bMatrix);
        setDmInv(dminv);
    };

    ~TriangleStrainConstraint(){};

    // This function should calculate a set of constraint satisfying points in the P matrix
    void projectConstraint(Eigen::VectorXd qN_1);

private:

};

#endif //QUIKDEFORM_TRIANGLESTRAINCONSTRAINT_H
