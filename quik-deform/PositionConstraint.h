//
// Created by Josh Wolper on 3/13/18.
//

#ifndef QUIKDEFORM_POSITIONCONSTRAINT_H
#define QUIKDEFORM_POSITIONCONSTRAINT_H

#include "Eigen/Eigen"
#include "Constraint.h"
#include <vector>
#include <iostream>

class PositionConstraint : public Constraint{
public:
    PositionConstraint(double weight,
                       Eigen::MatrixXd sMatrix,
                       Eigen::VectorXd p)
    {

        //Set our variables
        setW(weight);
        setS(sMatrix);
        setP(p);

        //Now set A and B = I_3
        Eigen::MatrixXd I = Eigen::Matrix<double, 3, 3>::Identity();

        setA(I);
        setB(I);

    };

    ~PositionConstraint(){};

    // This function should calculate a set of constraint satisfying points in the P matrix
    void projectConstraint(Eigen::VectorXd qN_1);

private:

};

#endif //QUIKDEFORM_POSITIONCONSTRAINT_H
