//
// Created by Josh Wolper on 3/22/18.
//

#ifndef QUIKDEFORM_TETSTRAINCONSTRAINT_H
#define QUIKDEFORM_TETSTRAINCONSTRAINT_H

#include "Eigen/Eigen"
#include "Constraint.h"
#include "SVD.h"
#include <vector>
#include <iostream>

class TetStrainConstraint : public Constraint{
public:
    TetStrainConstraint(double weight,
                        Eigen::MatrixXd sMatrix,
                        Eigen::VectorXd p,
                        Eigen::MatrixXd aMatrix,
                        Eigen::MatrixXd bMatrix,
                        std::vector<int> indeces,
                        double volume,
                        Eigen::MatrixXd dminv)
    {

        //Set our variables
        setW(weight * volume); //now when we multiply by weight we are actually multiplying by weight * volume!! Which is correct I think
        setS(sMatrix);
        setP(p);
        setVolume(volume); //store volume anyway just in case
        setIndeces(indeces);
        setA(aMatrix);
        setB(bMatrix);
        setDmInv(dminv);

    };

    ~TetStrainConstraint(){};

    // This function should calculate a set of constraint satisfying points in the P matrix
    void projectConstraint(Eigen::VectorXd qN_1);

private:

};

#endif //QUIKDEFORM_TETSTRAINCONSTRAINT_H
