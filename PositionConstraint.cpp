//
// Created by Josh Wolper on 3/13/18.
//

#include "PositionConstraint.h"

using namespace Eigen;

// local step. solve for the set of project current points that will satisfy this constraint
void PositionConstraint::projectConstraint(Eigen::VectorXd qN_1){

    //Since the constraint manifold for a point is only a single point, p_i will always be the same!
    //We don't actually have to do anything here!

    std::cout << "Solved position constraint! " << std::endl;

    return;
}
