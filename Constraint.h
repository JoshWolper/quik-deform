// Base constraint class for other constraints to extend.
// Abstract class so cannot be instantiated
// Satisfy function must be implemented by kids

#ifndef QUIKDEFORM_CONSTRAINT_H
#define QUIKDEFORM_CONSTRAINT_H

#include "Eigen/Eigen"

class Constraint{
public:
    Constraint(){};
    virtual ~Constraint(){};

    virtual Eigen::MatrixXd Satisfy(double timeStep){};
};

#endif //QUIKDEFORM_CONSTRAINT_H
