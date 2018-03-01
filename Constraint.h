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

    virtual void projectConstraint(Eigen::MatrixXd qN_1){};

    Eigen::MatrixXd getP(){ return *pMatrix; };
    Eigen::MatrixXd getA(){ return *aMatrix; };
    Eigen::MatrixXd getB(){ return *bMatrix; };
    Eigen::MatrixXd getS(){ return *sMatrix; };

    double getW() { return w; };
    int getCardinality() { return cardinality; };

private:
    double w; //constraint weight
    int cardinality; //number of particles involved in constraint
    Eigen::MatrixXd* pMatrix; //p matrix for all projected points of the constraint
    Eigen::MatrixXd* sMatrix; //selection matrix
    Eigen::MatrixXd* aMatrix; //A matrix
    Eigen::MatrixXd* bMatrix; //B matrix

};

#endif //QUIKDEFORM_CONSTRAINT_H
