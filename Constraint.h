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

    Eigen::VectorXd getP(){ return pMatrix; };
    Eigen::MatrixXd getA(){ return aMatrix; };
    Eigen::MatrixXd getB(){ return bMatrix; };
    Eigen::MatrixXd getS(){ return sMatrix; };
    double getW() { return w; };
    std::vector<int> getIndeces(){ return indeces; };

    void setP(Eigen::VectorXd P){ pMatrix = P;};
    void setA(Eigen::MatrixXd A){ aMatrix = A;};
    void setB(Eigen::MatrixXd B){ bMatrix = B;};
    void setS(Eigen::MatrixXd S){ sMatrix = S;};
    void setW(double weight){ w = weight; };

    void setIndeces(std::vector<int> ids){ indeces = ids; };

    int getCardinality() { return cardinality; };

private:
    double w; //constraint weight
    int cardinality; //number of particles involved in constraint
    Eigen::VectorXd pMatrix; //p matrix for all projected points of the constraint
    Eigen::MatrixXd sMatrix; //selection matrix
    Eigen::MatrixXd aMatrix; //A matrix
    Eigen::MatrixXd bMatrix; //B matrix
    std::vector<int> indeces; //index matrix

};

#endif //QUIKDEFORM_CONSTRAINT_H
