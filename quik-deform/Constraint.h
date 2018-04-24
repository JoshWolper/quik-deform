// Base constraint class for other constraints to extend.
// Abstract class so cannot be instantiated
// Satisfy function must be implemented by kids

#ifndef QUIKDEFORM_CONSTRAINT_H
#define QUIKDEFORM_CONSTRAINT_H

#include "Eigen/Eigen"
#include <iostream>

using namespace std;

class Constraint{
public:
    Constraint(){};
    virtual ~Constraint(){};

    virtual void projectConstraint(Eigen::VectorXd qN_1){ cout << "ERROR: projectConstraints function not implemented" << endl; return; };

    Eigen::VectorXd getP(){ return pMatrix; };
    Eigen::MatrixXd getA(){ return aMatrix; };
    Eigen::MatrixXd getB(){ return bMatrix; };
    Eigen::MatrixXd getS(){ return sMatrix; };
    double getW() { return w; };
    double getVolume() { return volume; };
    std::vector<int> getIndeces(){ return indeces; };
    Eigen::MatrixXd getDmInv(){ return DmInv; };
    Eigen::Matrix3d getDefGrad(){ return defGrad; };
    int getCardinality() { return cardinality; };
    Eigen::MatrixXd getDs(){ return Ds; };

    void setP(Eigen::VectorXd P){ pMatrix = P;};
    void setA(Eigen::MatrixXd A){ aMatrix = A;};
    void setB(Eigen::MatrixXd B){ bMatrix = B;};
    void setS(Eigen::MatrixXd S){ sMatrix = S;};
    void setW(double weight){ w = weight; };

    void setVolume(double vol){ volume = vol; };
    void setArea(double a){ area = a; };

    void setDmInv(Eigen::MatrixXd Dminv){ DmInv = Dminv; };
    void setIndeces(std::vector<int> ids){ indeces = ids; };
    void setDefGrad(Eigen::Matrix3d F){ defGrad = F; };
    void setDs(Eigen::MatrixXd Ds_arg){ Ds = Ds_arg; };


private:
    double w; //constraint weight
    int cardinality; //number of particles involved in constraint
    double volume;
    double area;
    Eigen::VectorXd pMatrix; //p matrix for all projected points of the constraint
    Eigen::MatrixXd sMatrix; //selection matrix
    Eigen::MatrixXd aMatrix; //A matrix
    Eigen::MatrixXd bMatrix; //B matrix
    std::vector<int> indeces; //index matrix
    Eigen::MatrixXd DmInv; //store for strain constraints
    Eigen::Matrix3d defGrad; //store current deformation gradient of constraint so we can print it
    Eigen::MatrixXd Ds;

};

#endif //QUIKDEFORM_CONSTRAINT_H
