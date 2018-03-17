// Extends the base Constraint class
// defines elastic constraints

#ifndef QUIKDEFORM_ELASTICCONSTRAINT_H
#define QUIKDEFORM_ELASTICCONSTRAINT_H

#include "Eigen/Eigen"
#include "Constraint.h"
#include <vector>

class StrainConstraint : public Constraint{
public:
    explicit StrainConstraint(std::vector<Eigen::Vector3d>& vertices,
                               std::vector<Eigen::Vector3i>& faces) {};
    ~StrainConstraint(){};

    // This function should calculate a set of constraint satisfying points in the P matrix
    void projectConstraint(Eigen::MatrixXd qN_1);

private:
    // do we need a class like this for the ElasticConstraint?
    struct Edge{
        int start;
        int end;

        Edge(int s, int e) : start(s), end(e) {};

        bool operator== (Edge other) const{
            return (this->start == other.start) ?
                   (this->end == other.end) : false;
        }
    };

    //both of these are indexed the same
    std::vector<Edge> edges;
};


#endif //QUIKDEFORM_ELASTICCONSTRAINT_H
