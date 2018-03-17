//
// Created by Josh Wolper on 3/16/18.
//

#include "GroundConstraint.h"

using namespace Eigen;

// local step. solve for the set of project current points that will satisfy this constraint
void GroundConstraint::projectConstraint(Eigen::VectorXd qN_1){

    //For ground constraints we need to either set the projected point to be the point itself (if not colliding with ground) OR to be the closest point to the surface if it has collided

    std::vector<int> ids = getIndeces(); //grab the indeces

    double floor = getFloor();

    VectorXd pTemp = VectorXd(3 * ids.size()); //have a slot for each index

    for(int i = 0; i < ids.size(); i++){

        int id = ids[i];
        double xVal = qN_1(id * 3 + 0);
        double yVal = qN_1(id * 3 + 1);
        double zVal = qN_1(id * 3 + 2);

        if(yVal > floor){ //set p val to be exact same position as current!

            pTemp(i*3 + 0) = xVal;
            pTemp(i*3 + 1) = yVal;
            pTemp(i*3 + 2) = zVal;

        }
        else{ //if below floor need to set y to 0

            pTemp(i*3 + 0) = xVal;
            pTemp(i*3 + 1) = 0;
            pTemp(i*3 + 2) = zVal;

        }

    }

    std::cout << "Ground constraint p: " << std::endl << pTemp << std::endl;

    setP(pTemp); //set the p

    return;
}
