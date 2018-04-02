#include "QuikDeformer.h"
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

using namespace Eigen;
using namespace std;

// Destructor
QuikDeformer::~QuikDeformer(){
    delete qMatrix;
    delete mMatrix;
    for (auto& c : constraints){
        delete c;
    }
}

// Function to read a given object file, fill a matrix q that is m X 3, and return the number of points total
void QuikDeformer::readObj(const std::string& fileName){

    ifstream inputfile;
    inputfile.open(fileName);
    if(!inputfile){
        cerr << "Unable to open " << fileName << "!" << std::endl;
        exit(1);
    }
    string line;
    while(getline(inputfile, line)){
        stringstream ss(line);
        if (line[0] == 'v'){
            Vector3d currPoint;
            ss.ignore();
            for (int i = 0; i < 3; i++){
                ss >> currPoint(i);
            }
            vertices.push_back(currPoint);
        }
        else if(line[0] == 'f'){
            Vector3i currFragment;
            ss.ignore();
            for (int i = 0; i < 3; i++){
                ss >> currFragment(i);
            }
            fragments.push_back(currFragment);
        }

    }

    numVertices = vertices.size();

    return;
}

void QuikDeformer::readVolumetric(const std::string& nodePath, const std::string& elePath, const std::string& facePath){

    //Open all the files and check for errors
    ifstream nodeFile, eleFile, faceFile;
    nodeFile.open(nodePath);
    if(!nodeFile){
        cerr << "Unable to open " << nodePath << "!" << std::endl;
        exit(1);
    }
    eleFile.open(elePath);
    if(!eleFile){
        cerr << "Unable to open " << elePath << "!" << std::endl;
        exit(1);
    }
    faceFile.open(facePath);
    if(!faceFile){
        cerr << "Unable to open " << facePath << "!" << std::endl;
        exit(1);
    }

    //NODE FILE
    string line;
    getline(nodeFile, line); //grab first line before starting (since it's not used)
    while( getline(nodeFile, line) ){
        stringstream ss(line);

        if (line[0] == '#'){
            break; //last line of file!
        }

        Vector3d currPoint;
        ss.ignore();
        double garbage;
        ss >> garbage; //throw out first value

        for (int i = 0; i < 3; i++){
            ss >> currPoint(i);
        }

        vertices.push_back(currPoint);

    }

    //cout << "Finished node file!" << endl;

    //ELE FILE
    getline(eleFile, line); //grab first line before starting (since it's not used)
    while( getline(eleFile, line) ){
        stringstream ss(line);

        if (line[0] == '#'){
            break; //last line of file!
        }

        vector<int> currTet;
        ss.ignore();
        int garbage, x;
        ss >> garbage; //throw away first val

        for (int i = 0; i < 4; i++){
            ss >> x;
            currTet.push_back(x);
        }

        tetrahedrons.push_back(currTet);

    }

    //cout << "Finished ele file!" << endl;

    //FACE FILE
    getline(faceFile, line); //grab first line before starting (since it's not used)
    while( getline(faceFile, line) ){
        stringstream ss(line);

        if (line[0] == '#'){
            break; //last line of file!
        }

        Vector3i currFragment;
        ss.ignore();
        int garbage;
        ss >> garbage; //throw away first value (it's garbage)

        for (int i = 0; i < 3; i++){
            ss >> currFragment(i);
        }

        fragments.push_back(currFragment);

    }

    numVertices = vertices.size();

    //cout << "First particle: " << vertices[0] << endl;
    //cout << "First tet: " << tetrahedrons[0][0] << " " <<  tetrahedrons[0][1] << " " << tetrahedrons[0][2] << " " << tetrahedrons[0][3] << endl;
    //cout << "First face: " << fragments[0] << endl;

    cout << "Num particles: " << vertices.size() << endl;
    cout << "Num tets: " << tetrahedrons.size() << endl;
    cout << "Num fragments: " << fragments.size() << endl;


    return;
}



// function called after readObj to construct the matrices we need
void QuikDeformer::setupMatrices(double mass, double vx, double vy, double vz) {
    // set up the q0 matrix
    qMatrix = new VectorXd(numVertices * 3, 1); //init our 3M by 1 vector, qMatrix
    vMatrix = new VectorXd(numVertices * 3, 1); //init our 3M by 1 vector, vMatrix
    fExtMatrix = new VectorXd(numVertices * 3, 1); //init our 3M by 1 vector, fExtMatrix


    for(int i = 0; i < numVertices; i++){
        (*qMatrix)((3*i) + 0) = vertices[i][0];
        (*qMatrix)((3*i) + 1) = vertices[i][1];
        (*qMatrix)((3*i) + 2) = vertices[i][2]; //fill each row iteratively

        (*vMatrix)((3*i) + 0) = vx;
        (*vMatrix)((3*i) + 1) = vy;
        (*vMatrix)((3*i) + 2) = vz; //fill each row iteratively

        (*fExtMatrix)((3*i) + 0) = 0;
        (*fExtMatrix)((3*i) + 1) = (gravityOn == true) ? (-9.81 * mass) : 0.0; //add gravity force to each point in our external force matrix
        (*fExtMatrix)((3*i) + 2) = 0;
    }

    // set up the mass matrix
    long numPoints = numVertices * 3; //eventually dynamically fill this when we are reading in an obj

    //Set the m x m diagonal mass matrix, M
    MatrixXd* temp = new MatrixXd(numPoints, numPoints);
    *temp = MatrixXd::Identity(numPoints, numPoints);
    *temp *= mass;
    mMatrix = temp;

    //Set the inverse mass matrix
    MatrixXd* temp2 = new MatrixXd(numPoints, numPoints);
    *temp2 = MatrixXd::Identity(numPoints, numPoints);
    *temp2 /= mass;
    invMassMatrix = temp2;
}


// helper function to just see what our matrices look like
void QuikDeformer::printMatrices() const {
    cout << "Our q matrix is: " << endl << *qMatrix << endl;
    cout << "Our v matrix is: " << endl << *vMatrix << endl;
    cout << "Our external force matrix is: " << endl << *fExtMatrix << endl;
    cout << "Our mass matrix M is: " << endl << *mMatrix << endl;
    cout << "Our inverse mass matrix is: " << endl << *invMassMatrix << endl;
}

void QuikDeformer::addWind(double wx, double wy, double wz, double windMag, bool windOsc){

    for(int i = 0; i < numVertices; i++){

        (*fExtMatrix)((3*i) + 0) += (wx * windMag);
        (*fExtMatrix)((3*i) + 1) += (wy * windMag); //update the forces to include wind effects in the specified direction and magnitude
        (*fExtMatrix)((3*i) + 2) += (wz * windMag);

    }

}

void QuikDeformer::addPositionConstraint(double weight, int posConstraintIndex){

    //Turn index into sMatrix and p Vector!

    MatrixXd sMatrix = MatrixXd(3, 3 * numVertices); //3 by 3M S matrix
    VectorXd p = VectorXd(3,1);

    p(0) = vertices[posConstraintIndex][0];
    p(1) = vertices[posConstraintIndex][1];
    p(2) = vertices[posConstraintIndex][2];

    sMatrix.setZero();
    sMatrix(0, (3 * posConstraintIndex + 0)) = 1;
    sMatrix(1, (3 * posConstraintIndex + 1)) = 1;
    sMatrix(2, (3 * posConstraintIndex + 2)) = 1;

    cout << "sMatrix is: " << endl << sMatrix << endl;

    constraints.push_back(new PositionConstraint(weight, sMatrix, p));

}

void QuikDeformer::addGroundConstraint(double weight, vector<int> posConstraintIndeces, double floorVal){

    //Turn index into sMatrix and p Vector!

    MatrixXd sMatrix = MatrixXd(3 * posConstraintIndeces.size(), 3 * numVertices); //3 by 3M S matrix
    VectorXd p = VectorXd(3 * posConstraintIndeces.size(), 1); //make our vector to hold all the points in the constraint

    sMatrix.setZero();

    for(int i = 0; i < posConstraintIndeces.size(); i++){

        p(i*3 + 0) = vertices[posConstraintIndeces[i]][0];
        p(i*3 + 1) = vertices[posConstraintIndeces[i]][1];
        p(i*3 + 2) = vertices[posConstraintIndeces[i]][2];

        sMatrix((3 * i + 0), (3 * posConstraintIndeces[i] + 0)) = 1;
        sMatrix((3 * i + 1), (3 * posConstraintIndeces[i] + 1)) = 1;
        sMatrix((3 * i + 2), (3 * posConstraintIndeces[i] + 2)) = 1;

    }

    cout << "sMatrix is: " << endl << sMatrix << endl;

    constraints.push_back(new GroundConstraint(weight, sMatrix, p, posConstraintIndeces, floorVal));

}


void QuikDeformer::add2DStrainConstraints(double strain2DWeight){

    //For each triangle in mesh
    for(int i = 0; i < fragments.size(); i++){

        Vector3i currTriangle = fragments[i];

        //Compute Dm
        MatrixXd Dm = MatrixXd(2,2);

        int id0 = 3 * (currTriangle[0] - 1);
        int id1 = 3 * (currTriangle[1] - 1);
        int id2 = 3 * (currTriangle[2] - 1);

        Dm(0,0) = (*qMatrix)(id1 + 0) - (*qMatrix)(id0 + 0); //top left = X1.x - X0.x
        Dm(1,0) = (*qMatrix)(id1 + 1) - (*qMatrix)(id0 + 1); //bottom left = X1.y - X0.y

        Dm(0,1) = (*qMatrix)(id2 + 0) - (*qMatrix)(id0 + 0); //top right = X2.x - X0.x
        Dm(1,1) = (*qMatrix)(id2 + 1) - (*qMatrix)(id0 + 1); //bottom right = X2.y - X0.y

        MatrixXd Dminv = Dm.inverse();

        double area = Dm.determinant() * 0.5;

        //cout << "Dm for triangle " << i << " :" << endl << Dm << endl;

        //SET S MATRIX
        MatrixXd sMat = MatrixXd(9, 3 * numVertices).setZero();
        for(int j = 0; j < currTriangle.size(); j++){
            //fill in S matrix for each index
            sMat((3 * j + 0), (3 * (currTriangle[j] - 1) + 0)) = 1;
            sMat((3 * j + 1), (3 * (currTriangle[j] - 1) + 1)) = 1;
            sMat((3 * j + 2), (3 * (currTriangle[j] - 1) + 2)) = 1;
        }
        //cout << "SMatrix: " << endl << sMat << endl;

        //SET BP MATRIX (okay to init to all zero)
        VectorXd Bp = VectorXd(9,1).setZero();

        //SET A MATRIX
        MatrixXd aMat = MatrixXd(9,12).setZero();
        buildTetStrainA(aMat, Dminv); //build aMat

        //Set B = Identity (9 by 9)
        Eigen::MatrixXd bMat = Eigen::MatrixXd(9, 9).setIdentity(); //identity since what we are passing for p is actually Bp

        //constraints.push_back(new TetStrainConstraint(strain3DWeight, sMat, Bp, aMat, bMat, currTet, volume, Dminv));

        cout << "Finished triangle " << i << " :" << endl;

    }

}

void QuikDeformer::buildTriangleStrainA(MatrixXd& A_matrix, MatrixXd& G){

    A_matrix.setZero();
    A_matrix(0,2)=G(0,0);A_matrix(0,4)=G(1,0);A_matrix(0,0)=-G(0,0)-G(1,0);
    A_matrix(1,2)=G(0,1);A_matrix(1,4)=G(1,1);A_matrix(1,0)=-G(0,1)-G(1,1);
    A_matrix(2,3)=G(0,0);A_matrix(2,5)=G(1,0);A_matrix(2,1)=-G(0,0)-G(1,0);
    A_matrix(3,3)=G(0,1);A_matrix(3,5)=G(1,1);A_matrix(3,1)=-G(0,1)-G(1,1);

    return;
}


void QuikDeformer::add3DStrainConstraints(double strain3DWeight){

    //For each tetrahedron in mesh
    for(int i = 0; i < tetrahedrons.size(); i++){

        vector<int> currTet = tetrahedrons[i];

        //Compute Dm
        MatrixXd Dm = MatrixXd(3,3);

        int id0 = 3 * (currTet[0] - 1); //might have to subtract 1 if NOT 0-indexed
        int id1 = 3 * (currTet[1] - 1);
        int id2 = 3 * (currTet[2] - 1);
        int id3 = 3 * (currTet[3] - 1);

        Dm(0,0) = (*qMatrix)(id1 + 0) - (*qMatrix)(id0 + 0); //top left = X1.x - X0.x
        Dm(1,0) = (*qMatrix)(id1 + 1) - (*qMatrix)(id0 + 1); //middle left = X1.y - X0.y
        Dm(2,0) = (*qMatrix)(id1 + 2) - (*qMatrix)(id0 + 2); //bottom left = X1.z - X0.z

        Dm(0,1) = (*qMatrix)(id2 + 0) - (*qMatrix)(id0 + 0); //top middle = X2.x - X0.x
        Dm(1,1) = (*qMatrix)(id2 + 1) - (*qMatrix)(id0 + 1); //middle middle = X2.y - X0.y
        Dm(2,1) = (*qMatrix)(id2 + 2) - (*qMatrix)(id0 + 2); //bottom middle = X2.z - X0.z

        Dm(0,2) = (*qMatrix)(id3 + 0) - (*qMatrix)(id0 + 0); //top right = X3.x - X0.x
        Dm(1,2) = (*qMatrix)(id3 + 1) - (*qMatrix)(id0 + 1); //middle right = X3.y - X0.y
        Dm(2,2) = (*qMatrix)(id3 + 2) - (*qMatrix)(id0 + 2); //bottom right = X3.z - X0.z

        MatrixXd Dminv = Dm.inverse();

        double volume = Dm.determinant() / 6.0;

        //cout << "Dm for tet " << i << " :" << endl << Dm << endl;

        //SET S MATRIX
        MatrixXd sMat = MatrixXd(12, 3 * numVertices).setZero();
        for(int j = 0; j < currTet.size(); j++){
            //fill in S matrix for each index
            sMat((3 * j + 0), (3 * (currTet[j] - 1) + 0)) = 1;
            sMat((3 * j + 1), (3 * (currTet[j] - 1) + 1)) = 1;
            sMat((3 * j + 2), (3 * (currTet[j] - 1) + 2)) = 1;
        }
        //cout << "SMatrix: " << endl << sMat << endl;

        //SET BP MATRIX (okay to init to all zero)
        VectorXd Bp = VectorXd(9,1).setZero();

        //SET A MATRIX
        MatrixXd aMat = MatrixXd(9,12).setZero();
        buildTetStrainA(aMat, Dminv); //build aMat

        //Set B = Identity (9 by 9)
        Eigen::MatrixXd bMat = Eigen::MatrixXd(9, 9).setIdentity(); //identity since what we are passing for p is actually Bp

        constraints.push_back(new TetStrainConstraint(strain3DWeight, sMat, Bp, aMat, bMat, currTet, volume, Dminv));

        cout << "Finished tet " << i << " :" << endl;

    }

}

void QuikDeformer::buildTetStrainA(MatrixXd& A_matrix, MatrixXd& G){

    A_matrix.setZero();
    A_matrix(0,3)=G(0,0);A_matrix(0,6)=G(1,0);A_matrix(0,9)=G(2,0);A_matrix(0,0)=-G(0,0)-G(1,0)-G(2,0);
    A_matrix(1,3)=G(0,1);A_matrix(1,6)=G(1,1);A_matrix(1,9)=G(2,1);A_matrix(1,0)=-G(0,1)-G(1,1)-G(2,1);
    A_matrix(2,3)=G(0,2);A_matrix(2,6)=G(1,2);A_matrix(2,9)=G(2,2);A_matrix(2,0)=-G(0,2)-G(1,2)-G(2,2);
    A_matrix(3,4)=G(0,0);A_matrix(3,7)=G(1,0);A_matrix(3,10)=G(2,0);A_matrix(3,1)=-G(0,0)-G(1,0)-G(2,0);
    A_matrix(4,4)=G(0,1);A_matrix(4,7)=G(1,1);A_matrix(4,10)=G(2,1);A_matrix(4,1)=-G(0,1)-G(1,1)-G(2,1);
    A_matrix(5,4)=G(0,2);A_matrix(5,7)=G(1,2);A_matrix(5,10)=G(2,2);A_matrix(5,1)=-G(0,2)-G(1,2)-G(2,2);
    A_matrix(6,5)=G(0,0);A_matrix(6,8)=G(1,0);A_matrix(6,11)=G(2,0);A_matrix(6,2)=-G(0,0)-G(1,0)-G(2,0);
    A_matrix(7,5)=G(0,1);A_matrix(7,8)=G(1,1);A_matrix(7,11)=G(2,1);A_matrix(7,2)=-G(0,1)-G(1,1)-G(2,1);
    A_matrix(8,5)=G(0,2);A_matrix(8,8)=G(1,2);A_matrix(8,11)=G(2,2);A_matrix(8,2)=-G(0,2)-G(1,2)-G(2,2);

}

void QuikDeformer::randomizeVertices(){

    /* initialize random seed: */
    srand (time(NULL));

    for(int i = 0; i < numVertices; i++){

        Vector3d newPoint;

        double newX, newY, newZ;
        double min, max;
        min = 1.2;
        max = 1.8;
        newX = randDouble(min, max);
        newY = randDouble(min, max);
        newZ = randDouble(min, max);

        newPoint[0] = newX;
        newPoint[1] = newY;
        newPoint[2] = newZ;

        vertices[i] = newPoint;

        //Update actual position matrix
        (*qMatrix)((3*i) + 0) = newX;
        (*qMatrix)((3*i) + 1) = newY;
        (*qMatrix)((3*i) + 2) = newZ;
    }

    return;
}

double QuikDeformer::randDouble(double min, double max)
{
    double r = (double)rand() / (double)RAND_MAX;
    return min + r * (max - min);
}



// runs the actual simulation and outputs the results appropriately
void QuikDeformer::runSimulation(double seconds, const std::string &outputFilePath, bool printsOn) {

    setPrintsOn(printsOn);

    //setup variables for current value and for "next" or "n+1" value!
    VectorXd qN = VectorXd(3 * numVertices, 1);
    VectorXd qN_1 = VectorXd(3 * numVertices, 1);
    VectorXd vN = VectorXd(3 * numVertices, 1);
    VectorXd vN_1 = VectorXd(3 * numVertices, 1);

    qN = *qMatrix;
    vN = *vMatrix;

    //Precompute and prefactor term for global step
    MatrixXd precomputedFactor = MatrixXd(numVertices, numVertices); //m by m matrix for this factor

    precomputedFactor = *mMatrix / (timeStep * timeStep); //first term of this factor

    //Now have to iterate over all constraints to add to our precompute factor!
    for(int i = 0; i < constraints.size(); i++){

        double w = constraints[i]->getW();
        MatrixXd S = constraints[i]->getS();
        MatrixXd Stranspose = S.transpose();
        MatrixXd A = constraints[i]->getA();
        MatrixXd Atranspose = A.transpose(); //need to do this because of the way transpose works! (does not replace original)

        precomputedFactor += (w * Stranspose * Atranspose * A * S);

    }

    //Perform sparse (simplicialLLT?) Cholesky decomposition to get L and Ltranspose!
    LLT<MatrixXd, Lower> lltOfA(precomputedFactor); // compute the Cholesky decomposition of A
    MatrixXd L = lltOfA.matrixL(); // retrieve factor L  in the decomposition
    MatrixXd Ltranspose = L.transpose();

    //Set up some variables to deal with time
    int stepsPerFrame = (int)ceil(1 / (timeStep / (1 / (double)frameRate))); //calculate how many steps per frame we should have based on our desired frame rate and dt!
    int step = 0;
    int frame = 0;

    int numFrames = seconds * frameRate;

    if(printsOn == true){cout << "Simulation steps begin: " << endl;}

    //Run the sim until we have the desired number of frames
    while(frame < numFrames){

        /*----Algorithm 1 begin (from proj dyn paper)------*/

        // Line 1 : Calculate sn
        VectorXd sn(3 * numVertices, 1);

        if(printsOn == true){cout << "made sn vector" << endl;}

        sn = qN + (timeStep * vN) + ((timeStep * timeStep) * *invMassMatrix * *fExtMatrix);

        if(printsOn == true){cout << "line 1 complete" << endl;}

        // Line 2 : qn+1 = sn
        qN_1 = sn;

        if(printsOn == true){cout << "line 2 complete" << endl;}

        // Lines 3-7 : solver loop
        for (auto i = 0; i < solverIterations; i++){

            // Lines 4-6 : Local Step (calc p_i for each constraint C_i)
            //#pragma omp parallel
            //#pragma omp for //parallelize with openMP!
            for(int j = 0; j < constraints.size(); j++) {
                if (printsOn == true) { cout << "Processing constraint " << j << "..." << endl; }

                constraints[j]->projectConstraint(qN_1); //project the constraint based on our calculated q n+1

                if (printsOn == true) {
                    MatrixXd aMat, bMat, sMat, Dminv, Ds;
                    Matrix3d defGrad;
                    aMat = constraints[j]->getA();
                    bMat = constraints[j]->getB();
                    sMat = constraints[j]->getS();
                    defGrad = constraints[j]->getDefGrad();
                    Dminv = constraints[j]->getDmInv();
                    Ds = constraints[j]->getDs();

                    //cout << "A Matrix: " << endl << aMat << endl;
                    //cout << "B Matrix: " << endl << bMat << endl;
                    //cout << "S Matrix: " << endl << sMat << endl;
                    cout << "Ds of constraint " << j << " : " << endl << Ds << endl;
                    cout << "Dminv of constraint " << j << " : " << endl << Dminv << endl;
                    cout << "F of constraint " << j << " : " << endl << defGrad << endl;
                }
            }

            if(printsOn == true){cout << "local step complete" << endl;}

            // Line 7 : Global Step (solve linear system and find qN_1)
            qN_1 = solveLinearSystem(sn, L, Ltranspose); //call this function to solve the global step!

            if(printsOn == true){cout << "global step complete" << endl;}
        }

        if(printsOn == true){cout << "lines 3-7 complete" << endl;}

        //Ground Collision Checks and Corrections
        for(int i = 0; i < numVertices; i++){ //check each position for y < 0

            double yVal = qN_1(3*i + 1);

            if(yVal < 0){

                qN_1(3*i + 1) = 0;

            }

        }


        // Line 9: vn+1 = (qn+1 - qn) / h
        vN_1 = (qN_1 - qN) / timeStep;

        if(printsOn == true){cout << "line 9 complete" << endl;}

        //At end of this loop set qN = qN+1 because we are moving to the next time now for the next loop!
        qN = qN_1;

        //do the same for vN
        vN = vN_1;

        if(printsOn == true){cout << "finished sim step" << endl;}

        if (step % stepsPerFrame == 0) {

            //Save the particles!
            cout << "INFO: Current Frame is " << frame << endl;
            writeObj(outputFilePath + std::to_string(frame) + ".obj", qN_1);
            frame = frame + 1; //update frame

        }

        step = step + 1;
    }

}

void QuikDeformer::runSimulation(double seconds, const std::string& outputFilePath, bool printsOn, std::vector<Eigen::VectorXd>& frames){

    setPrintsOn(printsOn);

    frames.clear(); //clear this vector just in case

    //setup variables for current value and for "next" or "n+1" value!
    VectorXd qN = VectorXd(3 * numVertices, 1);
    VectorXd qN_1 = VectorXd(3 * numVertices, 1);
    VectorXd vN = VectorXd(3 * numVertices, 1);
    VectorXd vN_1 = VectorXd(3 * numVertices, 1);

    qN = *qMatrix;
    vN = *vMatrix;

    //Precompute and prefactor term for global step
    MatrixXd precomputedFactor = MatrixXd(numVertices, numVertices); //m by m matrix for this factor

    precomputedFactor = *mMatrix / (timeStep * timeStep); //first term of this factor

    //Now have to iterate over all constraints to add to our precompute factor!
    for(int i = 0; i < constraints.size(); i++){

        double w = constraints[i]->getW();
        MatrixXd S = constraints[i]->getS();
        MatrixXd Stranspose = S.transpose();
        MatrixXd A = constraints[i]->getA();
        MatrixXd Atranspose = A.transpose(); //need to do this because of the way transpose works! (does not replace original)

        precomputedFactor += (w * Stranspose * Atranspose * A * S);

    }

    //Perform sparse (simplicialLLT?) Cholesky decomposition to get L and Ltranspose!
    LLT<MatrixXd, Lower> lltOfA(precomputedFactor); // compute the Cholesky decomposition of A
    MatrixXd L = lltOfA.matrixL(); // retrieve factor L  in the decomposition
    MatrixXd Ltranspose = L.transpose();

    //Set up some variables to deal with time
    int stepsPerFrame = (int)ceil(1 / (timeStep / (1 / (double)frameRate))); //calculate how many steps per frame we should have based on our desired frame rate and dt!
    int step = 0;
    int frame = 0;

    int numFrames = seconds * frameRate;

    if(printsOn == true){cout << "Simulation steps begin: " << endl;}

    //Run the sim until we have the desired number of frames
    while(frame < numFrames){

        /*----Algorithm 1 begin (from proj dyn paper)------*/

        // Line 1 : Calculate sn
        VectorXd sn(3 * numVertices, 1);

        if(printsOn == true){cout << "made sn vector" << endl;}

        sn = qN + (timeStep * vN) + ((timeStep * timeStep) * *invMassMatrix * *fExtMatrix);

        if(printsOn == true){cout << "line 1 complete" << endl;}

        // Line 2 : qn+1 = sn
        qN_1 = sn;

        if(printsOn == true){cout << "line 2 complete" << endl;}

        // Lines 3-7 : solver loop
        for (auto i = 0; i < solverIterations; i++){

            // Lines 4-6 : Local Step (calc p_i for each constraint C_i)
            //#pragma omp parallel
            //#pragma omp for //parallelize with openMP!
            for(int j = 0; j < constraints.size(); j++) {
                if (printsOn == true) { cout << "Processing constraint " << j << "..." << endl; }

                constraints[j]->projectConstraint(qN_1); //project the constraint based on our calculated q n+1

                if (printsOn == true) {
                    MatrixXd aMat, bMat, sMat, Dminv, Ds;
                    Matrix3d defGrad;
                    aMat = constraints[j]->getA();
                    bMat = constraints[j]->getB();
                    sMat = constraints[j]->getS();
                    defGrad = constraints[j]->getDefGrad();
                    Dminv = constraints[j]->getDmInv();
                    Ds = constraints[j]->getDs();

                    //cout << "A Matrix: " << endl << aMat << endl;
                    //cout << "B Matrix: " << endl << bMat << endl;
                    //cout << "S Matrix: " << endl << sMat << endl;
                    cout << "Ds of constraint " << j << " : " << endl << Ds << endl;
                    cout << "Dminv of constraint " << j << " : " << endl << Dminv << endl;
                    cout << "F of constraint " << j << " : " << endl << defGrad << endl;
                }
            }

            if(printsOn == true){cout << "local step complete" << endl;}

            // Line 7 : Global Step (solve linear system and find qN_1)
            qN_1 = solveLinearSystem(sn, L, Ltranspose); //call this function to solve the global step!

            if(printsOn == true){cout << "global step complete" << endl;}
        }

        if(printsOn == true){cout << "lines 3-7 complete" << endl;}

        //Ground Collision Checks and Corrections
        for(int i = 0; i < numVertices; i++){ //check each position for y < 0

            double yVal = qN_1(3*i + 1);

            if(yVal < 0){

                qN_1(3*i + 1) = 0;

            }

        }


        // Line 9: vn+1 = (qn+1 - qn) / h
        vN_1 = (qN_1 - qN) / timeStep;

        if(printsOn == true){cout << "line 9 complete" << endl;}

        //At end of this loop set qN = qN+1 because we are moving to the next time now for the next loop!
        qN = qN_1;

        //do the same for vN
        vN = vN_1;

        if(printsOn == true){cout << "finished sim step" << endl;}

        if (step % stepsPerFrame == 0) {

            //Save the particles, but in this version of the function we must just add them to the passed by reference vector of vectors!!
            cout << "INFO: Current Frame is " << frame << endl;

            frames.push_back(qN_1); //push back this frame

            frame = frame + 1; //update frame

        }

        step = step + 1;
    }

}


// global step. merge all projected points into a single set of points
MatrixXd QuikDeformer::solveLinearSystem(Eigen::MatrixXd sn, Eigen::MatrixXd L, Eigen::MatrixXd Ltranspose) {

    //First compute the right side of the expression to solve (our b term)
    MatrixXd b = MatrixXd(numVertices, 3); //m by m matrix for this factor

    b = (*mMatrix / (timeStep * timeStep)) * sn; //first term of this factor (notice this has sn multiplied, diff from Y!)

    //Now have to iterate over all constraints to add to our b term!
    for(int i = 0; i < constraints.size(); i++){

        double w = constraints[i]->getW();
        MatrixXd S = constraints[i]->getS();
        MatrixXd Stranspose = S.transpose();
        MatrixXd A = constraints[i]->getA();
        MatrixXd Atranspose = A.transpose(); //need to do this because of the way transpose works! (does not replace original)
        MatrixXd B = constraints[i]->getB();
        MatrixXd p = constraints[i]->getP();

        b += (w * Stranspose * Atranspose * B * p); //add term to b for this constraint

    }

    //First we must solve the equation Ly = b to get y!
    MatrixXd y = L.colPivHouseholderQr().solve(b);

    //Then we must solve the equation Ltranspose q = y to get q!
    MatrixXd q = Ltranspose.colPivHouseholderQr().solve(y);

    return q;
}

// writes data into a .obj file
void QuikDeformer::writeObj(const std::string &fileName, Eigen::VectorXd qMat) const {

    //TODO: Test this and make sure it actually reads our values from the position matrix we pass!

    std::ofstream out(fileName);

    out << "# obj file created by QuikDeformer" << std::endl << std::endl;

    // first write all the vector data
    for (int i = 0; i < numVertices; i++){
        out << "v  " << std::to_string(qMat(3*i + 0));
        out << "  " << std::to_string(qMat(3*i + 1));
        out << "  " << std::to_string(qMat(3*i + 2)) << std::endl;
    }
    out << std::endl;

    // then write all the face data
    for (auto& f : fragments){
        out << "f  " << std::to_string(f[0]);
        out << "  " << std::to_string(f[1]);
        out << "  " << std::to_string(f[2]) << std::endl;
    }
    out.close();
}


// write data into a .bgeo file
void QuikDeformer::writeBgeo(const std::string &fileName) const{
    // TODO: need to import Partio?
    /*
    Partio::ParticlesDataMutable* parts = Partio::create();
    Partio::ParticleAttribute positionHandle, velocityHandle, massHandle;
    massHandle = parts->addAttribute("mass", Partio::VECTOR, 1);
    positionHandle = parts->addAttribute("position", Partio::VECTOR, 3);
    velocityHandle = parts->addAttribute("velocity", Partio::VECTOR, 3);
    for (int i = 0; i < numParticles; i++){
        int idx = parts->addParticle();
        float* m = parts->dataWrite<float>(massHandle, idx);
        float* p = parts->dataWrite<float>(positionHandle, idx);
        float* v = parts->dataWrite<float>(velocityHandle, idx);
        m[0] = mass[i];
        for (int k = 0; k < 3; k++)
            p[k] = positions[i][k];
        for (int k = 0; k < 3; k++)
            v[k] = velocities[i][k];
    }

    Partio::write(fileName.c_str(), *parts);
    parts->release();
    */
}


