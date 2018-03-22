#include "QuikDeformer.h"

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
        (*fExtMatrix)((3*i) + 1) = -9.81 * mass; //add gravity force to each point in our external force matrix
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


// adds constraints
void QuikDeformer::addConstraint(const std::string &type) {
    if (type == "strain"){
        constraints.push_back(new StrainConstraint(vertices, fragments));
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
        Dm(1,1) = (*qMatrix)(id2 + 0) - (*qMatrix)(id0 + 0); //bottom right = X2.y - X0.y

        MatrixXd Dminv = Dm.inverse();

        cout << "Dm for triangle " << i << " :" << Dm << endl;

    }

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
            for(int j = 0; j < constraints.size(); j++){
                constraints[j]->projectConstraint(qN_1); //project the constraint based on our calculated q n+1
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


