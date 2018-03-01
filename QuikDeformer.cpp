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
    qMatrix = new MatrixXd(numVertices, 3); //init our M by 3 matrix, qMatrix
    vMatrix = new MatrixXd(numVertices, 3); //init our M by 3 matrix, vMatrix
    fExtMatrix = new MatrixXd(numVertices, 3); //init our M by 3 matrix, fExtMatrix


    for(int i = 0; i < numVertices; i++){
        (*qMatrix)(i,0) = vertices[i][0];
        (*qMatrix)(i,1) = vertices[i][1];
        (*qMatrix)(i,2) = vertices[i][2]; //fill each row iteratively

        (*vMatrix)(i,0) = vx;
        (*vMatrix)(i,1) = vy;
        (*vMatrix)(i,2) = vz; //fill each row iteratively

        (*fExtMatrix)(i,0) = 0;
        (*fExtMatrix)(i,1) = -9.81; //add gravity force to each point in our external force matrix
        (*fExtMatrix)(i,2) = 0;
    }

    // set up the mass matrix
    long numPoints = qMatrix->rows(); //eventually dynamically fill this when we are reading in an obj

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


// runs the actual simulation and outputs the results appropriately
void QuikDeformer::runSimulation(double seconds, const std::string &outputFilePath) {

    //setup variables for current value and for "next" or "n+1" value!
    MatrixXd qN = MatrixXd(numVertices, 3);
    MatrixXd qN_1 = MatrixXd(numVertices, 3);
    MatrixXd vN = MatrixXd(numVertices, 3);
    MatrixXd vN_1 = MatrixXd(numVertices, 3);

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


    // set up variables for calculating the right dt
    double time = 0.0;
    double frameTime = 1.0 / frameRate;
    int totalFrames = seconds * frameRate;

    for (int frame = 1; frame <= totalFrames; frame++){
        double currentFrameEndTime = frame * frameTime;

        // run the simulation until we're at the desired frame time
        while (time < currentFrameEndTime){

            //update time
            double dt = std::min(timeStep, currentFrameEndTime - time);
            time += dt;

            /*----Algorithm 1 begin (from proj dyn paper)------*/

            // Line 1 : Calculate sn
            MatrixXd sn(numVertices, 3);
            sn = qN + (timeStep * vN) + ((timeStep * timeStep) * *invMassMatrix * *fExtMatrix);

            // Line 2 : qn+1 = sn
            qN_1 = sn;

            // Lines 3-7 : solver loop
            for (auto i = 0; i < solverIterations; i++){

                // Lines 4-6 : Local Step (calc p_i for each constraint C_i)
                for(int j = 0; j < constraints.size(); j++){
                    constraints[j]->projectConstraint(qN_1); //project the constraint based on our calculated q n+1
                }

                // Line 7 : Global Step (solve linear system and find qN_1)
                qN_1 = solveLinearSystem(sn); //call this function to solve the global step!
            }

            // Line 9: vn+1 = (qn+1 - qn) / h
            vN_1 = (qN_1 - qN) / timeStep;

            //At end of this loop set qN = qN+1 because we are moving to the next time now for the next loop!
            qN = qN_1;
        }

        // print out the frame
        //writeObj(outputFilePath + std::to_string(frame) + ".obj", qN_1);
    }

}

// global step. merge all projected points into a single set of points
MatrixXd QuikDeformer::solveLinearSystem(Eigen::MatrixXd sn) {

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

    



    return MatrixXd();
}

// writes data into a .obj file
void QuikDeformer::writeObj(const std::string &fileName, Eigen::MatrixXd qMat) const {

    //TODO: Test this and make sure it actually reads our values from the position matrix we pass!

    std::ofstream out(fileName);

    out << "# obj file created by QuikDeformer" << std::endl << std::endl;

    // first write all the vector data
    for (int i = 0; i < numVertices; i++){
        out << "v  " << std::to_string(qMat(i,0));
        out << "  " << std::to_string(qMat(i,1));
        out << "  " << std::to_string(qMat(i,2)) << std::endl;
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


