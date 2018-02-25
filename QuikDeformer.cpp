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

    // also add velocities
    for(int i = 0; i < numVertices; i++){
        velocities.emplace_back(Vector3d(0.0, 0.0, 0.0));
    }
}

// function called after readObj to construct the matrices we need
void QuikDeformer::setupMatrices(double mass) {
    // set up the q0 matrix
    qMatrix = new MatrixXd(numVertices, 3); //init our M by 3 matrix, q

    for(int i = 0; i < numVertices; i++){
        (*qMatrix)(i,0) = vertices[i][0];
        (*qMatrix)(i,1) = vertices[i][1];
        (*qMatrix)(i,2) = vertices[i][2]; //fill each row iteratively
    }

    // set up the mass matrix
    long numPoints = qMatrix->rows(); //eventually dynamically fill this when we are reading in an obj

    //Set the m x m diagonal mass matrix, M
    MatrixXd* temp = new MatrixXd(numPoints, numPoints);
    *temp = MatrixXd::Identity(numPoints, numPoints);
    *temp *= mass;
    mMatrix = temp;
}


// helper function to just see what our matrices look like
void QuikDeformer::printMatrices() const {
    cout << "Our q0 matrix is: " << endl << *qMatrix << endl;
    cout << "Our mass matrix M is: " << endl << *mMatrix << endl;
}


// adds constraints
void QuikDeformer::addConstraint(const std::string &type) {
    if (type == "strain"){
        constraints.push_back(new StrainConstraint(vertices, fragments));
    }
}


// runs the actual simulation and outputs the results appropriately
void QuikDeformer::runSimulation(double seconds, const std::string &outputFilePath) {
    // set up some initial data
    Vector3d gravity(0.0, -9.81, 0.0);

    // start main loop
    // step 1: sn = qn + h * vn + h^2 * mMatrix^-1 * force_external
    MatrixXd sn(numVertices, 3);
    // step 2: qn+1 = sn
    MatrixXd qn_1 = sn;

    // set up variables for calculating the right dt
    double time = 0.0;
    double frameTime = 1.0 / frameRate;
    int totalFrames = seconds * frameRate;

    for (int frame = 1; frame <= totalFrames; frame++){
        double currentFrameEndTime = frame * frameTime;

        // run the simulation until we're at the desired frame time
        while (time < currentFrameEndTime){
            double dt = std::min(timeStep, currentFrameEndTime - time);
            time += dt;

            MatrixXd qn_1 = MatrixXd();

            for (auto i = 0; i < solverIterations; i++){
                // step 5 (local step): calculate every set of points which satisfy every constraint
                vector<MatrixXd> pMatricies;
                for(auto& c : constraints){
                    pMatricies.emplace_back(c->Satisfy(dt));
                }

                // step 7 (global step): solve linear systems
                qn_1 = solveLinearSystems(sn, pMatricies);
            }

            // step 9: vn+1 = (qn+1 - qn) / h
        }

        // print out the frame
        writeBgeo(outputFilePath + std::to_string(frame) + ".bgeo");
    }
}


// writes data into a .obj file
void QuikDeformer::writeObj(const std::string &fileName) const {
    std::ofstream out(fileName);

    out << "# obj file created by QuikDeformer" << std::endl << std::endl;

    // first write all the vector data
    for (auto& v : vertices){
        out << "v  " << std::to_string(v[0]);
        out << "  " << std::to_string(v[1]);
        out << "  " << std::to_string(v[2]) << std::endl;
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


// global step. merge all projected points into a single set of points
MatrixXd QuikDeformer::solveLinearSystems(Eigen::MatrixXd sn, std::vector<Eigen::MatrixXd> pMatricies) {
    // TODO
    return MatrixXd();
}