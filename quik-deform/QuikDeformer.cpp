#include "QuikDeformer.h"
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <math.h> /*acos*/

using namespace Eigen;
using namespace std;

#define PI 3.14159265

// Destructor
QuikDeformer::~QuikDeformer(){
    delete qMatrix;
    delete mMatrix;
    for (auto& c : constraints){
        delete c;
    }
}

// Function to read a given object file, fill a matrix q that is m X 3, and return the number of points total
void QuikDeformer::readObj(const std::string& fileName, int indexBase){

    int temp;

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
                ss >> temp;
                currFragment(i) = temp - indexBase; //subtract off the indexBase (0 or 1)
            }
            fragments.push_back(currFragment);
        }

    }

    numVertices = vertices.size();

    return;
}

void QuikDeformer::readVolumetric(const std::string& nodePath, const std::string& elePath, const std::string& facePath, int indexBase, bool scaleModel){

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

        if(scaleModel){

            double scaleFactor = 100;
            double dx = 0;
            double dy = 2;
            double dz = 0;

            currPoint(0) = (currPoint(0) / scaleFactor) + dx;
            currPoint(1) = (currPoint(1) / scaleFactor) + dy;
            currPoint(2) = (currPoint(2) / scaleFactor) + dz;

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
            currTet.push_back(x - indexBase); //subtract off the indexBase (0 or 1)
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
        int garbage, temp;
        ss >> garbage; //throw away first value (it's garbage)

        for (int i = 0; i < 3; i++){
            ss >> temp;
            currFragment(i) = temp - indexBase; //subtract off the indexBase (0 or 1)
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

void QuikDeformer::addWind(double wx, double wy, double wz, double windMag, bool windOsc, double oscAmp, double period){

    for(int i = 0; i < numVertices; i++){

        (*fExtMatrix)((3*i) + 0) += (wx * windMag);
        (*fExtMatrix)((3*i) + 1) += (wy * windMag); //update the forces to include wind effects in the specified direction and magnitude
        (*fExtMatrix)((3*i) + 2) += (wz * windMag);

    }

    //Set variables
    windX = wx;
    windY = wy;
    windZ = wz;
    windMagnitude = windMag;
    windOscillates = windOsc;
    oscillationAmplitude = oscAmp;
    windPeriod = period;

}

void QuikDeformer::updateWind(){

    //assumes we have windOsc ON, thus all params should be set!
    //start by grabbing the current elapsed time since wind started
    double elapsedTime = ( std::clock() - getStartTime() ) / (double) CLOCKS_PER_SEC;

    //Use function f(t) = A * sin( t / period )
    double func = oscillationAmplitude * sin(elapsedTime / windPeriod);


    //THIS ASSUMES THAT GRAVITY IS BASICALLY ALWAYS THE ONLY POSSIBLE FORCE (OTHER THAN WIND)!!!
    for(int i = 0; i < numVertices; i++){

        (*fExtMatrix)((3*i) + 0) = (windX * windMagnitude) + func;
        (*fExtMatrix)((3*i) + 1) = (windY * windMagnitude) + func;
        (*fExtMatrix)((3*i) + 2) = (windZ * windMagnitude) + func;

        if(gravityOn){
            (*fExtMatrix)((3*i) + 1) -= 9.81; //add gravity if need be
        }

    }

}

void QuikDeformer::addCollisionPlanes(const std::vector<Eigen::Vector3d> pCenters, const std::vector<double> pLengths, const std::vector<double> pWidths, const std::vector<Eigen::Vector3d> pNormals, double frictionCoeff){

    planeCenters = pCenters;
    planeLengths = pLengths;
    planeWidths = pWidths;
    planeNormals = pNormals;
    frictionCoefficient = frictionCoeff;

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

    //cout << "sMatrix is: " << endl << sMatrix << endl;

    constraints.push_back(new PositionConstraint(weight, sMatrix, p));

}

void QuikDeformer::add2DStrainConstraints(double strain2DWeight){

    //For each triangle in mesh
    for(int i = 0; i < fragments.size(); i++){

        Vector3i currTriangle = fragments[i];

        //Compute DmHat for triangle
        MatrixXd DmHat = MatrixXd(3,2).setZero();

        int id0 = 3 * currTriangle[0];
        int id1 = 3 * currTriangle[1];
        int id2 = 3 * currTriangle[2];

        DmHat(0,0) = (*qMatrix)(id1 + 0) - (*qMatrix)(id0 + 0); //top left = X1.x - X0.x
        DmHat(1,0) = (*qMatrix)(id1 + 1) - (*qMatrix)(id0 + 1); //middle left = X1.y - X0.y
        DmHat(2,0) = (*qMatrix)(id1 + 2) - (*qMatrix)(id0 + 2); //bottom left = X1.z - X0.z

        DmHat(0,1) = (*qMatrix)(id2 + 0) - (*qMatrix)(id0 + 0); //top right = X2.x - X0.x
        DmHat(1,1) = (*qMatrix)(id2 + 1) - (*qMatrix)(id0 + 1); //middle right = X2.y - X0.y
        DmHat(2,1) = (*qMatrix)(id2 + 2) - (*qMatrix)(id0 + 2); //bottom right = X2.z - X0.z

        //Now take the QR decomposition of DmHat
        HouseholderQR<MatrixXd> QRDecomp(DmHat);
        MatrixXd R = QRDecomp.matrixQR().triangularView<Upper>(); //grab the R matrix (upper triangular)
        MatrixXd Q = QRDecomp.householderQ();

        if(abs(Q.determinant() - 1) >= 1e-5){
            //cout << "Correcting Q and R for DmHat QR Error: det(Q) \n" << endl;
            Q.col(1) *= -1;
            R(1,1) *= -1;
        }

        /*/------------------------------------//
        //CHECK THAT QR Decomp IS CORRECT!!!
        Eigen::MatrixXd I3 = Eigen::MatrixXd(3, 3).setIdentity();
        MatrixXd temp33 = MatrixXd(3,3);
        MatrixXd temp32 = MatrixXd(3,2);
        temp33 = Q * Q.transpose() - I3;
        temp32 = Q*R - DmHat;
        if(temp33(0,0) >= 1e-5){ cout << "DmHat QR ERROR: Q * Qt \n" << endl; cout << "Q*Qt: \n" << Q*Q.transpose() << endl;}
        if(temp33(0,1) >= 1e-5){ cout << "DmHat QR ERROR: Q * Qt \n" << endl; cout << "Q*Qt: \n" << Q*Q.transpose() << endl;}
        if(temp33(0,2) >= 1e-5){ cout << "DmHat QR ERROR: Q * Qt \n" << endl; cout << "Q*Qt: \n" << Q*Q.transpose() << endl;}
        if(temp33(1,0) >= 1e-5){ cout << "DmHat QR ERROR: Q * Qt \n" << endl; cout << "Q*Qt: \n" << Q*Q.transpose() << endl;}
        if(temp33(1,1) >= 1e-5){ cout << "DmHat QR ERROR: Q * Qt \n" << endl; cout << "Q*Qt: \n" << Q*Q.transpose() << endl;}
        if(temp33(1,2) >= 1e-5){ cout << "DmHat QR ERROR: Q * Qt \n" << endl; cout << "Q*Qt: \n" << Q*Q.transpose() << endl;}
        if(temp33(2,0) >= 1e-5){ cout << "DmHat QR ERROR: Q * Qt \n" << endl; cout << "Q*Qt: \n" << Q*Q.transpose() << endl;}
        if(temp33(2,1) >= 1e-5){ cout << "DmHat QR ERROR: Q * Qt \n" << endl; cout << "Q*Qt: \n" << Q*Q.transpose() << endl;}
        if(temp33(2,2) >= 1e-5){ cout << "DmHat QR ERROR: Q * Qt \n" << endl; cout << "Q*Qt: \n" << Q*Q.transpose() << endl;}
        if(temp32(0,0) >= 1e-5){ cout << "DmHat QR ERROR: QR - DmHat \n" << endl; cout << "QR: \n" << Q*R << endl; cout << "DmHat: \n" << DmHat << endl; }
        if(temp32(0,1) >= 1e-5){ cout << "DmHat QR ERROR: QR - DmHat \n" << endl; cout << "QR: \n" << Q*R << endl; cout << "DmHat: \n" << DmHat << endl; }
        if(temp32(1,0) >= 1e-5){ cout << "DmHat QR ERROR: QR - DmHat \n" << endl; cout << "QR: \n" << Q*R << endl; cout << "DmHat: \n" << DmHat << endl; }
        if(temp32(1,1) >= 1e-5){ cout << "DmHat QR ERROR: QR - DmHat \n" << endl; cout << "QR: \n" << Q*R << endl; cout << "DmHat: \n" << DmHat << endl; }
        if(temp32(2,0) >= 1e-5){ cout << "DmHat QR ERROR: QR - DmHat \n" << endl; cout << "QR: \n" << Q*R << endl; cout << "DmHat: \n" << DmHat << endl; }
        if(temp32(2,1) >= 1e-5){ cout << "DmHat QR ERROR: QR - DmHat \n" << endl; cout << "QR: \n" << Q*R << endl; cout << "DmHat: \n" << DmHat << endl; }
        if(abs(Q.determinant() - 1) >= 1e-5){
            cout << "DmHat QR ERROR: det(Q) \n" << endl;
        }
        if(R(1,0) >= 1e-5){
            cout << "DmHat QR ERROR: R(1,0) \n" << endl;
        }
        if(R(2,0) >= 1e-5){
            cout << "DmHat QR ERROR: R(2,0) \n" << endl;
        }
        if(R(2,1) >= 1e-5){
            cout << "DmHat QR ERROR: R(2,1) \n" << endl;
        }
        //------------------------------------/*/

        //Set Dm as the top 2x2 of R! (throw out the zeros in third row)
        MatrixXd Dm = MatrixXd(2,2).setZero();
        Dm(0,0) = R(0,0); //top left
        Dm(1,0) = R(1,0); //bottom left
        Dm(0,1) = R(0,1); // top right
        Dm(1,1) = R(1,1); //bottom right

        MatrixXd Dminv = Dm.inverse();

        double area = abs(Dm.determinant()) * 0.5;

        //SET S MATRIX
        MatrixXd sMat = MatrixXd(9, 3 * numVertices).setZero(); //9 by 3m
        for(int j = 0; j < 3; j++){
            //fill in S matrix for each index
            sMat((3 * j + 0), (3 * currTriangle[j] + 0)) = 1;
            sMat((3 * j + 1), (3 * currTriangle[j] + 1)) = 1;
            sMat((3 * j + 2), (3 * currTriangle[j] + 2)) = 1;
        }

        //SET BP MATRIX (okay to init to all zero)
        VectorXd Bp = VectorXd(6,1).setZero(); //6 by 1

        //SET A MATRIX
        MatrixXd aMat = MatrixXd(6,9).setZero();
        buildTriangleStrainA(aMat, Dminv); //build aMat from DmInv!

        //Set B = Identity (6 by 6)
        Eigen::MatrixXd bMat = Eigen::MatrixXd(6, 6).setIdentity(); //identity since what we are passing for p is actually Bp

        //Form currTriangle into an int vector
        vector<int> triIndeces;
        triIndeces.push_back(currTriangle[0]);
        triIndeces.push_back(currTriangle[1]);
        triIndeces.push_back(currTriangle[2]);

        /*
        cout << "Weight: " << strain2DWeight << endl;
        cout << "sMat: \n" << sMat << endl;
        cout << "Bp: \n" << Bp << endl;
        cout << "aMat: \n" << aMat << endl;
        cout << "bMat: \n" << bMat << endl;
        cout << "area: " << area << endl;
        cout << "Dminv: \n" << Dminv << endl;
        cout << "DmHat: \n" << DmHat << endl;
        */

        constraints.push_back(new TriangleStrainConstraint(strain2DWeight, sMat, Bp, aMat, bMat, triIndeces, area, Dminv));

        //cout << "Finished triangle " << i << " :" << endl;

    }

}

//Build A matrix from DmInverse (as G)
void QuikDeformer::buildTriangleStrainA(MatrixXd& A_matrix, MatrixXd& G){

    A_matrix.setZero();
    double g, h, i, j;
    g = G(0,0); //top left of G
    h = G(0,1); //top right of G
    i = G(1,0); //bottom left of G
    j = G(1,1); //bottom right of G

    A_matrix(0,0) = -g - i;    A_matrix(0,3) = g;    A_matrix(0,6) = i; //defines the three entries in row one of A
    A_matrix(1,0) = -h - j;    A_matrix(1,3) = h;    A_matrix(1,6) = j;

    A_matrix(2,1) = -g - i;    A_matrix(2,4) = g;    A_matrix(2,7) = i; //defines the three entries in row three of A
    A_matrix(3,1) = -h - j;    A_matrix(3,4) = h;    A_matrix(3,7) = j;

    A_matrix(4,2) = -g - i;    A_matrix(4,5) = g;    A_matrix(4,8) = i; //defines the entries in row five of A
    A_matrix(5,2) = -h - j;    A_matrix(5,5) = h;    A_matrix(5,8) = j;

    return;
}


void QuikDeformer::add3DStrainConstraints(double strain3DWeight){

    //For each tetrahedron in mesh
    for(int i = 0; i < tetrahedrons.size(); i++){

        vector<int> currTet = tetrahedrons[i];

        //Compute Dm
        MatrixXd Dm = MatrixXd(3,3);

        int id0 = 3 * (currTet[0]); //might have to subtract 1 if NOT 0-indexed
        int id1 = 3 * (currTet[1]);
        int id2 = 3 * (currTet[2]);
        int id3 = 3 * (currTet[3]);

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

        double volume = abs(Dm.determinant()) / 6.0;

        //cout << "Dm for tet " << i << " :" << endl << Dm << endl;

        //SET S MATRIX
        MatrixXd sMat = MatrixXd(12, 3 * numVertices).setZero();
        for(int j = 0; j < currTet.size(); j++){
            //fill in S matrix for each index
            sMat((3 * j + 0), (3 * currTet[j] + 0)) = 1;
            sMat((3 * j + 1), (3 * currTet[j] + 1)) = 1;
            sMat((3 * j + 2), (3 * currTet[j] + 2)) = 1;
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

        //cout << "Finished tet " << i << " :" << endl;

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



// Runs our sim and outputs OBJECT FILES
void QuikDeformer::runSimulation(double seconds, const std::string &outputFilePath, bool printsOn) {

    setStartTime(std::clock()); //start the clock
    setPrintsOn(printsOn);

    //setup variables for current value and for "next" or "n+1" value!
    VectorXd qN = VectorXd(3 * numVertices, 1);
    VectorXd qN_1 = VectorXd(3 * numVertices, 1);
    VectorXd vN = VectorXd(3 * numVertices, 1);
    VectorXd vN_1 = VectorXd(3 * numVertices, 1);

    qN = *qMatrix;
    vN = *vMatrix;

    //Precompute and prefactor term for global step
    MatrixXd precomputedFactor = MatrixXd(numVertices * 3, numVertices * 3); //3m by 3m matrix for this factor
    SparseMatrix<double> m_system(numVertices*3, numVertices*3); //NEW: init sparse matrix for precomputation factor

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

    //Now we must turn our precomputed factor into a sparse matrix!
    m_system = precomputedFactor.sparseView();

    //Prune like Fanfu does in his code, supresses values smaller than some threshold (not sure why pass 0s though)
    m_system.prune(0,0);

    //Now set up the system by having the simplicialLLT thing do cholesky!
    m_solver.compute(m_system);

    //Set up some variables to deal with time
    int stepsPerFrame = (int)ceil(1 / (timeStep / (1 / (double)frameRate))); //calculate how many steps per frame we should have based on our desired frame rate and dt!
    int step = 0;
    int frame = 0;

    int numFrames = seconds * frameRate;

    cout << "Simulation steps per frame: " << stepsPerFrame << endl;

    if(printsOn == true){cout << "Simulation steps begin: " << endl;}

    //cout << "Precomputation time: " << ( std::clock() - getStartTime() ) / (double) CLOCKS_PER_SEC << " seconds" << endl;

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

            clock_t timeBeforeLocal = clock();

            // Lines 4-6 : Local Step (calc p_i for each constraint C_i)
            #pragma omp parallel for num_threads(4)
            for(int j = 0; j < constraints.size(); j++) {
                //cout << "Solving constraint " << j << endl;
                constraints[j]->projectConstraint(qN_1); //project the constraint based on our calculated q n+1
            }

            //cout << "Local Solver time: " << ( std::clock() - timeBeforeLocal ) / (double) CLOCKS_PER_SEC << " seconds" << endl;

            if(printsOn == true){cout << "local step complete" << endl;}

            clock_t timeBeforeGlobal = clock();

            // Line 7 : Global Step (solve linear system and find qN_1)
            qN_1 = solveLinearSystem(sn);

            //cout << "Global Solver time: " << ( std::clock() - timeBeforeGlobal ) / (double) CLOCKS_PER_SEC << " seconds" << endl;

            if(printsOn == true){cout << "global step complete" << endl;}
        }

        if(printsOn == true){cout << "lines 3-7 complete" << endl;}

        //Ground Collision Checks and Corrections
        Vector3d inputPoint, outputPoint;
        vector<int> collisionIndeces;
        collisionIndeces.clear(); //make sure this is empty
        for(int i = 0; i < numVertices; i++){ //check each position for y < 0

            //Calculate the indeces
            int id0 = 3*i + 0;
            int id1 = 3*i + 1;
            int id2 = 3*i + 2;

            //Construct input, and use function to get output
            inputPoint = Vector3d(qN_1(id0), qN_1(id1), qN_1(id2));
            outputPoint = planeCollision(inputPoint);

            //Now check if we DID correct this point or not to see if we should damp the velocity or not
            if((inputPoint[0] != outputPoint[0]) || (inputPoint[1] != outputPoint[1]) || (inputPoint[2] != outputPoint[2])){

                //Add this index to the collision list for this step!
                collisionIndeces.push_back(i);

            }

            //Assign output points to our position matrix!
            qN_1(id0) = outputPoint[0];
            qN_1(id1) = outputPoint[1];
            qN_1(id2) = outputPoint[2];

        }

        // Line 9: vn+1 = (qn+1 - qn) / h
        vN_1 = (qN_1 - qN) / timeStep;

        if(printsOn == true){cout << "line 9 complete" << endl;}

        //Now we must add frictional damping based on the points that are in collision right now!
        for(int i = 0; i < collisionIndeces.size(); i++){
            //Calculate the indeces
            int j = collisionIndeces[i];
            int id0 = 3*j + 0;
            int id1 = 3*j + 1;
            int id2 = 3*j + 2;

            vN_1(id0) *= frictionCoefficient;
            vN_1(id1) *= frictionCoefficient;
            vN_1(id2) *= frictionCoefficient;

            //cout << "Damped velocity of vertex " << j << endl;

        }

        //At end of this loop set qN = qN+1 because we are moving to the next time now for the next loop!
        qN = qN_1;

        //do the same for vN
        vN = vN_1;

        if(printsOn == true){cout << "finished sim step" << endl;}

        if (step % stepsPerFrame == 0) {

            //Get time
            double totalDuration;
            double currElapsedTime = getElapsedTime();
            clock_t start = getStartTime();
            totalDuration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
            //Current frame time = totalDuration - currElapsedTime
            double currFrameTime;
            currFrameTime = totalDuration - currElapsedTime;

            //Save the particles!
            cout << "INFO: Frame " << frame << " took " << currFrameTime << " seconds to process " << endl;

            setElapsedTime(totalDuration); //now set the new elapsed time

            writeObj(outputFilePath + std::to_string(frame) + ".obj", qN_1);
            frame = frame + 1; //update frame

        }

        //UPDATE WIND IF NEEDED
        if(windOscillates){
            updateWind();
        }

        step = step + 1;
    }

    double elapsedTime = getElapsedTime();
    cout << "Average frame time: " << elapsedTime / frame << " seconds" << endl;

}

//Runs our simulation and FILLS "FRAMES" (no writes)
void QuikDeformer::runSimulation(double seconds, bool printsOn, std::vector<Eigen::VectorXd>& frames){

    setStartTime(std::clock()); //start the clock
    setPrintsOn(printsOn);

    //setup variables for current value and for "next" or "n+1" value!
    VectorXd qN = VectorXd(3 * numVertices, 1);
    VectorXd qN_1 = VectorXd(3 * numVertices, 1);
    VectorXd vN = VectorXd(3 * numVertices, 1);
    VectorXd vN_1 = VectorXd(3 * numVertices, 1);

    qN = *qMatrix;
    vN = *vMatrix;

    //Precompute and prefactor term for global step
    MatrixXd precomputedFactor = MatrixXd(numVertices * 3, numVertices * 3); //3m by 3m matrix for this factor
    SparseMatrix<double> m_system(numVertices*3, numVertices*3); //NEW: init sparse matrix for precomputation factor

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

    //Now we must turn our precomputed factor into a sparse matrix!
    m_system = precomputedFactor.sparseView();

    //Prune like Fanfu does in his code, supresses values smaller than some threshold (not sure why pass 0s though)
    m_system.prune(0,0);

    //Now set up the system by having the simplicialLLT thing do cholesky!
    m_solver.compute(m_system);

    //Set up some variables to deal with time
    int stepsPerFrame = (int)ceil(1 / (timeStep / (1 / (double)frameRate))); //calculate how many steps per frame we should have based on our desired frame rate and dt!
    int step = 0;
    int frame = 0;

    int numFrames = seconds * frameRate;

    cout << "Simulation steps per frame: " << stepsPerFrame << endl;

    if(printsOn == true){cout << "Simulation steps begin: " << endl;}

    //cout << "Precomputation time: " << ( std::clock() - getStartTime() ) / (double) CLOCKS_PER_SEC << " seconds" << endl;

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

            clock_t timeBeforeLocal = clock();

            // Lines 4-6 : Local Step (calc p_i for each constraint C_i)
            #pragma omp parallel for num_threads(4)
            for(int j = 0; j < constraints.size(); j++) {
                constraints[j]->projectConstraint(qN_1); //project the constraint based on our calculated q n+1
            }

            //cout << "Local Solver time: " << ( std::clock() - timeBeforeLocal ) / (double) CLOCKS_PER_SEC << " seconds" << endl;

            if(printsOn == true){cout << "local step complete" << endl;}

            clock_t timeBeforeGlobal = clock();

            // Line 7 : Global Step (solve linear system and find qN_1)
            qN_1 = solveLinearSystem(sn);

            //cout << "Global Solver time: " << ( std::clock() - timeBeforeGlobal ) / (double) CLOCKS_PER_SEC << " seconds" << endl;

            if(printsOn == true){cout << "global step complete" << endl;}
        }

        if(printsOn == true){cout << "lines 3-7 complete" << endl;}

        //Ground Collision Checks and Corrections
        Vector3d inputPoint, outputPoint;
        vector<int> collisionIndeces;
        collisionIndeces.clear(); //make sure this is empty
        for(int i = 0; i < numVertices; i++){ //check each position for y < 0

            //Calculate the indeces
            int id0 = 3*i + 0;
            int id1 = 3*i + 1;
            int id2 = 3*i + 2;

            //Construct input, and use function to get output
            inputPoint = Vector3d(qN_1(id0), qN_1(id1), qN_1(id2));
            outputPoint = planeCollision(inputPoint);

            //Now check if we DID correct this point or not to see if we should damp the velocity or not
            if((inputPoint[0] != outputPoint[0]) || (inputPoint[1] != outputPoint[1]) || (inputPoint[2] != outputPoint[2])){

                //Add this index to the collision list for this step!
                collisionIndeces.push_back(i);

            }

            //Assign output points to our position matrix!
            qN_1(id0) = outputPoint[0];
            qN_1(id1) = outputPoint[1];
            qN_1(id2) = outputPoint[2];

        }


        // Line 9: vn+1 = (qn+1 - qn) / h
        vN_1 = (qN_1 - qN) / timeStep;

        if(printsOn == true){cout << "line 9 complete" << endl;}

        //Now we must add frictional damping based on the points that are in collision right now!
        for(int i = 0; i < collisionIndeces.size(); i++){
            //Calculate the indeces
            int j = collisionIndeces[i];
            int id0 = 3*j + 0;
            int id1 = 3*j + 1;
            int id2 = 3*j + 2;

            vN_1(id0) *= frictionCoefficient;
            vN_1(id1) *= frictionCoefficient;
            vN_1(id2) *= frictionCoefficient;

        }

        //At end of this loop set qN = qN+1 because we are moving to the next time now for the next loop!
        qN = qN_1;

        //do the same for vN
        vN = vN_1;

        if(printsOn == true){cout << "finished sim step" << endl;}

        if (step % stepsPerFrame == 0) {

            //Get time
            double totalDuration;
            double currElapsedTime = getElapsedTime();
            clock_t start = getStartTime();
            totalDuration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
            //Current frame time = totalDuration - currElapsedTime
            double currFrameTime;
            currFrameTime = totalDuration - currElapsedTime;

            //Save the particles!
            cout << "INFO: Frame " << frame << " took " << currFrameTime << " seconds to process " << endl;

            setElapsedTime(totalDuration); //now set the new elapsed time

            //Save the particles, but in this version of the function we must just add them to the passed by reference vector of vectors!!
            frames.push_back(qN_1); //push back this frame

            frame = frame + 1; //update frame

        }

        //UPDATE WIND IF NEEDED
        if(windOscillates) {
            updateWind();
        }

        step = step + 1;
    }

    double elapsedTime = getElapsedTime();
    cout << "Average frame time: " << elapsedTime / frame << " seconds" << endl;

}

Vector3d QuikDeformer::planeCollision(Eigen::Vector3d p){

    Vector3d newPoint = Vector3d(p[0], p[1], p[2]); //set new point to be the input point by default
    Vector3d rayP, center, normal, adjustment;
    double numerator, denom, temp, angle, multiplier;

    //Iterate over every plane, check if above/below, if below adjust the newPoint!
    for(int i = 0; i < planeCenters.size(); i++){

        center = planeCenters[i];
        normal = planeNormals[i].normalized();

        rayP = p - center; //ray from center to input point p

        numerator = normal.dot(rayP);
        denom = normal.norm() * rayP.norm();

        temp = numerator/denom;

        //Clamp to range of -1 to 1
        if(temp < -1){
            temp = -1;
        } else if(temp > 1){
            temp = 1;
        }

        angle = acos(temp) * 180.0 / PI;

        //cout << "Angle: " << angle << endl;

        if((angle > 90.0) && (angle < 270)){
            //point BELOW plane, must fix
            adjustment = normal.normalized(); //get the normal and normalize it

            //Determine what the "angle from plane" is
            if(angle < 180){
                angle = angle - 90;

            } else if (angle == 180){
                angle = 90; //so sin is 1
            } else{
                angle = 270 - angle;
            }

            multiplier = sin(angle * (PI / 180.0)) * rayP.norm();

            //cout << "Angle: " << angle << endl;
            //cout << "Norm: " << rayP.norm() << endl;
            //cout << "Multiplier: " << multiplier << endl;

            adjustment[0] = adjustment[0] * multiplier;
            adjustment[1] = adjustment[1] * multiplier;
            adjustment[2] = adjustment[2] * multiplier;

            newPoint = newPoint + adjustment; //update point to be moved up to surface

        }
        else{
            //point above plane, all good just continue loop!
            continue;
        }

    }

    //cout << "New point: " << newPoint << endl;

    return newPoint;
}

// FAST VERSION: global step. merge all projected points into a single set of points
MatrixXd QuikDeformer::solveLinearSystem(Eigen::MatrixXd sn) {

    clock_t globalPrecomputeTime = clock();

    //First compute the right side of the expression to solve (our b term)
    MatrixXd b = MatrixXd(numVertices * 3, 1); //3m by 1 matrix for this factor

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

    //cout << "Global Solver precompute time: " << ( std::clock() - globalPrecomputeTime ) / (double) CLOCKS_PER_SEC << " seconds" << endl;

    clock_t beforeSolve = clock();

    //Do the solve!
    MatrixXd q = m_solver.solve(b);

    //cout << "Global Solver time: " << ( std::clock() - beforeSolve ) / (double) CLOCKS_PER_SEC << " seconds" << endl;

    return q;
}


// SLOW VERSION: global step. merge all projected points into a single set of points
MatrixXd QuikDeformer::solveLinearSystem(Eigen::MatrixXd sn, Eigen::MatrixXd L, Eigen::MatrixXd Ltranspose) {

    clock_t globalPrecomputeTime = clock();

    //First compute the right side of the expression to solve (our b term)
    MatrixXd b = MatrixXd(numVertices * 3, 1); //3m by 1 matrix for this factor

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

    //cout << "Global Solver precompute time: " << ( std::clock() - globalPrecomputeTime ) / (double) CLOCKS_PER_SEC << " seconds" << endl;

    clock_t beforeFirstSolve = clock();

    //First we must solve the equation Ly = b to get y!
    MatrixXd y = L.colPivHouseholderQr().solve(b);

    //cout << "Global Solver first solve time: " << ( std::clock() - beforeFirstSolve ) / (double) CLOCKS_PER_SEC << " seconds" << endl;

    clock_t beforeSecondSolve = clock();

    //Then we must solve the equation Ltranspose q = y to get q!
    MatrixXd q = Ltranspose.colPivHouseholderQr().solve(y);

    //cout << "Global Solver second solve time: " << ( std::clock() - beforeSecondSolve ) / (double) CLOCKS_PER_SEC << " seconds" << endl;

    return q;
}

// writes data into a .obj file
void QuikDeformer::writeObj(const std::string &fileName, Eigen::VectorXd qMat) const {

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
        out << "f  " << std::to_string(f[0] + 1);
        out << "  " << std::to_string(f[1] + 1);
        out << "  " << std::to_string(f[2] + 1) << std::endl;
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


