#include "QuikDeformNode.h"
#include "global.h"
#include <istream>
#include <iostream>

// ---------------------------------------
//
//         Helper functions 
//
// ---------------------------------------


// macro for checking errors
#define McheckErr(stat,msg)			\
	if ( MS::kSuccess != stat ) {	\
		cerr << msg;				\
		return MS::kFailure;		\
	}


// extract original data from the mesh
void getMeshData(const MFnMesh& mesh,
	std::vector<Eigen::Vector3d>& originalVertices,
	std::vector<Eigen::Vector3i>& originalTriangles) {

	// reset all input containers
	originalVertices.clear();
	originalTriangles.clear();

	// -------------------------------
	// get vertices
	// -------------------------------
	// TODO: vertex data need to also include any transforms done to it like scaling 
	// maybe just also copy the transform? or apply the transform to the vertices after reading it?
	MPointArray meshVertices;
	mesh.getPoints(meshVertices);
	for (auto i = 0u; i < meshVertices.length(); i++) {
		originalVertices.emplace_back(Eigen::Vector3d(meshVertices[i][0], meshVertices[i][1], meshVertices[i][2]));
	}

	// -------------------------------
	// get triangles
	// -------------------------------
	MIntArray triangleCounts, triangleIndices;
	mesh.getTriangles(triangleCounts, triangleIndices);

	// triangleCounts contains number of triangles per face
	// sum it up to get total number of triangles
	int triangleNum = 0;
	for (auto i = 0u; i < triangleCounts.length(); i++) {
		triangleNum += triangleCounts[i];
	}

	for (int i = 0; i < triangleNum; i++) {
		originalTriangles.emplace_back(Eigen::Vector3i(triangleIndices[i * 3],
			triangleIndices[i * 3 + 1],
			triangleIndices[i * 3 + 2]));
	}
}


// extract original data from the mesh and also return a tetrahedralized version
void getTetMeshData(const MFnMesh& mesh,
	std::vector<Eigen::Vector3d>& originalVertices,
	std::vector<Eigen::Vector3i>& originalTriangles,
	std::vector<Eigen::Vector3d>& tetVertices,
	std::vector<Eigen::Vector3i>& tetTriangles,
	std::vector<std::vector<int>>& tetTetrahedrons,
	double tetVolume,
	bool printVals) {
	// reset all input containers
	originalVertices.clear();
	originalTriangles.clear();
	tetVertices.clear();
	tetTriangles.clear();
	tetTetrahedrons.clear();

	// -------------------------------
	// get vertices
	// -------------------------------
	// TODO: vertex data need to also include any transforms done to it like scaling 
	// maybe just also copy the transform? or apply the transform to the vertices after reading it?
	MPointArray meshVertices;
	mesh.getPoints(meshVertices);
	for (auto i = 0u; i < meshVertices.length(); i++) {
		originalVertices.emplace_back(Eigen::Vector3d(meshVertices[i][0], meshVertices[i][1], meshVertices[i][2]));
	}

	// -------------------------------
	// get triangles
	// -------------------------------
	MIntArray triangleCounts, triangleIndices;
	mesh.getTriangles(triangleCounts, triangleIndices);

	// triangleCounts contains number of triangles per face
	// sum it up to get total number of triangles
	int triangleNum = 0;
	for (auto i = 0u; i < triangleCounts.length(); i++) {
		triangleNum += triangleCounts[i];
	}

	for (int i = 0; i < triangleNum; i++) {
		originalTriangles.emplace_back(Eigen::Vector3i(triangleIndices[i * 3],
			triangleIndices[i * 3 + 1],
			triangleIndices[i * 3 + 2]));
	}

	// set up tetgen input/output
	tetgenio tetInput, tetOutput;
	tetgenio::facet *f;
	tetgenio::polygon *p;
	tetInput.firstnumber = 0;

	// populate the points
	tetInput.numberofpoints = meshVertices.length();
	tetInput.pointlist = new REAL[tetInput.numberofpoints * 3];
	for (int i = 0; i < tetInput.numberofpoints; i++) {
		tetInput.pointlist[i * 3] = meshVertices[i][0];
		tetInput.pointlist[i * 3 + 1] = meshVertices[i][1];
		tetInput.pointlist[i * 3 + 2] = meshVertices[i][2];
	}

	// populate the facets
	tetInput.numberoffacets = (int)originalTriangles.size();
	tetInput.facetlist = new tetgenio::facet[tetInput.numberoffacets];
	tetInput.facetmarkerlist = new int[tetInput.numberoffacets];
	for (int i = 0; i < tetInput.numberoffacets; i++) {
		f = &tetInput.facetlist[i];
		f->numberofpolygons = 1;
		f->polygonlist = new tetgenio::polygon[f->numberofpolygons];
		f->numberofholes = 0;
		f->holelist = NULL;
		p = &f->polygonlist[0];
		p->numberofvertices = 3;
		p->vertexlist = new int[p->numberofvertices];
		p->vertexlist[0] = originalTriangles[i][0];
		p->vertexlist[1] = originalTriangles[i][1];
		p->vertexlist[2] = originalTriangles[i][2];
	}

	std::string tetCmd = "pqa" + std::to_string(tetVolume);
	char* cmd = new char[tetCmd.length() + 1];
	strcpy(cmd, tetCmd.c_str());
	tetrahedralize(cmd, &tetInput, &tetOutput);
	//tetrahedralize("pq", &tetInput, &tetOutput);
	delete[] cmd;

	if (printVals) {
		MGlobal::displayInfo(std::string("finish tetgen test with " + std::to_string(tetOutput.numberofpoints) + " points and "
			+ std::to_string(tetOutput.numberoftetrahedra) + " tetrahedrons "
			+ std::to_string(tetOutput.numberoftrifaces) + " triFaces").c_str());
	}

	// get vertices
	for (int i = 0; i < tetOutput.numberofpoints; i++) {
		tetVertices.emplace_back(Eigen::Vector3d(tetOutput.pointlist[i * 3],
			tetOutput.pointlist[i * 3 + 1],
			tetOutput.pointlist[i * 3 + 2]));
	}
	// get triangles
	for (int i = 0; i < tetOutput.numberoftrifaces; i++) {
		tetTriangles.emplace_back(Eigen::Vector3i(tetOutput.trifacelist[i * 3],
			tetOutput.trifacelist[i * 3 + 1],
			tetOutput.trifacelist[i * 3 + 2]));
	}
	// get tetrahedrons
	for (int i = 0; i < tetOutput.numberoftetrahedra; i++) {
		std::vector<int> tetrahedron;
		tetrahedron.push_back(tetOutput.tetrahedronlist[i * 4]);
		tetrahedron.push_back(tetOutput.tetrahedronlist[i * 4 + 1]);
		tetrahedron.push_back(tetOutput.tetrahedronlist[i * 4 + 2]);
		tetrahedron.push_back(tetOutput.tetrahedronlist[i * 4 + 3]);
		tetTetrahedrons.push_back(tetrahedron);
	}
}


// helper function that prints out vertices
void printVertices(const std::vector<Eigen::Vector3d>& vertices) {
	// get vertices
	for (auto i = 0u; i < vertices.size(); i++) {
		std::string msg = "vertex " + std::to_string(i) + " is at "
			+ std::to_string(vertices[i][0]) + ", "
			+ std::to_string(vertices[i][1]) + ", "
			+ std::to_string(vertices[i][2]);
		MGlobal::displayInfo(msg.c_str());
	}
}

// helper function that prints out triangles
void printTriangles(const std::vector<Eigen::Vector3i>& triangles) {
	for (int i = 0; i < triangles.size(); i++) {
		std::string msg = "triangle " + std::to_string(i) + " has "
			+ std::to_string(triangles[i][0]) + ", "
			+ std::to_string(triangles[i][1]) + ", "
			+ std::to_string(triangles[i][2]);
		MGlobal::displayInfo(msg.c_str());
	}
}


// print the number of tetrahedrons
void printTetrahedrons(const std::vector<std::vector<int>>& tetrahedrons) {
	for (auto i = 0u; i < tetrahedrons.size(); i++) {
		std::string msg = "tetrahedron " + std::to_string(i) + " has "
			+ std::to_string(tetrahedrons[i][0]) + ", "
			+ std::to_string(tetrahedrons[i][1]) + ", "
			+ std::to_string(tetrahedrons[i][2]) + ", "
			+ std::to_string(tetrahedrons[i][3]);
		MGlobal::displayInfo(msg.c_str());
	}
}


// populated comutedFrames with data from savedFrame
void readSavedFrames(const MObject& savedFramesData, std::vector<MPointArray>& computedFrames) {
	MFnStringArrayData savedFramesArray(savedFramesData);
	MStringArray allFrames = savedFramesArray.array();
	
	computedFrames.clear();
	// each string entry is 1 frame of all vertex positions separated by ';' 
	// iterate through each frame
	for (auto i = 0u; i < allFrames.length(); i++) {
		MPointArray pointArray; 
		
		// parse each frame into array of vertices
		MStringArray currentFrame;
		allFrames[i].split(';', currentFrame);

		// iterate through each vertex and append the data into the MPointArray
		for (auto j = 0u; j < currentFrame.length(); j++) {
			MStringArray vertex;
			currentFrame[j].split(',', vertex);
			double x = vertex[0].asDouble();
			double y = vertex[1].asDouble();
			double z = vertex[2].asDouble();
			pointArray.append(x, y, z);
		}

		computedFrames.push_back(pointArray);
	}
}


// ---------------------------------------
//
//         Make QuikDeformNode  
//
// ---------------------------------------


// ---------------------------------------
// initialize attributes 
// ---------------------------------------
MObject QuikDeformNode::doCompute;
// simulation attributes
MObject QuikDeformNode::inputMesh;
MObject QuikDeformNode::outputMesh;
MObject QuikDeformNode::timeStep;
MObject QuikDeformNode::solverIterations;
MObject QuikDeformNode::secondsToSimulate;
MObject QuikDeformNode::frameRate;
MObject QuikDeformNode::currentFrame;

// object attributes
MObject QuikDeformNode::keepMesh;
MObject QuikDeformNode::tetVolume;
MObject QuikDeformNode::mass;
MObject QuikDeformNode::initialVelocity;
MObject QuikDeformNode::volumetric;
MObject QuikDeformNode::youngsModulus;
MObject QuikDeformNode::poissonRatio;
MObject QuikDeformNode::positionConstraints;
MObject QuikDeformNode::collisionConstraints;
MObject QuikDeformNode::frictionCoeff;

// external force attributes
MObject QuikDeformNode::doGravity;
MObject QuikDeformNode::doWind;
MObject QuikDeformNode::windDirection;
MObject QuikDeformNode::windMagnitude;
MObject QuikDeformNode::windAmplitude;
MObject QuikDeformNode::windPeriod;
MObject QuikDeformNode::windOscillation;

MTypeId QuikDeformNode::id(0x80000);
MObject QuikDeformNode::savedFrames;
MObject QuikDeformNode::savedMesh;


// create the node
void* QuikDeformNode::creator() {
	return new QuikDeformNode();
}


// initialize attribute fields
MStatus QuikDeformNode::initialize() {

	MFnUnitAttribute unitAttr;
	MFnNumericAttribute numAttr;
	MFnTypedAttribute typedAttr;

	MStatus returnStatus;

	// ---------------------------------------
	// create all attributes 
	// ---------------------------------------
	QuikDeformNode::doCompute = numAttr.create("doCompute", "dc", MFnNumericData::kBoolean, 1);
	// simulation attributes
	QuikDeformNode::inputMesh = typedAttr.create("inputMesh", "im", MFnData::kMesh);
	typedAttr.setStorable(true);
	QuikDeformNode::outputMesh = typedAttr.create("outputMesh", "om", MFnData::kMesh);
	typedAttr.setStorable(true);
	QuikDeformNode::timeStep = numAttr.create("timeStep", "ts", MFnNumericData::kDouble, 0.01);
	numAttr.setStorable(true);
	QuikDeformNode::solverIterations = numAttr.create("solverIterations", "si", MFnNumericData::kInt, 5);
	numAttr.setStorable(true);
	QuikDeformNode::secondsToSimulate = numAttr.create("secondsToSimulate", "f", MFnNumericData::kInt, 5);
	numAttr.setStorable(true);
	QuikDeformNode::frameRate = numAttr.create("frameRate", "fr", MFnNumericData::kInt, 24);
	numAttr.setStorable(true);
	QuikDeformNode::currentFrame = numAttr.create("currentFrame", "cf", MFnNumericData::kInt, 1);
	// object attributes
	QuikDeformNode::keepMesh = numAttr.create("keepMesh", "kp", MFnNumericData::kBoolean, 1);
	numAttr.setStorable(true);
	QuikDeformNode::tetVolume = numAttr.create("tetVolume", "tv", MFnNumericData::kDouble, 1.0);
	numAttr.setStorable(true);
	QuikDeformNode::mass = numAttr.create("mass", "m", MFnNumericData::kDouble, 1.0);
	numAttr.setStorable(true);
	QuikDeformNode::initialVelocity = numAttr.createPoint("initialVelocity", "iv");
	numAttr.setStorable(true);
	QuikDeformNode::volumetric = numAttr.create("volumetric", "v", MFnNumericData::kBoolean, 1);
	numAttr.setStorable(true);
	QuikDeformNode::youngsModulus = numAttr.create("youngsModulus", "E", MFnNumericData::kDouble, 5000);
	numAttr.setStorable(true);
	QuikDeformNode::poissonRatio = numAttr.create("poissonRatio", "nu", MFnNumericData::kDouble, 0.3);
	numAttr.setStorable(true);
	QuikDeformNode::positionConstraints = typedAttr.create("positionConstraints", "pcs", MFnData::kString);
	typedAttr.setStorable(true);
	QuikDeformNode::collisionConstraints = typedAttr.create("collisionConstraints", "ccs", MFnData::kString);
	typedAttr.setStorable(true);
	QuikDeformNode::frictionCoeff = numAttr.create("frictionCoeff", "fc", MFnNumericData::kDouble, 0.4);
	numAttr.setStorable(true);
	// external forces attributes
	QuikDeformNode::doGravity = numAttr.create("doGravity", "dg", MFnNumericData::kBoolean, 1);
	numAttr.setStorable(true);
	QuikDeformNode::doWind = numAttr.create("doWind", "dw", MFnNumericData::kBoolean, 0);
	numAttr.setStorable(true);
	QuikDeformNode::windDirection = numAttr.createPoint("windDirection", "wd");
	numAttr.setStorable(true);
	QuikDeformNode::windMagnitude = numAttr.create("windMagnitude", "wm", MFnNumericData::kDouble, 1);
	numAttr.setStorable(true);
	QuikDeformNode::windAmplitude = numAttr.create("windAmplitude", "wa", MFnNumericData::kDouble, 1);
	numAttr.setStorable(true);
	QuikDeformNode::windPeriod = numAttr.create("windPeriod", "wp", MFnNumericData::kDouble, 0.5);
	numAttr.setStorable(true);
	QuikDeformNode::windOscillation = numAttr.create("windOscillation", "wo", MFnNumericData::kBoolean, 0);
	numAttr.setStorable(true);

	QuikDeformNode::savedFrames = typedAttr.create("savedFrames", "sfs", MFnNumericData::kStringArray);
	typedAttr.setStorable(true);
	QuikDeformNode::savedMesh = typedAttr.create("savedMesh", "sm", MFnData::kMesh);
	typedAttr.setStorable(true);

	// ---------------------------------------
	// add all created attributes 
	// ---------------------------------------
	CHECK_MSTATUS(addAttribute(QuikDeformNode::doCompute));
	// simulation attributes
	CHECK_MSTATUS(addAttribute(QuikDeformNode::inputMesh));
	CHECK_MSTATUS(addAttribute(QuikDeformNode::outputMesh));
	CHECK_MSTATUS(addAttribute(QuikDeformNode::timeStep));
	CHECK_MSTATUS(addAttribute(QuikDeformNode::solverIterations));
	CHECK_MSTATUS(addAttribute(QuikDeformNode::secondsToSimulate));
	CHECK_MSTATUS(addAttribute(QuikDeformNode::frameRate));
	CHECK_MSTATUS(addAttribute(QuikDeformNode::currentFrame));
	// object attributes
	CHECK_MSTATUS(addAttribute(QuikDeformNode::keepMesh));
	CHECK_MSTATUS(addAttribute(QuikDeformNode::tetVolume));
	CHECK_MSTATUS(addAttribute(QuikDeformNode::mass));
	CHECK_MSTATUS(addAttribute(QuikDeformNode::initialVelocity));
	CHECK_MSTATUS(addAttribute(QuikDeformNode::volumetric));
	CHECK_MSTATUS(addAttribute(QuikDeformNode::youngsModulus));
	CHECK_MSTATUS(addAttribute(QuikDeformNode::poissonRatio));
	CHECK_MSTATUS(addAttribute(QuikDeformNode::positionConstraints));
	CHECK_MSTATUS(addAttribute(QuikDeformNode::collisionConstraints));
	CHECK_MSTATUS(addAttribute(QuikDeformNode::frictionCoeff));
	// external forces attributes
	CHECK_MSTATUS(addAttribute(QuikDeformNode::doGravity));
	CHECK_MSTATUS(addAttribute(QuikDeformNode::doWind));
	CHECK_MSTATUS(addAttribute(QuikDeformNode::windDirection));
	CHECK_MSTATUS(addAttribute(QuikDeformNode::windMagnitude));
	CHECK_MSTATUS(addAttribute(QuikDeformNode::windAmplitude));
	CHECK_MSTATUS(addAttribute(QuikDeformNode::windPeriod));
	CHECK_MSTATUS(addAttribute(QuikDeformNode::windOscillation));

	CHECK_MSTATUS(addAttribute(QuikDeformNode::savedFrames));
	CHECK_MSTATUS(addAttribute(QuikDeformNode::savedMesh));

	// ---------------------------------------
	// link created attributes 
	// ---------------------------------------
	// only link the following two b/c the compute function should only run when either
	// the compute button is clicked or the currentFrame changes
	CHECK_MSTATUS(attributeAffects(QuikDeformNode::doCompute, QuikDeformNode::outputMesh));
	CHECK_MSTATUS(attributeAffects(QuikDeformNode::currentFrame, QuikDeformNode::outputMesh));
	
	return MS::kSuccess;
}


MStatus QuikDeformNode::compute(const MPlug& plug, MDataBlock& data) {
	MStatus returnStatus;

	// ---------------------------------------
	// step 1: load saved data if any exists
	// ---------------------------------------
	// if there's no computedFrame data but there is data in savedFrames, read in the data and also the inputs
	if (computedFrames.empty()) {
		MDataHandle savedFramesDataHandle = data.outputValue(savedFrames);
		MObject savedFramesData = savedFramesDataHandle.data();
		if (!savedFramesData.isNull()) {
			currentConfiguration = getConfiguration(data);
			readSavedFrames(savedFramesData, computedFrames);
			if (!computedFrames.empty()) {
				MGlobal::displayInfo("successfully loaded saved frame data");
			}
			else {
				MGlobal::displayInfo("failed to load saved frame data");
			}
		}
	}
	// if there's no originalMesh but there is data in savedMesh, read in the mesh
	if (originalMesh.isNull()) {
		MDataHandle savedMeshDataHandle = data.outputValue(savedMesh);
		MObject savedMeshData = savedMeshDataHandle.asMeshTransformed();
		if (!savedMeshData.isNull()) {
			originalMesh = savedMeshData;
			MGlobal::displayInfo("successfully loaded saved mesh data");
		}
	}


	// ----------------------------------------
	// step 2: read input and compute output 
	// ----------------------------------------
	double computeStart = omp_get_wtime();

	MDataHandle doComputeData = data.inputValue(doCompute);
	bool doComputation = doComputeData.asBool();

	if (doComputation) {
		QuikDeformNodeInput newConfiguration = getConfiguration(data);

		// only compute output if we've never computed quikDeformer before or if input changed
		if (quikDeformer == nullptr || currentConfiguration != newConfiguration) {
			MGlobal::displayInfo(std::string("recomputing everything").c_str());

			// --------------------------------------------------------
			// step 2.1: get the mesh. tetrahedralize it if neccessary
			// --------------------------------------------------------

			// TODO: what happens when the originalMesh is the only thing changed?
			MDataHandle inputMeshData = data.inputValue(inputMesh);
			originalMesh = inputMeshData.asMeshTransformed();
			currentConfiguration = newConfiguration;

			// declare the containers that will contain data for the mesh we want to simulate
			std::vector<Eigen::Vector3d> meshVertices;
			std::vector<Eigen::Vector3i> meshTriangles;
			std::vector<std::vector<int>> meshTetrahedrons;
			unsigned int vertexNumber = 0u; // how many vertices are in the final mesh

			// if the simulation is volumetric, tetrahedralize the mesh 
			if (currentConfiguration.volumetric) {
				// ----- get data from the mesh and tetrahedralize it -----
				// containers for data from the original mesh
				std::vector<Eigen::Vector3d> originalVertices;
				std::vector<Eigen::Vector3i> originalTriangles;

				// tetrahedralize the original mesh
				getTetMeshData((MFnMesh)originalMesh, originalVertices, originalTriangles,
					meshVertices, meshTriangles, meshTetrahedrons, currentConfiguration.tetVolume, false);
				vertexNumber = originalVertices.size();

				// make a new mesh as the originalObj if keepMesh if false
				if (!currentConfiguration.keepMesh) {
					MFnMeshData dataCreator;
					MObject newObj = dataCreator.create();

					// set up the input values to make a new mesh with
					MPointArray vertexArray;
					MIntArray polygonCounts;
					MIntArray polygonConnects;
					for (auto i = 0; i < meshVertices.size(); i++) {
						vertexArray.append(meshVertices[i][0], meshVertices[i][1], meshVertices[i][2]);
					}
					for (auto i = 0; i < meshTriangles.size(); i++) {
						polygonCounts.append(3);
						polygonConnects.append(meshTriangles[i][2]); // reverse order to get the right surface normal
						polygonConnects.append(meshTriangles[i][1]);
						polygonConnects.append(meshTriangles[i][0]);
					}

					// make the new mesh
					MFnMesh newMesh;
					newMesh.create(vertexArray.length(), polygonCounts.length(), vertexArray, polygonCounts, polygonConnects, newObj);

					// fix vertex normal for newMesh
					for (auto j = 0; j < meshTriangles.size(); j++) {
						MVector p1(vertexArray[meshTriangles[j][2]]);
						MVector p2(vertexArray[meshTriangles[j][1]]);
						MVector p3(vertexArray[meshTriangles[j][0]]);
						MVector surfaceNormal = (p2 - p1) ^ (p3 - p1);
						newMesh.setFaceVertexNormal(surfaceNormal, j, meshTriangles[j][2]);
						newMesh.setFaceVertexNormal(surfaceNormal, j, meshTriangles[j][1]);
						newMesh.setFaceVertexNormal(surfaceNormal, j, meshTriangles[j][0]);
					}

					originalMesh = newObj;
					vertexNumber = meshVertices.size();
				}
			}
			// thin-shell simulation. no need to tetrahedralize. just use original data.
			else {
				getMeshData((MFnMesh)originalMesh, meshVertices, meshTriangles);
				vertexNumber = meshVertices.size();
			}

			// save the mesh data into savedMesh
			MDataHandle savedMeshDataHandle = data.outputValue(savedMesh);
			savedMeshDataHandle.set(originalMesh);


			// ----------------------------------------------------------
			// step 2.2: initialize QuikDeformer and run the simulation
			// ----------------------------------------------------------
			if (quikDeformer != nullptr) { delete quikDeformer; }

			// ----- initialize QuikDeformer -----
			quikDeformer = new QuikDeformer(
				meshVertices, meshTriangles, meshTetrahedrons,
				currentConfiguration.timeStep,
				currentConfiguration.solverIterations,
				currentConfiguration.frameRate,
				currentConfiguration.mass,
				currentConfiguration.initialVelocity[0],
				currentConfiguration.initialVelocity[1],
				currentConfiguration.initialVelocity[2],
				currentConfiguration.doGravity,
				currentConfiguration.volumetric);

			// ----- add strain constraints -----
			double E = currentConfiguration.youngsModulus;
			double nu = currentConfiguration.poissonRatio;
			double strainWeight = 2 * (E / ((double)2 * ((double)1 + nu)));
			if (currentConfiguration.volumetric) {
				quikDeformer->add3DStrainConstraints(strainWeight);
			}
			else {
				quikDeformer->add2DStrainConstraints(strainWeight);
			}

			// ----- add position constraints -----
			if (currentConfiguration.positionConstraints != "") {
				std::stringstream ss(currentConfiguration.positionConstraints);
				std::string indexString;
				while (std::getline(ss, indexString, ',')) {
					if (indexString != "") {
						int index = std::stoi(indexString);
						if (index < vertexNumber) {
							quikDeformer->addPositionConstraint(100000, index);
						}
					}
				}
			}

			// ----- add collision constraints -----
			vector<Eigen::Vector3d> planeCenters, planeNormals;
			vector<double> pLengths, pWidths; // not used
			if (currentConfiguration.collisionConstraints != "") {
				std::stringstream stream1(currentConfiguration.collisionConstraints);
				std::string planeString;
				while (std::getline(stream1, planeString, ';')) {
					if (planeString != "") {
						// tokenize the string of the format: "plane1,0,0,0,1,0,0"
						vector<std::string> tokens;
						std::stringstream stream2(planeString);
						std::string tokenString;
						while (std::getline(stream2, tokenString, ',')) {
							if (tokenString != "") {
								tokens.push_back(tokenString);
							}
						}
						// extract and add the info 
						Eigen::Vector3d center = Eigen::Vector3d(std::stod(tokens[1]), std::stod(tokens[2]), std::stod(tokens[3]));
						Eigen::Vector3d normal = Eigen::Vector3d(std::stod(tokens[4]), std::stod(tokens[5]), std::stod(tokens[6])).normalized();
						planeCenters.push_back(center);
						planeNormals.push_back(normal);
					}
				}
				quikDeformer->addCollisionPlanes(planeCenters, pLengths, pWidths, planeNormals, currentConfiguration.frictionCoeff);
			}

			// ----- add wind -----
			if (currentConfiguration.doWind) {
				quikDeformer->addWind(currentConfiguration.windDirection[0],
								  	  currentConfiguration.windDirection[1],
									  currentConfiguration.windDirection[2],
									  currentConfiguration.windMagnitude, 
									  currentConfiguration.windOscillation, 
									  currentConfiguration.windAmplitude, 
									  currentConfiguration.windPeriod);
			}

			// ----------------------------------------------------------
			// step 2.3: put the calculated frames into computedFrames
			// ----------------------------------------------------------

			std::vector<Eigen::VectorXd> tempFrames; // temp holder of computed frames
			quikDeformer->runSimulation(currentConfiguration.secondsToSimulate, false, tempFrames);

			// parse the output frames into maya data
			// also save the data into savedFrames
			computedFrames.clear();
			MStringArray savedFramesArray; // each string entry is 1 frame of all vertex positions
										   // separated by ';' 
			for (auto i = 0u; i < tempFrames.size(); i++) {
				MPointArray pointArray;
				std::stringstream ss;
				for (auto j = 0u; j < vertexNumber; j++) {
					double x = tempFrames[i][j * 3];
					double y = tempFrames[i][j * 3 + 1];
					double z = tempFrames[i][j * 3 + 2];

					pointArray.append(x, y, z);
					ss << x << "," << y << "," << z << ";";
				}
				computedFrames.push_back(pointArray);
				std::string str = ss.str();
				MString frameString(str.c_str());
				savedFramesArray.append(frameString);
			}

			// print out some times
			double totalTimeElapsed = omp_get_wtime() - computeStart;
			std::string timeDiff = "Computation complete. Time elapsed: " + std::to_string(totalTimeElapsed) + " seconds";
			MGlobal::displayInfo(timeDiff.c_str());

			// save the savedFrames into attributes
			MDataHandle savedFramesDataHandle = data.outputValue(savedFrames);
			MFnStringArrayData arrayData;
			savedFramesDataHandle.set(arrayData.create(savedFramesArray));
		}
		else {
			MGlobal::displayInfo("Input is the same. No recomputation needed.");
		}

		// don't forget to switch off doCompute
		doComputeData.set(false);
	}


	// ----------------------------------------------------
	// step 3: update the mesh according to currentFrame
	// ----------------------------------------------------
	// create output data
	MDataHandle outputMeshData = data.outputValue(outputMesh);
	MFnMeshData dataCreator;
	MObject newOutputObj = dataCreator.create();
	MFnMesh newMesh;
	newMesh.copy(originalMesh, newOutputObj);

	// update output
	MDataHandle currentFrameData = data.inputValue(currentFrame);
	int frameToDisplay = currentFrameData.asInt();
	int totalFrames = currentConfiguration.frameRate * currentConfiguration.secondsToSimulate;
	if (frameToDisplay <= totalFrames) {
		newMesh.setPoints(computedFrames[frameToDisplay - 1]);
		newMesh.setObject(newOutputObj);
		outputMeshData.set(newOutputObj);
		data.setClean(plug);
	}
	else {
		MGlobal::displayInfo(std::string("can't showing frame " + std::to_string(frameToDisplay) + ", because it has not been computed yet.").c_str());
	}

	return MStatus::kSuccess;
}


// reads the input from MDataBlock into a QuikDeformNodeInput object
QuikDeformNodeInput QuikDeformNode::getConfiguration(MDataBlock& data) {
	// ----- Make the required DataHandles -----
	// get simulation attributes
	MDataHandle timeStepData = data.inputValue(timeStep);
	MDataHandle solverIterationsData = data.inputValue(solverIterations);
	MDataHandle secondsToSimulateData = data.inputValue(secondsToSimulate);
	MDataHandle frameRateData = data.inputValue(frameRate);
	// get object attributes
	MDataHandle keepMeshData = data.inputValue(keepMesh);
	MDataHandle tetVolumeData = data.inputValue(tetVolume);
	MDataHandle massData = data.inputValue(mass);
	MDataHandle initialVelocityData = data.inputValue(initialVelocity);
	MDataHandle volumetricData = data.inputValue(volumetric);
	MDataHandle youngsModulusData = data.inputValue(youngsModulus);
	MDataHandle poissonRatioData = data.inputValue(poissonRatio);
	MDataHandle positionConstraintsData = data.inputValue(positionConstraints);
	MDataHandle collisionConstraintsData = data.inputValue(collisionConstraints);
	MDataHandle frictionCoeffData = data.inputValue(frictionCoeff);
	// get external forces attributes
	MDataHandle doGravityData = data.inputValue(doGravity);
	MDataHandle doWindData = data.inputValue(doWind);
	MDataHandle windDirectionData = data.inputValue(windDirection);
	MDataHandle windMagnitudeData = data.inputValue(windMagnitude);
	MDataHandle windAmplitudeData = data.inputValue(windAmplitude);
	MDataHandle windPeriodData = data.inputValue(windPeriod);
	MDataHandle windOscillationData = data.inputValue(windOscillation);

	// ----- get data from the DataHandles into the QuikDeformNodeInput -----
	QuikDeformNodeInput newConfiguration;
	// simulation attributes
	newConfiguration.timeStep = timeStepData.asDouble();
	newConfiguration.solverIterations = solverIterationsData.asInt();
	newConfiguration.secondsToSimulate = secondsToSimulateData.asInt();
	newConfiguration.frameRate = frameRateData.asInt();
	// object attributes
	newConfiguration.keepMesh = keepMeshData.asBool();
	newConfiguration.tetVolume = tetVolumeData.asDouble();
	newConfiguration.mass = massData.asDouble();
	newConfiguration.initialVelocity = initialVelocityData.asFloatVector();
	newConfiguration.volumetric = volumetricData.asBool();
	newConfiguration.youngsModulus = youngsModulusData.asDouble();
	newConfiguration.poissonRatio = poissonRatioData.asDouble();
	newConfiguration.positionConstraints = positionConstraintsData.asString().asChar();
	newConfiguration.collisionConstraints = collisionConstraintsData.asString().asChar();
	newConfiguration.frictionCoeff = frictionCoeffData.asDouble();
	// external force attributes
	newConfiguration.doGravity = doGravityData.asBool();
	newConfiguration.doWind = doWindData.asBool();
	newConfiguration.windDirection = windDirectionData.asFloatVector().normal();
	newConfiguration.windMagnitude = windMagnitudeData.asDouble();
	newConfiguration.windAmplitude = windAmplitudeData.asDouble();
	newConfiguration.windPeriod = windPeriodData.asDouble();
	newConfiguration.windOscillation = windOscillationData.asBool();

	return newConfiguration;
}


// Initialize Plugin upon loading
MStatus initializePlugin(MObject obj)
{
	MStatus status = MStatus::kSuccess;
	MFnPlugin plugin(obj, "CIS660", "1.0", "Any");

	status = plugin.registerNode("QuikDeformNode", QuikDeformNode::id, QuikDeformNode::creator, QuikDeformNode::initialize);

	// load the mel script that will add commands to menu
	char buffer[2048];
	std::string path = plugin.loadPath().asChar();
	sprintf_s(buffer, 2048, "source \"%s/QuikDeformNodeMenu.mel\";", path.c_str());
	MGlobal::executeCommand(buffer, true);

	if (!status) {
		status.perror("register QuikDeformNode failed");
		return status;
	}

	return status;
}

// Cleanup Plugin upon unloading
MStatus uninitializePlugin(MObject obj)
{
	MStatus status;
	MFnPlugin plugin(obj);

	status = plugin.deregisterNode(QuikDeformNode::id);

	if (!status) {
		status.perror("deregister QuikDeformNode failed");
	}
	return status;
}