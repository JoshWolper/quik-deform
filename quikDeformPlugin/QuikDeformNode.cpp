#include "QuikDeformNode.h"
#include "global.h"
#include <istream>
#include <iostream>

// macro for checking errors
#define McheckErr(stat,msg)			\
	if ( MS::kSuccess != stat ) {	\
		cerr << msg;				\
		return MS::kFailure;		\
	}


// helper function that gets vertices and triangles from a Maya mesh
void getMeshData(const MFnMesh& mesh,
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
		tetInput.pointlist[i * 3]	  = meshVertices[i][0];
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

// external force attributes
MObject QuikDeformNode::doGravity;
MObject QuikDeformNode::doWind;
MObject QuikDeformNode::windDirection;
MObject QuikDeformNode::windMagnitude;
MObject QuikDeformNode::windOscillation;

MTypeId QuikDeformNode::id(0x80000);



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
	QuikDeformNode::outputMesh = typedAttr.create("outputMesh", "om", MFnData::kMesh);
	QuikDeformNode::timeStep = numAttr.create("timeStep", "ts", MFnNumericData::kDouble, 0.01);
	QuikDeformNode::solverIterations = numAttr.create("solverIterations", "si", MFnNumericData::kInt, 5);
	QuikDeformNode::secondsToSimulate = numAttr.create("secondsToSimulate", "f", MFnNumericData::kInt, 5);
	QuikDeformNode::frameRate = numAttr.create("frameRate", "fr", MFnNumericData::kInt, 24);
	QuikDeformNode::currentFrame = numAttr.create("currentFrame", "cf", MFnNumericData::kInt, 1);
	// object attributes
	QuikDeformNode::keepMesh = numAttr.create("keepMesh", "kp", MFnNumericData::kBoolean, 1);
	QuikDeformNode::tetVolume = numAttr.create("tetVolume", "tv", MFnNumericData::kDouble, 1.0);
	QuikDeformNode::mass = numAttr.create("mass", "m", MFnNumericData::kDouble, 1.0);
	QuikDeformNode::initialVelocity = numAttr.createPoint("initialVelocity", "iv");
	QuikDeformNode::volumetric = numAttr.create("volumetric", "v", MFnNumericData::kBoolean, 1);
	QuikDeformNode::youngsModulus = numAttr.create("youngsModulus", "E", MFnNumericData::kDouble, 5000);
	QuikDeformNode::poissonRatio = numAttr.create("poissonRatio", "nu", MFnNumericData::kDouble, 0.3);
	QuikDeformNode::positionConstraints = typedAttr.create("positionConstraints", "pcs", MFnData::kString);
	// external forces attributes
	QuikDeformNode::doGravity = numAttr.create("doGravity", "dg", MFnNumericData::kBoolean, 1);
	QuikDeformNode::doWind = numAttr.create("doWind", "dw", MFnNumericData::kBoolean, 0);
	QuikDeformNode::windDirection = numAttr.createPoint("windDirection", "wd");
	QuikDeformNode::windMagnitude = numAttr.create("windMagnitude", "wm", MFnNumericData::kDouble, 0.01);
	QuikDeformNode::windOscillation = numAttr.create("windOscillation", "wo", MFnNumericData::kBoolean, 0);

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
	// external forces attributes
	CHECK_MSTATUS(addAttribute(QuikDeformNode::doGravity));
	CHECK_MSTATUS(addAttribute(QuikDeformNode::doWind));
	CHECK_MSTATUS(addAttribute(QuikDeformNode::windDirection));
	CHECK_MSTATUS(addAttribute(QuikDeformNode::windMagnitude));
	CHECK_MSTATUS(addAttribute(QuikDeformNode::windOscillation));

	// ---------------------------------------
	// link created attributes 
	// ---------------------------------------
	// recomputes the outputMesh
	CHECK_MSTATUS(attributeAffects(QuikDeformNode::doCompute, QuikDeformNode::outputMesh));
	CHECK_MSTATUS(attributeAffects(QuikDeformNode::currentFrame, QuikDeformNode::outputMesh));
	/*
	// simulation attributes
	CHECK_MSTATUS(attributeAffects(QuikDeformNode::timeStep, QuikDeformNode::outputMesh));
	CHECK_MSTATUS(attributeAffects(QuikDeformNode::inputMesh, QuikDeformNode::outputMesh)); 
	CHECK_MSTATUS(attributeAffects(QuikDeformNode::solverIterations, QuikDeformNode::outputMesh));
	CHECK_MSTATUS(attributeAffects(QuikDeformNode::secondsToSimulate, QuikDeformNode::outputMesh));
	CHECK_MSTATUS(attributeAffects(QuikDeformNode::frameRate, QuikDeformNode::outputMesh));
	
	// object attributes
	CHECK_MSTATUS(attributeAffects(QuikDeformNode::keepMesh, QuikDeformNode::outputMesh));
	CHECK_MSTATUS(attributeAffects(QuikDeformNode::tetVolume, QuikDeformNode::outputMesh));
	CHECK_MSTATUS(attributeAffects(QuikDeformNode::mass, QuikDeformNode::outputMesh));
	CHECK_MSTATUS(attributeAffects(QuikDeformNode::initialVelocity, QuikDeformNode::outputMesh));
	CHECK_MSTATUS(attributeAffects(QuikDeformNode::volumetric, QuikDeformNode::outputMesh));
	CHECK_MSTATUS(attributeAffects(QuikDeformNode::youngsModulus, QuikDeformNode::outputMesh));
	CHECK_MSTATUS(attributeAffects(QuikDeformNode::poissonRatio, QuikDeformNode::outputMesh));
	// external forces
	CHECK_MSTATUS(attributeAffects(QuikDeformNode::doGravity, QuikDeformNode::outputMesh));
	CHECK_MSTATUS(attributeAffects(QuikDeformNode::doWind, QuikDeformNode::outputMesh));
	CHECK_MSTATUS(attributeAffects(QuikDeformNode::windDirection, QuikDeformNode::outputMesh));
	CHECK_MSTATUS(attributeAffects(QuikDeformNode::windMagnitude, QuikDeformNode::outputMesh));
	CHECK_MSTATUS(attributeAffects(QuikDeformNode::windOscillation, QuikDeformNode::outputMesh));
	*/

	return MS::kSuccess;
}


MStatus QuikDeformNode::compute(const MPlug& plug, MDataBlock& data) {
	MStatus returnStatus;

	// if doCompute is true, then recompute everything based on input
	MDataHandle doComputeData = data.inputValue(doCompute);
	bool doComputation = doComputeData.asBool();
	if (doComputation) {
		// ---------------------------------------
		// get all input attributes from the node
		// ---------------------------------------
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

		// get external forces attributes
		MDataHandle doGravityData = data.inputValue(doGravity);
		MDataHandle doWindData = data.inputValue(doWind);
		MDataHandle windDirectionData = data.inputValue(windDirection);
		MDataHandle windMagnitudeData = data.inputValue(windMagnitude);
		MDataHandle windOscillationData = data.inputValue(windOscillation);

		// set up the newConfiguration
		QuikDeformNodeIO newConfiguration;
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
		// external force attributes
		newConfiguration.doGravity = doGravityData.asBool();
		newConfiguration.doWind = doWindData.asBool();
		newConfiguration.windDirection = windDirectionData.asVector().normal();
		newConfiguration.windMagnitude = windMagnitudeData.asDouble();
		newConfiguration.windOscillation = windOscillationData.asBool();

		// ---------------------------------------
		// step 2: recompute output if newInput is different
		// ---------------------------------------
		double computeStart = omp_get_wtime();

		//MGlobal::displayInfo("current position constraint");
		//MGlobal::displayInfo(currentConfiguration.positionConstraints.c_str());
		//MGlobal::displayInfo("new position constraint");
		//MGlobal::displayInfo(newConfiguration.positionConstraints.c_str());
		// compute output if we've never computed quikDeformer before or if an input attribute change
		if (quikDeformer == nullptr || currentConfiguration != newConfiguration) {
			MGlobal::displayInfo(std::string("recomputing everything").c_str());

			// TODO: what happens when the originalMesh is the only thing changed?
			// TODO: rotation kind of messes up the world matrix thing. try it without world Mesh again
			MDataHandle inputMeshData = data.inputValue(inputMesh);
			originaObj = inputMeshData.asMeshTransformed();
			currentConfiguration = newConfiguration;

			// ----- get data from the mesh and tetrahedralize it -----
			// original vertices and triangles from the Maya mesh
			std::vector<Eigen::Vector3d> originalVertices;
			std::vector<Eigen::Vector3i> originalTriangles;

			// tetgen generated vertices, triangles and tetrahedrons
			std::vector<Eigen::Vector3d> tetVertices;
			std::vector<Eigen::Vector3i> tetTriangles;
			std::vector<std::vector<int>> tetTetrahedrons;


			// tetrahedralize the original mesh
			getMeshData((MFnMesh)originaObj, originalVertices, originalTriangles,
				tetVertices, tetTriangles, tetTetrahedrons, currentConfiguration.tetVolume, false);

			// make a new mesh as the originalObj if keepMesh if false
			if (!currentConfiguration.keepMesh) {
				MFnMeshData dataCreator;
				MObject newObj = dataCreator.create();

				// set up the input values to make a new mesh with
				MPointArray vertexArray;
				MIntArray polygonCounts;
				MIntArray polygonConnects;
				for (auto i = 0; i < tetVertices.size(); i++) {
					vertexArray.append(tetVertices[i][0], tetVertices[i][1], tetVertices[i][2]);
				}
				for (auto i = 0; i < tetTriangles.size(); i++) {
					polygonCounts.append(3);
					polygonConnects.append(tetTriangles[i][2]); // reverse order to get the right surface normal
					polygonConnects.append(tetTriangles[i][1]);
					polygonConnects.append(tetTriangles[i][0]);
				}

				// make the new mesh
				MFnMesh newMesh;
				newMesh.create(vertexArray.length(), polygonCounts.length(), vertexArray, polygonCounts, polygonConnects, newObj);

				// fix vertex normal for newMesh
				for (auto j = 0; j < tetTriangles.size(); j++) {
					MVector p1(vertexArray[tetTriangles[j][2]]);
					MVector p2(vertexArray[tetTriangles[j][1]]);
					MVector p3(vertexArray[tetTriangles[j][0]]);
					MVector surfaceNormal = (p2 - p1) ^ (p3 - p1);
					newMesh.setFaceVertexNormal(surfaceNormal, j, tetTriangles[j][2]);
					newMesh.setFaceVertexNormal(surfaceNormal, j, tetTriangles[j][1]);
					newMesh.setFaceVertexNormal(surfaceNormal, j, tetTriangles[j][0]);
				}

				originaObj = newObj;
			}

			//WIND PARAMETERS
			bool windOn = false;
			double wx = 1; //wind direction
			double wy = 0;
			double wz = 0;
			double windMag = 1.5; //wind magnitude
			bool windOsc = false; //whether the wind oscillates or is constant

								  //Weight PARAMETERS
			double E = currentConfiguration.youngsModulus;
			double nu = currentConfiguration.poissonRatio;
			double lame_lambda = E * nu / (((double)1 + nu) * ((double)1 - (double)2 * nu));
			double lame_mu = E / ((double)2 * ((double)1 + nu));

			double tetStrainWeight = 2 * lame_mu;
			double volumeWeight = 3 * lame_lambda;

			// ----- run QuikDeformer to get the computed frames -----
			if (quikDeformer != nullptr) { delete quikDeformer; }

			quikDeformer = new QuikDeformer(
				tetVertices, tetTriangles, tetTetrahedrons,
				currentConfiguration.timeStep,
				currentConfiguration.solverIterations,
				currentConfiguration.frameRate,
				currentConfiguration.mass,
				currentConfiguration.initialVelocity[0],
				currentConfiguration.initialVelocity[1],
				currentConfiguration.initialVelocity[2],
				currentConfiguration.doGravity,
				currentConfiguration.volumetric);

			// add strain constraints
			if (currentConfiguration.volumetric) {
				quikDeformer->add3DStrainConstraints(tetStrainWeight);
			}
			else {
				// TODO: thin shelled triangle constraint
			}

			// add position constraints
			if (currentConfiguration.positionConstraints != "") {
				std::stringstream ss(currentConfiguration.positionConstraints);
				std::string indexString;
				while(std::getline(ss, indexString, ',')){
					if (indexString != "") {
						int index = std::stoi(indexString);
						quikDeformer->addPositionConstraint(100000, index);
					}
				}
			}


			std::vector<Eigen::VectorXd> tempFrames; // temp holder of computed frames
			quikDeformer->runSimulation(currentConfiguration.secondsToSimulate, false, tempFrames);

			// parse the output frames into maya data
			// TODO: what happens when only secondsToSimulate changes?
			computedFrames.clear();
			for (auto i = 0u; i < tempFrames.size(); i++) {
				MPointArray array;
				// only copy over vertices for the mesh that we're currently using
				auto vertexNumber = currentConfiguration.keepMesh ? originalVertices.size() : tetVertices.size();
				for (auto j = 0; j < vertexNumber; j++) {
					array.append(tempFrames[i][j * 3], tempFrames[i][j * 3 + 1], tempFrames[i][j * 3 + 2]);
				}
				computedFrames.push_back(array);
			}

			double timeElapsed = omp_get_wtime() - computeStart;;
			std::string timeDiff = "Computation complete. Time elapsed: " + std::to_string(timeElapsed) + " seconds";
			MGlobal::displayInfo(timeDiff.c_str());

			
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
	newMesh.copy(originaObj, newOutputObj);

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