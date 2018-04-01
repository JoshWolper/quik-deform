#include "QuikDeformNode.h"


// macro for checking errors
#define McheckErr(stat,msg)			\
	if ( MS::kSuccess != stat ) {	\
		cerr << msg;				\
		return MS::kFailure;		\
	}

// helper function for getting triangles from a mesh
void getTrianglesHelper(const MFnMesh& mesh, std::vector<std::vector<int>>& indices) {
	MIntArray triangleCounts, triangleIndices;
	mesh.getTriangles(triangleCounts, triangleIndices);

	int triangleNum = 0;
	for (auto i = 0u; i < triangleCounts.length(); i++) {
		triangleNum += triangleCounts[i];
	}

	for (int i = 0; i < triangleNum; i++) {
		std::vector<int> triangle;
		triangle.push_back(triangleIndices[i * 3]);
		triangle.push_back(triangleIndices[i * 3 + 1]);
		triangle.push_back(triangleIndices[i * 3 + 2]);
		indices.push_back(triangle);
	}

}


// ---------------------------------------
// initialize attributes 
// ---------------------------------------
// simulation attributes
MObject QuikDeformNode::inputMesh;
MObject QuikDeformNode::outputMesh;
MObject QuikDeformNode::timeStep;
MObject QuikDeformNode::solverIterations;
MObject QuikDeformNode::framesToSimulate;
MObject QuikDeformNode::frameRate;
MObject QuikDeformNode::currentFrame;
MObject QuikDeformNode::mass;
MObject QuikDeformNode::initialVelocity;

// constraint attributes
MObject QuikDeformNode::doStrainConstraint;
MObject QuikDeformNode::strainConstraintWeight;
MObject QuikDeformNode::doVolumeConstraint;
MObject QuikDeformNode::volumeConstraintWeight;
MObject QuikDeformNode::doBendingConstraint;
MObject QuikDeformNode::bendingConstraintWeight;

// external force attributes
MObject QuikDeformNode::doGravity;
MObject QuikDeformNode::gravityForce;
MObject QuikDeformNode::doWind;
MObject QuikDeformNode::windForce;

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
	// simulation attributes
	QuikDeformNode::inputMesh = typedAttr.create("inputMesh", "im", MFnData::kMesh);
	QuikDeformNode::outputMesh = typedAttr.create("outputMesh", "om", MFnData::kMesh);
	QuikDeformNode::timeStep = numAttr.create("timeStep", "ts", MFnNumericData::kDouble, 0.01);
	QuikDeformNode::solverIterations = numAttr.create("solverIterations", "si", MFnNumericData::kInt, 5);
	QuikDeformNode::framesToSimulate = numAttr.create("framesToSimulate", "f", MFnNumericData::kInt, 24);
	QuikDeformNode::frameRate = numAttr.create("frameRate", "fr", MFnNumericData::kInt, 24);
	QuikDeformNode::currentFrame = numAttr.create("currentFrame", "cf", MFnNumericData::kInt, 24);
	QuikDeformNode::mass = numAttr.create("mass", "m", MFnNumericData::kDouble, 1.0);
	QuikDeformNode::initialVelocity = numAttr.createPoint("initialVelocity", "iv");
	// constraint attributes
	QuikDeformNode::doStrainConstraint = numAttr.create("doStrainConstraint", "dsc", MFnNumericData::kBoolean, 0);
	QuikDeformNode::strainConstraintWeight = numAttr.create("strainConstraintWeight", "scw", MFnNumericData::kDouble, 1);
	QuikDeformNode::doVolumeConstraint = numAttr.create("doVolumeConstraint", "dvc", MFnNumericData::kBoolean, 0);
	QuikDeformNode::volumeConstraintWeight = numAttr.create("volumeConstraintWeight", "vcw", MFnNumericData::kDouble, 1);
	QuikDeformNode::doBendingConstraint = numAttr.create("doBendingConstraint", "dbc", MFnNumericData::kBoolean, 0);
	QuikDeformNode::bendingConstraintWeight = numAttr.create("bendingConstraintWeight", "bcw", MFnNumericData::kDouble, 1);
	// external forces attributes
	QuikDeformNode::doGravity = numAttr.create("doGravity", "dg", MFnNumericData::kBoolean, 0);
	QuikDeformNode::gravityForce = numAttr.createPoint("gravityForce", "gf");
	QuikDeformNode::doWind = numAttr.create("doWind", "dw", MFnNumericData::kBoolean, 0);
	QuikDeformNode::windForce = numAttr.createPoint("windForce", "wf");

	// ---------------------------------------
	// add all created attributes 
	// ---------------------------------------
	// simulation attributes
	CHECK_MSTATUS(addAttribute(QuikDeformNode::inputMesh));
	CHECK_MSTATUS(addAttribute(QuikDeformNode::outputMesh));
	CHECK_MSTATUS(addAttribute(QuikDeformNode::timeStep));
	CHECK_MSTATUS(addAttribute(QuikDeformNode::solverIterations));
	CHECK_MSTATUS(addAttribute(QuikDeformNode::framesToSimulate));
	CHECK_MSTATUS(addAttribute(QuikDeformNode::frameRate));
	CHECK_MSTATUS(addAttribute(QuikDeformNode::currentFrame));
	CHECK_MSTATUS(addAttribute(QuikDeformNode::mass));
	CHECK_MSTATUS(addAttribute(QuikDeformNode::initialVelocity));
	// constraint attributes
	CHECK_MSTATUS(addAttribute(QuikDeformNode::doStrainConstraint));
	CHECK_MSTATUS(addAttribute(QuikDeformNode::strainConstraintWeight));
	CHECK_MSTATUS(addAttribute(QuikDeformNode::doVolumeConstraint));
	CHECK_MSTATUS(addAttribute(QuikDeformNode::volumeConstraintWeight));
	CHECK_MSTATUS(addAttribute(QuikDeformNode::doBendingConstraint));
	CHECK_MSTATUS(addAttribute(QuikDeformNode::bendingConstraintWeight));
	// external forces attributes
	CHECK_MSTATUS(addAttribute(QuikDeformNode::doGravity));
	CHECK_MSTATUS(addAttribute(QuikDeformNode::gravityForce));
	CHECK_MSTATUS(addAttribute(QuikDeformNode::doWind));
	CHECK_MSTATUS(addAttribute(QuikDeformNode::windForce));

	// ---------------------------------------
	// link created attributes 
	// ---------------------------------------
	attributeAffects(QuikDeformNode::currentFrame, QuikDeformNode::outputMesh);
	/*
	returnStatus = attributeAffects(LSystemNode::time, LSystemNode::outputMesh);
	McheckErr(returnStatus, "ERROR in attributeAffects\n");
	returnStatus = attributeAffects(LSystemNode::angle, LSystemNode::outputMesh);
	McheckErr(returnStatus, "ERROR in attributeAffects\n");
	returnStatus = attributeAffects(LSystemNode::stepSize, LSystemNode::outputMesh);
	McheckErr(returnStatus, "ERROR in attributeAffects\n");
	returnStatus = attributeAffects(LSystemNode::grammar, LSystemNode::outputMesh);
	McheckErr(returnStatus, "ERROR in attributeAffects\n");
	*/


	return MS::kSuccess;
}


MStatus QuikDeformNode::compute(const MPlug& plug, MDataBlock& data) {
	MStatus returnStatus;

	// get simulation attributes
	MDataHandle inputMeshData = data.inputValue(inputMesh); // TODO: is it too much wasted effort to get the new input Mesh at every compute?
	MDataHandle timeStepData = data.inputValue(timeStep);
	MDataHandle solverIterationsData = data.inputValue(solverIterations);
	MDataHandle framesToSimulateData = data.inputValue(framesToSimulate);
	MDataHandle frameRateData = data.inputValue(frameRate);
	MDataHandle currentFrameData = data.inputValue(currentFrame);
	MDataHandle massData = data.inputValue(mass);
	MDataHandle initialVelocityData = data.inputValue(initialVelocity);

	MObject originalObj = inputMeshData.asMesh(); // this returns a local space mesh https://tinyurl.com/ycgcmc49
	MFnMesh originalMesh = originalObj;
	double dt = timeStepData.asDouble();
	int iter = solverIterationsData.asInt();
	int totalFrames = framesToSimulateData.asInt();
	int curFrame = currentFrameData.asInt();

	MGlobal::displayInfo(("total frames is " + std::to_string(totalFrames)).c_str());
	MGlobal::displayInfo(("current frame is " + std::to_string(curFrame)).c_str());

	// get constraint attributes
	MDataHandle doStrainConstraintData = data.inputValue(doStrainConstraint);
	bool doStrain = doStrainConstraintData.asBool();
	MDataHandle doVolumeConstraintData = data.inputValue(doVolumeConstraint);
	bool doVolume = doVolumeConstraintData.asBool();
	MDataHandle doBendingConstraintData = data.inputValue(doBendingConstraint);
	bool doBending = doBendingConstraintData.asBool();

	//TODO: get external forces attributes


	// create output data
	MDataHandle outputMeshData = data.outputValue(outputMesh);
	MFnMeshData dataCreator;
	MObject newOutputObj = dataCreator.create();
	MFnMesh newMesh;
	newMesh.copy(originalObj, newOutputObj);


	// get the vertices and faces
	MFloatPointArray vertices;
	originalMesh.getPoints(vertices);
	std::vector<std::vector<int>> indices;
	getTrianglesHelper(originalMesh, indices);

	/*
	for (unsigned int i = 0u; i < vertices.length(); i++) {
	MFloatPoint vertex = vertices[i];
	std::string msg = "vertex " + std::to_string(i) + " is at " + std::to_string(vertex[0]) + ", " + std::to_string(vertex[1]) + ", " + std::to_string(vertex[2]);
	MGlobal::displayInfo(msg.c_str());
	}

	// get the faces
	std::vector<std::vector<int>> indices;
	getTrianglesHelper(originalMesh, indices);

	MGlobal::displayInfo("printing triangleVertices");
	for (auto i = 0u; i < indices.size(); i++) {
	std::string msg = "triangle " + std::to_string(i) + " has " + std::to_string(indices[i][0]) + ", "
	+ std::to_string(indices[i][1]) + ", "	+ std::to_string(indices[i][2]);
	MGlobal::displayInfo(msg.c_str());
	}
	*/

	// test if tetgen works
	tetgenio tetInput, tetOutput;
	tetgenio::facet *f;
	tetgenio::polygon *p;
	tetInput.firstnumber = 1;
	
	// populate the points
	tetInput.numberofpoints = vertices.length();
	tetInput.pointlist = new REAL[tetInput.numberofpoints * 3];
	for (int i = 0; i < tetInput.numberofpoints; i++) {
		tetInput.pointlist[i * 3]	  = vertices[i][0];
		tetInput.pointlist[i * 3 + 1] = vertices[i][1];
		tetInput.pointlist[i * 3 + 2] = vertices[i][2];
	}

	// populate the facets
	tetInput.numberoffacets = indices.size();
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
		p->vertexlist[0] = indices[i][0];
		p->vertexlist[1] = indices[i][1];
		p->vertexlist[2] = indices[i][2];
	}

	tetgenbehavior tetCmd;
	tetCmd.parse_commandline("pq");
	tetgenH::tetrahedralize(&tetCmd, &tetInput, &tetOutput);

	// print the number of points
	for (int i = 0; i < tetOutput.numberofpoints; i++) {
		std::string msg = "vertex " + std::to_string(i) + " is at " 
			+ std::to_string(tetOutput.pointlist[i * 3]) + ", " 
			+ std::to_string(tetOutput.pointlist[i * 3] + 1) + ", "
			+ std::to_string(tetOutput.pointlist[i * 3] + 2);
		MGlobal::displayInfo(msg.c_str());
	}
	// print the number of tetrahedrons
	for (int i = 0; i < tetOutput.numberoftetrahedra; i++) {
		std::string msg = "tetrahedron " + std::to_string(i) + " has "
			+ std::to_string(tetOutput.tetrahedronlist[i * 4]) + ", "
			+ std::to_string(tetOutput.tetrahedronlist[i * 4 + 1]) + ", "
			+ std::to_string(tetOutput.tetrahedronlist[i * 4 + 2]) + ", "
			+ std::to_string(tetOutput.tetrahedronlist[i * 4 + 3]);
		MGlobal::displayInfo(msg.c_str());
	}
	// print the number of triangles
	for (int i = 0; i < tetOutput.numberoftrifaces; i++) {
		std::string msg = "triFace " + std::to_string(i) + " has "
			+ std::to_string(tetOutput.trifacelist[i * 3]) + ", "
			+ std::to_string(tetOutput.trifacelist[i * 3 + 1]) + ", "
			+ std::to_string(tetOutput.trifacelist[i * 3 + 2]);
		MGlobal::displayInfo(msg.c_str());
	}


	// move the vertices 
	// for now just compute all frames from the get go. assume attributes won't change
	if (initialPosition.length() == 0) {
		// fill the initial position
		initialPosition = vertices;
	}

	// use QuikDeformer to calculate new vertices
	/*
	TODO: figure out way to pass the following into QuikDeformer object constructor:
	All vertices
	All fragments // 1 BASED INDEXING
	All tetrahedrons (from tetgen)
	*/


	// compute all frames
	if (computedFrames.size() < totalFrames) {
		for (auto i = computedFrames.size(); i < totalFrames; i++) {
			if (i == 0) {
				computedFrames.push_back(initialPosition);
			}
			else {
				computedFrames.push_back(computedFrames[i - 1]);
			}

			for (unsigned int j = 0u; j < computedFrames[i].length(); j++) {
				computedFrames[i][j][2] += 0.2f;
			}
		}
	}

	// update output
	if (curFrame <= totalFrames) {
		newMesh.setPoints(computedFrames[curFrame - 1]);
		newMesh.setObject(newOutputObj);
		outputMeshData.set(newOutputObj);

		data.setClean(plug);
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