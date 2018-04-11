/**
	Defines the QuikDeformNode class. Can be applied in Maya as a node in the dependency graph
	to simulate a deformable object using the QuikDeformer class.
 */

#pragma once

// Maya header files
#include <maya/MFnPlugin.h>
#include <maya/MArgDatabase.h>
#include <maya/MArgList.h>
#include <maya/MObject.h>
#include <maya/MGlobal.h>
#include <maya/MPxCommand.h>
#include <maya/MSyntax.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MSimple.h>
#include <maya/MSelectionList.h>
#include <maya/MDagPath.h>
#include <maya/MFnMesh.h>
#include <maya/MFnMeshData.h>
#include <maya/MPoint.h>
#include <maya/MPointArray.h>
#include <maya/MItMeshPolygon.h>
#include <maya/MVector.h>
#include <maya/MFloatVector.h>

// QuikDeform header files
#include <Eigen\Eigen>
#include <tetgen.h>
#include "global.h"
#include "QuikDeformer.h"

// stl and other header files
#include <iostream>
#include <string>


// a simple struct that lets QuikDeformNode pass inputs into QuikDeformer
struct QuikDeformNodeIO {
	// simulation attributes
	double timeStep;
	int solverIterations;
	int secondsToSimulate;
	int frameRate;

	// object attributes
	double tetVolume;
	double mass;
	MVector initialVelocity;
	bool volumetric;
	double youngsModulus;
	double poissonRatio;

	// external force
	bool doGravity;
	bool doWind;
	MVector windDirection;
	double windMagnitude;
	bool windOscillation;

	bool operator==(const QuikDeformNodeIO& rhs) const {
		return
			// simulation
			timeStep == rhs.timeStep &&
			solverIterations == rhs.solverIterations &&
			secondsToSimulate == rhs.secondsToSimulate &&
			frameRate == rhs.frameRate &&
			// object 
			tetVolume == rhs.tetVolume &&
			mass == rhs.mass &&
			initialVelocity == rhs.initialVelocity &&
			volumetric == rhs.volumetric &&
			youngsModulus == rhs.youngsModulus &&
			poissonRatio == rhs.poissonRatio &&
			// external forces
			doGravity == rhs.doGravity &&
			doWind == rhs.doWind &&
			windDirection == rhs.windDirection &&
			windMagnitude == rhs.windMagnitude &&
			windOscillation == rhs.windOscillation;
	}

	bool operator!=(const QuikDeformNodeIO& rhs) const {
		return !((*this) == rhs);
	}

	// for debug
	std::string print() {
		return "time step is " + std::to_string(timeStep) +
			"\niter is " + std::to_string(solverIterations) +
			"\nsec is " + std::to_string(secondsToSimulate) +
			"\nframerate is " + std::to_string(frameRate) +
			"\ntetVolume is " + std::to_string(tetVolume) +
			"\nmass is " + std::to_string(mass) +
			"\ninitialVelocity is " + std::to_string(initialVelocity[0]) + ", " +
			std::to_string(initialVelocity[1]) + ", " +
			std::to_string(initialVelocity[2]) +
			"\nvolumetric is " + std::to_string(volumetric) +
			"\nE is " + std::to_string(youngsModulus) +
			"\nnu is " + std::to_string(poissonRatio);
	}
};


// define a custom maya quikDeform Node
class QuikDeformNode : public MPxNode
{
public:
	QuikDeformNode() : quikDeformer(nullptr) {};
	virtual ~QuikDeformNode() {
		delete quikDeformer; 
	};
	virtual MStatus compute(const MPlug& plug, MDataBlock& data);
	static void* creator();
	static MStatus initialize();

	// simulation attributes
	static MObject inputMesh;
	static MObject outputMesh;
	static MObject timeStep;
	static MObject solverIterations;
	static MObject secondsToSimulate;
	static MObject frameRate;
	static MObject currentFrame;
	
	// object attributes
	static MObject tetVolume;
	static MObject mass;
	static MObject initialVelocity;
	static MObject volumetric; // if we're doing volumetric or thin-shell simulation TODO: make this into an enum
	static MObject youngsModulus;
	static MObject poissonRatio;

	// external forces attributes
	static MObject doGravity;
	static MObject doWind;
	static MObject windDirection;
	static MObject windMagnitude;
	static MObject windOscillation;

	static MTypeId id;

private:
	// basic node stats
	QuikDeformer* quikDeformer;
	MObject originaObj;
	std::vector<MPointArray> computedFrames;
	std::vector<MObject> computedObjs;
	QuikDeformNodeIO currentConfiguration;
};
