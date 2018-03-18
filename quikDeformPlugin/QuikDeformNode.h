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

// QuikDeform header files
#include <Eigen\Eigen>
#include "global.h"
#include "QuikDeformer.h"

// stl and other header files
#include <iostream>
#include <string>


// defines a custom maya quikDeform Node
class QuikDeformNode : public MPxNode
{
public:
	QuikDeformNode() {};
	virtual ~QuikDeformNode() {};
	virtual MStatus compute(const MPlug& plug, MDataBlock& data);
	static void* creator();
	static MStatus initialize();

	// simulation attributes
	static MObject timeStep;
	static MObject solverIterations;
	static MObject framesToSimulate;
	static MObject frameRate;
	static MObject mass;
	static MObject initialVelocity;
	
	// constraint attributes
	static MObject doStrainConstraint;
	static MObject strainConstraintWeight;
	static MObject doVolumeConstraint;
	static MObject volumeConstraintWeight;
	static MObject doBendingConstraint;
	static MObject bendingConstraintWeight;

	// external forces attributes
	static MObject doGravity;
	static MObject gravityForce;
	static MObject doWind;
	static MObject windForce;

	static MTypeId id;
};