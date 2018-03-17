#pragma once

#include <maya/MFnPlugin.h>
#include <maya/MArgDatabase.h>
#include <maya/MArgList.h>
#include <maya/MObject.h>
#include <maya/MGlobal.h>
#include <maya/MPxCommand.h>
#include <maya/MSyntax.h>


// custom Maya command
class QuikDeformPluginCmd : public MPxCommand
{
public:
	virtual MStatus doIt(const MArgList& args);
	static void *creator() { return new QuikDeformPluginCmd; }
	static MSyntax newSyntax();
};
