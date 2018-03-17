//
// Copyright (C) 
// 
// File: quikDeformPluginCmd.cpp
//
// MEL Command: quikDeformPlugin
//

#include <maya/MSimple.h>
#include <Eigen\Core>
#include <Eigen\Dense>

#include "quikDeformPluginCmd.h"

// define the long and short names for flags
const char* flag_name_long = "-name";
const char* flag_name = "-n";
const char* flag_id_long = "-idNum";
const char* flag_id = "-i";

// define the MSyntax function used to parse syntax
MSyntax QuikDeformPluginCmd::newSyntax()
{
	MSyntax syntax;

	syntax.addFlag(flag_name, flag_name_long, MSyntax::kString);
	syntax.addFlag(flag_id, flag_id_long, MSyntax::kUnsigned);

	return syntax;
}


MStatus QuikDeformPluginCmd::doIt( const MArgList& args ){
	// default values for flag commands
	MString name = "John Smith";
	unsigned int id = 0;

	MArgDatabase argData(syntax(), args);

	if (argData.isFlagSet(flag_name)){
		argData.getFlagArgument(flag_name, 0, name);
	}
	if (argData.isFlagSet(flag_id)){
		argData.getFlagArgument(flag_id, 0, id);
	}

	MString command = "confirmDialog -title \"Hello Maya\" -message \"Name: " + name + "\\nID: " + id + "\"";
	MStatus stat = MGlobal::executeCommand(command);

	return stat;
}


// Initialize Plugin upon loading
MStatus initializePlugin(MObject obj)
{
	MStatus stat;
	MFnPlugin plugin(obj, "CIS660", "1.0", "Any");
	stat = plugin.registerCommand("quikDeformCmd", QuikDeformPluginCmd::creator, QuikDeformPluginCmd::newSyntax);
	if (!stat) stat.perror("registerCommand failed");
	return stat;
}

// Cleanup Plugin upon unloading
MStatus uninitializePlugin(MObject obj)
{
	MStatus stat;
	MFnPlugin plugin(obj);
	stat = plugin.deregisterCommand("quikDeformCmd");
	if (!stat) stat.perror("deregisterCommand failed");
	return stat;
}