---------------------------------------------------------------------------------
			Installation instructions
---------------------------------------------------------------------------------
- just compile and load the mll file in Maya
- In order to have QuikDeformNode's GUI working correctly, copy the following file:
		AEQuikDeformNodeTemplate.mel
  into a folder where your Maya look for scripts by default, such as this following folder:
		C:\Users\[username]\Documents\maya\2017\prefs\scripts


---------------------------------------------------------------------------------
			Todo
---------------------------------------------------------------------------------
QuikDeformer



Maya integration
- combine mesh data with transform data - DONE
- update the GUI to better reflect the parameters - DONE
- hook up GUI with quikDeformer and recompute everytime a value changes - DONE

- dynamically add more vertices and edges to the surface of the mesh
	- look into addPolygon function()
- make a GUI menu in Mel to allow initial data setting
- GUI to set point constraints
	figure out how to get the vertex index given a selected vertex
- save data when saving Maya scene 
	nAttr.setStorable(true)?
- GUI to set ground plane
- compute button for the node


// Maya documentation
https://help.autodesk.com/view/MAYAUL/2017/ENU/?guid=__files_Dependency_graph_plugins_The_basics_htm