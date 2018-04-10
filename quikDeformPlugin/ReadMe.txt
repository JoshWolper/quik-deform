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

- update the GUI to better reflect the parameters
- hook up all the parameters with actual values in quikDeformer
- dynamically add more vertices and edges to the surface of the mesh
	- look into addPolygon function()
- make a GUI menu in Mel to allow initial data setting
- GUI to set point constraints
	figure out how to get the vertex index given a selected vertex
- GUI to set ground plane
- compute button for the node