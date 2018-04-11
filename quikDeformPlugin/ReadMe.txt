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
- add tetGen's a slider to GUI - DONE


- dynamically add more vertices and edges to the surface of the mesh
	- look into addPolygon function()
	- how do we know which vertices are on the surface?
		original vertices are still at the front, but new vertices are tagged on seemingly randomly 
		triangle index list is rearranged
	- might not work
	- just use what's already in maya
- make a GUI menu in Mel to allow initial data setting
- GUI to set point constraints
	figure out how to get the vertex index given a selected vertex
- save data when saving Maya scene 
	nAttr.setStorable(true)?
- GUI to set ground plane
- subdivision slider
	dynamically change a to allow subdivisions
- progress bar?
	https://help.autodesk.com/cloudhelp/2017/ENU/Maya-Tech-Docs/CommandsPython/progressBar.html
- compute button for the node (MUST HAVE otherwise sliders are useless)
	have a textfield that reads an attribute that says "need to compute"
	whenever compute runs, just check if config changed. if so, display the "need to compute" textfield
	whenever the compute button is clicked, run the simulation based on current configuration
	https://stackoverflow.com/questions/49631282/how-to-add-command-button-to-maya-aetemplate


// Maya documentation
https://help.autodesk.com/view/MAYAUL/2017/ENU/?guid=__files_Dependency_graph_plugins_The_basics_htm