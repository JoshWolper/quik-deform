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
- dynamically add more vertices and edges to the surface of the mesh - DONE
- compute button for the node - DONE
	works for multiple QuikDeformNodes
	"need to recompute" text that shows up when you change inputs - DONE but has bugs so not using atm
- set point constraints in attribute editor
- 

- make a GUI menu in Mel to allow initial data setting
	- should have all the data fields
		sim, obj, external force attributes
	- have a window for selecting obj's to simulate 
	- have a window for selecting groundplanes
	- have a window for selecting point constraints
- Attribute Editor set ground/point constraints
	- have a window in the attribute editor that shows what indices are currently selected
		- if an item in the window is selected, you can remove it with the remove button
		- if an item in the viewport is selected, you can add it with the add button
	- feed vertex index data and plane data into C++

- allow creating multiple quikDeformNodes (smart naming)
- save data when saving Maya scene 
	nAttr.setStorable(true)?
- subdivision slider
	dynamically change a to allow subdivisions
- progress bar?
	https://help.autodesk.com/cloudhelp/2017/ENU/Maya-Tech-Docs/CommandsPython/progressBar.html
	https://help.autodesk.com/cloudhelp/2017/ENU/Maya-Tech-Docs/Commands/show.html?componentBox.html&cat=Windows
	progress bar mel command


// Maya documentation
https://help.autodesk.com/view/MAYAUL/2017/ENU/?guid=__files_Dependency_graph_plugins_The_basics_htm