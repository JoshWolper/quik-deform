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
	"need to recompute" text that shows up when you change inputs - kind of done but has bugs so not using atm
- set point constraints in attribute editor - DONE
- set ground constraints in the attribute editor - DONE
- make a GUI menu in Mel to allow initial data setting - DONE
- allow creating multiple quikDeformNodes simultaneously - DONE
- make sure thin-shell works with maya planes - DONE
	works but only in certain configurations


- save data when saving Maya scene 
	nAttr.setStorable(true)?
		this stores the parameters but not the positions since they're stored and retrieved at run time
	need to keyframe every position
- make recompute happen when you change the initial mesh
	move originalVertices and triangles into inputIO and compare it when necessary


- figure out why cloth is so slow
- add wind parameters to GUI

- test if rendering works
- starting frame
- subdivision slider
	dynamically change a to allow subdivisions
- progress bar?
	https://help.autodesk.com/cloudhelp/2017/ENU/Maya-Tech-Docs/CommandsPython/progressBar.html
	https://help.autodesk.com/cloudhelp/2017/ENU/Maya-Tech-Docs/Commands/show.html?componentBox.html&cat=Windows
	progress bar mel command


// Maya documentation
https://help.autodesk.com/view/MAYAUL/2017/ENU/?guid=__files_Dependency_graph_plugins_The_basics_htm