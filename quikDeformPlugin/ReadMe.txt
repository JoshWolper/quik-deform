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
- figure out how to run tetgen in code and feed tetrahedrons (just vector<vector<int>>) into quikDeformer
- figure out how to represent mesh info once computed
	- copy the mesh and change the vertex positions 
	- change the vertex positions of the mesh directly
- hook up all the parameters with actual values in quikDeformer