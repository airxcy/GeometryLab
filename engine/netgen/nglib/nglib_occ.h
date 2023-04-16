#ifndef NGLIB_OCC_HPP_INCLUDED
#define NGLIB_OCC_HPP_INCLUDED

/// Data type for NETGEN OpenCascade geometry
typedef void * Ng_OCC_Geometry;
typedef void * Ng_OCC_TopTools_IndexedMapOfShape;

// **********************************************************
// **   OpenCascade Geometry / Meshing Utilities           **
// **********************************************************

// Create new OCC Geometry Object
 Ng_OCC_Geometry * Ng_OCC_NewGeometry ();

// Delete an OCC Geometry Object
 Ng_Result Ng_OCC_DeleteGeometry (Ng_OCC_Geometry * geom);

// Loads geometry from STEP file
 Ng_OCC_Geometry * Ng_OCC_Load_STEP (const char * filename);

// Loads geometry from IGES file
 Ng_OCC_Geometry * Ng_OCC_Load_IGES (const char * filename);

// Loads geometry from BREP file
 Ng_OCC_Geometry * Ng_OCC_Load_BREP (const char * filename);

// Set the local mesh size based on geometry / topology
 Ng_Result Ng_OCC_SetLocalMeshSize (Ng_OCC_Geometry * geom,
                                              Ng_Mesh * mesh,
                                              Ng_Meshing_Parameters * mp);

// Mesh the edges and add Face descriptors to prepare for surface meshing
 Ng_Result Ng_OCC_GenerateEdgeMesh (Ng_OCC_Geometry * geom,
                                              Ng_Mesh * mesh,
                                              Ng_Meshing_Parameters * mp);

// Mesh the surfaces of an OCC geometry
 Ng_Result Ng_OCC_GenerateSurfaceMesh (Ng_OCC_Geometry * geom,
                                                 Ng_Mesh * mesh,
                                                 Ng_Meshing_Parameters * mp);

// Get the face map of an already loaded OCC geometry
 Ng_Result Ng_OCC_GetFMap(Ng_OCC_Geometry * geom,
                                    Ng_OCC_TopTools_IndexedMapOfShape * FMap);

 void Ng_OCC_Uniform_Refinement (Ng_OCC_Geometry * geom,
					   Ng_Mesh * mesh);
 void Ng_OCC_Generate_SecondOrder (Ng_OCC_Geometry * geom,
					   Ng_Mesh * mesh);
#endif // NGLIB_OCC_HPP_INCLUDED
