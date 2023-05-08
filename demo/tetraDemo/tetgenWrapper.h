#ifndef TETGEN_WRAPPER
#define TETGEN_WRAPPER
#include "tetgen/tetgen.h"
#include "VolumeMesh.h"
typedef VolumeMesh<REAL, int> TetMesh;
typedef XMesh<REAL, int> TriMesh;
class TetgenWrapper
{
public:
	char tetvcoding[4][3] =
	{
		{1,2,3},
		{2,0,3},
		{0,1,3},
		{1,0,2}
	};
	VolumeMesh<REAL,int> mesh;
	std::vector< std::vector<REAL> > steinerVertex;

	tetgenbehavior tetparam;
	tetgenio in, out;

	void initSurfaceMesh(TriMesh& egm);
	bool translateOutput();
	void run();

};
#endif // !TETGEN_WRAPPER
