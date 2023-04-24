#ifndef TETGEN_WRAPPER
#define TETGEN_WRAPPER
#include "tetgen/tetgen.h"
#include "EigenMeshD.h"
#include <string>

class TetgenWrapper
{
public:

	void fromEigen(EigenMeshD& egm);
	bool toSTD();
	void run();
	void Vis();

	char tetvcoding[4][3] =
	{
		{1,2,3},
		{2,0,3},
		{0,1,3},
		{1,0,2}
	};

	tetgenbehavior tetparam;
	tetgenio in,out;

	std::vector<std::vector<REAL > > V;
	std::vector<std::vector<int> > T;
	std::vector<std::vector<int> > F;
};
#endif // !TETGEN_WRAPPER
