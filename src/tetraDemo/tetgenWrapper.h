#ifndef TETGEN_WRAPPER
#define TETGEN_WRAPPER
#include "tetgen/tetgen.h"
#include "EigenMeshD.h"
#include <string>

class TetgenWrapper
{
public:

	void fromEigen(EigenMeshD& egm);
	bool toEigen(EigenMeshD& egm);
	void run();

	char tetvcoding[4][3] =
	{
		{1,2,3},
		{2,0,3},
		{0,1,3},
		{1,0,2}
	};

	tetgenbehavior tetparam;
	tetgenio in,out;


};
#endif // !TETGEN_WRAPPER
