#ifndef MESHIO_H
#define MESHIO_H
#include <string>
#include "XMesh.h"

template<class T_REAL,class T_IDX>
void loadOBJ(std::string fpath,XMesh<T_REAL,T_IDX>* m);

template<class T_REAL, class T_IDX>
void loadSTL(std::string fpath,XMesh<T_REAL, T_IDX>* m);



#endif
