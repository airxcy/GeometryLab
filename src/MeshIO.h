#ifndef MESHIO_H
#define MESHIO_H
#include <string>
#include "XMesh.h"

void loadOBJ(std::string fpath,XMesh* m);
void loadSTL(std::string fpath,XMesh* m);

#endif
