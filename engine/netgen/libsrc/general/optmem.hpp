#ifndef FILE_OPTMEM
#define FILE_OPTMEM

/**************************************************************************/
/* File:   optmem.hh                                                      */
/* Author: Joachim Schoeberl                                              */
/* Date:   04. Apr. 97                                                    */
/**************************************************************************/

namespace netgen
{

/** 
    Optimized Memory allocation classes
*/

class BlockAllocator
{
private:
  ///
  unsigned size, blocks;
  ///
  void * freelist;
  ///
  NgArray<char*> bablocks;
  mutex block_allocator_mutex;
public:
  ///
   BlockAllocator (unsigned asize, unsigned ablocks = 100);
  ///
   ~BlockAllocator ();
  ///
   void * Alloc ();
  ///
   void Free (void * p);
};

}

#endif
