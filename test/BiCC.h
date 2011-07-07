#ifndef BICC_H
#define BICC_H

unsigned int* BiCC_CreateMat(unsigned int rows, unsigned int cols);
void BiCC_ReleaseMat(unsigned int* mat);
int BiCC_Do(const unsigned int* adj_sub,
            unsigned int rows,
            unsigned int cols,
            unsigned int* label,
            unsigned int* label_M);

#endif  // BICC_H
    
