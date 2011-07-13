#ifndef BICC_H
#define BICC_H

#include <vector>

class BiCC
{
public:
    BiCC(unsigned int rows, unsigned int cols);
    
    void doInit(unsigned int rows, unsigned int cols);

    int findComponents(const std::vector<unsigned int>& adj_sub);
    std::vector<unsigned int> getLabelMatrix(){ return m_vuiLabelMatrix; };
    std::vector<unsigned int> getLabelVector(){ return m_vuiLabelVector; };

    unsigned int getRowLabel(unsigned int i);
    unsigned int getColLabel(unsigned int j);
    unsigned int getLabelElement(unsigned int k);
    
private:
    std::vector<unsigned int> m_vuiLabelMatrix;
    std::vector<unsigned int> m_vuiLabelVector;
    std::vector<unsigned int> m_vuiAdj;

    unsigned int m_iNRows;
    unsigned int m_iNCols;
    unsigned int m_iCurrentLabel;

    void followRow(unsigned int i);
    void followCol(unsigned int j);

    bool areAdjacent(unsigned int i, unsigned int j);
    bool unlabeledAdjacent(unsigned int i, unsigned int j);

    void setLabelM(unsigned int i, unsigned int j);
    void setLabel(unsigned int i, unsigned int j);
    void setRowLabel(unsigned int i);
};

#endif  // BICC_H
    
