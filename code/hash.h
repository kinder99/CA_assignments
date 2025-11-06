#ifndef HASH_H
#define HASH_H

#include "defines.h"
#include "particlesystem.h"
#include <vector>

class Hash
{
public:
    Hash();
    Hash(int spacing, int maxNum, ParticleSystem* system);

    int hashCoordinates(int x, int y, int z);
    Vec3 intCoordinates(Vec3 coord);
    int hashPos(int nr);
    void create(int nr);
    void query(int nr, int maxDist);

    std::vector<int>* getGrid(){return grid;}
    std::vector<int>* getCells(){return cells;}
    std::vector<int>* getIDs(){return particleIDs;}

    void setSystem(ParticleSystem* system);
    void setSpacing(int spacing){this->spacing = spacing;}

    int getQuerySize(){return querySize;}
protected:
    int spacing;
    int size;
    std::vector<int>* grid;
    std::vector<int>* cells;
    std::vector<int>* particleIDs;
    int querySize;
    ParticleSystem* system;
};

#endif // HASH_H
