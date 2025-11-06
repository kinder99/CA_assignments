#include "hash.h"

Hash::Hash(int spacing, int maxNum, ParticleSystem* system):spacing(spacing), system(system){
    size = 2 * maxNum;
    grid = new std::vector<int>(size + 1, 0);
    cells = new std::vector<int>(maxNum, 0);
    particleIDs = new std::vector<int>(10*maxNum, 0);
    querySize = 0;
}

int Hash::hashCoordinates(int x, int y, int z){
    int hash = (x * 346782478) ^ (y * 453218576) ^ (z * 821357954);
    return std::abs(hash)%size;
}

Vec3 Hash::intCoordinates(Vec3 coord){
    int x = std::floor(coord.x()/spacing);
    int y = std::floor(coord.y()/spacing);
    int z = std::floor(coord.z()/spacing);
    return Vec3(x,y,z);
}

int Hash::hashPos(int nr){
    Vec3 pos = system->getParticle(nr)->pos;
    return hashCoordinates(pos.x(),pos.y(),pos.z());
}

void Hash::create(int nr){
    delete grid;
    delete cells;
    grid = new std::vector<int>(size + 1, 0);
    cells = new std::vector<int>(size, 0);
    int numObj = std::min(nr, (int) cells->size());
    //the vectors have already been initialized with zeros in the constructor

    for(int i = 0; i<numObj; i++){
        int hash = hashPos(i);
        grid->at(hash)++;
    }

    int start = 0;
    for(int i = 0; i<size; i++){
        start += grid->at(i);
        grid->at(i) = start;
    }
    grid->at(size) = start;

    for(int i = 0; i<numObj; i++){
        int h = hashPos(i);
        grid->at(h)--;
        cells->at(grid->at(h)) = i;
    }
}

void Hash::query(int nr, int maxDist){
    Vec3 temp_pos0 = system->getParticle(nr)->pos;
    Vec3 pos0 = Vec3(temp_pos0.x() - maxDist, temp_pos0.y() - maxDist, temp_pos0.z() - maxDist);
    Vec3 p0 = intCoordinates(pos0);

    Vec3 temp_pos1 = system->getParticle(nr)->pos;
    Vec3 pos1 = Vec3(temp_pos1.x() + maxDist, temp_pos1.y() + maxDist, temp_pos1.z() + maxDist);
    Vec3 p1 = intCoordinates(pos1);

    querySize = 0;

    for(int xi = p0.x(); xi<p1.x(); xi++){
        for(int yi = p0.y(); yi<p1.y(); yi++){
            for(int zi = p0.z(); zi<p1.z(); zi++){
                int hash = hashCoordinates(xi, yi, zi);
                int start = grid->at(hash);
                int end = grid->at(hash+1);

                for(int i = start; i<end; i++){
                    particleIDs->at(querySize) = cells->at(i);
                    querySize++;
                }
            }
        }
    }
}









