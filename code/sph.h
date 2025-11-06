#ifndef SPH_H
#define SPH_H
#include "particle.h"
#include "particlesystem.h"
#include "integrators.h"
#include "hash.h"
#include <math.h>

class SPH : public Force
{
public:
    SPH(ParticleSystem system, double width, double height, double depth);
    ParticleSystem getSystem(){return system;}
    void setSystem(ParticleSystem system){this->system = system;}
    void computeDensityPressure();
    virtual void apply();
    Vec3 spiky(Vec3 r, double h);
    double visco(Vec3 r, double h);
protected:
    ParticleSystem system;
    Hash* hash;
    IntegratorSymplecticEuler integrator;
    double width, height, depth;
    double h = 15;
    double poly6 = 315/(64 * M_PI * std::pow(h,9)); //muller's poly6 kernel
    double gasConstant = 1;
    double restDensity = 1000;
    double viscosity = 0.001;
};

#endif // SPH_H
