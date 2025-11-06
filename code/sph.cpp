#include "sph.h"

SPH::SPH(ParticleSystem system, double width, double height, double depth):system(system),width(width),depth(depth){

}

void SPH::computeDensityPressure(){
    for(Particle* p : system.getParticles()){ //iterate over every particle
        p->pressure = 0;
        for(Particle* p2 : system.getParticles()){ //get all other particles (including current one)
            double dist = (p->pos - p2->pos).norm();
            if(dist < h*h){
                p->density += p->mass * poly6 * std::pow((h*h - dist),3);
            }
        }
        p->pressure = gasConstant * (p->density * restDensity);
    }
}

Vec3 SPH::spiky(Vec3 r, double h){
    float r_norm = r.norm();
    if(r_norm > h) return {0.f, 0.f, 0.f};
    return -r * 45.f / (M_PI * std::pow(h, 6) * r_norm) * std::pow(h-r_norm, 2.f);
}

double SPH::visco(Vec3 r, double h){
    float r_norm = r.norm();
    if(r_norm > h) return 0;
    return 45.f / (M_PI * std::pow(h, 5)) * (1 - r_norm/h);
}

void SPH::apply(){
    computeDensityPressure();
    for(int i = 0; i<system.getNumParticles(); i++){
        Particle* pi = system.getParticle(i);
        Vec3 a_p(0.f, 0.f, 0.f);
        Vec3 a_v(0.f, 0.f, 0.f);

        float rho_i = pi->density;
        float press_i = pi->pressure;
        float frac_i = press_i / (rho_i*rho_i);
        hash->query(i, h);
        for(int nr = 0; nr<hash->getQuerySize(); nr++) {
            Particle* pj = system.getParticle(nr);
            if(pi == pj) continue;
            Vec3 r = (pj->pos - pi->pos);
            if(r.norm() > h) continue;

            float rho_j = pi->density;
            float press_j = pi->pressure;
            float frac_j = press_j / (rho_j*rho_j);
            float Pij = pj->mass * (frac_i + frac_j);

            a_p += Pij * spiky(r, h);

            Vec3 Vij = viscosity * pj->mass * (pj->vel - pi->vel) / (rho_i * rho_j);
            a_v += Vij * this->visco(r, h);
        }
        // apply force
        pi->force += pi->mass * (a_p + a_v);
    }
}
