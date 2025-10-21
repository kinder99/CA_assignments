#include "integrators.h"
#include <iostream>

// timestep 0.5, default system params, 10 steps (10 times 1 step)


void IntegratorEuler::step(ParticleSystem &system, double dt) {
    //10 steps :
    //analytical : x = 5.26670, v = -1.89764
    //numerical : x = 5.38485, v = -1.96082
    double t0 = system.getTime();
    Vecd x0 = system.getState();
    Vecd dx = system.getDerivative();
    Vecd x1 = x0 + dt*dx;
    system.setState(x1);
    system.setTime(t0+dt);
    system.updateForces();
}

void IntegratorSymplecticEuler::step(ParticleSystem &system, double dt) {
    //10 steps :
    //analytical : x = 5.26670, v = -1.89764
    //numerical : x = 5.15630, v = -1.85609
    double t0 = system.getTime();
    Vecd dx = system.getState();
    Vecd v1 = system.getVelocities() + dt*system.getAccelerations();
    system.setVelocities(v1);
    Vecd p1 = system.getPositions() + dt*v1;
    system.setPositions(p1);
    system.setTime(t0+dt);
    system.updateForces();
}

void IntegratorMidpoint::step(ParticleSystem &system, double dt) {
    //10 steps :
    //analytical : x = 5.26670, v = -1.89764
    //numerical : x = 5.26578, v = -1.90063
    double t0 = system.getTime();
    Vecd x0 = system.getState();
    Vecd dx = system.getDerivative();
    Vecd midx = x0+dt*dx/2;
    system.setState(midx);
    system.setTime(t0 + dt/2);
    system.updateForces();
    Vecd dv1 = system.getDerivative();
    Vecd x1 = x0+dt*dv1;
    system.setState(x1);
    system.setTime(t0+dt);
    system.updateForces();
}

void IntegratorRK2::step(ParticleSystem &system, double dt) {
    //10 steps :
    //analytical : x = 5.26670, v = -1.89764
    //numerical : x = 5.26567, v = -1.90136
    double t0 = system.getTime();
    Vecd x0 = system.getState();
    Vecd k1 = system.getDerivative();
    Vecd xk1 = x0 + dt*k1;
    system.setTime(t0+dt);
    system.setState(xk1);
    system.updateForces();

    Vecd k2 = system.getDerivative();
    Vecd x1 = x0+dt/2*(k1+k2);

    system.setState(x1);
    system.updateForces();
}

void IntegratorRK4::step(ParticleSystem &system, double dt) {
    //10 steps :
    //analytical : x = 5.26670, v = -1.89764
    //numerical : x = 5.26670, v = -1.89763
    double t0 = system.getTime();
    Vecd x0 = system.getState();
    Vecd k1 = system.getDerivative();
    Vecd xk1 = x0+dt/2*k1;
    system.setState(xk1);
    system.setTime(t0+dt/2);
    system.updateForces();

    Vecd k2 = system.getDerivative();
    Vecd xk2 = x0+dt/2*k2;
    system.setTime(t0+dt/2);
    system.setState(xk2);
    system.updateForces();

    Vecd k3 = system.getDerivative();
    Vecd xk3 = x0+dt*k3;
    system.setTime(t0+dt);
    system.setState(xk3);
    system.updateForces();

    Vecd k4 = system.getDerivative();
    Vecd x = x0 + dt/6*(k1 + 2*k2 + 2*k3 + k4);
    system.setTime(t0+dt);
    system.setState(x);
    system.updateForces();
}

void IntegratorVerlet::step(ParticleSystem &system, double dt) { //broken
    //10 steps :
    //analytical : x = 5.26670, v = -1.89764
    //numerical : x = 5.15630, v = -1.85609
    double t0 = system.getTime();
    Vecd pt = system.getPositions();
    Vecd pmt = system.getPreviousPositions();
    if(t0 == 0.0){
        pmt = pt - system.getVelocities()*dt;
    }
    Vecd pdt = pt + (pt - pmt) + dt*dt*system.getAccelerations();
    system.setPreviousPositions(pt);
    system.setPositions(pdt);
    Vecd vdt = (pdt - pt)/dt;
    system.setVelocities(vdt);
    system.setTime(t0+dt);
    system.updateForces();
}
