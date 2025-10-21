#include "forces.h"

void ForceConstAcceleration::apply() {
    for (Particle* p : particles) {
        // TODO
        p->force += p->mass * this->getAcceleration();
    }
}

void ForceDrag::apply() {
    for (Particle* p : particles) {
        p->force += -this->klinear * p->vel; //Stokes drag
        p->force += -this->kquadratic * p->vel.norm() * p->vel;
    }
}

void ForceSpring::apply() {
    if (particles.size() < 2) return;
    Particle* p1 = getParticle1();
    Particle* p2 = getParticle2();

    auto spring_member = this->getSpringConstant() * ((p2->pos - p1->pos).norm()-this->getRestLength());
    Vecd divide = (p2->pos - p1->pos)/(p2->pos - p1->pos).norm();
    auto damping_member = this->getDampingCoeff()*(p2->vel - p1->vel).dot(divide) ;
    auto f1 = (spring_member + damping_member)*divide;

    p1->force += f1;
    p2->force += -f1;
}

void ForceGravitation::apply() {
    // for (int i = 0; i<particles.max_size(); i++) {
    //     for (int j = 0; j<particles.max_size(); j++) {
    //         Particle* p_i = particles.at(i);
    //         Particle* p_j = particles.at(j);
    //         if (p_i == p_j) {p_i->force += Vecd(0);}
    //         else {
    //             auto first = (this->G * p_i->mass * p_j->mass)/std::pow(std::sqrt(std::pow((p_j->pos.x() - p_i->pos.x()),2)+std::pow((p_j->pos.y() - p_i->pos.y()),2)+std::pow((p_j->pos.z() - p_i->pos.z()),2)),2);
    //             auto second = (p_i->pos - p_j->pos)/(p_i->pos - p_j->pos).norm();
    //             p_i->force += first * second;
    //         }
    //     }
    // }

    for (Particle* p_j : particles) {
        const Particle* p = getAttractor();
        auto first = (getConstant() * p->mass * p_j->mass)/((p->pos - p_j->pos).norm()*(p->pos - p_j->pos).norm());
        auto second = (p->pos - p_j->pos)/(p->pos - p_j->pos).norm();
        auto third = (2/(1+std::exp(-this->a*(((p->pos - p_j->pos).norm()*(p->pos - p_j->pos).norm()))/(this->b*this->b)))-1);
        p_j->force += first * second * third;
    }
}
