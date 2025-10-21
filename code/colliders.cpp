#include "colliders.h"
#include <cmath>

/*
 * Generic function for collision response from contact plane
 */
void Collider::resolveCollision(Particle* p, const Collision& col, double kElastic, double kFriction) const
{
    Vec3 past_postion = p->prevPos;
    Vec3 plane_normal = col.normal;
    double d = -(col.normal.dot(col.position));

    //get predicted state
    Vec3 predicted_postion = p->pos;
    Vec3 predicted_velocity = p->vel;

    //collision response
    Vec3 new_position = predicted_postion - (1+kElastic)*(plane_normal.dot(predicted_postion)+d)*(plane_normal);
    Vec3 new_velocity = -kElastic * (plane_normal.dot(predicted_velocity)*plane_normal) + ((1-kFriction)*(predicted_velocity - plane_normal.dot(predicted_velocity)*plane_normal));

    p->pos = new_position;
    p->vel = new_velocity;
}

/*
 * Plane
 */
bool ColliderPlane::isInside(const Particle* p) const
{
    if(planeN.dot(p->pos) + planeD <= 0){
        return true;
    }
    return false;
}


bool ColliderPlane::testCollision(const Particle* p, Collision& colInfo) const
{

    Vec3 r = (p->pos - p->prevPos);
    double lambda = -(planeN.dot(p->prevPos)+planeD)/(planeN.dot(r));
    if(0<=lambda && lambda<=1){
        colInfo.normal = planeN;
        colInfo.position = p->prevPos+lambda*r;
        return true;
    }
    return false;
}



/*
 * Sphere
 */
bool ColliderSphere::isInside(const Particle* p) const
{
    if((p->pos-this->getCenter()).dot((p->pos-this->getCenter()).transpose())>this->getRadius()){
        return true;
    }
    return false;
}


bool ColliderSphere::testCollision(const Particle* p, Collision& colInfo) const
{
    Vec3 p0 = p->prevPos;
    Vec3 p1 = p->pos;
    Vec3 C = this->getCenter();
    double r = this->getRadius();
    Vec3 v = (p1 - p0);
    double a = v.dot(v);
    double b = 2*v.dot(p0 - C);
    double c = C.dot(C) + p0.dot(p0) - 2*(p0.dot(C)) - r*r;
    double delta = b*b - 4*a*c;
    double lambda = (-b-std::sqrt(delta))/(2*a);
    if(lambda < 0){
        lambda = (-b+std::sqrt(delta))/(2*a);
    }
    if(0<=lambda && lambda<=1){
        colInfo.position = p0+lambda*v;
        colInfo.normal = -(colInfo.position - this->getCenter());
        return true;
    }
    return false;
}



/*
 * AABB
 */
bool ColliderAABB::isInside(const Particle* p) const
{
    // TODO
    return false;
}


bool ColliderAABB::testCollision(const Particle* p, Collision& colInfo) const
{
    // TODO
    return false;
}
