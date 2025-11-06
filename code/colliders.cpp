#include "colliders.h"
#include <cmath>
#include <iostream>

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
    if(std::sqrt((p->pos-this->getCenter()).dot((p->pos-this->getCenter()).transpose()))>this->getRadius()){
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
        colInfo.normal = (colInfo.position - this->getCenter()).normalized();
        return true;
    }
    return false;
}

void ColliderSphere::resolveCollision(Particle* p, const Collision& col, double kElastic, double kFriction) const
{
    //define tangent plane to collision then call generic resolve

    ColliderPlane plane = ColliderPlane(col.normal, -(col.normal.dot(col.position)));

    plane.resolveCollision(p, col, kElastic, kFriction);
}

/*
 * AABB
 */
bool ColliderAABB::isInside(const Particle* p) const
{
    Vec3 position = p->prevPos;
    Vec3 min = this->getMin();
    Vec3 max = this->getMax();
    //check if point is within the interval min-max for all axes
    if(position.x() >= min.x() && position.x() <= max.x() && position.y() >= min.y() && position.y() <= max.y() && position.z() >= min.z() && position.z() <= max.z()){
        return true;
    }
    return false;
}


static bool inInterval(Vec3 pos, Vec3 min, Vec3 max) {
    return (min.x() <= pos.x() && pos.x() <= max.x()) && (min.y() <= pos.y() && pos.y() <= max.y()) && (min.z() <= pos.z() && pos.z() <= max.z());
}

bool ColliderAABB::testCollision(const Particle* p, Collision& colInfo) const // <= this is currently broken. i hate this. very, very much.
//particle agglomerate to the collided surface instead of bouncing off
{
    Vec3 p0 = p->prevPos;
    Vec3 p1 = p->pos;
    Vec3 r = (p1 - p0);
    Vec3 r_inv = Vec3(1/r.x(),1/r.y(),1/r.z());
    Vec3 min = this->getMin();
    Vec3 max = this->getMax();

    double entry = 0.0;
    double exit = 1.0;
    Vec3 normal = Vec3(0,0,0);

    //check if particle moves
    if(r == Vec3(0,0,0)){
        return false;
    }

    //both positions are inside the aabb
    if(inInterval(p0, min, max) && inInterval(p1, min, max)){
        return false;
    }

    //iterate over all axes
    for(int i = 0 ; i < 3 ; i++) {
        // check if direction of movement is parallel to current axis
        if(r[i] == 0) {
            if (p0[i] < min[i] || p0[i] > max[i]) {
                return false;
            }
        }
        else { //multiply by inverse of r instead of dividing (actually useless considering i added a check for parallel case but i felt smart thinking of this so it stays)
            double t0 = (min[i] - p0[i])*r_inv[i];
            double t1 = (max[i] - p0[i])*r_inv[i];

            if(t0 > t1) {
                std::swap(t0, t1); //we want t1>t0, always
            }

            if (t0 > entry) {
                entry = t0; //get point of entry
                if(r[i]<0){ //assign normals according to direction
                    normal[i] = 1.0;
                }
                else {
                    normal[i] = -1.0;
                }
            }
            exit = std::min(exit, t1); //get point of exit
            if (entry > exit){
                return false; //check for overlap
            }
        }
    }
    colInfo.normal = normal;
    colInfo.position = p0 + r * entry;
    return true;
}


void ColliderAABB::resolveCollision(Particle* p, const Collision& col, double kElastic, double kFriction) const
{
    //define tangent plane to collision then call generic resolve

    ColliderPlane plane = ColliderPlane(col.normal, -(col.normal.dot(col.position)));

    plane.resolveCollision(p, col, kElastic, kFriction);
}
