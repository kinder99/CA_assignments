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
    Vec3 position = p->pos;
    Vec3 min = this->getMin();
    Vec3 max = this->getMax();
    //check if point is within the interval min-max for all axes
    if(position.x() >= min.x() && position.x() <= max.x() && position.y() >= min.y() && position.y() <= max.y() && position.z() >= min.z() && position.z() <= max.z()){
        return true;
    }
    return false;
}

bool ColliderAABB::testCollision(const Particle* p, Collision& colInfo) const
{
    Vec3 p0 = p->prevPos;
    Vec3 p1 = p->pos;
    Vec3 r = p1 - p0;
    Vec3 r_inv = Vec3(1/r.x(),1/r.y(),1/r.z());
    Vec3 min = this->getMin();
    Vec3 max = this->getMax();

    //check if ray is parallel to any of the planes of the aabb
    if(r.x() == 0){
        if(p0.x() <= min.x() && p0.x() >= max.x()){
            return false;
        }
    }
    if(r.y() == 0){
        if(p0.y() <= min.y() && p0.y() >= max.y()){
            return false;
        }
    }
    if(r.z() == 0){
        if(p0.z() <= min.z() && p0.z() >= max.z()){
            return false;
        }
    }

    //compute slab range for all axes
    //according to wikipedia, t front and back can be computed with t = (p-o)/r
    //we can avoid dividing by 0 by multiplying by the inverse of r instead
    double back_x = (min.x()-p0.x())*r_inv.x();
    double front_x = (max.x()-p0.x())*r_inv.x();

    double back_y = (min.y()-p0.y())*r_inv.y();
    double front_y = (max.y()-p0.y())*r_inv.y();

    double back_z = (min.z()-p0.z())*r_inv.z();
    double front_z = (max.z()-p0.z())*r_inv.z();

    //slab segments
    double t_close_x = std::min(back_x,front_x);
    double t_far_x = std::max(back_x,front_x);

    double t_close_y = std::min(back_y,front_y);
    double t_far_y = std::max(back_y,front_y);

    double t_close_z = std::min(back_z,front_z);
    double t_far_z = std::max(back_z,front_z);

    //intersections of slab segments
    //for normal, get axis of smallest value
    double t_back = std::max({t_close_x, t_close_y, t_close_z});
    Vec3 point_back = p0 + t_back*r;
    double t_front = std::min({t_far_x, t_far_y, t_far_z});
    Vec3 point_front = p0 + t_front*r;
    if(t_back <= t_front){
        if(t_back < 0){
            colInfo.position = point_front;
            if(t_far_x < t_far_y && t_far_x < t_far_z){
                colInfo.normal = Vec3(-1,0,0);
            }
            else if(t_far_y < t_far_x && t_far_y < t_far_z){
                colInfo.normal = Vec3(0,-1,0);
            }
            else{
                colInfo.normal = Vec3(0,0,-1);
            }
        }
        else{
            colInfo.position = point_back;
            if(t_close_x > t_close_y && t_close_x > t_close_z){
                colInfo.normal = Vec3(1,0,0);
            }
            else if(t_close_y > t_close_x && t_close_y > t_close_z){
                colInfo.normal = Vec3(0,1,0);
            }
            else{
                colInfo.normal = Vec3(0,0,1);
            }
        }
        return true;
    }
    return false;
}
