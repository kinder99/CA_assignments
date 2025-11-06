#ifndef SCENESPH_H
#define SCENESPH_H

#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include "scene.h"
#include "widgetsph.h"
#include "particlesystem.h"
#include "integrators.h"
#include "colliders.h"
#include "hash.h"
#include "sph.h"

class SceneSPH : public Scene
{
    Q_OBJECT
public:
    SceneSPH();
    virtual ~SceneSPH();

    virtual void initialize();
    virtual void reset();
    virtual void update(double dt);
    virtual void paint(const Camera& cam);

    virtual void mousePressed(const QMouseEvent* e, const Camera& cam);
    virtual void mouseMoved(const QMouseEvent* e, const Camera& cam);

    virtual void getSceneBounds(Vec3& bmin, Vec3& bmax) {
        bmin = Vec3(-110, -10, -110);
        bmax = Vec3( 110, 100,  110);
    }
    virtual unsigned int getNumParticles() { return system.getNumParticles(); }

    virtual QWidget* sceneUI() { return widget; }

public slots:
    void updateSimParams();

protected:
    WidgetSPH* widget = nullptr;

    QOpenGLShaderProgram* shader = nullptr;
    QOpenGLVertexArrayObject* vaoSphereL = nullptr;
    QOpenGLVertexArrayObject* vaoSphereH = nullptr;
    QOpenGLVertexArrayObject* vaoCube    = nullptr;
    QOpenGLVertexArrayObject* vaoFloor   = nullptr;
    unsigned int numFacesSphereL = 0, numFacesSphereH = 0;

    IntegratorSymplecticEuler integrator;
    ParticleSystem system;
    ForceConstAcceleration* fGravity;

    ColliderPlane colliderFloor, colliderWallNorth, colliderWallWest, colliderWallSouth, colliderWallEast;

    double kBounce, kFriction;
    double width, height, depth;
    //double emitRate;
    //double maxParticleLife;

    Hash* hash;
    SPH* sph;
    int mouseX, mouseY;
};

#endif // SCENESPH_H
