#include "scenesph.h"
#include "glutils.h"
#include "model.h"
#include <QOpenGLFunctions_3_3_Core>
#include <random>

SceneSPH::SceneSPH() {
    widget = new WidgetSPH();
    connect(widget, SIGNAL(updatedParameters()), this, SLOT(updateSimParams()));
}


SceneSPH::~SceneSPH() {
    if (widget)     delete widget;
    if (shader)     delete shader;
    if (vaoFloor)   delete vaoFloor;
    if (vaoSphereH) delete vaoSphereH;
    if (vaoSphereL) delete vaoSphereL;
    if (vaoCube)    delete vaoCube;
    if (fGravity)   delete fGravity;
}


void SceneSPH::initialize() {
    // load shader
    shader = glutils::loadShaderProgram(":/shaders/phong.vert", ":/shaders/phong.frag");

    // create floor VAO
    Model quad = Model::createQuad();
    vaoFloor = glutils::createVAO(shader, &quad, buffers);
    glutils::checkGLError();

    // create particle VAOs
    Model sphereLowres = Model::createIcosphere(1);
    vaoSphereL = glutils::createVAO(shader, &sphereLowres, buffers);
    numFacesSphereL = sphereLowres.numFaces();
    glutils::checkGLError();

    // create forces
    fGravity = new ForceConstAcceleration();
    system.addForce(fGravity);

    // scene description
    colliderFloor.setPlane(Vec3(0, 1, 0), 0);
    colliderWallNorth.setPlane(Vec3(1,0,0),0);
    colliderWallSouth.setPlane(Vec3(-1,0,0),0);
    colliderWallEast.setPlane(Vec3(0,0,1),0);
    colliderWallWest.setPlane(Vec3(0,0,-1),0);

    hash = new Hash(2, widget->getWidth() * widget->getHeight() * widget->getDepth(), &system);
    sph = new SPH(system, width, height, depth);
}

void SceneSPH::reset()
{
    // update values from UI
    updateSimParams();

    //initialize randomness
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> distr(-0.5,0.5);

    // erase all particles
    fGravity->clearInfluencedParticles();
    system.deleteParticles();
    //deadParticles.clear();
    double tx;
    double ty;
    double tz;
    Vec3 pos;
    Particle* p;
    for(int i = 0; i<width; i++){
        for(int j = 0; j<height; j++){
            for(int k = 0; k<depth; k++){
                int idx = width*i + height*j + k;

                tx = i*widget->getSizeX() - width;
                ty = j*widget->getSizeY();
                tz = k*widget->getSizeZ() - depth;

                pos = Vec3(tx + distr(gen), ty+5, tz + distr(gen));

                p = new Particle();
                p->id = idx;
                p->pos = pos;
                p->prevPos = pos;
                p->vel = Vec3(0,0,0);
                p->mass = 1;
                p->radius = 1.0;
                p->color = Vec3(25/255.0, 151/255.0, 136/255.0);

                system.addParticle(p);
                fGravity->addInfluencedParticle(p);
            }
        }
    }

    hash->setSpacing(1.8*(p->radius));
    hash->create((int) system.getNumParticles());

    system.addForce(sph);
    for(Particle* p: system.getParticles()){
        sph->addInfluencedParticle(p);
    }
}

void SceneSPH::updateSimParams()
{
    // get gravity from UI and update force
    double g = widget->getGravity();
    fGravity->setAcceleration(Vec3(0, -g, 0));

    // get other relevant UI values and update simulation params
    kBounce = 0.5;
    kFriction = 0;
    width = widget->getWidth();
    height = widget->getHeight();
    depth = widget->getDepth();
    //maxParticleLife = widget->getLifetime();
    //emitRate = widget->getEmitRate();

}

void SceneSPH::paint(const Camera& camera) {

    QOpenGLFunctions* glFuncs = nullptr;
    glFuncs = QOpenGLContext::currentContext()->functions();

    shader->bind();

    // camera matrices
    QMatrix4x4 camProj = camera.getPerspectiveMatrix();
    QMatrix4x4 camView = camera.getViewMatrix();
    shader->setUniformValue("ProjMatrix", camProj);
    shader->setUniformValue("ViewMatrix", camView);

    // lighting
    const int numLights = 1;
    const QVector3D lightPosWorld[numLights] = {QVector3D(100,500,100)};
    const QVector3D lightColor[numLights] = {QVector3D(1,1,1)};
    QVector3D lightPosCam[numLights];
    for (int i = 0; i < numLights; i++) {
        lightPosCam[i] = camView.map(lightPosWorld[i]);  // map = matrix * vector
    }
    shader->setUniformValue("numLights", numLights);
    shader->setUniformValueArray("lightPos", lightPosCam, numLights);
    shader->setUniformValueArray("lightColor", lightColor, numLights);

    // draw floor
    vaoFloor->bind();
    QMatrix4x4 modelMat;
    modelMat.scale(20, 1, 20);
    shader->setUniformValue("ModelMatrix", modelMat);
    shader->setUniformValue("matdiff", 0.8f, 0.8f, 0.8f);
    shader->setUniformValue("matspec", 0.0f, 0.0f, 0.0f);
    shader->setUniformValue("matshin", 0.0f);
    glFuncs->glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

    //draw walls
    //north
    modelMat = QMatrix4x4();
    modelMat.rotate(90, QVector3D(1,0,0));
    modelMat.translate(0, -20, 0);
    modelMat.scale(20, 1, 20);
    modelMat.translate(0, 0, -1);
    shader->setUniformValue("ModelMatrix", modelMat);
    glFuncs->glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);


    //south
    modelMat = QMatrix4x4();
    modelMat.rotate(90, QVector3D(1,0,0));
    modelMat.translate(0, 20, 0);
    modelMat.scale(20, 1, 20);
    modelMat.translate(0, 0, -1);
    shader->setUniformValue("ModelMatrix", modelMat);
    glFuncs->glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);


    //west
    modelMat = QMatrix4x4();
    modelMat.rotate(90, QVector3D(0,0,1));
    modelMat.translate(0, 20, 0);
    modelMat.scale(20, 1, 20);
    modelMat.translate(1, 0, 0);
    shader->setUniformValue("ModelMatrix", modelMat);
    glFuncs->glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);


    //east
    modelMat = QMatrix4x4();
    modelMat.rotate(90, QVector3D(0,0,1));
    modelMat.translate(0, -20, 0);
    modelMat.scale(20, 1, 20);
    modelMat.translate(1, 0, 0);
    shader->setUniformValue("ModelMatrix", modelMat);
    glFuncs->glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

    // draw ramp
    // modelMat = QMatrix4x4();
    // modelMat.rotate(30.0, QVector3D(1, 0, 0));
    // modelMat.translate(0, -6, 0);
    // modelMat.scale(100, 1, 100);
    // modelMat.translate(0, 0, -1);
    // shader->setUniformValue("ModelMatrix", modelMat);
    // glFuncs->glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

    // draw the particles
    vaoSphereL->bind();
    for (const Particle* particle : system.getParticles()) {
        Vec3   p = particle->pos;
        Vec3   c = particle->color;
        double r = particle->radius;

        modelMat = QMatrix4x4();
        modelMat.translate(p[0], p[1], p[2]);
        modelMat.scale(r);
        shader->setUniformValue("ModelMatrix", modelMat);

        shader->setUniformValue("matdiff", GLfloat(c[0]), GLfloat(c[1]), GLfloat(c[2]));
        shader->setUniformValue("matspec", 1.0f, 1.0f, 1.0f);
        shader->setUniformValue("matshin", 100.f);

        glFuncs->glDrawElements(GL_TRIANGLES, 3*numFacesSphereL, GL_UNSIGNED_INT, 0);
    }
}

void SceneSPH::update(double dt) {
    // integration step
    Vecd ppos = system.getPositions();
    integrator.step(system, dt);
    system.setPreviousPositions(ppos);

    Collision colInfo;
    for (Particle* p : system.getParticles()) {
        p->color = Vec3(25/255.0, 151/255.0, 136/255.0);
        if (colliderFloor.testCollision(p, colInfo)) {
            colliderFloor.resolveCollision(p, colInfo, kBounce, kFriction);
        }
        if (colliderWallNorth.testCollision(p, colInfo)) {
            colliderWallNorth.resolveCollision(p, colInfo, kBounce, kFriction);
        }
        if (colliderWallWest.testCollision(p, colInfo)) {
            colliderWallWest.resolveCollision(p, colInfo, kBounce, kFriction);
        }
        if (colliderWallSouth.testCollision(p, colInfo)) {
            colliderWallSouth.resolveCollision(p, colInfo, kBounce, kFriction);
        }
        if (colliderWallEast.testCollision(p, colInfo)) {
            colliderWallEast.resolveCollision(p, colInfo, kBounce, kFriction);
        }
    }
}

void SceneSPH::mousePressed(const QMouseEvent* e, const Camera&)
{
    mouseX = e->pos().x();
    mouseY = e->pos().y();
}

void SceneSPH::mouseMoved(const QMouseEvent* e, const Camera& cam)
{
    int dx = e->pos().x() - mouseX;
    int dy = e->pos().y() - mouseY;
    mouseX = e->pos().x();
    mouseY = e->pos().y();

    //Vec3 disp = cam.worldSpaceDisplacement(dx, -dy, cam.getEyeDistance());

    // example
    // if (e->buttons() & Qt::RightButton) {
    //     if (!e->modifiers()) {
    //         // move fountain
    //         fountainPos += disp;
    //     }
    //     else if (e->modifiers() & Qt::ShiftModifier){
    //         // move box
    //         colliderBox.setFromCenterSize(
    //             colliderBox.getCenter() + disp,
    //             colliderBox.getSize());
    //     }
    // }
}
