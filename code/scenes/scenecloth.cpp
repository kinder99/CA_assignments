#include "scenecloth.h"
#include "glutils.h"
#include "model.h"
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLBuffer>


SceneCloth::SceneCloth() {
    widget = new WidgetCloth();
    connect(widget, SIGNAL(updatedParameters()), this, SLOT(updateSimParams()));
    connect(widget, SIGNAL(freeAnchors()), this, SLOT(freeAnchors()));
}

SceneCloth::~SceneCloth() {
    if (widget)      delete widget;
    if (shaderPhong) delete shaderPhong;
    if (vaoSphereS)  delete vaoSphereS;
    if (vaoSphereL)  delete vaoSphereL;
    if (vaoCube)     delete vaoCube;
    if (vaoMesh)     delete vaoMesh;
    if (vboMesh)     delete vboMesh;
    if (iboMesh)     delete iboMesh;

    system.deleteParticles();
    if (fGravity)  delete fGravity;
    for (ForceSpring* f : springsStretch) delete f;
    for (ForceSpring* f : springsShear) delete f;
    for (ForceSpring* f : springsBend) delete f;
}

void SceneCloth::initialize() {

    // load shaders
    shaderPhong = glutils::loadShaderProgram(":/shaders/phong.vert", ":/shaders/phong.frag");
    shaderCloth = glutils::loadShaderProgram(":/shaders/cloth.vert", ":/shaders/cloth.geom", ":/shaders/cloth.frag");

    // create sphere VAOs
    Model sphere = Model::createIcosphere(3);
    vaoSphereL = glutils::createVAO(shaderPhong, &sphere, buffers);
    numFacesSphereL = sphere.numFaces();
    glutils::checkGLError();

    sphere = Model::createIcosphere(1);
    vaoSphereS = glutils::createVAO(shaderPhong, &sphere, buffers);
    numFacesSphereS = sphere.numFaces();
    glutils::checkGLError();

    // create cube VAO
    Model cube = Model::createCube();
    vaoCube = glutils::createVAO(shaderPhong, &cube, buffers);
    glutils::checkGLError();


    // create cloth mesh VAO
    vaoMesh = new QOpenGLVertexArrayObject();
    vaoMesh->create();
    vaoMesh->bind();
    vboMesh = new QOpenGLBuffer(QOpenGLBuffer::Type::VertexBuffer);
    vboMesh->create();
    vboMesh->bind();
    vboMesh->setUsagePattern(QOpenGLBuffer::UsagePattern::DynamicDraw);
    vboMesh->allocate(1000*1000*3*3*sizeof(float)); // sync with widget max particles
    shaderCloth->setAttributeBuffer("vertex", GL_FLOAT, 0, 3, 0);
    shaderCloth->enableAttributeArray("vertex");
    iboMesh = new QOpenGLBuffer(QOpenGLBuffer::Type::IndexBuffer);
    iboMesh->create();
    iboMesh->bind();
    iboMesh->setUsagePattern(QOpenGLBuffer::UsagePattern::StaticDraw);
    iboMesh->allocate(1000*1000*2*3*sizeof(unsigned int));
    vaoMesh->release();

    // create gravity force
    fGravity = new ForceConstAcceleration();
    system.addForce(fGravity);

    // TODO: in my solution setup, these were the colliders
    //colliderBall.setCenter(Vec3(40,-20,0));
    //colliderBall.setRadius(30);
    //colliderCube.setFromCenterSize(Vec3(-60,30,0), Vec3(60, 40, 60));
    //colliderWalls.setFromCenterSize(Vec3(0, 0, 0), Vec3(200, 200, 200));
}

void SceneCloth::reset()
{
    // we only update numParticles on resets
    updateSimParams();

    // reset particles
    system.deleteParticles();

    // reset forces
    system.clearForces();
    fGravity->clearInfluencedParticles();
    for (ForceSpring* f : springsStretch) delete f;
    springsStretch.clear();
    for (ForceSpring* f : springsShear) delete f;
    springsShear.clear();
    for (ForceSpring* f : springsBend) delete f;
    springsBend.clear();
    springs.clear();

    // cloth props
    Vec2 dims = widget->getDimensions();
    Vec2i dimParticles = widget->getNumParticles();
    numParticlesX = dimParticles.x();
    numParticlesY = dimParticles.y();
    clothWidth  = dims[0];
    clothHeight = dims[1];
    double edgeX = dims[0]/numParticlesX;
    double edgeY = dims[1]/numParticlesY;
    particleRadius = widget->getParticleRadius();

    // create particles
    numParticles = numParticlesX * numParticlesY;
    fixedParticle = std::vector<bool>(numParticles, false);

    for (int i = 0; i < numParticlesX; i++) {
        for (int j = 0; j < numParticlesY; j++) {

            int idx = i*numParticlesY + j;
            double tx;
            double ty;
            Vec3 pos;
            Particle* p;

            switch (widget->getFixed()) { //this is the worst switch i've ever written it's very funny
                case 0:
                    if(i==0){fixedParticle[idx] = true;}
                    else {fixedParticle[idx] = false;}

                    tx = i*edgeX - 0.5*clothWidth;
                    ty = j*edgeY - 0.5*clothHeight;
                    pos = Vec3(ty+edgeY, 70 - tx - edgeX, 0);

                    p = new Particle();
                    p->id = idx;
                    p->pos = pos;
                    p->prevPos = pos;
                    p->vel = Vec3(0,0,0);
                    p->mass = 1;
                    p->radius = particleRadius;
                    p->color = Vec3(235/255.0, 51/255.0, 36/255.0);

                    system.addParticle(p);
                    fGravity->addInfluencedParticle(p);
                    break;
                case 1:
                    if(i==0 && (j<3 || j>numParticlesY-4)){fixedParticle[idx] = true;}
                    else {fixedParticle[idx] = false;}

                    tx = i*edgeX - 0.5*clothWidth;
                    ty = j*edgeY - 0.5*clothHeight;
                    pos = Vec3(ty+edgeY, 70 - tx - edgeX, 0);

                    p = new Particle();
                    p->id = idx;
                    p->pos = pos;
                    p->prevPos = pos;
                    p->vel = Vec3(0,0,0);
                    p->mass = 1;
                    p->radius = particleRadius;
                    p->color = Vec3(235/255.0, 51/255.0, 36/255.0);

                    system.addParticle(p);
                    fGravity->addInfluencedParticle(p);
                    break;
                case 2:
                    if((i==0 || i==numParticlesX-1) && (j<3 || j>numParticlesY-4)){fixedParticle[idx] = true;}
                    else {fixedParticle[idx] = false;}

                    tx = i*edgeX - 0.5*clothWidth;
                    ty = j*edgeY - 0.5*clothHeight;
                    pos = Vec3(ty+edgeY, 70 - tx - edgeX, 0);

                    p = new Particle();
                    p->id = idx;
                    p->pos = pos;
                    p->prevPos = pos;
                    p->vel = Vec3(0,0,0);
                    p->mass = 1;
                    p->radius = particleRadius;
                    p->color = Vec3(235/255.0, 51/255.0, 36/255.0);

                    system.addParticle(p);
                    fGravity->addInfluencedParticle(p);
                    break;
                default:
                    fixedParticle[idx] = false;

                    tx = i*edgeX - 0.5*clothWidth;
                    ty = j*edgeY - 0.5*clothHeight;
                    pos = Vec3(ty+edgeY, 70 - tx - edgeX, 0);

                    p = new Particle();
                    p->id = idx;
                    p->pos = pos;
                    p->prevPos = pos;
                    p->vel = Vec3(0,0,0);
                    p->mass = 1;
                    p->radius = particleRadius;
                    p->color = Vec3(235/255.0, 51/255.0, 36/255.0);

                    system.addParticle(p);
                    fGravity->addInfluencedParticle(p);
                    break;
            }
        }
    }

    // forces: gravity
    system.addForce(fGravity);

    double ks = widget->getStiffness();
    double kd = widget->getDamping();

    for(int i = 0; i<numParticlesX-1; i++){
        for(int j = 0; j<numParticlesY-1; j++){
            int idx = i*numParticlesY + j;
            //int idy = j*numParticlesX + i;

            //streching springs
            ForceSpring* stretchRight = new ForceSpring();
            stretchRight->setParticlePair(system.getParticle(idx),system.getParticle(idx+1));
            stretchRight->setRestLength(edgeY);
            stretchRight->setSpringConstant(ks);
            stretchRight->setDampingCoeff(kd);
            system.addForce(stretchRight);
            springsStretch.push_back(stretchRight);
            springs.push_back(stretchRight);


            ForceSpring* stretchBottom = new ForceSpring();
            stretchBottom->setParticlePair(system.getParticle(idx),system.getParticle(idx+numParticlesY));
            stretchBottom->setRestLength(edgeX);
            stretchBottom->setSpringConstant(ks);
            stretchBottom->setDampingCoeff(kd);
            system.addForce(stretchBottom);
            springsStretch.push_back(stretchBottom);
            springs.push_back(stretchBottom);

            //shear springs
            if(idx-numParticlesY > 0){ //check if not on first row
                ForceSpring* shearTopRight = new ForceSpring();
                shearTopRight->setParticlePair(system.getParticle(idx-numParticlesY+1),system.getParticle(idx));
                shearTopRight->setRestLength(std::sqrt(edgeX*edgeX + edgeY*edgeY));
                shearTopRight->setSpringConstant(ks);
                shearTopRight->setDampingCoeff(kd);
                system.addForce(shearTopRight);
                springsShear.push_back(shearTopRight);
                springs.push_back(shearTopRight);
            }

            if(idx+numParticlesY < numParticlesX*numParticlesY){ //check if not on bottom row
                ForceSpring* shearBottomRight = new ForceSpring();
                shearBottomRight->setParticlePair(system.getParticle(idx+numParticlesY+1),system.getParticle(idx));
                shearBottomRight->setRestLength(std::sqrt(edgeX*edgeX + edgeY*edgeY));
                shearBottomRight->setSpringConstant(ks);
                shearBottomRight->setDampingCoeff(kd);
                system.addForce(shearBottomRight);
                springsShear.push_back(shearBottomRight);
                springs.push_back(shearBottomRight);
            }

            //bend springs
            if(j < numParticlesY-2){ //check if not on second to last column
                ForceSpring* bendRight = new ForceSpring();
                bendRight->setParticlePair(system.getParticle(idx),system.getParticle(idx+2));
                bendRight->setRestLength(2*edgeY);
                bendRight->setSpringConstant(ks);
                bendRight->setDampingCoeff(kd);
                system.addForce(bendRight);
                springsBend.push_back(bendRight);
                springs.push_back(bendRight);
            }

            if(i < numParticlesX-2){
                ForceSpring* bendBottom = new ForceSpring();
                bendBottom->setParticlePair(system.getParticle(idx),system.getParticle(idx+2*numParticlesY));
                bendBottom->setRestLength(2*edgeX);
                bendBottom->setSpringConstant(ks);
                bendBottom->setDampingCoeff(kd);
                system.addForce(bendBottom);
                springsBend.push_back(bendBottom);
                springs.push_back(bendBottom);
            }
        }
    }
    // Code for PROVOT layout
    updateSprings();

    // update index buffer
    iboMesh->bind();
    numMeshIndices = (numParticlesX - 1)*(numParticlesY - 1)*2*3;
    int* indices = new int[numMeshIndices];
    int idx = 0;
    for (int i = 0; i < numParticlesX-1; i++) {
        for (int j = 0; j < numParticlesY-1; j++) {
            indices[idx  ] = i*numParticlesY + j;
            indices[idx+1] = (i+1)*numParticlesY + j;
            indices[idx+2] = i*numParticlesY + j + 1;
            indices[idx+3] = i*numParticlesY + j + 1;
            indices[idx+4] = (i+1)*numParticlesY + j;
            indices[idx+5] = (i+1)*numParticlesY + j + 1;
            idx += 6;
        }
    }
    void* bufptr = iboMesh->mapRange(0, numMeshIndices*sizeof(int),
                                     QOpenGLBuffer::RangeInvalidateBuffer | QOpenGLBuffer::RangeWrite);
    memcpy(bufptr, (void*)(indices), numMeshIndices*sizeof(int));
    iboMesh->unmap();
    iboMesh->release();
    delete[] indices;
    glutils::checkGLError();
}


void SceneCloth::updateSprings()
{
    double ks = widget->getStiffness();
    double kd = widget->getDamping();

    // here I update all ks and kd parameters.
    // idea: if you want to enable/disable a spring type, you can set ks to 0 for these
    for (ForceSpring* f : springsStretch) {
        f->setDampingCoeff(kd);
        f->setSpringConstant(ks);
    }
    for (ForceSpring* f : springsShear) {
        f->setDampingCoeff(kd);
        f->setSpringConstant(ks);
    }
    for (ForceSpring* f : springsBend) {
        f->setDampingCoeff(kd);
        f->setSpringConstant(ks);
    }
}

void SceneCloth::relaxation(int n){
    for(int i = 0; i<n; i++){
        for(ForceSpring* spring : springs){
            Particle* p1 = spring->getParticle1();
            Particle* p2 = spring->getParticle2();
            Vec3 d = p2->pos - p1->pos;
            double dist = d.norm();
            double expected_dist = spring->getRestLength();

            if(dist <= expected_dist){
                continue;
            }
            else{
                double correction = dist - expected_dist;
                if(fixedParticle[p1->id] && fixedParticle[p2->id]){
                    continue;
                }
                else if(fixedParticle[p2->id]){
                    p1->pos += correction * d.normalized();
                }
                else if(fixedParticle[p1->id]){
                    p2->pos -= correction * d.normalized();
                }
                else{
                    p1->pos += correction/2 * d.normalized();
                    p2->pos -= correction/2 * d.normalized();
                }
            }
        }
    }
}

void SceneCloth::updateSimParams()
{
    double g = widget->getGravity();
    fGravity->setAcceleration(Vec3(0, -g, 0));

    updateSprings();

    for (Particle* p : system.getParticles()) {
        p->radius = widget->getParticleRadius();
    }

    showParticles = widget->showParticles();
}

void SceneCloth::freeAnchors()
{
    fixedParticle = std::vector<bool>(numParticles, false);
}

void SceneCloth::paint(const Camera& camera)
{
    QOpenGLFunctions* glFuncs = nullptr;
    glFuncs = QOpenGLContext::currentContext()->functions();

    shaderPhong->bind();
    shaderPhong->setUniformValue("normalSign", 1.0f);

    // camera matrices
    QMatrix4x4 camProj = camera.getPerspectiveMatrix();
    QMatrix4x4 camView = camera.getViewMatrix();
    shaderPhong->setUniformValue("ProjMatrix", camProj);
    shaderPhong->setUniformValue("ViewMatrix", camView);

    // lighting
    const int numLights = 1;
    const QVector3D lightPosWorld[numLights] = {QVector3D(80,80,80)};
    const QVector3D lightColor[numLights] = {QVector3D(1,1,1)};
    QVector3D lightPosCam[numLights];
    for (int i = 0; i < numLights; i++) {
        lightPosCam[i] = camView.map(lightPosWorld[i]);
    }
    shaderPhong->setUniformValue("numLights", numLights);
    shaderPhong->setUniformValueArray("lightPos", lightPosCam, numLights);
    shaderPhong->setUniformValueArray("lightColor", lightColor, numLights);

    // draw the particle spheres
    QMatrix4x4 modelMat;
    if (showParticles) {
        vaoSphereS->bind();
        shaderPhong->setUniformValue("matspec", 1.0f, 1.0f, 1.0f);
        shaderPhong->setUniformValue("matshin", 100.f);
        for (int i = 0; i < numParticles; i++) {
            const Particle* particle = system.getParticle(i);
            Vec3   p = particle->pos;
            Vec3   c = particle->color;
            if (fixedParticle[i])      c = Vec3(63/255.0, 72/255.0, 204/255.0);
            if (i == selectedParticle) c = Vec3(1.0,0.9,0);

            modelMat = QMatrix4x4();
            modelMat.translate(p[0], p[1], p[2]);
            modelMat.scale(particle->radius);
            shaderPhong->setUniformValue("ModelMatrix", modelMat);
            shaderPhong->setUniformValue("matdiff", GLfloat(c[0]), GLfloat(c[1]), GLfloat(c[2]));
            glFuncs->glDrawElements(GL_TRIANGLES, 3*numFacesSphereS, GL_UNSIGNED_INT, 0);
        }
    }

    // TODO: draw colliders and walls

    shaderPhong->release();


    // update cloth mesh VBO coords
    vboMesh->bind();
    float* pos = new float[3*numParticles];
    for (int i = 0; i < numParticles; i++) {
        pos[3*i  ] = system.getParticle(i)->pos.x();
        pos[3*i+1] = system.getParticle(i)->pos.y();
        pos[3*i+2] = system.getParticle(i)->pos.z();
    }
    void* bufptr = vboMesh->mapRange(0, 3*numParticles*sizeof(float),
                       QOpenGLBuffer::RangeInvalidateBuffer | QOpenGLBuffer::RangeWrite);
    memcpy(bufptr, (void*)(pos), 3*numParticles*sizeof(float));
    vboMesh->unmap();
    vboMesh->release();
    delete[] pos;

    // draw mesh
    shaderCloth->bind();
    shaderCloth->setUniformValue("ProjMatrix", camProj);
    shaderCloth->setUniformValue("ViewMatrix", camView);
    shaderCloth->setUniformValue("NormalMatrix", camView.normalMatrix());
    shaderCloth->setUniformValue("matdiffFront", 0.7f, 0.0f, 0.0f);
    shaderCloth->setUniformValue("matspecFront", 1.0f, 1.0f, 1.0f);
    shaderCloth->setUniformValue("matshinFront", 100.0f);
    shaderCloth->setUniformValue("matdiffBack", 0.7f, 0.3f, 0.0f);
    shaderCloth->setUniformValue("matspecBack", 0.0f, 0.0f, 0.0f);
    shaderCloth->setUniformValue("matshinBack", 0.0f);
    shaderCloth->setUniformValue("numLights", numLights);
    shaderCloth->setUniformValueArray("lightPos", lightPosCam, numLights);
    shaderCloth->setUniformValueArray("lightColor", lightColor, numLights);
    vaoMesh->bind();
    glFuncs->glDrawElements(GL_TRIANGLES, numMeshIndices, GL_UNSIGNED_INT, 0);
    vaoMesh->release();
    shaderCloth->release();

    glutils::checkGLError();
}


void SceneCloth::update(double dt)
{
    // fixed particles: no velocity, no force acting
    for (int i = 0; i < numParticles; i++) {
        if (fixedParticle[i]) {
            Particle* p = system.getParticle(i);
            p->vel = Vec3(0,0,0);
            p->force = Vec3(0,0,0);
            p->pos = p->prevPos;
        }
    }

    // integration step
    Vecd ppos = system.getPositions();
    integrator.step(system, dt);
    system.setPreviousPositions(ppos);

    for (int i = 0; i < numParticles; i++) {
        if (fixedParticle[i]) {
            Particle* p = system.getParticle(i);
            p->vel = Vec3(0,0,0);
            p->force = Vec3(0,0,0);
            p->pos = p->prevPos;
        }
    }

    // user interaction
    if (selectedParticle >= 0) {
        Particle* p = system.getParticle(selectedParticle);
        p->pos = cursorWorldPos;
        p->vel = Vec3(0,0,0);

        // TODO: test and resolve for collisions during user movement
    }

    relaxation(5);

    // collisions
    for (Particle* p : system.getParticles()) {
        // TODO: test and resolve collisions
    }

    // needed after we have done collisions and relaxation, since spring forces depend on p and v
    system.updateForces();
}


void SceneCloth::mousePressed(const QMouseEvent* e, const Camera& cam)
{
    grabX = e->pos().x();
    grabY = e->pos().y();

    if (!(e->modifiers() & Qt::ControlModifier)) {

        Vec3 rayDir = cam.getRayDir(grabX, grabY);
        Vec3 origin = cam.getPos();

        selectedParticle = -1;
        double min_dist = std::numeric_limits<double>::max();
        for (int i = 0; i < numParticles; i++) {
            Vec3 A = origin - system.getParticle(i)->pos;
            double dist = (A.cross(rayDir)).norm()/rayDir.norm();
            if(dist < min_dist){
                min_dist = dist;
                selectedParticle = i;
            }
        }

        if (selectedParticle >= 0) {
            cursorWorldPos = system.getParticle(selectedParticle)->pos;
        }
    }
}

void SceneCloth::mouseMoved(const QMouseEvent* e, const Camera& cam)
{
    int dx = e->pos().x() - grabX;
    int dy = e->pos().y() - grabY;
    grabX = e->pos().x();
    grabY = e->pos().y();

    if (e->modifiers() & Qt::ControlModifier) {
    }
    else if (e->modifiers() & Qt::ShiftModifier) {

    }
    else {
        if (selectedParticle >= 0) {
            double d = -(system.getParticle(selectedParticle)->pos - cam.getPos()).dot(cam.zAxis());
            Vec3 disp = cam.worldSpaceDisplacement(dx, -dy, d);
            cursorWorldPos += disp;
        }
    }
}

void SceneCloth::mouseReleased(const QMouseEvent*, const Camera&)
{
    selectedParticle = -1;
}

void SceneCloth::keyPressed(const QKeyEvent* e, const Camera&)
{
    if (selectedParticle >= 0 && e->key() == Qt::Key_F) {
        fixedParticle[selectedParticle] = true;
        Particle* p = system.getParticle(selectedParticle);
        p->prevPos = p->pos;
        p->vel = Vec3(0,0,0);
        p->force = Vec3(0,0,0);
    }
}
