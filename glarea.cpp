// Basé sur :
// CC-BY Edouard.Thiel@univ-amu.fr - 22/01/2019

#include "glarea.h"

GLArea::GLArea(QWidget *parent) : QOpenGLWidget(parent)
{
    QSurfaceFormat sf;
    sf.setDepthBufferSize(24);
    sf.setSamples(16);
    setFormat(sf);

    setEnabled(true);                   // événements clavier et souris
    setFocusPolicy(Qt::StrongFocus);    // accepte focus
    setFocus();                         // donne le focus

    timer = new QTimer(this);
    timer->setInterval(20);           // msec
    connect (timer, SIGNAL(timeout()), this, SLOT(onTimeout()));
    timer->start();
    elapsedTimer.start();

    // Initialisation des composants de bullet
    collisionConfiguration = new btDefaultCollisionConfiguration();
    dispatcher = new btCollisionDispatcher(collisionConfiguration);
    overlappingPairCache = new btDbvtBroadphase();
    solver = new btSequentialImpulseConstraintSolver;
    dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);
    dynamicsWorld->setGravity(btVector3(0, -10, 0));
}
GLArea::~GLArea()
{
    delete timer;
    makeCurrent();
    tearGLObjects();
    doneCurrent();

    // Delete all bullet components
    for (int i = dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
    {
        btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[i];
        btRigidBody* body = btRigidBody::upcast(obj);
        if (body && body->getMotionState())
        {
            delete body->getMotionState();
        }
        dynamicsWorld->removeCollisionObject(obj);
        delete obj;
    }
    for (int j = 0; j < collisionShapes.size(); j++)
    {
        btCollisionShape* shape = collisionShapes[j];
        collisionShapes[j] = 0;
        delete shape;
    }
    delete dynamicsWorld;
    delete solver;
    delete overlappingPairCache;
    delete dispatcher;
    delete collisionConfiguration;
    collisionShapes.clear();
}

void GLArea::initializeGL()
{
    initializeOpenGLFunctions();
    glClearColor(0.5f,0.5f,1.0f,1.0f);
    glEnable(GL_DEPTH_TEST);

    makeGLObjects();

    program_sol = new QOpenGLShaderProgram(this);
    program_sol->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/simple.vsh");
    program_sol->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/simple.fsh");
    if (! program_sol->link()) {
        qWarning("Failed to compile and link shader program:");
        qWarning() << program_sol->log();
    }
    program_sol->setUniformValue("texture", 0);

    program_sphere = new QOpenGLShaderProgram(this);
    program_sphere->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/sphere.vsh");
    program_sphere->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/sphere.fsh");
    if (! program_sphere->link()) {
        qWarning("Failed to compile and link shader program:");
        qWarning() << program_sphere->log();
    }
    program_sphere->setUniformValue("texture", 0);
}


void GLArea::makeGLObjects()
{
    GLfloat vertices_sol[] = {
       -groundBox.x(), groundBox.y(),-groundBox.z(),
       -groundBox.x(), groundBox.y(), groundBox.z(),
        groundBox.x(), groundBox.y(), groundBox.z(),
        groundBox.x(), groundBox.y(), groundBox.z(),
        groundBox.x(), groundBox.y(),-groundBox.z(),
       -groundBox.x(), groundBox.y(),-groundBox.z()
    };

    GLfloat texCoords_sol[] = {
            0.0f, 0.0f,
            0.0f, 1.0f,
            1.0f, 1.0f,
            1.0f, 1.0f,
            1.0f, 0.0f,
            0.0f, 0.0f
    };

    QVector<GLfloat> vertData_sol;
    for (int i = 0; i < 6; ++i) {
        // coordonnées sommets
        for (int j = 0; j < 3; j++)
            vertData_sol.append(vertices_sol[i*3+j]);
        // coordonnées texture
        for (int j = 0; j < 2; j++)
            vertData_sol.append(texCoords_sol[i*2+j]);
    }

    // Ground
    btCollisionShape* groundShape = new btBoxShape(btVector3(groundBox.x(), groundBox.y(), groundBox.z()));
    collisionShapes.push_back(groundShape);
    btTransform groundTransform;
    groundTransform.setIdentity();
    groundTransform.setOrigin(btVector3(0, 0, 0));
    btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(0., myMotionState, groundShape, btVector3(0, 0, 0));
    btRigidBody* body = new btRigidBody(rbInfo);
    dynamicsWorld->addRigidBody(body);

    vbo_sol.create();
    vbo_sol.bind();
    vbo_sol.allocate(vertData_sol.constData(), vertData_sol.count() * int(sizeof(GLfloat)));

    // Q1
    //objects.push_back(new Sphere(QVector3D(20, 10, 0), 1, 5, dynamicsWorld, collisionShapes, QVector3D(0, 0, 0)));
    //objects.push_back(new Sphere(QVector3D(22, 14, 0), 1, 10., dynamicsWorld, collisionShapes, QVector3D(0, 0, 0)));
    //objects.push_back(new Sphere(QVector3D(24, 16, 0), 1, 20., dynamicsWorld, collisionShapes, QVector3D(0, 0, 0)));

    // Q2
    //objects.push_back(new Box(QVector3D(5, 10, 0), QVector3D(1, 1, 1), 1, dynamicsWorld, collisionShapes));
    //objects.push_back(new Box(QVector3D(10, 14, 0), QVector3D(4, 2, 3), 8, dynamicsWorld, collisionShapes));
    //objects.push_back(new Box(QVector3D(15, 16, 0), QVector3D(2, 8, 5), 9, dynamicsWorld, collisionShapes));

    /*
     * Q3 */
    for(unsigned int i = 0; i < 10; i++){
        for(unsigned int j = 0; j < 30; j++){
            objects.push_back(new Box(QVector3D(0+i, groundBox.y()+j, 5), QVector3D(1, 0.5f, 1), 10, dynamicsWorld, collisionShapes));
        }
    }

    objects.push_back(new Sphere(QVector3D(0, 15, 10), 1, 20., dynamicsWorld, collisionShapes, QVector3D(0, 10, -100)));

    // Création de textures
    QImage image_sol(":/textures/ground.jpg");
    if (image_sol.isNull())
        qDebug() << "load image ground.jpg failed";
    textures[0] = new QOpenGLTexture(image_sol);
}


void GLArea::tearGLObjects()
{
    vbo_sol.destroy();
    for (int i = 0; i < 1; i++)
        delete textures[i];
}


void GLArea::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);
    windowRatio = float(w) / h;
}


void GLArea::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Matrice de projection
    QMatrix4x4 projectionMatrix;
    projectionMatrix.perspective(45.0f, windowRatio, 1.0f, 1000.0f);

    // Matrice de vue (caméra)
    QMatrix4x4 viewMatrix;
    viewMatrix.translate(xPos, yPos, zPos);
    viewMatrix.rotate(xRot, 1, 0, 0);
    viewMatrix.rotate(yRot, 0, 1, 0);
    viewMatrix.rotate(zRot, 0, 0, 1);

    // Affichage du sol
    vbo_sol.bind();
    program_sol->bind(); // active le shader program du sol

    QMatrix4x4 modelMatrixSol;
    modelMatrixSol.translate(0.0f, 0.0f, 0.0f);
    program_sol->setUniformValue("projectionMatrix", projectionMatrix);
    program_sol->setUniformValue("viewMatrix", viewMatrix);
    program_sol->setUniformValue("modelMatrix", modelMatrixSol);

    program_sol->setAttributeBuffer("in_position", GL_FLOAT, 0, 3, 5 * sizeof(GLfloat));
    program_sol->setAttributeBuffer("in_uv", GL_FLOAT, 3 * sizeof(GLfloat), 2, 5 * sizeof(GLfloat));
    program_sol->enableAttributeArray("in_position");
    program_sol->enableAttributeArray("in_uv");

    textures[0]->bind();
    glDrawArrays(GL_TRIANGLES, 0, 6);
    textures[0]->release();

    program_sol->disableAttributeArray("in_position");
    program_sol->disableAttributeArray("colAttr");
    program_sol->release();

    program_sphere->bind();
    program_sphere->setUniformValue("projectionMatrix", projectionMatrix);
    program_sphere->setUniformValue("viewMatrix", viewMatrix);

    for(unsigned int i = 0; i < objects.size(); i++){
        objects[i]->display(program_sphere);
    }

    program_sphere->release();
}


void GLArea::keyPressEvent(QKeyEvent *ev)
{
    float da = 0.1f;

    switch(ev->key()) {
    case Qt::Key_A :
        xRot -= da;
        break;

    case Qt::Key_Q :
        xRot += da;
        break;

    case Qt::Key_Z :
        yRot -= da;
        break;

    case Qt::Key_S :
        yRot += da;
        break;

    case Qt::Key_E :
        zRot -= da;
        break;

    case Qt::Key_D :
        zRot += da;
        break;
        }
    update();
}


void GLArea::keyReleaseEvent(QKeyEvent *ev)
{
    qDebug() << __FUNCTION__ << ev->text();
}


void GLArea::mousePressEvent(QMouseEvent *ev)
{
    lastPos = ev->pos();
}


void GLArea::mouseReleaseEvent(QMouseEvent *ev)
{
    qDebug() << __FUNCTION__ << ev->x() << ev->y() << ev->button();
}


void GLArea::mouseMoveEvent(QMouseEvent *ev)
{
    int dx = ev->x() - lastPos.x();
    int dy = ev->y() - lastPos.y();

    if (ev->buttons() & Qt::LeftButton) {
        xRot += dy;
        yRot += dx;
        update();
    } else if (ev->buttons() & Qt::RightButton) {
        xPos += dx/10.0f;
        yPos -= dy/10.0f;
        update();
    } else if (ev->buttons() & Qt::MidButton) {
        xPos += dx/10.0f;
        zPos += dy;
        update();
    }

    lastPos = ev->pos();
}


void GLArea::onTimeout()
{
    static qint64 old_chrono = elapsedTimer.elapsed(); // static : initialisation la première fois et conserve la dernière valeur
    qint64 chrono = elapsedTimer.elapsed();
    dt = (chrono - old_chrono) / 1000.0f;

    dynamicsWorld->stepSimulation(dt);

    for (int j = dynamicsWorld->getNumCollisionObjects() - 1; j >= 0; j--)
    {
        btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[j];
        btRigidBody* body = btRigidBody::upcast(obj);
        btTransform trans;
        if (body && body->getMotionState())
        {
            body->getMotionState()->getWorldTransform(trans);
        }
        else
        {
            trans = obj->getWorldTransform();
        }
        for(unsigned int k = 0; k < objects.size(); k++){
            if(j == objects[k]->getBulletObjectIndex()) {
                //printf("world pos object %d = %f,%f,%f\n", j, float(trans.getOrigin().getX()), float(trans.getOrigin().getY()), float(trans.getOrigin().getZ()));
                objects[k]->setPosition(QVector3D(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ()));
                break;
            }
        }
    }

    old_chrono = chrono;
    update();
}
