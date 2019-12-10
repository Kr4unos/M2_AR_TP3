// Basé sur :
// CC-BY Edouard.Thiel@univ-amu.fr - 22/01/2019

#ifndef GLAREA_H
#define GLAREA_H

#include <QKeyEvent>
#include <QTimer>
#include <QElapsedTimer>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>
#include <QDebug>
#include <QSurfaceFormat>
#include <QMatrix4x4>
#include <stdio.h>

#include "btBulletDynamicsCommon.h"
#include "globject.h"
#include "sphere.h"
#include "box.h"

class GLArea : public QOpenGLWidget,
               protected QOpenGLFunctions
{
    Q_OBJECT

public:
    explicit GLArea(QWidget *parent = nullptr);
    ~GLArea() override;

protected slots:
    void onTimeout();

protected:
    void initializeGL() override;
    void doProjection();
    void resizeGL(int w, int h) override;
    void paintGL() override;
    void keyPressEvent(QKeyEvent *ev) override;
    void keyReleaseEvent(QKeyEvent *ev) override;
    void mousePressEvent(QMouseEvent *ev) override;
    void mouseReleaseEvent(QMouseEvent *ev) override;
    void mouseMoveEvent(QMouseEvent *ev) override;

private:
    float xRot=20.0f, yRot=0.0f, zRot=0.0f;
    float xPos=0.0f,  yPos=0.0f, zPos=-50.0f;
    QTimer *timer = nullptr;
    QElapsedTimer elapsedTimer;
    float dt = 0;
    float windowRatio = 1.0f;
    QPoint lastPos;

    QOpenGLShaderProgram *program_sol;
    QOpenGLShaderProgram *program_sphere;
    QOpenGLBuffer vbo_sol;

    std::vector<GLObject*> objects;
    QVector3D groundBox = QVector3D(50, 0, 50);

    btDiscreteDynamicsWorld* dynamicsWorld;
    btBroadphaseInterface* overlappingPairCache;
    btCollisionDispatcher* dispatcher;
    btDefaultCollisionConfiguration* collisionConfiguration;
    btSequentialImpulseConstraintSolver* solver;
    btAlignedObjectArray<btCollisionShape*> collisionShapes;

    QOpenGLTexture *textures[2];

    void makeGLObjects();
    void tearGLObjects();
};

#endif // GLAREA_H
