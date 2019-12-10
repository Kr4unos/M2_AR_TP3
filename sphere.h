#ifndef SPHERE_H
#define SPHERE_H

#include "globject.h"

class Sphere : public GLObject
{
private:
    float rayon;
    int nbTriangles;
public:
    Sphere(QVector3D position, float rayon, float mass, btDiscreteDynamicsWorld* dynamicsWorld, btAlignedObjectArray<btCollisionShape*> &collisionShapes, QVector3D linearVelocity);

    void display(QOpenGLShaderProgram *program) override;
    void build() override;
    void buildBulletObject(btDiscreteDynamicsWorld* dynamicsWorld, btAlignedObjectArray<btCollisionShape*> &collisionShapes, btScalar shape, btVector3 origin, btScalar mass, btVector3 linearVelocity);
    ~Sphere();
};

#endif // SPHERE_H
