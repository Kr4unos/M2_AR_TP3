#ifndef BOX_H
#define BOX_H

#include "globject.h"
#include <random>

class Box : public GLObject
{
private:
    QVector3D size;
public:
    Box(QVector3D position, QVector3D size, float mass, btDiscreteDynamicsWorld* dynamicsWorld, btAlignedObjectArray<btCollisionShape*> &collisionShapes);

    void display(QOpenGLShaderProgram *program) override;
    void build() override;
    void buildBulletObject(btDiscreteDynamicsWorld* dynamicsWorld, btAlignedObjectArray<btCollisionShape*> &collisionShapes, btVector3 shape, btVector3 origin, btScalar mass, btVector3 linearVelocity);
    ~Box();
};

#endif // BOX_H
