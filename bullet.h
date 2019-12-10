#ifndef BULLET_H
#define BULLET_H


#include <vector>

class GLObject;

class Bullet
{
        int generateBox(btVector3 shape, btVector3 origin, btScalar mass);
        int generateSphere(btScalar shape, btVector3 origin, btScalar mass, btVector3 linearVelocity);
        void simulate(float dt, std::vector<GLObject*> objects);
};

#endif // BULLET_H
