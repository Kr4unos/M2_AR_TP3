#include "box.h"
#include "bullet.h"

Box::Box(QVector3D position, QVector3D size, float mass, btDiscreteDynamicsWorld* dynamicsWorld, btAlignedObjectArray<btCollisionShape*> &collisionShapes)
{
    this->position = position;
    this->size = size;
    this->buildBulletObject(dynamicsWorld, collisionShapes, btVector3(size.x(), size.y(), size.z()), btVector3(position.x(), position.y(), position.z()), btScalar(mass), btVector3(0, 0, 0));
    this->build();
}

void Box::display(QOpenGLShaderProgram *program)
{
     vbo.bind();

     program->setAttributeBuffer("in_position", GL_FLOAT, 0, 3, 6 * sizeof(GLfloat));
     program->setAttributeBuffer("in_uv", GL_FLOAT, 3 * sizeof(GLfloat), 3, 6 * sizeof(GLfloat));
     program->enableAttributeArray("in_position");
     program->enableAttributeArray("in_uv");

     QMatrix4x4 modelMatrix;
     modelMatrix.setToIdentity();
     modelMatrix.translate(position);
     program->setUniformValue("modelMatrix", modelMatrix);

     glDrawArrays(GL_TRIANGLES, 0, nbTriangles*3);

     program->disableAttributeArray("modelMatrix");
     program->disableAttributeArray("in_position");
     program->disableAttributeArray("in_uv");

     vbo.release();
}

void Box::build(){
    GLfloat vertices_box[] = {

        -size.x(), -size.y(), -size.z(),
        -size.x(), -size.y(), size.z(),
        -size.x(), size.y(), size.z(),

        size.x(), size.y(), -size.z(),
        -size.x(), -size.y(), -size.z(),
        -size.x(), size.y(), -size.z(),

        size.x(),-size.y(), size.z(),
        -size.x(),-size.y(), -size.z(),
        size.x(),-size.y(), -size.z(),

        size.x(), size.y(),-size.z(),
        size.x(), -size.y(),-size.z(),
        -size.x(), -size.y(),-size.z(),

        -size.x(), -size.y(),-size.z(),
        -size.x(), size.y(), size.z(),
        -size.x(), size.y(),-size.z(),

        size.x(), -size.y(), size.z(),
        -size.x(), -size.y(), size.z(),
        -size.x(), -size.y(),-size.z(),

        -size.x(), size.y(), size.z(),
        -size.x(), -size.y(), size.z(),
        size.x(), -size.y(), size.z(),

        size.x(), size.y(), size.z(),
        size.x(), -size.y(),-size.z(),
        size.x(), size.y(),-size.z(),

        size.x(), -size.y(),-size.z(),
        size.x(), size.y(), size.z(),
        size.x(), -size.y(), size.z(),

        size.x(), size.y(), size.z(),
        size.x(), size.y(),-size.z(),
        -size.x(), size.y(),-size.z(),

        size.x(), size.y(), size.z(),
        -size.x(), size.y(),-size.z(),
        -size.x(), size.y(), size.z(),

        size.x(), size.y(), size.z(),
        -size.x(), size.y(), size.z(),
        size.x(), -size.y(), size.z()

    };

    nbTriangles = 12;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);

    float r = dis(gen);
    float g = dis(gen);
    float b = dis(gen);

    GLfloat rgb_box[] = {
            r, g, b
    };

    QVector<GLfloat> vertData_box;
    for (int i = 0; i < nbTriangles*9; ++i) {
        // coordonnées sommets
        for (int j = 0; j < 3; j++)
            vertData_box.append(vertices_box[i*3+j]);
        // coordonnées texture
        for (int j = 0; j < 3; j++)
            vertData_box.append(rgb_box[j]);
    }

    vbo.create();
    vbo.bind();
    vbo.allocate(vertData_box.constData(), vertData_box.count() * int(sizeof(GLfloat)));
}

void Box::buildBulletObject(btDiscreteDynamicsWorld* dynamicsWorld, btAlignedObjectArray<btCollisionShape*> &collisionShapes, btVector3 shape, btVector3 origin, btScalar mass, btVector3 linearVelocity)
{
    btCollisionShape* boxShape = new btBoxShape(shape);
    collisionShapes.push_back(boxShape);

    btTransform groundTransform;
    groundTransform.setIdentity();
    groundTransform.setOrigin(origin);

    btVector3 localInertia(0, 0, 0);
    if (mass != 0.0f) boxShape->calculateLocalInertia(mass, localInertia);

    btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, boxShape, localInertia);
    btRigidBody* body = new btRigidBody(rbInfo);
    body->setLinearVelocity(linearVelocity);
    dynamicsWorld->addRigidBody(body);

    this->bulletObjectIndex = dynamicsWorld->getCollisionObjectArray().size() - 1;
}

Box::~Box()
{
    vbo.destroy();
}
