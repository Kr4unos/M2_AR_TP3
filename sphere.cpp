#include "sphere.h"
#include "bullet.h"

Sphere::Sphere(QVector3D position, float rayon, float mass, btDiscreteDynamicsWorld* dynamicsWorld, btAlignedObjectArray<btCollisionShape*> &collisionShapes, QVector3D linearVelocity)
{
    this->position = position;
    this->rayon = rayon;
    this->buildBulletObject(dynamicsWorld, collisionShapes, btScalar(rayon), btVector3(position.x(), position.y(), position.z()), btScalar(mass), btVector3(linearVelocity.x(), linearVelocity.y(), linearVelocity.z()));
    this->build();
}

void Sphere::display(QOpenGLShaderProgram *program)
{
     vbo.bind();

     program->setAttributeBuffer("in_position", GL_FLOAT, 0, 3, 6 * sizeof(GLfloat));
     program->setAttributeBuffer("in_uv", GL_FLOAT, 3 * sizeof(GLfloat), 3, 6 * sizeof(GLfloat));
     program->enableAttributeArray("in_position");
     program->enableAttributeArray("in_uv");

     QMatrix4x4 modelMatrix;
     modelMatrix.setToIdentity();
     modelMatrix.translate(position);
     modelMatrix.scale(rayon);
     program->setUniformValue("modelMatrix", modelMatrix);

     glDrawArrays(GL_TRIANGLES, 0, nbTriangles*3);

     program->disableAttributeArray("modelMatrix");
     program->disableAttributeArray("in_position");
     program->disableAttributeArray("in_uv");

     vbo.release();

}

void Sphere::build(){

    unsigned stack_count = 10;
    unsigned slice_count = 10;
    nbTriangles = (stack_count-1)*(slice_count-1)*2;

    GLfloat* vertices_sphere = new GLfloat[nbTriangles*3*3];
    std::vector<QVector3D> pts;

    for (unsigned j = 0 ; j < stack_count ; ++j)
    {
        for (unsigned i = 0 ; i < slice_count ; ++i)
        {
            float x, y, z;
            float theta  = -M_PI+(2*M_PI*j/(stack_count-1));
            float lambda = -M_PI/2+(M_PI*i/(slice_count-1)) ;
            x = 1 * cos(theta)*cos(lambda);
            y = 1 * sin(theta)*cos(lambda);
            z = 1 * sin(lambda);

            pts.push_back(QVector3D(x,y,z));
        }
    }

    for (unsigned j = 0 ; j < stack_count-1 ; ++j)
    {
        for (unsigned i = 0 ; i < slice_count-1 ; ++i)
        {
            unsigned c = (slice_count-1)*18;

            vertices_sphere[j*c+i*18  ] = pts[j*slice_count+i][0];
            vertices_sphere[j*c+i*18+1] = pts[j*slice_count+i][1];
            vertices_sphere[j*c+i*18+2] = pts[j*slice_count+i][2];

            vertices_sphere[j*c+i*18+3] = pts[j*slice_count+i+1][0];
            vertices_sphere[j*c+i*18+4] = pts[j*slice_count+i+1][1];
            vertices_sphere[j*c+i*18+5] = pts[j*slice_count+i+1][2];

            vertices_sphere[j*c+i*18+6] = pts[(j+1)*slice_count+i+1][0];
            vertices_sphere[j*c+i*18+7] = pts[(j+1)*slice_count+i+1][1];
            vertices_sphere[j*c+i*18+8] = pts[(j+1)*slice_count+i+1][2];

            vertices_sphere[j*c+i*18+9] = pts[(j+1)*slice_count+i+1][0];
            vertices_sphere[j*c+i*18+10] = pts[(j+1)*slice_count+i+1][1];
            vertices_sphere[j*c+i*18+11] = pts[(j+1)*slice_count+i+1][2];

            vertices_sphere[j*c+i*18+12] = pts[(j+1)*slice_count+i][0];
            vertices_sphere[j*c+i*18+13] = pts[(j+1)*slice_count+i][1];
            vertices_sphere[j*c+i*18+14] = pts[(j+1)*slice_count+i][2];

            vertices_sphere[j*c+i*18+15] = pts[j*slice_count+i][0];
            vertices_sphere[j*c+i*18+16] = pts[j*slice_count+i][1];
            vertices_sphere[j*c+i*18+17] = pts[j*slice_count+i][2];

        }
    }

    GLfloat rgb_sphere[] = {
            1.0f, 0.0f, 0.0f
        };

    QVector<GLfloat> vertData_sphere;
    for (int i = 0; i < nbTriangles*9; ++i) {
        // coordonnées sommets
        for (int j = 0; j < 3; j++)
            vertData_sphere.append(vertices_sphere[i*3+j]);
        // coordonnées texture
        for (int j = 0; j < 3; j++)
            vertData_sphere.append(rgb_sphere[j]);
    }

    vbo.create();
    vbo.bind();
    vbo.allocate(vertData_sphere.constData(), vertData_sphere.count() * int(sizeof(GLfloat)));
}

void Sphere::buildBulletObject(btDiscreteDynamicsWorld *dynamicsWorld, btAlignedObjectArray<btCollisionShape *> &collisionShapes, btScalar shape, btVector3 origin, btScalar mass, btVector3 linearVelocity)
{
    btCollisionShape* colShape = new btSphereShape(shape);
    collisionShapes.push_back(colShape);

    /// Create Dynamic Objects
    btTransform startTransform;
    startTransform.setIdentity();

    bool isDynamic = (mass != 0.f);
    btVector3 localInertia(0, 0, 0);
    if (isDynamic) colShape->calculateLocalInertia(mass, localInertia);

    startTransform.setOrigin(origin);

    //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
    btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
    btRigidBody* body = new btRigidBody(rbInfo);
    body->setLinearVelocity(linearVelocity);
    dynamicsWorld->addRigidBody(body);

    this->bulletObjectIndex = dynamicsWorld->getCollisionObjectArray().size() - 1;
}

Sphere::~Sphere()
{
    vbo.destroy();
}
