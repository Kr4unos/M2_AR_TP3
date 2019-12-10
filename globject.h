#ifndef GLOBJECT_H
#define GLOBJECT_H

#include <QVector3D>
#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>
#include <math.h>
#include "btBulletDynamicsCommon.h"

class GLObject
{
    public:
        virtual void display(QOpenGLShaderProgram *program) = 0;
        virtual void build() = 0;

        inline QVector3D getPosition() const { return position; }
        inline void setPosition(QVector3D newPosition) { position = newPosition; }
        inline QOpenGLBuffer getVbo() const { return vbo; }
        inline int getBulletObjectIndex() const { return bulletObjectIndex; }
        virtual ~GLObject();

    protected:
        QVector3D position;
        QOpenGLBuffer vbo;
        int bulletObjectIndex;
        int nbTriangles;
};

#endif // GLOBJECT_H
