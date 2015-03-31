//
// btVoxelShape.h
//
// Created by Cristian Zaloj on 31 Mar 2015
//

#pragma once

#ifndef btVoxelShape_h__
#define btVoxelShape_h__

#include <tuple>
#include "btBulletCollisionCommon.h"

typedef long long btVoxelIndex;
struct btVoxelWorldIndex {
    btVoxelIndex x;
    btVoxelIndex y;
    btVoxelIndex z;
};
struct btVoxelCollisionPiece {
    btCollisionShape* shape;
    btCollisionObject* object;
    btVector3 offset;
};

class btIVoxelAPI {
public:
    virtual void getLocalBoundingBox(btVector3& outMin, btVector3& outMax) const = 0;
    virtual void calculateLocalInertia(btScalar mass, btVector3& inertia) const = 0;
    virtual void clampTraversal(btVoxelWorldIndex& min, btVoxelWorldIndex& max) const = 0;

    virtual const btVoxelCollisionPiece* getCollisionObject(const btVoxelWorldIndex& pos) const = 0;
};

class btVoxelShape : public btConcaveShape {
public:
    btVoxelShape(btIVoxelAPI* chunk);

    virtual void getAabb(const btTransform& t, btVector3& aabbMin, btVector3& aabbMax) const;
    virtual btScalar getMargin() const;
    virtual void setMargin(btScalar collisionMargin);
    virtual void setLocalScaling(const btVector3& scaling);
    virtual const btVector3& getLocalScaling() const;
    virtual const char* getName() const;
    virtual void calculateLocalInertia(btScalar mass, btVector3& inertia) const;

    virtual void processAllTriangles(btTriangleCallback* callback, const btVector3& aabbMin, const btVector3& aabbMax) const;

    btIVoxelAPI* world;
};

#endif // !btVoxelShape_h__
