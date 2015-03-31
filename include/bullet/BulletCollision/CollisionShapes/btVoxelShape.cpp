#include "btVoxelShape.h"

#include <stdexcept>

#include "LinearMath/btAabbUtil2.h"

btVoxelShape::btVoxelShape(btIVoxelAPI* api) :
world(api) {
    m_shapeType = VOXEL_WORLD_PROXYTYPE;
}
void btVoxelShape::getAabb(const btTransform& t, btVector3& aabbMin, btVector3& aabbMax) const {
    // Get local untransformed bounding box
    btVector3 extentMin(0.0, 0.0, 0.0);
    btVector3 extentMax(0.0, 0.0, 0.0);
    world->getLocalBoundingBox(extentMin, extentMax);

    // Transform chunk boundaries
    btTransformAabb(extentMin, extentMax, 0.0, t, aabbMin, aabbMax);
}
btScalar btVoxelShape::getMargin() const {
    return 0.0;
}
void btVoxelShape::setMargin(btScalar collisionMargin) {
    throw std::logic_error("The method or operation is not implemented.");
}
void btVoxelShape::setLocalScaling(const btVector3& scaling) {
    throw std::logic_error("The method or operation is not implemented.");
}
const btVector3& btVoxelShape::getLocalScaling() const {
    throw std::logic_error("The method or operation is not implemented.");
}
const char* btVoxelShape::getName() const {
    return "Voxel Shape";
}
void btVoxelShape::calculateLocalInertia(btScalar mass, btVector3& inertia) const {
    world->calculateLocalInertia(mass, inertia);
}

void btVoxelShape::processAllTriangles(btTriangleCallback* callback, const btVector3& aabbMin, const btVector3& aabbMax) const {
    // TODO(Cristian): More API stuff
    return;
}
