/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "btVoxelSphereCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "btBoxBoxDetector.h"
#include "BulletCollision/CollisionDispatch/btCollisionObjectWrapper.h"
#include "BulletCollision/CollisionShapes/btVoxelShape.h"
#define USE_PERSISTENT_CONTACTS 1

btVoxelSphereCollisionAlgorithm::btVoxelSphereCollisionAlgorithm(bool _swap, btPersistentManifold* mf, const btCollisionAlgorithmConstructionInfo& ci, const btCollisionObjectWrapper* body0Wrap, const btCollisionObjectWrapper* body1Wrap)
: btActivatingCollisionAlgorithm(ci,body0Wrap,body1Wrap),
    swap(_swap) {
	if (!m_manifoldPtr && m_dispatcher->needsCollision(body0Wrap->getCollisionObject(),body1Wrap->getCollisionObject()))
	{
		m_manifoldPtr = m_dispatcher->getNewManifold(body0Wrap->getCollisionObject(),body1Wrap->getCollisionObject());
		m_ownManifold = true;
	}
}

btVoxelSphereCollisionAlgorithm::~btVoxelSphereCollisionAlgorithm()
{
	if (m_ownManifold)
	{
		if (m_manifoldPtr)
			m_dispatcher->releaseManifold(m_manifoldPtr);
	}
}

void btVoxelSphereCollisionAlgorithm::processCollision(const btCollisionObjectWrapper* b0, const btCollisionObjectWrapper* b1, const btDispatcherInfo& dispatchInfo, btManifoldResult* resultOut)
{
	if (!m_manifoldPtr)
		return;

	/// report a contact. internally this will be kept persistent, and contact reduction is done
	resultOut->setPersistentManifold(m_manifoldPtr);
#ifndef USE_PERSISTENT_CONTACTS	
	m_manifoldPtr->clearManifold();
#endif //USE_PERSISTENT_CONTACTS

    /************************************************************************/
    /* START CUSTOM COLLISION CODE                                          */
    /************************************************************************/
    
    const btCollisionObjectWrapper* chunk = b1->getCollisionShape()->getShapeType() == VOXEL_WORLD_PROXYTYPE ? b1 : b0;
    const btCollisionObjectWrapper* body = chunk == b1 ? b0 : b1;
    
    btVoxelShape* voxShape = (btVoxelShape*)chunk->getCollisionShape();
    btIVoxelAPI* voxAPI = voxShape->world;

    btTransform chunkTransform = chunk->getWorldTransform();
    btTransform teleportTransform = chunkTransform;
    auto basis = chunkTransform.getBasis();
    btVector3 origin = chunkTransform.getOrigin();
    btTransform bodyLocal = chunkTransform.inverse() * body->getWorldTransform();

    btVector3 traverseMin(0, 0, 0);
    btVector3 traverseMax(0, 0, 0);
    body->getCollisionShape()->getAabb(bodyLocal, traverseMin, traverseMax);

    btVoxelWorldIndex cMin;
    btVoxelWorldIndex cMax;
    cMin.x = static_cast<btVoxelIndex>(traverseMin.x());
    cMin.y = static_cast<btVoxelIndex>(traverseMin.y());
    cMin.z = static_cast<btVoxelIndex>(traverseMin.z());
    cMax.x = static_cast<btVoxelIndex>(traverseMax.x());
    cMax.y = static_cast<btVoxelIndex>(traverseMax.y());
    cMax.z = static_cast<btVoxelIndex>(traverseMax.z());
    voxAPI->clampTraversal(cMin, cMax);

    btVoxelWorldIndex vi;
    for (vi.y = cMin.y; vi.y <= cMax.y; vi.y++) {
        for (vi.z = cMin.z; vi.z <= cMax.z; vi.z++) {
            for (vi.x = cMin.x; vi.x <= cMax.x; vi.x++) {
                auto co = voxAPI->getCollisionObject(vi);
                if (!co) continue;

                // Move shape to voxel position
                teleportTransform.setOrigin(origin + (basis * (btVector3((btScalar)vi.x, (btScalar)vi.y, (btScalar)vi.z) + co->offset)));
                btCollisionObjectWrapper wrapper(nullptr, co->shape, co->object, teleportTransform, 0, 0);

                // Dispatch collision from shapes
                auto alg = m_dispatcher->findAlgorithm(&wrapper, body);
                alg->processCollision(&wrapper, body, dispatchInfo, resultOut);
            }
        }
    }

    /************************************************************************/
    /* END CUSTOM COLLISION CODE                                            */
    /************************************************************************/

#ifdef USE_PERSISTENT_CONTACTS
	//  refreshContactPoints is only necessary when using persistent contact points. otherwise all points are newly added
	if (m_ownManifold)
	{
		resultOut->refreshContactPoints();
	}
#endif //USE_PERSISTENT_CONTACTS

}

btScalar btVoxelSphereCollisionAlgorithm::calculateTimeOfImpact(btCollisionObject* /*body0*/, btCollisionObject* /*body1*/, const btDispatcherInfo& /*dispatchInfo*/, btManifoldResult* /*resultOut*/)
{
	//not yet
	return 1.f;
}
