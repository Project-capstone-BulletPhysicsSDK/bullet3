/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2015 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "BasicExample.h"

#include "btBulletDynamicsCommon.h"
#define ARRAY_SIZE_Y 10
#define ARRAY_SIZE_X 10
#define ARRAY_SIZE_Z 10

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"

struct BasicExample : public CommonRigidBodyBase
{
	BasicExample(struct GUIHelperInterface* helper)
		: CommonRigidBodyBase(helper)
	{
	}
	virtual ~BasicExample() {}
	virtual void initPhysics();
	virtual void renderScene();
	void resetCamera()
	{
		float dist = 1;
		float pitch = -35;
		float yaw = 52;
		float targetPos[3] = {5, 5, 5};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}
};

void BasicExample::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();
	//m_dynamicsWorld->setGravity(btVector3(0,0,0));
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);

	///create a few basic rigid bodies
	btBoxShape* groundShape = createBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));

	//groundShape->initializePolyhedralFeatures();
	//btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);

	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, -50, 0));

	{
		btScalar mass(0.);
		createRigidBody(mass, groundTransform, groundShape, btVector4(0, 0, 1, 1));
	}

	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance

		btBoxShape* colShape = createBoxShape(btVector3(1, 1, 1));
		m_collisionShapes.push_back(colShape);

		btCollisionShape* colShape2 = new btSphereShape(btScalar(.5));
		m_collisionShapes.push_back(colShape2);

		btCollisionShape* colShape3 = new btSphereShape(btScalar(2));
		m_collisionShapes.push_back(colShape2);


		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		for (int k = 0; k < ARRAY_SIZE_Y; k++)
		{
			for (int i = 0; i < ARRAY_SIZE_X; i++)
			{
				for (int j = 0; j < ARRAY_SIZE_Z; j++)
				{
					if (k == 0|| i == ARRAY_SIZE_X -1 || i == 0 || j == ARRAY_SIZE_Z -1 || j == 0){
						//set position
						startTransform.setOrigin(btVector3(
						btScalar(2 * i),
						btScalar(2* k),
						btScalar(2 * j)));
							createRigidBody(0.f, startTransform, colShape);

					
					}

				}
			}
		}
		startTransform.setOrigin(btVector3(
						btScalar(10),
						btScalar(1),
						btScalar(10)));
							createRigidBody(0.f, startTransform, colShape3);
		
		btScalar mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			colShape2->calculateLocalInertia(mass, localInertia);

		int numBalls = 200;
		for (int i = 0; i < numBalls; i++){
			int offset = rand() % 100 - 50;
			int offset2 = rand() % 100 - 50;

			startTransform.setOrigin(btVector3(
				btScalar(10 + 0.02*offset),
				btScalar(10 * i + 100),
				btScalar(10+ 0.02*offset2)));
			btRigidBody* body;
			body ->setRestitution(1);
			body = createRigidBody(mass, startTransform, colShape2);
			body ->setRestitution(1);
		}
	}

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void BasicExample::renderScene()
{
	CommonRigidBodyBase::renderScene();
}

CommonExampleInterface* BasicExampleCreateFunc(CommonExampleOptions& options)
{
	return new BasicExample(options.m_guiHelper);
}

B3_STANDALONE_EXAMPLE(BasicExampleCreateFunc)
