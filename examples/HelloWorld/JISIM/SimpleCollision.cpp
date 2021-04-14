#include <stdio.h>
#include <vector>
#include "btBulletDynamicsCommon.h"	//you may need to change this

#include <iostream>
#include <chrono>
#include <ctime>

/*
 Every library except for softbody seems to be required to compile any
 simulation program
*/

btDynamicsWorld* world;	//every physical object go to the world
btDispatcher* dispatcher;	//what collision algorithm to use?
btCollisionConfiguration* collisionConfig;	//what collision algorithm to use?
btBroadphaseInterface* broadphase;	//should Bullet examine every object, or just what close to each other
btConstraintSolver* solver;					//solve collisions, apply forces, impulses
std::vector<btRigidBody*> bodies;

btRigidBody* addSphere(float rad,float x,float y,float z,float mass)
{
	btTransform t;	//position and rotation
	t.setIdentity();
	t.setOrigin(btVector3(x,y,z));	//put it to x,y,z coordinates
	btSphereShape* sphere=new btSphereShape(rad);	//it's a sphere, so use sphereshape
	btVector3 inertia(0,0,0);	//inertia is 0,0,0 for static object, else
	if(mass!=0.0)
		sphere->calculateLocalInertia(mass,inertia);	//it can be determined by this function (for all kind of shapes)
	
	btMotionState* motion=new btDefaultMotionState(t);	//set the position (and motion)
	btRigidBody::btRigidBodyConstructionInfo info(mass,motion,sphere,inertia);	//create the constructioninfo, able to create multiple bodies with the same info
	btRigidBody* body=new btRigidBody(info);	//create the body 
	world->addRigidBody(body);	//put it into the world
	bodies.push_back(body);	//store in a vector for cleanup
	return body;
}


void init()
{
	//pretty much initialize everything logically
	collisionConfig=new btDefaultCollisionConfiguration();
	dispatcher=new btCollisionDispatcher(collisionConfig);
	broadphase=new btDbvtBroadphase();
	solver=new btSequentialImpulseConstraintSolver();
	world=new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collisionConfig);
	world->setGravity(btVector3(0,-10,0));	//gravity on Earth
	
	//similar to createSphere
	btTransform t;
	t.setIdentity();
	t.setOrigin(btVector3(0,0,0));
	btStaticPlaneShape* plane=new btStaticPlaneShape(btVector3(0,1,0),0);
	btMotionState* motion=new btDefaultMotionState(t);
	btRigidBody::btRigidBodyConstructionInfo info(0.0,motion,plane);
	btRigidBody* body=new btRigidBody(info);
	world->addRigidBody(body);
	bodies.push_back(body);
	
	addSphere(1.0,0,20,0,1.0);	//add a new sphere above the ground 
}


int main()
{

    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();
    
	int i =0;
	init();
	
	for (i = 0; i < 150; i++)
	{
		btRigidBody* sphere = addSphere(2.0,0,20,0,1.0);
		sphere->setLinearVelocity(btVector3(20.0,0,0));

		
		world->stepSimulation(1.f / 60.f, 10);

		//print positions of all objects
		for (int j = world->getNumCollisionObjects() - 1; j >= 0; j--)
		{
			btCollisionObject* obj = world->getCollisionObjectArray()[j];
			btRigidBody* body = btRigidBody::upcast(obj);
			btTransform trans;
			if (body && body->getMotionState())
			{
				body->getMotionState()->getWorldTransform(trans);
			}
			else
			{
				trans = obj->getWorldTransform();
			}
			printf("world pos object %d = %f,%f,%f\n", j, float(trans.getOrigin().getX()), float(trans.getOrigin().getY()), float(trans.getOrigin().getZ()));
		}
	}

	for(int i=0;i<bodies.size();i++)
	{
		world->removeCollisionObject(bodies[i]);
		btMotionState* motionState=bodies[i]->getMotionState();
		btCollisionShape* shape=bodies[i]->getCollisionShape();
		delete bodies[i];
		delete shape;
		delete motionState;
	}
	delete dispatcher;
	delete collisionConfig;
	delete solver;
	delete broadphase;
	delete world;

    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end-start;
    std::time_t end_time = std::chrono::system_clock::to_time_t(end);
  
    std::cout << "finished computation at " << std::ctime(&end_time)
              << "elapsed time: " << elapsed_seconds.count() << "s\n";
}
