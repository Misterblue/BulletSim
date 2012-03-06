/*
 * Copyright (c) Contributors, http://opensimulator.org/
 * See CONTRIBUTORS.TXT for a full list of copyright holders.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyrightD
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the OpenSimulator Project nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE DEVELOPERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "PrimObject.h"
#include "btBulletDynamicsCommon.h"
#include "BulletSim.h"

PrimObject::PrimObject(WorldData* world, ShapeData* data) {

	m_worldData = world;
	m_id = data->ID;
	
	// TODO: correct collision shape creation
	btCollisionShape* shape = NULL;

	// Unpack ShapeData
	IDTYPE id = data->ID;
	btVector3 position = data->Position.GetBtVector3();
	btQuaternion rotation = data->Rotation.GetBtQuaternion();
	btVector3 scale = data->Scale.GetBtVector3();
	btVector3 velocity = data->Velocity.GetBtVector3();
	btScalar maxScale = scale.m_floats[scale.maxAxis()];
	btScalar mass = btScalar(data->Mass);
	btScalar friction = btScalar(data->Friction);
	btScalar restitution = btScalar(data->Restitution);
	bool isStatic = (data->Static == 1);
	bool isCollidable = (data->Collidable == 1);

	// Save the ID for this shape in the user settable variable (used to know what is colliding)
	shape->setUserPointer((void*)id);
	
	// Create a starting transform
	btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(position);
	startTransform.setRotation(rotation);

	// Building a rigid body
	btVector3 localInertia(0, 0, 0);
	shape->calculateLocalInertia(mass, localInertia);

	// Create the motion state and rigid body
	SimMotionState* motionState = new SimMotionState(data->ID, startTransform, &(m_worldData->updatesThisFrame));
	btRigidBody::btRigidBodyConstructionInfo cInfo(mass, motionState, shape, localInertia);
	btRigidBody* body = new btRigidBody(cInfo);
	motionState->RigidBody = body;

	this->SetPhysicalProperties(friction, restitution, velocity);

	// Set the dynamic and collision flags (for static and phantom objects)
	this->SetProperties(isStatic, isCollidable, false, mass);

	m_body = body;
	world->dynamicsWorld->addRigidBody(body);
}

PrimObject::~PrimObject(void) {
	if (m_body)
	{
		// Remove the object from the world
		m_worldData->dynamicsWorld->removeCollisionObject(m_body);

		// If we added a motionState to the object, delete that
		btMotionState* motionState = m_body->getMotionState();
		if (motionState)
			delete motionState;
		
		// Delete the rest of the memory allocated to this object
		btCollisionShape* shape = m_body->getCollisionShape();
		if (shape) 
			delete shape;

		// finally make the object itself go away
		delete m_body;

		m_body = NULL;
	}
}

bool PrimObject::SetProperties(const bool isStatic, const bool isSolid, const bool genCollisions, const float mass) {
	// NOTE: From the author of Bullet: "If you want to change important data 
	// from objects, you have to remove them from the world first, then make the 
	// change, and re-add them to the world." It's my understanding that some of
	// the following changes, including mass, constitute "important data"
	m_worldData->dynamicsWorld->removeRigidBody(m_body);
	SetObjectProperties(isStatic, isSolid, genCollisions, mass);
	m_worldData->dynamicsWorld->addRigidBody(m_body);

	// Why is this commented out?
	// m_worldData.dynamicsWorld->updateSingleAabb(body);
	m_body->activate(false);
	return false;
}

// TODO: generalize these parameters so we can model the non-physical/phantom/collidable objects of OpenSimulator
void PrimObject::SetObjectProperties(bool isStatic, bool isSolid, bool genCollisions, float mass)
{
	// BSLog("PrimObject::SetObjectProperties: id=%u, isStatic=%d, isSolid=%d, genCollisions=%d, mass=%f", 
	//					m_id, isStatic, isSolid, genCollisions, mass);
	SetObjectDynamic(!isStatic, mass);		// this handles the static part
	SetCollidable(isSolid);
	if (genCollisions)
	{
		// for the moment, everything generates collisions
		// TODO: Add a flag to CollisionFlags that is checked in StepSimulation on whether to pass up or not
	}
}

bool PrimObject::SetDynamic(const bool isPhysical, const float mass)
{
	return false;
}

bool PrimObject::SetScaleMass(const float scale, const float mass)
{
	return false;
}

void PrimObject::SetObjectDynamic(bool isDynamic, float mass)
{
	const btVector3 ZERO_VECTOR(0.0, 0.0, 0.0);

	// BSLog("PrimObject::SetObjectDynamic: isDynamic=%d, mass=%f", isDynamic, mass);
	if (isDynamic)
	{
		m_body->setCollisionFlags(m_body->getCollisionFlags() & ~btCollisionObject::CF_STATIC_OBJECT);
		// We can't deactivate a physical object because Bullet does not tell us when
		// the object goes idle. The lack of this event means small velocity values are
		// left on the object and the viewer displays the object floating off.
		// m_body->setActivationState(DISABLE_DEACTIVATION);
		// Change for if Bullet has been modified to call MotionState when body goes inactive

		// Recalculate local inertia based on the new mass
		btVector3 localInertia(0, 0, 0);
		m_body->getCollisionShape()->calculateLocalInertia(mass, localInertia);

		// Set the new mass
		m_body->setMassProps(mass, localInertia);
		// BSLog("SetObjectDynamic: dynamic. ID=%d, Mass = %f", CONVLOCALID(m_body->getCollisionShape()->getUserPointer()), mass);
		m_body->updateInertiaTensor();

		// NOTE: Workaround for issue http://code.google.com/p/bullet/issues/detail?id=364
		// when setting mass
		m_body->setGravity(m_body->getGravity());

		// if there are any constraints on this object, recalcuate transforms for new mass
		m_worldData->constraints->RecalculateAllConstraints(m_id);
	}
	else
	{
		m_body->setCollisionFlags(m_body->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);
		// m_body->setActivationState(WANTS_DEACTIVATION);
		// force the static object to be inactive
		m_body->forceActivationState(ISLAND_SLEEPING);

		// Clear all forces for this object
		m_body->setLinearVelocity(ZERO_VECTOR);
		m_body->setAngularVelocity(ZERO_VECTOR);
		m_body->clearForces();

		// Set the new mass (the caller should be passing zero)
		m_body->setMassProps(mass, ZERO_VECTOR);
		// BSLog("SetObjectDynamic: not dynamic. ID=%d, Mass = %f", CONVLOCALID(m_body->getCollisionShape()->getUserPointer()), mass);
		m_body->updateInertiaTensor();
		m_body->setGravity(m_body->getGravity());
	}
	// don't force activation so we don't undo the "ISLAND_SLEEPING" for static objects
	m_body->activate(false);
	return;
}

void PrimObject::SetCollidable(bool collidable)
{
	// Toggle the CF_NO_CONTACT_RESPONSE on or off for the object to enable/disable phantom behavior
	if (collidable)
		m_body->setCollisionFlags(m_body->getCollisionFlags() & ~btCollisionObject::CF_NO_CONTACT_RESPONSE);
	else
		m_body->setCollisionFlags(m_body->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE);

	m_body->activate(false);
	return;
}

bool PrimObject::SetPhysicalProperties(const btScalar friction, const btScalar restitution, const btVector3& velocity) {
	return false;
}

btVector3 PrimObject::GetObjectPosition()
{
	btTransform xform = m_body->getWorldTransform();
	return xform.getOrigin();
}

bool PrimObject::SetObjectTranslation(btVector3& position, btQuaternion& rotation)
{
	const btVector3 ZERO_VECTOR(0.0, 0.0, 0.0);

	// Build a transform containing the new position and rotation
	btTransform transform;
	transform.setIdentity();
	transform.setOrigin(position);
	transform.setRotation(rotation);

	// Clear all forces for this object
	m_body->setLinearVelocity(ZERO_VECTOR);
	m_body->setAngularVelocity(ZERO_VECTOR);
	m_body->clearForces();

	// Set the new transform for the rigid body and the motion state
	m_body->setWorldTransform(transform);
	m_body->getMotionState()->setWorldTransform(transform);

	m_body->activate(false);
	return true;
}

bool PrimObject::SetObjectVelocity(btVector3& velocity)
{
	m_body->setLinearVelocity(velocity);
	m_body->activate(false);
	return true;
}

bool PrimObject::SetObjectAngularVelocity(btVector3& angularVelocity)
{
	m_body->setAngularVelocity(angularVelocity);
	m_body->activate(true);
	return true;
}

bool PrimObject::SetObjectForce(btVector3& force)
{
	m_body->applyCentralForce(force);
	m_body->activate(false);
	return true;
}

bool PrimObject::SetObjectScaleMass(btVector3& scale, float mass, bool isDynamic)
{
	// BSLog("SetObjectScaleMass: mass=%f", mass);
	const btVector3 ZERO_VECTOR(0.0, 0.0, 0.0);

	btCollisionShape* shape = m_body->getCollisionShape();

	// NOTE: From the author of Bullet: "If you want to change important data 
	// from objects, you have to remove them from the world first, then make the 
	// change, and re-add them to the world." It's my understanding that some of
	// the following changes, including mass, constitute "important data"
	m_worldData->dynamicsWorld->removeRigidBody(m_body);

	// Clear all forces for this object
	m_body->setLinearVelocity(ZERO_VECTOR);
	m_body->setAngularVelocity(ZERO_VECTOR);
	m_body->clearForces();

	// Set the new scale
	AdjustScaleForCollisionMargin(shape, scale);

	// apply the mass and dynamicness
	SetObjectDynamic(isDynamic, mass);

	// Add the rigid body back to the simulation
	m_worldData->dynamicsWorld->addRigidBody(m_body);

	// Calculate a new AABB for this object
	m_worldData->dynamicsWorld->updateSingleAabb(m_body);

	if (m_worldData->constraints)
	{
		m_worldData->constraints->RecalculateAllConstraints(m_id);
	}

	m_body->activate(false);
	return true;
}

bool PrimObject::SetObjectCollidable(bool collidable)
{
	SetCollidable(collidable);
	return true;
}

// Adjust how gravity effects the object
// neg=fall quickly, 0=1g, 1=0g, pos=float up
bool PrimObject::SetObjectBuoyancy(float buoy)
{
	float grav = m_worldData->params->gravity * (1.0f - buoy);
	m_body->setGravity(btVector3(0, 0, grav));

	m_body->activate(false);

	return true;
}

// Bullet has a 'collisionMargin' that makes the mesh a little bigger. This routine
// reduces the scale of the underlying mesh to make the mesh plus margin the
// same size as the original mesh.
// TODO: figure out of this works correctly. For the moment set gCollisionMargin to zero.
//    Bullet tries to use scale only on the shape and does not include the margin in the scale
//    calculation. This is true in most cases but there seem to be some shapes that get
//    this wrong. 
void PrimObject::AdjustScaleForCollisionMargin(btCollisionShape* shape, btVector3& scale)
{
	btVector3 aabbMin;
	btVector3 aabbMax;
	btTransform transform;
	transform.setIdentity();
	
	// we use the constant margin because SPHERE getMargin does not return the real margin
	// btScalar margin = shape->getMargin();
	btScalar margin = m_worldData->params->collisionMargin;
	// the margin adjustment only has to happen to our compound shape. Internal shapes don't need it.
	// if (shape->isCompound() && margin > 0.01)
	// looks like internal shapes need it after all
	if (margin > 0.01)
	{
		shape->getAabb(transform, aabbMin, aabbMax);
		// the AABB contains the margin already
		btScalar xExtent = aabbMax.x() - aabbMin.x();
		btScalar xAdjustment = (xExtent - margin - margin) / xExtent;
		btScalar yExtent = aabbMax.y() - aabbMin.y();
		btScalar yAdjustment = (yExtent - margin - margin) / yExtent;
		btScalar zExtent = aabbMax.z() - aabbMin.z();
		btScalar zAdjustment = (zExtent - margin - margin) / zExtent;
		// BSLog("Adjust: m=%f, xE=%f, xA=%f, yE=%f, yA=%f, zE=%f, zA=%f, sX=%f, sY=%f, sZ=%f", margin,
		// 	xExtent, xAdjustment, yExtent, yAdjustment, zExtent, zAdjustment,
		// 	scale.x(), scale.y(), scale.z());
		shape->setLocalScaling(btVector3(scale.x()*xAdjustment, scale.y()*yAdjustment, scale.z()*zAdjustment));
	}
	else
	{
		// margin is small enough we won't fiddle with the adjustment
		shape->setLocalScaling(btVector3(scale.x(), scale.y(), scale.z()));
	}
	return;
}

void PrimObject::UpdateParameter(const char* parm, const float val)
{
	btScalar btVal = btScalar(val);

	if (strcmp(parm, "lineardamping") == 0)
	{
		m_body->setDamping(btVal, m_worldData->params->angularDamping);
		return;
	}
	if (strcmp(parm, "angulardamping") == 0)
	{
		m_body->setDamping(m_worldData->params->linearDamping, btVal);
		return;
	}
	if (strcmp(parm, "deactivationtime") == 0)
	{
		m_body->setDeactivationTime(btVal);
		return;
	}
	if (strcmp(parm, "linearsleepingthreshold") == 0)
	{
		m_body->setSleepingThresholds(btVal, m_worldData->params->angularSleepingThreshold);
	}
	if (strcmp(parm, "angularsleepingthreshold") == 0)
	{
		m_body->setSleepingThresholds(m_worldData->params->linearSleepingThreshold, btVal);
	}
	if (strcmp(parm, "ccdmotionthreshold") == 0)
	{
		m_body->setCcdMotionThreshold(btVal);
	}
	if (strcmp(parm, "ccdsweptsphereradius") == 0)
	{
		m_body->setCcdSweptSphereRadius(btVal);
	}
	return;
}

void PrimObject::UpdatePhysicalParameters(btScalar frict, btScalar resti, const btVector3& velo)
{
	// Tweak continuous collision detection parameters
	if (m_worldData->params->ccdMotionThreshold > 0.0f)
	{
		m_body->setCcdMotionThreshold(btScalar(m_worldData->params->ccdMotionThreshold));
		m_body->setCcdSweptSphereRadius(btScalar(m_worldData->params->ccdSweptSphereRadius));
	}
	m_body->setDamping(m_worldData->params->linearDamping, m_worldData->params->angularDamping);
	m_body->setDeactivationTime(m_worldData->params->deactivationTime);
	m_body->setSleepingThresholds(m_worldData->params->linearSleepingThreshold, m_worldData->params->angularSleepingThreshold);
	m_body->setContactProcessingThreshold(m_worldData->params->contactProcessingThreshold);

	m_body->setFriction(frict);
	m_body->setRestitution(resti);
	m_body->setLinearVelocity(velo);
	// per http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?t=3382
	m_body->setInterpolationLinearVelocity(btVector3(0, 0, 0));
	m_body->setInterpolationAngularVelocity(btVector3(0, 0, 0));
	m_body->setInterpolationWorldTransform(m_body->getWorldTransform());
}