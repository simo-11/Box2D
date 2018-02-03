#include "EpJoint.h"

Settings *EpDebug::settings=nullptr;
EpDebug::EpDebug()
{
	vxA = nullptr;
	velocityIterations=positionIterations = 0;
}
EpDebug::~EpDebug()
{
	if (vxA != nullptr) {
		b2Free(vxA);
	}
}

void EpDebug::EndInitVelocityConstraints
	(b2ElasticPlasticJoint * joint, const b2SolverData & data)
{
	if (settings == nullptr) {
		vo = 0;
		po = 0;
		return;
	}
	velocityIterations = settings->velocityIterations;
	positionIterations= settings->positionIterations;
	if (vxA != nullptr && 2 * velocityIterations == vo
		&& 2 * positionIterations == po) {
		// no changes
		return;
	}
	if (vxA != nullptr) {
		b2Free(vxA);
	}
	vo = 2 * velocityIterations;
	po = 2 * positionIterations;
	int size = (9 * vo + 6 * po) * sizeof(float);
	vxA=(float*)b2Alloc(size);
	int i = vo;
	vxB = &vxA[i]; i += vo;
	vyA = &vxA[i]; i += vo;
	vyB = &vxA[i]; i += vo;
	vaA = &vxA[i]; i += vo;
	vaB = &vxA[i]; i += vo;
	ix = &vxA[i]; i += vo;
	iy = &vxA[i]; i += vo;
	ia = &vxA[i]; i += vo;
	pxA = &vxA[i]; i += po;
	pxB = &vxA[i]; i += po;
	pyA = &vxA[i]; i += po;
	pyB = &vxA[i]; i += po;
	paA = &vxA[i]; i += po;
	paB = &vxA[i]; i += po;
}

void EpDebug::BeginVelocityIteration
	(b2ElasticPlasticJoint * joint, const b2SolverData & data)
{
	if (vo == 0) {
		return;
	}
	int iA = joint->GetIslandIndexForA();
	int iB = joint->GetIslandIndexForB();
	b2Velocity* va = data.velocities;
	int i = 2*joint->velocityIteration;
	if (i >=vo) {
		return;
	}
	vxA[i] = va[iA].v.x;
	vxB[i] = va[iB].v.x;
	vyA[i] = va[iA].v.y;
	vyB[i] = va[iB].v.y;
	vaA[i] = va[iA].w;
	vaB[i] = va[iB].w;
	b2Vec2 rf = joint->GetReactionForce(1.f);
	ix[i] = rf.x;
	iy[i] = rf.y;
	ia[i] = joint->GetReactionTorque(1.f);
}

void EpDebug::EndVelocityIteration
	(b2ElasticPlasticJoint * joint, const b2SolverData & data)
{
	if (vo == 0) {
		return;
	}
	int iA = joint->GetIslandIndexForA();
	int iB = joint->GetIslandIndexForB();
	b2Velocity* va = data.velocities;
	int i = 2 * joint->velocityIteration+1;
	if (i >= vo) {
		return;
	}
	vxA[i] = va[iA].v.x;
	vxB[i] = va[iB].v.x;
	vyA[i] = va[iA].v.y;
	vyB[i] = va[iB].v.y;
	vaA[i] = va[iA].w;
	vaB[i] = va[iB].w;
	b2Vec2 rf = joint->GetReactionForce(1.f);
	ix[i] = rf.x;
	iy[i] = rf.y;
	ia[i] = joint->GetReactionTorque(1.f);
}

void EpDebug::BeginPositionIteration
	(b2ElasticPlasticJoint * joint, const b2SolverData & data)
{
	if (po == 0) {
		return;
	}
	int iA = joint->GetIslandIndexForA();
	int iB = joint->GetIslandIndexForB();
	b2Position* va = data.positions;
	int i = 2 * joint->positionIteration;
	if (i >= po) {
		return;
	}
	pxA[i] = va[iA].c.x;
	pxB[i] = va[iB].c.x;
	pyA[i] = va[iA].c.y;
	pyB[i] = va[iB].c.y;
	paA[i] = va[iA].a;
	paB[i] = va[iB].a;
}

void EpDebug::EndPositionIteration
	(b2ElasticPlasticJoint * joint, const b2SolverData & data)
{
	if (po == 0) {
		return;
	}
	int iA = joint->GetIslandIndexForA();
	int iB = joint->GetIslandIndexForB();
	b2Position* va = data.positions;
	int i = 2 * joint->positionIteration+1;
	if (i >= po) {
		return;
	}
	pxA[i] = va[iA].c.x;
	pxB[i] = va[iB].c.x;
	pyA[i] = va[iA].c.y;
	pyB[i] = va[iB].c.y;
	paA[i] = va[iA].a;
	paB[i] = va[iB].a;
}
