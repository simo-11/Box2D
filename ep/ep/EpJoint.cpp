#include "EpJoint.h"
#include <stdio.h>
#include <imgui/imgui.h>

Settings *EpDebug::settings=nullptr;

SelectedEPJoint::SelectedEPJoint(){
	id = 0;
	dep = false;
	next = nullptr;
	joint = nullptr;
	epd = nullptr;
}
SelectedEPJoint::~SelectedEPJoint() {
	if (nullptr != epd) {
		delete epd;
		epd = nullptr;
	}
}
EpDebug* SelectedEPJoint::getEpDebug(){
	if (nullptr == epd) {
		epd = new EpDebug();
	}
	return epd;
}

void SelectedEPJoint::StartDebug()
{
	getEpDebug();
	joint->SetDebugListener(epd);
	dep = true;
}

void SelectedEPJoint::StopDebug()
{
	joint->SetDebugListener(nullptr);
	delete epd;
	epd = nullptr;
	dep = false;
}


EpDebug::EpDebug()
{
	vxA = nullptr;
	velocityIterations=positionIterations = 0;
	vo = po = 0;
	showA = showB= false;
}
EpDebug::~EpDebug()
{
	if (vxA != nullptr) {
		b2Free(vxA);
	}
}

bool EpDebug::IsWanted(b2Body *b) {
	return b->IsActive() && (b->GetType() == b2_dynamicBody);
}
void EpDebug::EndInitVelocityConstraints
	(b2ElasticPlasticJoint * joint, const b2SolverData & data)
{
	if (settings == nullptr) {
		vo = 0;
		po = 0;
		return;
	}
	showA = IsWanted(joint->GetBodyA());
	showB = IsWanted(joint->GetBodyB()); 
	if (!showA && !showB) {
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

#define BS 100
void EpDebug::Ui(Test *t, SelectedEPJoint* j) {
	ImGui::SetNextWindowSize(ImVec2(550, 680), ImGuiSetCond_FirstUseEver);
	char buff[BS];
	snprintf(buff, BS, "EpDebug for joint %d",j->id);
	if (!ImGui::Begin(buff))
	{
		ImGui::End();
		return;
	}
	EpDebug *epd = j->epd;
	ImGui::BeginGroup();
	ImGui::Text("Details for last iteration");
	int vc = epd->vo;
	ImVec2 graphSize(120,30);
	if (vc > 0) {
		if (epd->showA) {
			ImGui::PlotLines("vxA", epd->vxA, vc, 0, NULL, FLT_MAX, FLT_MAX, graphSize);
			ImGui::PlotLines("vyA", epd->vyA, vc, 0, NULL, FLT_MAX, FLT_MAX, graphSize);
			ImGui::PlotLines("vaA", epd->vaA, vc, 0, NULL, FLT_MAX, FLT_MAX, graphSize);
		}
		if (epd->showB) {
			ImGui::PlotLines("vxB", epd->vxB, vc, 0, NULL, FLT_MAX, FLT_MAX, graphSize);
			ImGui::PlotLines("vyB", epd->vyB, vc, 0, NULL, FLT_MAX, FLT_MAX, graphSize);
			ImGui::PlotLines("vaB", epd->vaB, vc, 0, NULL, FLT_MAX, FLT_MAX, graphSize);
		}
		ImGui::PlotLines("ix", epd->ix, vc, 0, NULL, FLT_MAX, FLT_MAX, graphSize);
		ImGui::PlotLines("iy", epd->iy, vc, 0, NULL, FLT_MAX, FLT_MAX, graphSize);
		ImGui::PlotLines("ia", epd->ia, vc, 0, NULL, FLT_MAX, FLT_MAX, graphSize);
	}
	vc = 2*j->joint->positionIteration;
	if (vc > 0) {
		if (epd->showA) {
			ImGui::PlotLines("pxA", epd->pxA, vc, 0, NULL, FLT_MAX, FLT_MAX, graphSize);
			ImGui::PlotLines("pyA", epd->pyA, vc, 0, NULL, FLT_MAX, FLT_MAX, graphSize);
			ImGui::PlotLines("paA", epd->paA, vc, 0, NULL, FLT_MAX, FLT_MAX, graphSize);
		}
		if (epd->showB) {
			ImGui::PlotLines("pxB", epd->pxB, vc, 0, NULL, FLT_MAX, FLT_MAX, graphSize);
			ImGui::PlotLines("pyB", epd->pyB, vc, 0, NULL, FLT_MAX, FLT_MAX, graphSize);
			ImGui::PlotLines("paB", epd->paB, vc, 0, NULL, FLT_MAX, FLT_MAX, graphSize);
		}
	}
	ImGui::EndGroup();
	if (ImGui::IsItemHovered()) {
		t->HighLightJoint(j->joint);
	}
	ImGui::End();
}