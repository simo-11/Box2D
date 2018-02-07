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
#define LX1 380
#define LX2 450
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
	int vc = epd->vo;
	if (vc > 0) {
		ImGui::Text("%d velocity iterations",epd->velocityIterations);
		ImGui::SameLine(LX1); ImGui::Text("min");
		ImGui::SameLine(LX2); ImGui::Text("max");
		if (epd->showA) {
			xyPlot("vxA", epd->vxA, vc);
			xyPlot("vyA", epd->vyA, vc);
			xyPlot("vaA", epd->vaA, vc);
		}
		if (epd->showB) {
			xyPlot("vxB", epd->vxB, vc);
			xyPlot("vyB", epd->vyB, vc);
			xyPlot("vaB", epd->vaB, vc);
		}
		xyPlot("ix", epd->ix, vc);
		xyPlot("iy", epd->iy, vc);
		xyPlot("ia", epd->ia, vc);
		vc = 2 * j->joint->positionIteration;
		ImGui::Text("%d position iterations", 
			j->joint->positionIteration);
		if (epd->showA) {
			xyPlot("pxA", epd->pxA, vc);
			xyPlot("pyA", epd->pyA, vc);
			xyPlot("paA", epd->paA, vc);
		}
		if (epd->showB) {
			xyPlot("pxB", epd->pxB, vc);
			xyPlot("pyB", epd->pyB, vc);
			xyPlot("paB", epd->paB, vc);
		}
	}
	ImGui::EndGroup();
	if (ImGui::IsItemHovered()) {
		t->HighLightJoint(j->joint);
	}
	ImGui::End();
}

void EpDebug::xyPlot(const char * label, float * v, int count)
{
	if (count == 0) {
		return;
	}
	ImVec2 graphSize(300, 50);
	float min=FLT_MAX, max=-FLT_MAX;
	for (int i = 0; i < count; i++) {
		float f = v[i];
		min = b2Min(min, f);
		max = b2Max(max, f);
	}
	ImGui::PlotLines(label, v, count, 0, NULL, min, max, graphSize);
	ImGui::SameLine();
	ImGui::SameLine(LX1); ImGui::Text("% 6.3f",min);
	ImGui::SameLine(LX2); ImGui::Text("% 6.3f",max);
}
