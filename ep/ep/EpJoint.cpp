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
	StopDebug();
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
	if (nullptr != epd) {
		delete epd;
		epd = nullptr;
	}
	if (joint != nullptr) {
		joint->SetDebugListener(nullptr);
	}
	dep = false;
}


EpDebug::EpDebug()
{
	xyData = nullptr;
	velocityIterations=positionIterations = 0;
	vo = po = 0;
	showA = showB= false;
}
EpDebug::~EpDebug()
{
	if (xyData != nullptr) {
		b2Free(xyData);
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
	if (xyData != nullptr && 2 * velocityIterations == vo
		&& 2 * positionIterations == po
		&& epDebugSteps==settings->epDebugSteps) {
		// move data if needed
		int dataPointsInOneStep, size;
		if (stepsStored+1 >= epDebugSteps) {
			dataPointsInOneStep = 2 * velocityIterations;
			size = dataPointsInOneStep*(epDebugSteps - 1)*sizeof(float);
			for (int i = 0; i < 9; i++) {
				int d = i*dataPointsInOneStep*epDebugSteps;
				int s = d + dataPointsInOneStep;
				memmove(&vxA[d], &vxA[s], size);
			}
			dataPointsInOneStep = 2 * positionIterations;
			size = dataPointsInOneStep*(epDebugSteps - 1)*sizeof(float);
			for (int i = 0; i < 6; i++) {
				int d = i * dataPointsInOneStep*epDebugSteps;
				int s = d + dataPointsInOneStep;
				memmove(&pxA[d], &pxA[s], size);
			}
			dataPointsInOneStep = velocityIterations;
			size = velocityIterations*(epDebugSteps - 1)*sizeof(float);
			for (int i = 0; i < 3; i++) {
				int d = i * dataPointsInOneStep*epDebugSteps;
				int s = d + dataPointsInOneStep;
				memmove(&cdotx[d], &cdotx[s], size);
			}
		}
		else {
			stepsStored++;
		}
		return;
	}
	if (xyData != nullptr) {
		b2Free(xyData);
	}
	epDebugSteps = settings->epDebugSteps;
	vo = 2 * velocityIterations;
	po = 2 * positionIterations;
	int size = (9 * vo + 6 * po+3*velocityIterations) * 
		epDebugSteps*sizeof(float);
	xyData=(float*)b2Alloc(size);
	stepsStored = 0;
	vxA = xyData;
	int vInc = vo*epDebugSteps;
	int pInc = po*epDebugSteps;
	int dInc = velocityIterations*epDebugSteps;
	int i = vInc;
	vxB = &vxA[i]; i += vInc;
	vyA = &vxA[i]; i += vInc;
	vyB = &vxA[i]; i += vInc;
	vaA = &vxA[i]; i += vInc;
	vaB = &vxA[i]; i += vInc;
	ix = &vxA[i]; i += vInc;
	iy = &vxA[i]; i += vInc;
	ia = &vxA[i]; i += vInc;
	pxA = &vxA[i]; i += pInc;
	pxB = &vxA[i]; i += pInc;
	pyA = &vxA[i]; i += pInc;
	pyB = &vxA[i]; i += pInc;
	paA = &vxA[i]; i += pInc;
	paB = &vxA[i]; i += pInc;
	cdotx = &vxA[i]; i += dInc;
	cdoty = &vxA[i]; i += dInc;
	cdotz = &vxA[i]; i += dInc;
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
	int i = vo*stepsStored+2*joint->velocityIteration;
	if (i >=vo*epDebugSteps) {
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
	int i = vo*stepsStored + 2 * joint->velocityIteration+1;
	if (i >= vo*epDebugSteps) {
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
	int vi = velocityIterations*stepsStored + joint->velocityIteration;
	cdotx[vi] = joint->Cdot.x;
	cdoty[vi] = joint->Cdot.y;
	cdotz[vi] = joint->Cdot.z;
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
	int i = po*stepsStored+ 2 * joint->positionIteration;
	if (i >= po*epDebugSteps) {
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
	pi = joint->positionIteration;
	int iA = joint->GetIslandIndexForA();
	int iB = joint->GetIslandIndexForB();
	b2Position* va = data.positions;
	int i = po*stepsStored + 2 * joint->positionIteration+1;
	if (i >= po*epDebugSteps) {
		return;
	}
	pxA[i] = va[iA].c.x;
	pxB[i] = va[iB].c.x;
	pyA[i] = va[iA].c.y;
	pyB[i] = va[iB].c.y;
	paA[i] = va[iA].a;
	paB[i] = va[iB].a;
	// fill values if multiple steps are plotted
	if (epDebugSteps <= 1) {
		return;
	}
	int dmax = po*(stepsStored+1);
	for (int di=i; di < dmax; di++) {
		pxA[di] = pxA[i];
		pxB[di] = pxB[i];
		pyA[di] = pyA[i];
		pyB[di] = pyB[i];
		paA[di] = paA[i];
		paB[di] = paB[i];
	}
}

#define BS 100
#define LX1 380
#define LX2 450
#define LX3 520
void EpDebug::Ui(Test *t, SelectedEPJoint* j) {
	ImGui::SetNextWindowSize(ImVec2(550, 760), ImGuiSetCond_FirstUseEver);
	char buff[BS];
	snprintf(buff, BS, "EpDebug for joint %d",j->id);
	if (!ImGui::Begin(buff))
	{
		ImGui::End();
		return;
	}
	float32 b, g;
	b=g=0.f;
	if (j->joint == NULL) {
		ImGui::Text("Joint is no longer active");
	}
	else {
		g = j->joint->m_gamma;
		b = j->joint->m_bias;
	}
	EpDebug *epd = j->epd;
	ImGui::BeginGroup();
	int vc = epd->vo*(epd->stepsStored+1);
	if (vc > 0) {
		ImGui::Text("%d vis, g=%6.3g, b=%6.3g",
			epd->velocityIterations,g,b);
		ImGui::SameLine(LX1); ImGui::Text("min");
		ImGui::SameLine(LX2); ImGui::Text("max");
		ImGui::SameLine(LX3); ImGui::Text("final");
		int cdotCount = (epd->stepsStored + 1)*epd->velocityIterations;
		xyPlot("cdotx", epd->cdotx, cdotCount);
		xyPlot("cdoty", epd->cdoty, cdotCount);
		xyPlot("cdotz", epd->cdotz, cdotCount);
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
		if (epd->epDebugSteps > 1) {
			vc = epd->po*(epd->stepsStored + 1);
			ImGui::Text("max %d position iterations",
				epd->po/2);
		}
		else {
			vc = 2 * epd->pi;
			ImGui::Text("%d position iterations",
				epd->pi);
		}
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
	ImVec2 graphSize(300, 40);
	float min=FLT_MAX, max=-FLT_MAX;
	for (int i = 0; i < count; i++) {
		float f = v[i];
		min = b2Min(min, f);
		max = b2Max(max, f);
	}
	ImGui::PlotLines(label, v, count, 0, NULL, min, max, graphSize);
	ImGui::SameLine();
	ImGui::SameLine(LX1); ImGui::Text("% 6.3g",min);
	ImGui::SameLine(LX2); ImGui::Text("% 6.3g",max);
	ImGui::SameLine(LX3); ImGui::Text("% 6.3g", v[count-1]);
}
