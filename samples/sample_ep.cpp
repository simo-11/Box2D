// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT
// EP / Beam modifications by Simo Nikula

#include "beam.h"
#include "draw.h"
#include "sample.h"
#include "settings.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <GLFW/glfw3.h>
#include <imgui.h>

#include <vector>
#include <iterator>
// This sample studies stiff to flexible body behaviour.
class EpBeam : public Sample
{
public:
	explicit EpBeam( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 0.0f, 0.0f };
			g_camera.m_zoom = 25.0f * 0.35f;
		}
		Beam::reset();
		m_restart = true;
		m_tipId = b2_nullBodyId;
		m_groundId = b2_nullBodyId;
		m_shapeIdFloor1 = b2_nullShapeId;
		m_shapeIdFloor2 = b2_nullShapeId;
	}
	void CreateWorld(){
		if ( m_groundId.index1 == 0 )
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			m_groundId = b2CreateBody( m_worldId, &bodyDef );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Vec2 p1 = { -40.0f, -2.0f };
			b2Vec2 p2 = { 0.0f, -3.f };
			b2Vec2 p3 = { 40.0f, -2.0f };
			b2Segment segment = { p1, p2 };
			m_shapeIdFloor1=b2CreateSegmentShape( m_groundId, &shapeDef, &segment );
			segment = { p2, p3 };
			m_shapeIdFloor2 = b2CreateSegmentShape( m_groundId, &shapeDef, &segment );
		}
		else
		{
			Cleanup();
			b2Segment segment = b2Shape_GetSegment( m_shapeIdFloor1 );
			float y = -1.f * ( 1 + 2 );
			segment.point2.y = y;
			b2Shape_SetSegment( m_shapeIdFloor1, &segment );
			segment = b2Shape_GetSegment( m_shapeIdFloor2 );
			segment.point1.y = y;
			b2Shape_SetSegment( m_shapeIdFloor2, &segment );
		}
		m_restart = false;
	}
	~EpBeam()
	{
		Cleanup();
	}
	void Cleanup()
	{
		while ( !m_beams.empty() )
		{
			delete m_beams.back();
			m_beams.pop_back();
		}
	}
	void UpdateUI() override
	{
		float height = 400.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, g_camera.m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 240.0f, height ) );

		ImGui::Begin( "EpCantilever", nullptr, ImGuiWindowFlags_NoResize );
		ImGui::PushItemWidth( 100.0f );

		if ( ImGui::SliderFloat( "Beam Length", &Beam::L, 1,20) )
		{
			m_restart = true;
		}
		if ( ImGui::SliderFloat( "Beam Height", &Beam::h, Beam::L/100, Beam::L/5) )
		{
			m_restart = true;
		}
		if ( ImGui::SliderFloat( "Beam Density", &Beam::density, 500, 10000 ) )
		{
			m_restart = true;
		}
		if ( ImGui::SliderFloat( "Beam Young's modulus", &Beam::E, 1E9, 1E12, "%.3E" ) )
		{
			m_restart = true;
		}
		if ( ImGui::SliderFloat( "Beam Yield stress", &Beam::fy, 1E6, 1E9, "%.3E" ) )
		{
			m_restart = true;
		}
		ImGui::SliderFloat( "Beam Rotation", &Beam::rotation, 0.f, 2 * B2_PI );
		if ( ImGui::TreeNodeEx( "Beam Boundary conditions", 
			ImGuiTreeNodeFlags_CollapsingHeader ) )
		{
			for ( int i = 0; i < 4; i++ )
			{
				int mask = 1 << i;
				bool v = Beam::flags & mask;
				char* label = Beam::flag_labels[i];
				if ( ImGui::Checkbox( label, &v ) )
				{
					Beam::flags ^= mask;
				}
			}
		}
		if ( ImGui::SmallButton( "Restart with current settings" ) )
		{
			m_restart = true;
		}
		ImGui::PopItemWidth();
		ImGui::End();
	}
	void MouseDown( b2Vec2 p, int button, int mod )
	{
		switch ( mod )
		{
			case GLFW_MOD_ALT:
			{
				Beam* beam = new Beam( m_worldId, p, Beam::rotation, Beam::flags );
				m_beams.push_back( beam );
			}
			break;
			default:
			Sample::MouseDown( p, button, mod );
				break;
		}
	}
	void Step( Settings& settings ) override
	{
		if ( m_restart )
		{
			Restart();
		}
		Sample::Step( settings );
		if ( !settings.pause )
		{
			float timeStep = settings.hertz > 0.0f ? 1.0f / settings.hertz : 0.0f;
			b2UpdateData updateData = { timeStep, &g_draw, &g_camera };
			for ( const auto& beam : m_beams )
			{
				beam->DoBeamAnalysis( updateData );
			}
		}
		g_draw.DrawString( 5, m_textLine, "Create Beams using ALT-MB1");
		m_textLine += m_textIncrement;
	}


	void Restart() override
	{
		CreateWorld();
	}

	static Sample* Create( Settings& settings )
	{
		return new EpBeam( settings );
	}

	b2BodyId m_tipId, m_groundId;
	bool m_restart;
	b2ShapeId m_shapeIdFloor1,m_shapeIdFloor2;
	std::vector<b2ShapeId> m_shapesToDeleteOnRestart;
	std::vector<Beam*> m_beams;
};
static int sampleEpCantileverIndex = 
RegisterSample( "ElasticPlastic", "Beam", EpBeam::Create );