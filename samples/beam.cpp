// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT
// Beam modifications by Simo Nikula

#include "beam.h"
#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <assert.h>
#include <memory>

Beam::Beam( b2WorldId worldId, b2Vec2 position, float rotation, int beamFlags)
{
	m_L=Beam::L;
	m_w=Beam::w;
	m_h = Beam::h;
	m_density = Beam::density;
	m_E = Beam::E;
	m_fy = Beam::fy;
	m_Wp = Beam::Wp();
	m_worldId = worldId;
	m_impl = Beam::GetSelectedImplementation();
	m_contacts = nullptr;
	m_joints = nullptr;
	m_contactCount = 0;
	m_jointCount = 0;
	m_groundIdStart = b2_nullBodyId;
	m_groundIdEnd = b2_nullBodyId;
	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.position = position;
	b2Rot rot = b2MakeRot( rotation );
	if ( beamFlags & (BeamFlags_ClampedAtStart | BeamFlags_HingeAtStart) )
	{
		m_groundIdStart = b2CreateBody( worldId, &bodyDef );
	}
	if ( beamFlags & (BeamFlags_ClampedAtEnd | BeamFlags_HingeAtEnd) )
	{
		bodyDef.position.x = position.x+rot.c*m_L;
		bodyDef.position.y = position.y + rot.s * m_L;
		m_groundIdEnd = b2CreateBody( worldId, &bodyDef );
	}
	float hx = 0.5f * m_L;
	float hh = 0.5f * m_h;
	b2Polygon box = b2MakeBox( hx, hh );
	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.density = m_density;
	bodyDef.type = b2_dynamicBody;
	bodyDef.isAwake = false;
	bodyDef.position = {position.x+rot.c*hx, position.y+rot.s*hx };
	bodyDef.rotation = rot;
	m_bodyId = b2CreateBody( worldId, &bodyDef );
	m_shapes.push_back(b2CreatePolygonShape( m_bodyId, &shapeDef, &box ));
	if ( beamFlags & BeamFlags_ClampedAtStart )
	{
		b2Vec2 pivot = position;
		b2WeldJointDef jointDef = b2DefaultWeldJointDef();
		jointDef.bodyIdA = m_groundIdStart;
		jointDef.bodyIdB = m_bodyId;
		jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
		jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
		b2CreateWeldJoint( worldId, &jointDef );
	}
	else if ( beamFlags & BeamFlags_HingeAtStart)
	{
		b2Vec2 pivot = position;
		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.bodyIdA = m_groundIdStart;
		jointDef.bodyIdB = m_bodyId;
		jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
		jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
		b2CreateRevoluteJoint( worldId, &jointDef );
	}
	if ( beamFlags & BeamFlags_ClampedAtEnd )
	{
		b2Vec2 pivot = { position.x + rot.c * m_L, position.y + rot.s * m_L };
		b2WeldJointDef jointDef = b2DefaultWeldJointDef();
		jointDef.bodyIdA = m_groundIdEnd;
		jointDef.bodyIdB = m_bodyId;
		jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
		jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
		b2CreateWeldJoint( worldId, &jointDef );
	}else if ( beamFlags & BeamFlags_HingeAtEnd )
	{
		b2Vec2 pivot = { position.x + rot.c * m_L, position.y + rot.s * m_L };
		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.bodyIdA = m_groundIdEnd;
		jointDef.bodyIdB = m_bodyId;
		jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
		jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
		b2CreateRevoluteJoint( worldId, &jointDef );
	}
}

float Beam::L, Beam::w, Beam::h, Beam::density, Beam::E, Beam::fy, Beam::rotation;
int Beam::flags;
char* Beam::flag_labels[4];
std::shared_ptr<std::vector<const char*>> implementationLabels;
std::shared_ptr<std::vector<const char*>> validImplementationLabels;
std::shared_ptr<std::vector<int>> validImplementations;
int Beam::selectedImplementationIndex;

namespace beam
{
bool static_init_done;
void static_init()
{
	if ( static_init_done )
	{
		return;
	}
	Beam::flag_labels[0] = "ClampedAtStart (1)";
	Beam::flag_labels[1] = "ClampedAtEnd (2)";
	Beam::flag_labels[2] = "HingeAtStart (4)";
	Beam::flag_labels[3] = "HingeAtEnd (8)";
	implementationLabels = std::make_shared<std::vector<const char*>>();
	validImplementationLabels = std::make_shared<std::vector<const char*>>();
	validImplementations = std::make_shared<std::vector<int>>();
	implementationLabels->push_back( "Rigid" );
	implementationLabels->push_back( "RigidPlastic" );
	Beam::selectedImplementationIndex = 0;
	static_init_done = true;
}
void static_cleanup()
{
	implementationLabels.reset();
	validImplementationLabels.reset();
	validImplementations.reset();
	static_init_done = false;
}
} // namespace

void Beam::reset()
{
Beam::L = 10.0f;
Beam::w = 1.0f;
Beam::h = 0.5f;
Beam::density = 7800.f;
Beam::E = 210E9;
Beam::fy = 350E6;
Beam::rotation = 0E0;
Beam::flags = 0;
beam::static_init();
}

float Beam::Wp()
{
	// for rectangular
	return Beam::fy*Beam::w * Beam::h * Beam::h / 4.f;
}

float Beam::I()
{
	return Beam::w * Beam::h * Beam::h *Beam::h/ 12.f;
}

bool Beam::isValid( BeamImplementation bi)
{
	switch ( bi )
	{
		case BeamImplementation_Rigid:
			return true;
		case BeamImplementation_RigidPlastic:
			float y = Beam::Wp() * Beam::L * Beam::L / 
				(2*Beam::E*Beam::I());
			return y < 0.1f*Beam::L;
	}
	return false;
}

void Beam::UpdateValidImplementations()
{
	validImplementations->clear();
	validImplementationLabels->clear();
	int v_count = implementationLabels->size();
	for ( int i = 0; i < v_count; i++ )
	{
		auto implementation = BeamImplementation( i );
		if ( isValid( implementation ) )
		{
			validImplementations->push_back( i );
			validImplementationLabels->push_back( (*implementationLabels)[i] );
		}
		else
		{
			if ( i == selectedImplementationIndex )
			{
				selectedImplementationIndex = 0;
			}
		}
	}
}

std::vector<const char*> Beam::GetValidImplementationLabels()
{
	if ( validImplementationLabels->empty() )
	{
		UpdateValidImplementations();
	}
	return *validImplementationLabels;
}

BeamImplementation Beam::GetSelectedImplementation()
{
	return BeamImplementation( selectedImplementationIndex );
}

void Beam::SetSelectedImplementation( BeamImplementation bi )
{
	selectedImplementationIndex=static_cast<int>( bi );
}

void Beam::SetSelectedImplementation( const char* offer)
{
	int index = 0;
	for ( const char* s : *implementationLabels )
	{
		if ( strcmp( offer, s ) )
		{
			selectedImplementationIndex = index;
			return;
		}
		index++;
	}
	
}

void Beam::DoBeamAnalysis( b2UpdateData updateData )
{
	if (!b2Body_IsAwake(m_bodyId)) {
		return;
	}
	CleanLoads();
	CollectLoads( updateData );
	if ( m_loads.empty() )
	{
		return;
	}
	switch ( m_impl )
	{
		case BeamImplementation_Rigid:
			return;
		case BeamImplementation_RigidPlastic:
			std::unique_ptr<RigidPlasticSolver> solver( new RigidPlasticSolver() );
			solver->solve(this);
			break;
	}
}

void Beam::Cleanup()
{
	beam::static_cleanup();
}

void Beam::CollectLoads( b2UpdateData& updateData )
{
	Load* load = new Load();
	load->p = b2Body_GetLocalCenterOfMass( m_bodyId );
	load->f = b2World_GetGravity( m_worldId ) * b2Body_GetMass(m_bodyId);
	m_loads.push_back( load );
	CollectJoints();
	for ( int i = 0; i < m_jointCount; i++ )
	{
		b2JointId jointId = m_joints[i];
		b2BodyId ba = b2Joint_GetBodyA( jointId );
		b2Vec2 anchor;
		if ( ba.index1 == m_bodyId.index1 )
		{
			anchor = b2Joint_GetLocalAnchorA( jointId );
		}
		else
		{
			anchor = b2Joint_GetLocalAnchorB( jointId );
		}
		Load* load = new Load();
		load->p = anchor;
		load->f = b2Joint_GetConstraintForce( jointId );
		load->m = b2Joint_GetConstraintTorque( jointId );
		m_loads.push_back( load );
	}
	int contactCount = b2Body_GetContactCapacity( m_bodyId );
	if ( contactCount != m_contactCount )
	{
		if ( m_contacts != nullptr )
		{
			free( m_contacts );
		}
		m_contactCount = contactCount;
		m_contacts = (b2ContactData*)malloc( m_contactCount * sizeof( b2ContactData ) );
	}
	if ( m_contacts != nullptr )
	{
		float inv_h = 1.f / updateData.timeStep;
		contactCount = b2Body_GetContactData( m_bodyId, m_contacts, m_contactCount );
		for ( int i = 0; i < contactCount; i++ )
		{
			b2ContactData cd = m_contacts[i];
			bool useA = false;
			b2BodyId ba = b2Shape_GetBody( cd.shapeIdA );
			if ( ba.index1 == m_bodyId.index1 )
			{
				useA = true;
			}
			b2Manifold manifold = cd.manifold;
			b2Vec2 normal = manifold.normal;
			b2Vec2 tangent = b2RightPerp( normal );
			for ( int pi = 0; pi < manifold.pointCount; pi++ )
			{
				b2ManifoldPoint manifoldPoint = manifold.points[pi];
				b2Vec2 anchor;
				if ( useA )
				{
					anchor = manifoldPoint.anchorA;
				}
				else
				{
					anchor = manifoldPoint.anchorB;
				}
				float nf = inv_h * manifoldPoint.normalImpulse;
				float tf = inv_h * manifoldPoint.tangentImpulse;
				Load* load = new Load();
				load->p = anchor;
				load->f = b2MulSV( nf, normal );
				m_loads.push_back( load );
				load = new Load();
				load->p = anchor;
				load->f = b2MulSV( tf, tangent );
				m_loads.push_back( load );
			}
		}
	}
}

void Beam::CollectJoints()
{
	int jointCount = b2Body_GetJointCount( m_bodyId );
	if ( jointCount != m_jointCount )
	{
		if ( m_joints != nullptr )
		{
			free( m_joints );
		}
		m_jointCount = jointCount;
		m_joints = (b2JointId*)malloc( m_jointCount * sizeof( b2JointId ) );
	}
	if ( m_joints != nullptr )
	{
		jointCount = b2Body_GetJoints( m_bodyId, m_joints, m_jointCount );
	}
}

Beam::~Beam()
{
	CleanLoads();
	CollectJoints();
	for ( int i = 0; i < m_jointCount; i++ )
	{
		b2DestroyJoint( m_joints[i] );
	}
	b2DestroyBody( m_bodyId );
	if ( m_contacts != nullptr )
	{
		free( m_contacts );
	}
	if ( m_joints != nullptr )
	{
		free( m_joints );
	}
}

void Beam::CleanLoads()
{
	while ( !m_loads.empty() )
	{
		delete m_loads.back();
		m_loads.pop_back();
	}
}

RigidPlasticSolver::~RigidPlasticSolver()
{
}

void RigidPlasticSolver::solve( Beam* beam )
{
	std::vector<b2Vec2> pv;
	std::vector<float> m;
	float mMax,mMaxAbs=0.f, hingeX;
	for (auto load:beam->m_loads )
	{
		pv.push_back( load->p );
	}
	for ( auto p : pv )
	{
		float mAtX = 0.f;
		for ( auto load : beam->m_loads )
		{
			float dm = load->m;
			float dmy = load->f.y * (load->p.x-p.x);
			float dmx= load->f.x * ( load->p.y - p.y );
			mAtX += dm + dmy + dmx;
		}
		m.push_back( mAtX );
		float mAbs = b2AbsFloat( mAtX );
		if ( mAbs > mMaxAbs )
		{
			mMax = mAtX;
			mMaxAbs = mAbs;
			hingeX = p.x;
		}
	}
	if ( mMaxAbs < beam->m_Wp)
	{
		return;
	}
	else
	{
	}
}

Solver::~Solver()
{
}
