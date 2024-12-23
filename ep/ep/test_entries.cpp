/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
* 
* Modifications for ep by Simo Nikula
*/

#include "test.h"
#include "beam.h"
#include "frame.h"
#include "ep_car.h"
#include "im_gui_demo.h"


TestEntry g_testEntries[] =
{
	{ "Beam", Beam::Create },
	{ "EmptyBeam", EmptyBeam::Create },
	{ "Car", EPCar::Create },
	{ "Frame", Frame::Create },
	{ "AxialBeam", AxialBeam::Create },
	{ "ElasticBeam", ElasticBeam::Create },
	{ "ImGui", ImGuiDemo::Create },
	{ NULL, NULL }
};
