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
*/

#include "Box2D/Common/b2Settings.h"
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

b2Version b2_version = {2, 3, 2};

// Memory allocators. Modify these to use your own allocator.
void* b2Alloc(int32 size)
{
	return malloc(size);
}

void b2Free(void* mem)
{
	free(mem);
}

// You can modify this to use your logging facility.
void b2Log(const char* string, ...)
{
	va_list args;
	va_start(args, string);
	vprintf(string, args);
	va_end(args);
}

FILE* fout = NULL;
int epLogBytes;
int maxEpLogBytes = 1 << 23;
void epLogClose() {
	if (fout!=NULL && fout != stdout) {
		fclose(fout);
		fout = NULL;
	}
	epLogActive = true;
}
void epLog(const char* string, ...) {
	if (!epLogActive || !epLogEnabled) {
		return;
	}
	if (!fout) {
		const char * fn = "ep-log.txt";
		fout = fopen(fn, "w");
		if (fout) {
			b2Log("ep-logging to %s, maxEpLogBytes=%d\n",fn,maxEpLogBytes);
			epLogBytes = 0;
		}else{
			b2Log("Could not open %s, ep-logging to stdout", fn);
			fout = stdout;
		}
	}
	va_list args;
	va_start(args, string);
	int bytes = vfprintf(fout, string, args);
	if ( bytes< 0) {
		b2Log("epLog failed\n");
		perror(string);
	}
	va_end(args);
	fflush(fout);
	epLogBytes += bytes;
	if (epLogBytes > maxEpLogBytes) {
		epLogEnabled = false;
		b2Log("ep-logging stopped after %d bytes\n", epLogBytes);
		epLogClose();
	}
}

bool epLogActive = true;
bool epLogEnabled = false;