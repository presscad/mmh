#include "stdafx.h"
#include "../baselib/motion/motion.h"
#include "../baselib/motion/motionloader.h"
#include "version.h"


void savePose(Posture& pose, const char* fn)
{
	BinaryFile bf;
	bf.openWrite(fn);
	bf.pack(MOT_VERSION_STRING[MOT_RECENT_VERSION]);
	bf.packInt(MOT_RECENT_VERSION);	
	pose.pack(bf, MOT_RECENT_VERSION);
	bf.close();
}

void loadPose(Posture& pose, const char* fn)
{
	BinaryFile bf;	
	bf.openRead(fn);
	bf.unpackStr();
	int version=bf.unpackInt();	
	pose.unpack(bf,version);
	bf.close();		
}
