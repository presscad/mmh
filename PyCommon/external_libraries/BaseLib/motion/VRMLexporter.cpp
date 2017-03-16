#include "stdafx.h"
#include "motion.h"
#include "motionloader.h"
#include "VRMLexporter.h"

namespace sz0
{

	struct replace : public Operator
	{
		replace (TString const& pattern, TString const& replacepattern):mPattern(pattern), mReplacepattern(replacepattern){}
		virtual void calc(TString& c) const;
		TString const& mPattern;
		TString const& mReplacepattern;
	};
	
	void replace::calc(TString& c) const
	{
		TString out;
		
		bitvectorn index;
		index.setSize(c.length());
		index.clearAll();

		for(int i=0; i<c.length()-mPattern.length()+1; i++)
		{
			if(c.subString(i, i+mPattern.length())==mPattern)
			{
				index.setAt(i);
				i+=mPattern.length()-1;
			}
		}

		int count=index.count();

		out.reserve(c.length()-mPattern.length()*count+mReplacepattern.length()*count+1);
		out.empty();

		char character[2];
		character[1]=0;
		int i;
		for(i=0; i<c.length()-mPattern.length()+1; i++)
		{
			if(index[i])
			{
				out.add(mReplacepattern);
				i+=mPattern.length()-1;
			}
			else
			{
				character[0]=c[i];
				out.concat(character);				
			}
		}

		if(mPattern.length()>1)
			out.add(c.subString(i));

		c=out;
	}
}

TString space(int level)
{
	static TString temp;
	temp.empty();
	temp.add("\n");
	for(int i=0; i<level; i++)
		temp.add("    ");
	return temp;
}


TString nameId(Bone& bone)
{
	static int curSite=0;
	TString out(bone.NameId);

	if(TString("Site")==bone.NameId)
	{
		out.add("%d", curSite++);
	}
	return out;
}

void packShape(Bone& bone,FILE* file, int level, MotionLoader* pLoader)
{
	TString shape;
	shape.add("Shape {\nappearance Appearance {\nmaterial Material {\ndiffuseColor 0 0.3529 1\nambientIntensity 0.2523\n");
	shape.add( "specularColor 0.7829 1.075 1.24\nshininess 0.544\ntransparency 0.2\nemissiveColor 0 0.3529 1}}\n");
	shape.add( "geometry DEF %s-FACES IndexedFaceSet {\n", nameId(bone).ptr());
	shape.add( "  ccw TRUE\n  solid FALSE\n  coord DEF %s-COORD Coordinate {\n  point [", nameId(bone).ptr());

	vector3 offset;
	bone.getOffset(offset);

	m_real width=1.0;
	vector3 v1(0,0,0);
	vector3 v3(offset);
	vector3 v2;
	v2=(v1+v3)*0.5;

	shape.add( " %f %f %f,\n", v1.x, v1.y, v1.z);
	shape.add( " %f %f %f,\n",v2.x+width, v2.y, v2.z+width);
	shape.add( " %f %f %f,\n",v2.x+width,v2.y, v2.z-width);
	shape.add( " %f %f %f,\n",v2.x-width,v2.y, v2.z+width);
	shape.add( " %f %f %f,\n",v2.x-width,v2.y, v2.z-width);
	shape.add( " %f %f %f]\n", v3.x,v3.y, v3.z);
	shape.add( "}\n");
	shape.add( "  coordIndex [0, 1, 2, -1,0,3,1,-1,0, 4, 3, -1,0, 2, 4,-1,1,3,5,-1,2, 1, 5,-1,3,4,5,-1,4,2,5,-1]\n");
	shape.add( "  }\n}\n");

	shape.op0(sz0::replace("\n", space(level)));
	fprintf(file, "%s", shape.ptr());
}

void packAnimation(Bone& bone, FILE* file, int level, Motion const& mot, int start, int end )
{
	TString anim;
	
	vectorn keys;
	keys.linspace(0, 1, end-start);
	
	if(level==1)
	{
		anim.add("DEF TIMER TimeSensor { loop TRUE cycleInterval %f },\n", mot.FrameTime()*(end-start));

		// root joint
		anim.add("DEF %s-POS-INTERP PositionInterpolator {\n key [", bone.NameId);
		for(int i=start; i<end; i++)
			anim.add("%f,", keys[i-start]);

		anim.add("]\nkeyValue [");
		
		for(int i=start; i<end; i++)
			anim.add("%f %f %f, \n", mot.Pose(i).m_aTranslations[0].x, mot.Pose(i).m_aTranslations[0].y, mot.Pose(i).m_aTranslations[0].z);

		anim.add("] },\n");
	}

	for(int i=0; i<mot.NumJoints(); i++)
	{
		if(&bone==&mot.skeleton().getBoneByRotJointIndex(i))
		{
			// joint i
			anim.add("DEF %s-ROT-INTERP OrientationInterpolator {\n key [", bone.NameId);
			for(int j=start; j<end; j++)
				anim.add("%f,", keys[j-start]);

			anim.add("]\nkeyValue [");

			for(int iframe=start; iframe<end; iframe++)
			{
				quater q;
				q=mot.Pose(iframe).m_aRotations[i];
				vector3 v=q.rotationVector();
				m_real angle=v.length();
				v/=angle;

				anim.add("%f %f %f %f, \n", v.x, v.y, v.z, angle);
			}

			anim.add("] },\n");

			break;
		}
	}		

	anim.op0(sz0::replace("\n", space(level)));
	fprintf(file, "%s", anim.ptr());
}
void packTransform(Bone& bone, FILE* file, int level, Motion const& mot, int start, int end )
{
	TString transform;
	transform.add("DEF %s Transform {\n", nameId(bone).ptr());
	vector3 offset;
	bone.getOffset(offset);
	transform.add("  translation %f %f %f\n", offset.x, offset.y, offset.z);

	transform.add("  children [\n");
	transform.op0(sz0::replace("\n", space(level)));
	fprintf(file, "%s", transform.ptr());
	transform.empty();

	if(bone.m_pChildHead)
	{
		//if(level==0)
		packAnimation(bone, file, level+1, mot, start, end);

		for(Bone* node=(Bone*)bone.m_pChildHead; node!=NULL; node=(Bone*)(node->m_pSibling))
		{
			packShape(*node, file, level+1, &mot.skeleton());

			if(node->m_pChildHead)
				packTransform(*node, file, level+1, mot, start, end );
		}
	}

	transform.add("]\n");
	if(level==0)
	{
		for(int i=0; i<mot.NumJoints(); i++)
		{
			char* name=mot.skeleton().getBoneByRotJointIndex(i).NameId;
			if(i==0)
			{				
				transform.add("ROUTE TIMER.fraction_changed TO %s-POS-INTERP.set_fraction\n", name);
				transform.add("ROUTE %s-POS-INTERP.value_changed TO %s.set_translation\n", name,name);				
			}
			transform.add("ROUTE TIMER.fraction_changed TO %s-ROT-INTERP.set_fraction\n", name);
			transform.add("ROUTE %s-ROT-INTERP.value_changed TO %s.set_rotation\n", name,name);

		}
	}
	transform.add("}\n");
	transform.op0(sz0::replace("\n", space(level)));
	fprintf(file, "%s", transform.ptr());
}
void MotionUtil::exportVRML(Motion const& mot, const char* filename, int start, int end)
{
	FILE* file=fopen(filename, "wt");

	if(end>mot.NumFrames())
		end=mot.NumFrames();

	fprintf(file, "#VRML V2.0 utf8\n\n# Produced by 3D Studio MAX VRML97 exporter, Version 3.01, Revision 0\n# MAX File: 방향지시.max, Date: Fri Aug 25 16:57:48 2000\n");
	packTransform(mot.skeleton().getBoneByRotJointIndex(0), file, 0, mot, start, end);
	
	fclose(file);
}