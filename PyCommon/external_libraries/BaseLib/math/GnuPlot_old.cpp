#include "stdafx.h"
#include "math/mathclass.h"
#include "utility/util.h"
#include ".\gnuPlot.h"
#include <stdio.h>
#include <process.h>

bool gnuPlot_OLD::g_bPlot=false;
int gnuPlot_OLD::g_sx=640;
int gnuPlot_OLD::g_sy=480;
intervalN gnuPlot_OLD::mRange;
bool gnuPlot_OLD::g_bYUP=true;
void gnuPlot_OLD::setSyscallState(bool bPlot)
{
	gnuPlot_OLD::g_bPlot=bPlot;
}			 

void gnuPlot_OLD::setImageSize(int x, int y)
{
	gnuPlot_OLD::g_sx=x;
	gnuPlot_OLD::g_sy=y;
}

bool gnuPlot_OLD::setYUP(bool b)
{
	bool prev=gnuPlot_OLD::g_bPlot;
	gnuPlot_OLD::g_bYUP=b;
	return prev;
}

void gnuPlot_OLD::systemCall(const char* path, const char* filename, bool bwait)
{
	char szCurrDirectory[512];
	GetCurrentDirectory(512, szCurrDirectory);
	
	TString currDir(szCurrDirectory);

	for(int i=0; i<currDir.length(); i++)
	{
		if(currDir[currDir.length()-i-1]=='\\')
		{
			currDir=currDir.left(-i-1);
			currDir+="\\gnuPlot";
			printf("%s\n", currDir.ptr());
			break;
		}
	}

	TString program;

	program=currDir+"\\bin\\wgnuPlot";

	TString workingFolder;
	workingFolder=currDir+"\\"+path;
	
	SetCurrentDirectory(workingFolder);
	
	printf("%s %s\n", program.ptr(), filename);

	int mode;
	if(bwait)
		mode=_P_WAIT;
	else
		mode=_P_NOWAIT ;
	if(_spawnl(mode, program.ptr(), program.ptr(), filename, 0)==-1)
	{
		int noerr=GetLastError();

		
		TCHAR* pTemp = NULL;
        int nLen = ::FormatMessage(
                        FORMAT_MESSAGE_ALLOCATE_BUFFER | 
                        FORMAT_MESSAGE_IGNORE_INSERTS |
                        FORMAT_MESSAGE_FROM_SYSTEM,
                        NULL, 
                        noerr,
                        MAKELANGID( LANG_NEUTRAL, SUBLANG_DEFAULT ),
                        (LPTSTR)&pTemp, 
                        1, 
                        NULL );
        
		printf("Spawn error : %d %s\n", noerr, pTemp);

        ::LocalFree( pTemp );
	}

	SetCurrentDirectory(szCurrDirectory);
}

void gnuPlot_OLD::plotArrow(const char* filename, const matrixN& mat, const char* title, const char* xlabel, const char* ylabel)
{
	TString fn, dir, lfn;
	bool bPNG;
	
	processFilename(filename, bPNG, fn, dir, lfn);
	
	writeData(TString("../gnuPlot/")+fn+".dat", mat);

	// write script file
	FILE* script;
	script=fopen(TString("../gnuPlot/")+fn+".dem","wt");
	if(!script) Msg::error("gnuPlot file open error %s",filename);

	if(bPNG)
	{
		fprintf(script, "set terminal png size %d,%d\n",g_sx, g_sy);
		fprintf(script, "set output \'%s\'\n", (lfn+".PNG").ptr());
	}

	if(title)
		fprintf(script, "set title \'%s\'\nset key off\n", title);

	if(mRange.size()==2)
	{
		fprintf(script, "set xrange [%f:%f]\n", mRange.start()[0], mRange.end()[0]);
		fprintf(script, "set yrange [%f:%f]\n", mRange.start()[1], mRange.end()[1]);
	}

	fprintf(script, "set xlabel \"%s\"\n", xlabel);
	fprintf(script, "set ylabel \"%s\"\n", ylabel);

	fprintf(script,	"plot '%s' using 1:2:3:4 with vectors\n", (const char*)(lfn+".dat"));	
	if(!bPNG)
		fprintf(script, "pause -1 \"Hit return to continue\"\n");
	fclose(script);

	if(gnuPlot_OLD::g_bPlot || bPNG)
		systemCall(dir, lfn+".dem", bPNG);
}

void gnuPlot_OLD::plot2DSignal(const char* filename, const matrixN& data, const char* title, const char* xlabel, 
						const char* ylabel)
{
	matrixN newData;

	newData.setSize(data.row()-1, 4);

	vectorN delta;
	for(int i=0; i<data.row()-1; i++)
	{
		newData[i].setValue(0,2, data[i]);

		delta.sub(data[i+1], data[i]);

		newData[i].setValue(2,4, delta);
	}

	plotArrow(filename, newData, title, xlabel, ylabel);
}

void gnuPlot_OLD::mergedPlotScattered(const TStrings& filenames, int dim, const char* title, const char* xlabel, 
								const char* ylabel, const char* zlabel)
{
	TString fn, dir, lfn;
	TStrings titles;
	titles.trimSamePrefix(filenames);

	bool bPNG;
	processFilename(filenames[0], bPNG, fn, dir, lfn);
	// gnuPlot/fn.dem
	FILE* script;
	TString scriptfn=TString("../gnuPlot/")+dir+"/merge"+title+".dem";
	script=fopen(scriptfn,"wt");
	if(!script) Msg::error("gnuPlot file open error %s",scriptfn.ptr());

	if(bPNG)
	{
		fprintf(script, "set terminal png size %d,%d\n",g_sx, g_sy);
		fprintf(script, "set output \'%s\'\n", (lfn+".PNG").ptr());
	}

	if(title)
	{
		fprintf(script, "set title \'%s\'\n", title);
	}
	fprintf(script, "set key on\n");

	if(dim==2)
	{
		fprintf(script, "set xlabel \"%s\"\n", xlabel);
		fprintf(script, "set ylabel \"%s\"\n", ylabel);

		if(mRange.size()==dim)
		{
			fprintf(script, "set xrange [%f:%f]\n", mRange.start()[0], mRange.end()[0]);
			fprintf(script, "set yrange [%f:%f]\n", mRange.start()[1], mRange.end()[1]);
		}

		fprintf(script,	"plot '%s' title '%s'\\\n", (const char*)(lfn+".dat"), titles[0].ptr());
		
		for(int i=1; i<filenames.size(); i++)
		{
			processFilename(filenames[i], bPNG, fn, dir, lfn);
			fprintf(script, ",'%s' title '%s'\\\n", (const char*)(lfn+".dat"), titles[i].ptr());
		}
		fprintf(script, "\n\n");
	}
	else
	{	
		if(g_bYUP)
		{
			fprintf(script, "set xlabel \"%s\"\n", xlabel);
			fprintf(script, "set ylabel \"%s\"\n", zlabel);
			fprintf(script, "set zlabel \"%s\"\n", ylabel);

			if(mRange.size()==dim)
			{
				fprintf(script, "set xrange [%f:%f]\n", mRange.start()[0], mRange.end()[0]);
				fprintf(script, "set yrange [%f:%f]\n", mRange.start()[2], mRange.end()[2]);
				fprintf(script, "set zrange [%f:%f]\n", mRange.start()[1], mRange.end()[1]);			
			}
		}
		else
		{
			fprintf(script, "set xlabel \"%s\"\n", xlabel);
			fprintf(script, "set ylabel \"%s\"\n", ylabel);
			fprintf(script, "set zlabel \"%s\"\n", zlabel);

			if(mRange.size()==dim)
			{
				fprintf(script, "set xrange [%f:%f]\n", mRange.start()[0], mRange.end()[0]);
				fprintf(script, "set yrange [%f:%f]\n", mRange.start()[1], mRange.end()[1]);
				fprintf(script, "set zrange [%f:%f]\n", mRange.start()[2], mRange.end()[2]);			
			}
		}

		fprintf(script,	"splot '%s' title '%s'\\\n", (const char*)(lfn+".dat"), titles[0].ptr());

		for(int i=1; i<filenames.size(); i++)
		{
			processFilename(filenames[i], bPNG, fn, dir, lfn);
			fprintf(script, ",'%s' title '%s'\\\n", (const char*)(lfn+".dat"), titles[i].ptr());
		}
		fprintf(script, "\n\n");
	}

	if(!bPNG)
		fprintf(script, "pause -1 \"Hit return to continue\"\n");

	fclose(script);


	if(gnuPlot_OLD::g_bPlot || bPNG)
		systemCall(dir, scriptfn, bPNG);
}

void gnuPlot_OLD::plotScattered(const char* filename, const matrixN& mat, const char* title, const char* xlabel, const char* ylabel, const char* zlabel)
{
	TString fn, dir, lfn;
	bool bPNG;
	processFilename(filename, bPNG, fn, dir, lfn);

	writeData(TString("../gnuPlot/")+fn+".dat", mat);

	// gnuPlot/fn.dem
	FILE* script;
	script=fopen(TString("../gnuPlot/")+fn+".dem","wt");
	if(!script) Msg::error("gnuPlot_OLD file open error %s",filename);

	if(bPNG)
	{
		fprintf(script, "set terminal png size %d,%d\n",g_sx, g_sy);
		fprintf(script, "set output \'%s\'\n", (lfn+".PNG").ptr());
	}
	
	if(title)
	{
		fprintf(script, "set title \'%s\'\nset key off\n", title);
	}

	if(mat.column()==2)
	{
		fprintf(script, "set xlabel \"%s\"\n", xlabel);
		fprintf(script, "set ylabel \"%s\"\n", ylabel);

		if(mRange.size()==mat.column())
		{
			fprintf(script, "set xrange [%f:%f]\n", mRange.start()[0], mRange.end()[0]);
			fprintf(script, "set yrange [%f:%f]\n", mRange.start()[1], mRange.end()[1]);
		}

		fprintf(script,	"plot '%s'\n", (const char*)(lfn+".dat"));		
	}
	else
  	{	
		if(g_bYUP)
		{
			fprintf(script, "set xlabel \"%s\"\n", xlabel);
			fprintf(script, "set ylabel \"%s\"\n", zlabel);
			fprintf(script, "set zlabel \"%s\"\n", ylabel);

			if(mRange.size()==mat.column())
			{
				fprintf(script, "set xrange [%f:%f]\n", mRange.start()[0], mRange.end()[0]);
				fprintf(script, "set yrange [%f:%f]\n", mRange.start()[2], mRange.end()[2]);
				fprintf(script, "set zrange [%f:%f]\n", mRange.start()[1], mRange.end()[1]);			
			}
		}
		else
		{
			fprintf(script, "set xlabel \"%s\"\n", xlabel);
			fprintf(script, "set ylabel \"%s\"\n", ylabel);
			fprintf(script, "set zlabel \"%s\"\n", zlabel);

			if(mRange.size()==mat.column())
			{
				fprintf(script, "set xrange [%f:%f]\n", mRange.start()[0], mRange.end()[0]);
				fprintf(script, "set yrange [%f:%f]\n", mRange.start()[1], mRange.end()[1]);
				fprintf(script, "set zrange [%f:%f]\n", mRange.start()[2], mRange.end()[2]);			
			}
		}
		fprintf(script,	"splot '%s'\n", (const char*)(lfn+".dat"));		
	}

	if(!bPNG)
		fprintf(script, "pause -1 \"Hit return to continue\"\n");

	fclose(script);

	
	if(gnuPlot_OLD::g_bPlot || bPNG)
		systemCall(dir, lfn+".dem", bPNG);
	
	// script for line fitting.
	if(mat.column()==2)
	{
		// gnuPlot/line.fnc
		script=fopen("../gnuPlot/line.fnc", "wt");
		fprintf(script, "l(x) = y0 + m*x\n");
		fclose(script);

		// gnuPlot/fn_fit.dem		
		script=fopen(TString("../gnuPlot/")+fn+"_fit.dem","wt");
		if(!script) Msg::error("gnuPlot file open error %s",filename);

		//	set title 'data for fit demo'
		fprintf(script, "plot '%s.dat'\n", fn.ptr());
		fprintf(script, "#set xlabel \"speed\"\n");
		fprintf(script, "#set ylabel \"deltaTurn\"\n");
		fprintf(script, "#set yrange [-0.5:0.5]\n");
		fprintf(script, "load 'line.fnc'\n");
		fprintf(script, "y0=0.0\n");
		fprintf(script, "m=0.0\n");
		fprintf(script, "show variables\n");
		fprintf(script, "fit l(x) '%s.dat' via y0, m\n", fn.ptr());
		fprintf(script,	"plot '%s.dat', l(x)\n", fn.ptr());
		fprintf(script, "pause -1 \"Hit return to continue\"\n");
		fclose(script);
	}
}


void gnuPlot_OLD::writeData(const char* filename, const matrixN& mat)
{	
	FILE* data;
	data=fopen(filename,"wt");
	if(!data) Msg::error("gnuPlot_OLD file open error %s",filename);
	
	for(int i=0; i<mat.row(); i++)
	{

		if(mat.column()==2)
			fprintf(data,"%f %f\n", mat[i].x(), mat[i].y());
		else if(mat.column()==3)	
		{
			if(g_bYUP)
			{	// x, z, y 를 사용했다. 이유는 y축을 위로 향하게 plotting 하고 싶어서.		
				fprintf(data,"%f %f %f\n", mat[i].x(), mat[i].z(), mat[i].y());
			}
			else
				fprintf(data,"%f %f %f\n", mat[i].x(), mat[i].y(), mat[i].z());
		}
		else if(mat.column()==4)
			fprintf(data,"%f %f %f %f\n", mat[i][0], mat[i][1], mat[i][2], mat[i][3]);
		else
		{
			TString temp;
			for(int j=0; j<mat.column(); j++)
				temp.add("%f ", mat[i][j]);
			fprintf(data, "%s\n", temp.ptr());
		}
	}
	fclose(data);
}

void gnuPlot_OLD::processFilename(const char* filename, bool& bPNG, TString& fn, TString& dir, TString& lfn)
{
	fn=filename;

	fn.makeUpper();

	bPNG=false;
	if(fn.right(4)==".PNG")
	{
		bPNG=true;
		fn=fn.left(-4);		
	}

	// split fn into dir+lfn
	int currDir;
	if((currDir=fn.findChar(0, '/'))!=-1)
	{
		lfn=fn.right(-1*currDir-1);
		dir=fn.left(currDir);
	}
	else
		lfn=fn;
}

void gnuPlot_OLD::setRange(const intervalN& range)
{
	mRange=range;
}

void gnuPlot_OLD::unsetRange()
{
	// draw using default setting.
	mRange.setSize(0);
}

