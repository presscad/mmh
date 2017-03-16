#include "stdafx.h"
#include "mathclass.h"
#include "delaunaytriangulation.h"
#include "../utility/textfile.h"

DelaunayTriangulation::DelaunayTriangulation()
{
}

DelaunayTriangulation::~DelaunayTriangulation(void)
{
}


extern double Factorial(int a);
extern double FactorialPerFactorial(int a, int b);

double combinationR(int n, int k)
{
	k=MAX(k, n-k);
	return (FactorialPerFactorial(n,k)/Factorial(n-k));
}

int combination(int n, int k)
{
	return int(combinationR(n,k)+0.5);
}

void DelaunayTriangulation::triangulate()
{
	FILE* outfile;
	outfile=fopen("input","wt");

	// make inputfile
	fprintf(outfile, "%d %d\n", dimension(), numSite());
	ASSERT(dimension()==3);
	for(int i=0; i<numSite(); i++)
	{
		for(int j=0; j<dimension(); j++)
		{
			fprintf(outfile,"%g ", (double)(site(i).getValue(j)));
		}
		fprintf(outfile,"\n");
	}

	fclose(outfile);
	// command string
	_flushall();
	system("qdelaunay QJ Pp i < input > output");
	_flushall();

	CTextFile file;
	file.OpenReadFile("output");
	int numSimplex=atoi(file.GetToken());
	
	m_aSimplicials.setSize(numSimplex, dimension()+1);
	for(int i=0; i<numSimplex; i++)
	{
		bool bLineChanged;
		char* token=file.GetToken(bLineChanged);
		ASSERT(bLineChanged);
		for(int j=0; j<m_aSimplicials.cols(); j++)
		{
			m_aSimplicials[i][j]=atoi(token);
			token=file.GetToken();			
		}
		file.Undo();
	}
}