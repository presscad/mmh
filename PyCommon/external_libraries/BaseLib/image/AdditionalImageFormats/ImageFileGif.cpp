#include "stdafx.h"
#include "stdimage.h"
#include "Image.h"
#include "ImageFileGif.h"
#include "ImagePixelPtr.h"

#define WIDTHBYTES(bits)    (((bits) + 31) / 32 * 4)

struct {
	unsigned int Width;
	unsigned int Height;
	unsigned char ColorMap[3][MAXCOLORMAPSIZE];
	unsigned int BitPixel;
	unsigned int ColorResolution;
	unsigned int BackGround;
	unsigned int AspectRatio;
} GifScreen;

struct {
	int transparent;
	int delayTime;
	int inputFlag;
	int disposal;
} Gif89={-1,-1,-1,0};



////////////////////////////////////////////////////////////////////////
// GIFFile class & function for read


GIFFile::GIFFile()
{
	m_GIFErrorText="No Error"; // yet
}

GIFFile::~GIFFile()
{
	// nothing
}


//
//	swap Rs and Bs
//
//	Note! this does its stuff on buffers with a whole number of pixels
//	per data row!!
//

BOOL GIFFile::BGRFromRGB(BYTE *buf, UINT widthPix, UINT height)
{
	if (buf==NULL)
		return FALSE;

	UINT col, row;
	for (row=0;row<height;row++) {
		for (col=0;col<widthPix;col++) {
			LPBYTE pRed, pGrn, pBlu;
			pRed = buf + row * widthPix * 3 + col * 3;
			pGrn = buf + row * widthPix * 3 + col * 3 + 1;
			pBlu = buf + row * widthPix * 3 + col * 3 + 2;

			// swap red and blue
			BYTE tmp;
			tmp = *pRed;
			*pRed = *pBlu;
			*pBlu = tmp;
		}
	}
	return TRUE;
}

////////////////////////////////////////////////////////////////////////////////
//
//	char * GIFFile::GIFReadInputFile(CString path, 
//								  UINT *width, 
//								  UINT *height)
//
//	Give a path.
//	It will read the .GIF at that path, allocate a buffer and give you
//	an RGB buffer back. width and height are modified to reflect the image dim's.
//
//	m_GIFErrorText is modifed to reflect error status
//
//

BYTE * GIFFile::GIFReadFileToRGB(TString path, 
								  UINT *width, 
								  UINT *height)
{
	UCHAR			buf[16];
	UCHAR			c;
	UCHAR			localColorMap[3][MAXCOLORMAPSIZE];
	int				useGlobalColormap;
	int				bitPixel;
	int				imageCount	=0;
	char			version[4];
	FILE 			*fd;          
	int 			w=0;
	int				h=0;	

	if (path=="") {
		m_GIFErrorText="No Name Given";
		return NULL;
	}

	BYTE *bigBuf;

	fd=fopen(path,"rb");
	if (fd==NULL) {                       
		m_GIFErrorText="Cant open GIF :\n" + path;
		return NULL;
	}

	// read GIF file header
	if (!ReadOK(fd,buf,6)) {
		m_GIFErrorText="Error reading GIF Magic #\n"+path;
		fclose(fd);
		return NULL;
	}
	
	// need the string "GIF" in the header
	if (strncmp((char *)buf,"GIF",3)!=0) {
		m_GIFErrorText="Error, "+path+" is not a valid .GIF file";
		fclose(fd);
		return NULL;
	}	

	strncpy(version,(char *)(buf+3),3);
	version[3]='\0';

	// only handle v 87a and 89a
	if ((strcmp(version,"87a")!=0)&&(strcmp(version,"89a")!=0)) {
		m_GIFErrorText="Error, Bad GIF Version number";
		fclose(fd);
		return NULL;
	}	

	// screen description
	if (!ReadOK(fd,buf,7)) {
		m_GIFErrorText="Error, failed to GIF read screen descriptor.\nGiving up";
		fclose(fd);
		return NULL;
	}

	GifScreen.Width		=	LM_to_uint((UCHAR)buf[0],(UCHAR)buf[1]);
	GifScreen.Height	=	LM_to_uint((UCHAR)buf[2],(UCHAR)buf[3]);
	GifScreen.BitPixel	=	2 << ((UCHAR)buf[4] & 0x07);
	GifScreen.ColorResolution = ((((UCHAR)buf[4] & 0x70) >> 3) + 1);
	GifScreen.BackGround=	(UCHAR)buf[5];									// background color...
	GifScreen.AspectRatio=	(UCHAR)buf[6];
            

	// read colormaps
	if (BitSet((UCHAR)buf[4],LOCALCOLORMAP)) {
		if (!ReadColorMap(fd,GifScreen.BitPixel,GifScreen.ColorMap)) {
			m_GIFErrorText="Error reading GIF colormap";
			fclose(fd);
			return NULL;                                             
		}
	}

	// non-square pixels, so what?	
	if ((GifScreen.AspectRatio!=0 ) && (GifScreen.AspectRatio!=49)) {
		m_GIFErrorText="Non-square pixels in GIF image.\nIgnoring that fact...";
	}

	// there can be multiple images in a GIF file... uh?
	// what the hell do we do with multiple images?
	// so, we'll be interested in just the first image, cause we're lazy

	for(;;) {	
		// read a byte;
		if (!ReadOK(fd,&c,1)) {
			m_GIFErrorText="Unexpected EOF in GIF.\nGiving up";
			fclose(fd);
			return NULL; 
		}
	
		// image terminator
		if (c==';') {
		}
	
		if (c=='!') {
			if (!ReadOK(fd,&c,1)) {
				m_GIFErrorText="Error on extension read.\nGiving up";
				fclose(fd);
				return NULL;       
			}
			DoExtension(fd,c);
			continue;
		}
	
		if (c!=',') {
			// Ignoring c
			continue;
		}
	
		// read image header
		if (!ReadOK(fd,buf,9)) {
			m_GIFErrorText="Error on dimension read\nGiving up";
			fclose(fd);
			return NULL;                     
		}
	
		useGlobalColormap=!BitSet((UCHAR)buf[8],LOCALCOLORMAP);
	
		bitPixel=1<<(((UCHAR)buf[8]&0x07)+1);
	
	    // let's see if we have enough mem to continue?

		long bufsize;

		if ((int)buf[5]>4) {
			//AfxMessageBox("This GIF file claims to be > 2000 bytes wide!",MB_OK | MB_ICONINFORMATION);
		}
		if ((int)buf[7]>4) {
			//AfxMessageBox("This GIF file claims to be > 2000 bytes high!",MB_OK | MB_ICONINFORMATION);
		}		                                                       
		
		w=LM_to_uint((UCHAR)buf[4],(UCHAR)buf[5]);		
		h=LM_to_uint((UCHAR)buf[6],(UCHAR)buf[7]);
		
		if ((w<0) || (h<0)) {
			m_GIFErrorText="Negative image dimensions!\nGiving up";
			fclose(fd);
			return NULL;
		}
				
		bufsize=(long)w*(long)h;
		bufsize*=3;
		bigBuf= (BYTE *) new char [bufsize];
		
		if (bigBuf==NULL) {
			m_GIFErrorText="Out of Memory in GIFRead";
			fclose(fd);
			return NULL;
		}
			
	
		if (!useGlobalColormap) {
			if (!ReadColorMap(fd,bitPixel,localColorMap)) {
				m_GIFErrorText="Error reading GIF colormap\nGiving up";
				delete [] bigBuf;
				fclose(fd);
				return NULL;                     
			}
	
		 	//read image
			if (!ReadImage(fd, bigBuf, w, h, localColorMap, BitSet((UCHAR)buf[8],INTERLACE))) {
				m_GIFErrorText="Error reading GIF file\nLocalColorMap\nGiving up";
				delete [] bigBuf;
				fclose(fd);
				return NULL; 				
			}
		} else {
			if (!ReadImage(fd, bigBuf, w, h, GifScreen.ColorMap, BitSet((UCHAR)buf[8],INTERLACE))) {
				m_GIFErrorText="Error reading GIF file\nGIFScreen Colormap\nGiving up";
				delete [] bigBuf;
				fclose(fd);
				return NULL; 			
			}
		}
		break;
	}

	*width=w;
	*height=h;
		
	fclose(fd);
	return bigBuf;
}

static int ReadColorMap(FILE *fd,
			int number,
			UCHAR buffer[3][MAXCOLORMAPSIZE])
{
	int 	i;
	UCHAR rgb[3];
	
	for (i=0;i < number; ++i) {
		if (!ReadOK(fd,rgb,sizeof(rgb))) {
			return FALSE;
		}
		
		buffer[CM_RED][i]=rgb[0];
		buffer[CM_GREEN][i]=rgb[1];
		buffer[CM_BLUE][i]=rgb[2];
	}	
	return TRUE;
}

static int DoExtension(FILE *fd, int label)
{
	static char buf[256];
	char	*str;

	switch(label) {
	case 0x01  :
		str="Plain Text Ext";
		break;
	case 0xff :
		str= "Appl ext";
		break;
	case 0xfe :
		str="Comment Ext";
		while (GetDataBlock(fd,(UCHAR *)buf)!=0) {
			//AfxMessageBox(buf, MB_OK | MB_ICONINFORMATION);
		}
		return FALSE;
		break;
	case 0XF9 :
		str="Graphic Ctrl Ext";
		(void)GetDataBlock(fd,(UCHAR *)buf);
		Gif89.disposal	=(buf[0]>>2)		&0x7;
		Gif89.inputFlag	=(buf[0]>>1)		&0x1;
		Gif89.delayTime	=LM_to_uint(buf[1],buf[2]);
		if ((buf[0]&0x1)!=0)
			Gif89.transparent=buf[3];
	
		while (GetDataBlock(fd,(UCHAR *)buf)!=0);
		return FALSE;
		break;
	default :
		str=buf;
		sprintf(buf,"UNKNOWN (0x%02x)",label);
		break;
	}
	
	while (GetDataBlock(fd,(UCHAR *)buf)!=0);

	return FALSE;
}

int ZeroDataBlock=FALSE;
static int GetDataBlock(FILE *fd, UCHAR *buf)
{
	UCHAR count;

	if (!ReadOK(fd,&count,1)) {
		//m_GIFErrorText="Error in GIF DataBlock Size";
		return -1;
	}

	ZeroDataBlock=count==0;

	if ((count!=0) && (!ReadOK(fd,buf,count))) {
		//m_GIFErrorText="Error reading GIF datablock";
		return -1;
	}
	return count;
}

static int GetCode(FILE *fd, int code_size, int flag)
{
	static UCHAR buf[280];
	static int curbit, lastbit, done, last_byte;
	int i,j,ret;
	UCHAR count;

	if (flag) {
		curbit=0;
		lastbit=0;
		done=FALSE;
		return 0;
	}

	if ((curbit+code_size) >=lastbit) {
		if (done) {
			if (curbit >=lastbit) {
				//m_GIFErrorText="Ran off the end of my bits";
				return 0;
			}
			return -1;
		}
		buf[0]=buf[last_byte-2];	
		buf[1]=buf[last_byte-1];

		if ((count=GetDataBlock(fd,&buf[2]))==0)
			done=TRUE;

		last_byte=2+count;

		curbit=(curbit - lastbit) + 16;

		lastbit = (2+count)*8;
	}
	ret=0;
	for (i=curbit,j=0; j<code_size;++i,++j)
		ret|=((buf[i/8]&(1<<(i% 8)))!=0)<<j;

	curbit+=code_size;

	return ret;
}

static int LZWReadByte(FILE *fd, int flag, int input_code_size)
{
	static int fresh=FALSE;
	int code, incode;
	static int code_size, set_code_size;
	static int max_code, max_code_size;
	static int firstcode, oldcode;
	static int clear_code, end_code;

	static unsigned short  next[1<<MAX_LZW_BITS];
	static UCHAR  vals[1<<MAX_LZW_BITS];
	static UCHAR  stack [1<<(MAX_LZW_BITS+1)];
	static UCHAR  *sp;
	
	register int i;

	if (flag) {
		set_code_size=input_code_size;
		code_size=set_code_size+1;
		clear_code=1<<set_code_size;
		end_code = clear_code+1;
		max_code = clear_code+2;
		max_code_size=2*clear_code;

		GetCode(fd,0,TRUE);

		fresh=TRUE;
	
		for(i=0;i<clear_code;++i) {
			next[i]=0;
			vals[i]=i;
		}

		for (;i<(1<<MAX_LZW_BITS);++i)
			next[i]=vals[0]=0;
	
		sp=stack;

		return 0;
		} else if (fresh) {
			fresh=FALSE;
			do {
				firstcode=oldcode=GetCode(fd,code_size,FALSE);
			} while (firstcode==clear_code);
			return firstcode;
		}

		if (sp > stack)
			return *--sp;

		while ((code= GetCode(fd,code_size,FALSE)) >=0) {
			if (code==clear_code) {
				for (i=0;i<clear_code;++i) {
					next[i]=0;
					vals[i]=i;
				}
				for (;i<(1<<MAX_LZW_BITS);++i)	
					next[i]=vals[i]=0;
				code_size=set_code_size+1;
				max_code_size=2*clear_code;
				max_code=clear_code+2;
				sp=stack;
				firstcode=oldcode=GetCode(fd,code_size,FALSE);
				return firstcode;
			} else if (code==end_code) {
				int count;
				UCHAR buf[260];
		
				if (ZeroDataBlock)
					return -2;

				while ((count=GetDataBlock(fd,buf)) >0);

				if (count!=0)
					//AfxMessageBox("Missing EOD in GIF data stream (common occurrence)",MB_OK);
				return -2;	
			}

			incode = code;

			if (code >= max_code) {
				*sp++=firstcode;
				code=oldcode;
			}

			while (code >=clear_code) {
				*sp++=vals[code];
				if (code==(int)next[code]) {
					//m_GIFErrorText="Circular table entry, big GIF Error!";
					return -1;
				}
				code=next[code];
			}

			*sp++ = firstcode=vals[code];

			if ((code=max_code) <(1<<MAX_LZW_BITS)) {
				next[code]=oldcode;
				vals[code]=firstcode;
				++max_code;
				if ((max_code >=max_code_size) &&
					(max_code_size < (1<<MAX_LZW_BITS))) {
					 max_code_size*=2;
					++code_size;
				}
			}

		oldcode=incode;

		if (sp > stack)
			return *--sp;
	}
	return code;
}   


static BOOL ReadImage(	FILE *fd,
						BYTE  * bigMemBuf,
						int width, int height,
						UCHAR cmap[3][MAXCOLORMAPSIZE],
						int interlace)
{
	UCHAR c;
	int color;
	int xpos=0, ypos=0, pass=0;
	long curidx;

	if (!ReadOK(fd,& c,1)) {
		return FALSE;
	}

	if (LZWReadByte(fd,TRUE,c)<0) {
		return FALSE;
	}
	
	while ((color=LZWReadByte(fd,FALSE,c))>=0) {
        curidx=(long)xpos+(long)ypos*(long)width;
        curidx*=3;     
        
		*(bigMemBuf+curidx)=cmap[0][color];
		*(bigMemBuf+curidx+1)=cmap[1][color];
		*(bigMemBuf+curidx+2)=cmap[2][color];				

		++xpos;
		if (xpos==width) {
			xpos=0;
			if (interlace) {
				switch (pass) {
				case 0:
				case 1:
					ypos+=8; break;
				case 2:
					ypos+=4; break;
				case 3:
					ypos+=2; break;
				}

				if (ypos>=height) {
					++pass;
					switch (pass) {
					case 1: ypos=4;break;
					case 2: ypos=2;break;
					case 3: ypos=1;break;
					default : goto fini;
					}
				}
			} else {
				++ypos;
			}
		}
		if (ypos >=height)
			break;
	}

fini :

	if (LZWReadByte(fd,FALSE,c)>=0) {

	}
	return TRUE;
}

//
//	gets image dimensions
//	returns -1,-1 on bad read
//

#include <io.h>

void GIFFile::GIFGetDimensions(TString path, UINT *width, UINT *height)
{
	UCHAR			buf[16];
	UCHAR			c;
	char			version[4];
	FILE 			*fd;          
	int 			w=0;
	int				h=0;	
	
	// _access() Determine file-access permission. #include <io.h>

	if (_access(path,0)!=0) {
		*width=(UINT)-1;*height=(UINT)-1;
		return;
	}
	
	fd=fopen(path,"rb");
	if (fd==NULL) {                       
		*width=(UINT)-1;*height=(UINT)-1;
		return;
	}

	// read GIF file header
	if (!ReadOK(fd,buf,6))
		goto bail;

	// need the string "GIF" in the header
	if (strncmp((char *)buf,"GIF",3)!=0)
		goto bail;

	strncpy(version,(char *)(buf+3),3);
	version[3]='\0';

	// only handle v 87a and 89a
	if ((strcmp(version,"87a")!=0)&&(strcmp(version,"89a")!=0))
		goto bail;

	// screen description
	if (!ReadOK(fd,buf,7))
		goto bail;

	GifScreen.Width		=	LM_to_uint((UCHAR)buf[0],(UCHAR)buf[1]);
	GifScreen.Height	=	LM_to_uint((UCHAR)buf[2],(UCHAR)buf[3]);
	GifScreen.BitPixel	=	2<<((UCHAR)buf[4]&0x07);
	GifScreen.ColorResolution=((((UCHAR)buf[4]&0x70)>>3)+1);
	GifScreen.BackGround=	(UCHAR)buf[5];									// background color...
	GifScreen.AspectRatio=	(UCHAR)buf[6];
            

	// read colormaps
	if (BitSet((UCHAR)buf[4],LOCALCOLORMAP))
		if (!ReadColorMap(fd,GifScreen.BitPixel,GifScreen.ColorMap))
			goto bail;                                         
 
	if (!ReadOK(fd,&c,1))
		goto bail;

	if (c!=',')
		goto bail;

	// read image header
	if (!ReadOK(fd,buf,9))
		goto bail;                    
	
	*width=LM_to_uint((UCHAR)buf[4],(UCHAR)buf[5]);		
	*height=LM_to_uint((UCHAR)buf[6],(UCHAR)buf[7]);
		
	if ((*width<0) || (*height<0))
		goto bail;

	// good
	fclose(fd);
	return;

bail:      
	// bad
	fclose(fd);
	*width=(UINT)-1;*height=(UINT)-1;
	return;
}
                                                                          


////////////
//
//	GIF writing section
//
////////////

/* a code_int must be able to hold 2**BITS values of type int, and also -1
 */


static int 				Width, Height;
static int				curx, cury;
static long 			CountDown;
static unsigned long	cur_accum = 0;
static int				cur_bits = 0;
static unsigned char	*buffer;


/*
 * Bump the 'curx' and 'cury' to point to the next pixel
 */
void GIFFile::BumpPixel()
{
    /*
     * Bump the current X position
     */
    ++curx;

    if( curx == Width ) {
        curx = 0;
		++cury;
    }
}

/*******************************************************************************
* Return the next pixel from the image
*******************************************************************************/

int GIFFile::GIFNextPixel( )
{
    unsigned long index;
    int r;
    
    if( CountDown == 0 )
        return EOF;

    --CountDown;
    
    index= (unsigned long)curx + (unsigned long)cury * (unsigned long)Width;
    
	r = *(buffer+index);

    BumpPixel();

    return r;
}

/*******************************************************************************
*	here's the entry point. 
*	file ptr, screen width, height, background color, bits per pixel and
*	arrays of color values (0-255)
*******************************************************************************/
BOOL GIFFile::GIFWriteFileFrom256Color(unsigned char  * buf,
							TString name,
							int GWidth, 
							int GHeight,
							int BackGround,
							int Red[], int Green[], int Blue[])
{                       
	FILE *fp;
	int B;
	int RWidth, RHeight;
	int LeftOfs, TopOfs;
	int Resolution;
	int ColorMapSize;
	int InitCodeSize;
	int i;
	int BitsPerPixel = 8;

	fp=fopen(name,"wb");
	if (fp==NULL) {
		m_GIFErrorText="Can't open GIF for writing";
		return FALSE;
	}
	
	ColorMapSize = 1 << BitsPerPixel;

	buffer=buf;

	RWidth = Width = GWidth;
	RHeight = Height = GHeight;
	LeftOfs = TopOfs = 0;

	cur_accum = 0;
	cur_bits = 0;

	Resolution = BitsPerPixel;

	CountDown = (long)Width * (long) Height;

	if (BitsPerPixel <=1)
	InitCodeSize=2;
	else
	InitCodeSize = BitsPerPixel;

	curx = cury =0;

	fwrite("GIF87a",1,6,fp);

	Putword(RWidth,fp);
	Putword(RHeight,fp);

	B=0x80;
	
	B |=(Resolution -1) << 5;

	B |=(BitsPerPixel - 1);

	fputc(B,fp);

	fputc(BackGround,fp);
	
	fputc(0,fp);

	for(i=0; i<ColorMapSize; ++i) {
		fputc(Red[i],fp);
		fputc(Green[i],fp);
		fputc(Blue[i],fp);
	}

	fputc(',',fp);

	Putword(LeftOfs,fp);
	Putword(TopOfs,fp);
	Putword(Width,fp);
	Putword(Height,fp);

	fputc(0x00,fp);

    /*
     * Write out the initial code size
     */
    fputc( InitCodeSize, fp );
    /*
     * Go and actually compress the data
     */

    compress(  InitCodeSize+1, fp);

    /*
     * Write out a Zero-length packet (to end the series)
     */
    fputc( 0, fp );

    /*
     * Write the GIF file terminator
     */
    fputc( ';', fp );

    /*
     * And close the file
     */
    fclose( fp );

	return TRUE;
}

/*******************************************************************************
 * Write out a word to the GIF file
*******************************************************************************/

void GIFFile::Putword(int w, FILE *fp )
{
    fputc( w & 0xff, fp );
    fputc( (w / 256) & 0xff, fp );
}


/***************************************************************************
 *
 *  GIFCOMPR.C       - GIF Image compression routines
 *
 *  Lempel-Ziv compression based on 'compress'.  GIF modifications by
 *  David Rowley (mgardi@watdcsu.waterloo.edu)
 *
 ***************************************************************************/

/*
 * General DEFINEs
 */

#define BITS    12

#define HSIZE  5003            /* 80% occupancy */

typedef        unsigned char   char_type;

/*
 *
 * GIF Image compression - modified 'compress'
 *
 * Based on: compress.c - File compression ala IEEE Computer, June 1984.
 *
 * By Authors:  Spencer W. Thomas       (decvax!harpo!utah-cs!utah-gr!thomas)
 *              Jim McKie               (decvax!mcvax!jim)
 *              Steve Davies            (decvax!vax135!petsd!peora!srd)
 *              Ken Turkowski           (decvax!decwrl!turtlevax!ken)
 *              James A. Woods          (decvax!ihnp4!ames!jaw)
 *              Joe Orost               (decvax!vax135!petsd!joe)
 *
 */

static int n_bits;                        /* number of bits/code */
static int maxbits = BITS;                /* user settable max # bits/code */
static code_int maxcode;                  /* maximum code, given n_bits */
static code_int maxmaxcode = (code_int)1 << BITS; /* should NEVER generate this
code */

#define MAXCODE(n_bits)        (((code_int) 1 << (n_bits)) - 1)

static count_int htab [HSIZE];
static unsigned short codetab [HSIZE];
#define HashTabOf(i)       htab[i]
#define CodeTabOf(i)    codetab[i]

static code_int free_ent = 0;                  /* first unused entry */

/*
 * block compression parameters -- after all codes are used up,
 * and compression rate changes, start over.
 */
static int clear_flg = 0;

/*
 * compress pixels to GIF packets
 *
 * Algorithm:  use open addressing double hashing (no chaining) on the
 * prefix code / next character combination.  We do a variant of Knuth's
 * algorithm D (vol. 3, sec. 6.4) along with G. Knott's relatively-prime
 * secondary probe.  Here, the modular division first probe is gives way
 * to a faster exclusive-or manipulation.  Also do block compression with
 * an adaptive reset, whereby the code table is cleared when the compression
 * ratio decreases, but after the table fills.  The variable-length output
 * codes are re-sized at this point, and a special CLEAR code is generated
 * for the decompressor.  Late addition:  construct the table according to
 * file size for noticeable speed improvement on small files.  Please direct
 * questions about this implementation to ames!jaw.
 */

static int g_init_bits;
static FILE* g_outfile;

static int ClearCode;
static int EOFCode;

/*******************************************************************************
*
*******************************************************************************/

void GIFFile::compress( int init_bits, FILE* outfile)
{
    register long fcode;
    register code_int i /* = 0 */;
    register int c;
    register code_int ent;
    register code_int disp;
    register int hshift;

    /*
     * Set up the globals:  g_init_bits - initial number of bits
     *                      g_outfile   - pointer to output file
     */
    g_init_bits = init_bits;
    g_outfile = outfile;

    /*
     * Set up the necessary values
     */
    clear_flg = 0;
    maxcode = MAXCODE(n_bits = g_init_bits);

    ClearCode = (1 << (init_bits - 1));
    EOFCode = ClearCode + 1;
    free_ent = ClearCode + 2;

    char_init();

    ent = GIFNextPixel( );

    hshift = 0;
    for ( fcode = (long) HSIZE;  fcode < 65536L; fcode *= 2L )
        ++hshift;
    hshift = 8 - hshift;                /* set hash code range bound */

    cl_hash( (count_int) HSIZE);            /* clear hash table */

    output( (code_int)ClearCode );

    while ( (c = GIFNextPixel( )) != EOF ) {	/* } */

        fcode = (long) (((long) c << maxbits) + ent);
        i = (((code_int)c << hshift) ^ ent);    /* xor hashing */

        if ( HashTabOf (i) == fcode ) {
            ent = CodeTabOf (i);
            continue;
        } else if ( (long)HashTabOf (i) < 0 )      /* empty slot */
            goto nomatch;
        disp = HSIZE - i;           /* secondary hash (after G. Knott) */
        if ( i == 0 )
            disp = 1;
probe:
        if ( (i -= disp) < 0 )
            i += HSIZE;

        if ( HashTabOf (i) == fcode ) {
            ent = CodeTabOf (i);
            continue;
        }
        if ( (long)HashTabOf (i) > 0 )
            goto probe;
nomatch:
        output ( (code_int) ent );
        ent = c;
        if ( free_ent < maxmaxcode ) {	/* } */
            CodeTabOf (i) = free_ent++; /* code -> hashtable */
            HashTabOf (i) = fcode;
        } else
                cl_block();
    }
    /*
     * Put out the final code.
     */
    output( (code_int)ent );
    output( (code_int) EOFCode );
}

/*****************************************************************
 * TAG( output )
 *
 * Output the given code.
 * Inputs:
 *      code:   A n_bits-bit integer.  If == -1, then EOF.  This assumes
 *              that n_bits =< (long)wordsize - 1.
 * Outputs:
 *      Outputs code to the file.
 * Assumptions:
 *      Chars are 8 bits long.
 * Algorithm:
 *      Maintain a BITS character long buffer (so that 8 codes will
 * fit in it exactly).  Use the VAX insv instruction to insert each
 * code in turn.  When the buffer fills up empty it and start over.
 */

static unsigned long masks[] = { 0x0000, 0x0001, 0x0003, 0x0007, 0x000F,
                                  0x001F, 0x003F, 0x007F, 0x00FF,
                                  0x01FF, 0x03FF, 0x07FF, 0x0FFF,
                                  0x1FFF, 0x3FFF, 0x7FFF, 0xFFFF };

void GIFFile::output( code_int  code)
{
    cur_accum &= masks[ cur_bits ];

    if( cur_bits > 0 )
        cur_accum |= ((long)code << cur_bits);
    else
        cur_accum = code;

    cur_bits += n_bits;

    while( cur_bits >= 8 ) {
        char_out( (unsigned int)(cur_accum & 0xff) );
        cur_accum >>= 8;
        cur_bits -= 8;
    }

    /*
     * If the next entry is going to be too big for the code size,
     * then increase it, if possible.
     */
   if ( free_ent > maxcode || clear_flg ) {
        if( clear_flg ) {
            maxcode = MAXCODE (n_bits = g_init_bits);
            clear_flg = 0;
        } else {
            ++n_bits;
            if ( n_bits == maxbits )
                maxcode = maxmaxcode;
            else
                maxcode = MAXCODE(n_bits);
        }
    }
	
	if( code == EOFCode ) {
        /*
         * At EOF, write the rest of the buffer.
         */
        while( cur_bits > 0 ) {
            char_out( (unsigned int)(cur_accum & 0xff) );
            cur_accum >>= 8;
            cur_bits -= 8;
        }
	
        flush_char();
	
        fflush( g_outfile );
	
        if( ferror( g_outfile ) ) {
			//AfxMessageBox("Write Error in GIF file",MB_OK);
			TRACE("Write Error in GIF file",MB_OK);
		}
    }
}

void GIFFile::cl_block()
{
	cl_hash((count_int)HSIZE);
	free_ent=ClearCode+2;
	clear_flg=1;

	output((code_int)ClearCode);
}

void GIFFile::cl_hash(register count_int hsize)

{
	register count_int *htab_p = htab+hsize;

	register long i;
	register long m1 = -1L;

	i = hsize - 16;

	do {
		*(htab_p-16)=m1;
		*(htab_p-15)=m1;
		*(htab_p-14)=m1;
		*(htab_p-13)=m1;
		*(htab_p-12)=m1;
		*(htab_p-11)=m1;
		*(htab_p-10)=m1;
		*(htab_p-9)=m1;
		*(htab_p-8)=m1;
		*(htab_p-7)=m1;
		*(htab_p-6)=m1;
		*(htab_p-5)=m1;
		*(htab_p-4)=m1;
		*(htab_p-3)=m1;
		*(htab_p-2)=m1;
		*(htab_p-1)=m1;
		
		htab_p-=16;
	} while ((i-=16) >=0);

	for (i+=16;i>0;--i)
		*--htab_p=m1;
}

/*******************************************************************************
*	GIF specific
*******************************************************************************/

static int a_count;

void GIFFile::char_init()
{
	a_count=0;
}

static char accum[256];

void GIFFile::char_out(int c)
{
	accum[a_count++]=c;
	if (a_count >=254)
		flush_char();
}

void GIFFile::flush_char()
{
	if (a_count > 0) {
		fputc(a_count,g_outfile);
		fwrite(accum,1,a_count,g_outfile);
		a_count=0;
	}
}


/////////////////////////////////////////////////////////////////////////////
// CBaseImage


BOOL CImage::SaveGIF(LPCTSTR lpszFileName)
{

	BYTE	*tmp = NULL;

	int		nWidth, nHeight;
	int		nRealWidth;
	int		i;
	int red[256], blue[256], green[256];

	nWidth = GetWidth();
	nHeight = GetHeight();
	nRealWidth = GetRealWidth();

	if( GetBitCount() == 8 )
	{
		tmp = new BYTE[ nWidth * nHeight * GetBitCount() ];
		if( tmp == NULL ) return FALSE;

		CPixelPtr ptr(*this);

		for( i=0; i<nHeight ; i++)
			memcpy( tmp+(nWidth*i), ptr[i], nWidth);

//		CPalette	*pPal;
		PALETTEENTRY	palEntry[256];

		ASSERT(0);
/*		if( GetPalette() != NULL )
		{
			GetPalette()->GetPaletteEntries( 0, 256, palEntry );
		
			// figure out the palette entries
			for (i=0;i<256;i++) {
				red[i]=palEntry[i].peRed;
				green[i]=palEntry[i].peGreen;
				blue[i]=palEntry[i].peBlue;
			}
		}
		else*/
		{
//			AfxMessageBox("256 color image must have palette!");
			TRACE("256 color image must have palette!");
			return FALSE;
		}
	}
	else if( GetBitCount() == 1 )
	{
		tmp = new BYTE[ nWidth * nHeight];
		if( tmp == NULL ) return FALSE;

		BYTE	*pBuf;
		BYTE* lpBinary;
		LPSTR lpDIBHdr;
		int		nPixelPerByte;
		int		nWidthDW;
		int		x, y;

		pBuf = tmp;

		lpDIBHdr  = (LPSTR) ::GlobalLock( GetHandle() );
		lpBinary = (BYTE*) ::FindDIBBits(lpDIBHdr);

		nPixelPerByte = 8;
		nWidthDW = nRealWidth;

		BYTE* ptmp = lpBinary + nWidthDW*(nHeight-1);
		for( y=0; y<nHeight; y++ )
		{
			for( x=0; x<nWidth; x++ )
			{
				//if( lpBinary[x/nPixelPerByte] & ( 0x80 >> (x%nPixelPerByte)) )
				if( ptmp[x/nPixelPerByte] & ( 0x80 >> (x%nPixelPerByte)) )
					*pBuf++ = 255;
				else
					*pBuf++ = 0;
				
			}
			//lpBinary -= nWidthDW;
			ptmp -= nWidthDW;
		}

		::GlobalUnlock( lpDIBHdr );

//		CPalette	*pPal;
		PALETTEENTRY	palEntry[256];

		ASSERT(0);
		/*if( GetPalette() != NULL )
		{
			GetPalette()->GetPaletteEntries( 0, 256, palEntry );
		
			// figure out the palette entries
			for ( i=0; i<256; i++ ) {
				red[i]= palEntry[i].peRed;
				green[i]=palEntry[i].peGreen;
				blue[i]=palEntry[i].peBlue;
			}
		}
		else*/
		{
//			AfxMessageBox("8bit image must have palette!");
			TRACE("8bit image must have palette!");
			return FALSE;
		}

/*
//		For Check
		CImage	grayImg;

		grayImg.Create( nWidth, nHeight, 8);
		CPixelPtr	ptr(grayImg);

		for( y=0; y<nHeight ; y++)
			memcpy( ptr[y], tmp+(nWidth*y), nWidth);
		
		theApp.MakeNewDoc( (CMyImage*) &grayImg );
*/
	}
	else if( GetBitCount() == 24 )
	{
//		AfxMessageBox("Not Implemented for 24 bit color!");
		TRACE("Not Implemented for 24 bit color!");
		return FALSE;

		CColorPixelPtr ptr(*this);
	}

	GIFFile theGifThing;
	if (!theGifThing.GIFWriteFileFrom256Color(tmp,
							lpszFileName,
							nWidth, 
							nHeight,
							0,		// background color
							red, green, blue)) {
//		AfxMessageBox(theGifThing.m_GIFErrorText);
		TRACE(theGifThing.m_GIFErrorText);
	}

	if( tmp ) delete[] tmp;

	return TRUE;
}

BOOL CImage::LoadGIF(LPCTSTR lpszFileName)
{
	GIFFile theGifThing;
	int		nWidth, nHeight;
	BYTE *tmp;
	
	// read the GIF to a packed buffer of RGB bytes
	tmp=theGifThing.GIFReadFileToRGB( lpszFileName, 
						(UINT*)&nWidth, 
						(UINT*)&nHeight);
	if (tmp==NULL) {
//		AfxMessageBox(theGifThing.m_GIFErrorText);
		TRACE(theGifThing.m_GIFErrorText);
		return FALSE;
	}

	// do this before DWORD-alignment!!!
	// swap red and blue for display
	theGifThing.BGRFromRGB(tmp, nWidth, nHeight);
/*
	// 256 palette의 경우만 따로 처리하는 것은 실패
	if( GifScreen.BitPixel == 256 )
	{
		// 256 Palette을 사용하는 경우
		Create( nWidth, nHeight, 8 );

		LPSTR lpDIBHdr  = (LPSTR) ::GlobalLock( GetHandle() );
		LPSTR lpPalette = lpDIBHdr + sizeof(BITMAPINFO);

		memcpy ( lpPalette, GifScreen.ColorMap[0], 256*3 );

		GlobalUnlock( lpDIBHdr );

		CPixelPtr ptr(*this);

		int x,y;

		for( y=0; y<nHeight; y++)
		{
			for( x=0; x<nWidth; x++)
			{
				// 3을 곱한 이유는 tmp가 true color이기 때문
				ptr[y][x] = tmp[y*(3*nHeight)+3*x];
			}
		}
	}
*/
	if ( Create( nWidth, nHeight, 24 ) == FALSE )
	{
//		AfxMessageBox("Can't create image in LoadGIF!");
		TRACE("Can't create image in LoadGIF!");
		delete[] tmp;
		return FALSE;
	}

	CColorPixelPtr	ptr( *this );

	for( int i=0; i<nHeight ; i++)
		memcpy( ptr[i], tmp+(nWidth*3*i), nWidth*3);
//	}

	delete[] tmp;

	return TRUE;
}


