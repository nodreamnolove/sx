#ifndef _MathFunction_H
#define _MathFunction_H

#define    MAXDAFEIPTNUM    4
#define IS_INSIDE(x,y,dx,dy)	( (min(dx,dy) >= max(x,y) || max(dx,dy) <= min(x,y)) ? 0:1 )
#define max(x,y)	(x > y ? x:y)
#define min(x,y)	(x > y ? y:x)
#define MAX_U(x,y)	(abs(x) > abs(y) ? x:y)
#define MIN_U(x,y)	(abs(x) > abs(y) ? y:x)

#endif