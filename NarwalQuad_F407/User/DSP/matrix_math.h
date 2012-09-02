/*
 * matrix_math.h
 *
 * Created: 9/17/2011 1:49:56 PM
 *  Author: GrubyGrub
 */


#ifndef MATRIX_MATH_H_
#define MATRIX_MATH_H_

#define MAXSIZE 7

//============================================================================//
//                        MATH LIBRARY from RotoMotion                        //
//============================================================================//
/*----------------------------------------------------------------------------*/
/* This will add vector a with vector b and return vector c
/* c(n,1) + a(n,1) + b(n,1)
/*----------------------------------------------------------------------------*/
static inline void VVadd( float *a, float *b, float *c, int n )
{
   int  i ;


   for( i=0 ; i<n ; ++i ) c[i] = a[i] + b[i] ;
}

/*----------------------------------------------------------------------------*/
/* This will zero out the matrix A
/* A = zeros (n,m)
/*----------------------------------------------------------------------------*/
static inline void Minit( float *A, int n, int m )
{
   int  i, j ;


   for(i=0 ; i<n ; ++i ) for( j=0 ; j<m ; ++j ) A[i*m+j] = 0.0 ;
}

/*----------------------------------------------------------------------------*/
/* This will generate an identity matrix I
/* A(n,n) = eye (n)
/*----------------------------------------------------------------------------*/
static inline void eye ( float *I, int n )
{
   int  i ;


   Minit( I, n, n ) ; for( i=0 ; i<n ; ++i ) I[i*n+i] = 1.0 ;
}

/*----------------------------------------------------------------------------*/
/* This will multiply scalar value s will all elements of matrix A(m,n)
/* placing result into matrix B(m,n)
/* B(m,n) = s. * A(m,n)
/*----------------------------------------------------------------------------*/
static inline void sMmult( float s, float *A, float *B, int m, int n )
{
   int  i, j ;


   for( i=0 ; i<m ; ++i ) for ( j=0 ; j<n ; ++j ) B[i*n+j] = s * A[i*n+j] ;
}

/*----------------------------------------------------------------------------*/
/* This will multiply matrix A and matrix B return in matrix C
/* C(m,p) = A(m,n) * B(n,p)
/*----------------------------------------------------------------------------*/
static inline void MMmult( float *A, float *B, float *C, int m, int n, int p )
{
   float  s ;
   int     i, j, k ;


   for( i=0 ; i<m ; ++i )
   {
      for( j=0 ; j<p ; ++j )
      {
         s = 0.0 ;

         for( k=0 ; k<n ; ++k ) s += A[i*n+k] * B[k*p+j] ;

         C[i*p+j] = s ;
      }
   }
}

/*----------------------------------------------------------------------------*/
/* This will multiply matrix A and vector b return in vector c
/* c(m,1) = A(m,n) * b(n,1)
/*----------------------------------------------------------------------------*/
static inline void MVmult( float *A, float *b, float *c, int m, int n )
{
   float  s ;
   int     i, j ;


   for( j=0; j<m; ++j )
   {
      s = 0.0 ;

      for( i=0 ; i<n ; ++i ) s += A[j*n+i] * b[i] ;

      c[j] = s ;
   }
}

/*----------------------------------------------------------------------------*/
/* This will multiply vector a and matrix B return in vector c
/* c(1,n) = a(1,m) * B(m,n)
/*----------------------------------------------------------------------------*/
static inline void VMmult( float *a, float *B, float *c, int m, int n )
{
   float  s ;
   int     i, j ;


   for( i=0 ; i<n ; ++i )
   {
      s = 0.0 ;

      for( j=0 ; j<m ; ++j ) s += a[j] * B[j*n+i] ;

      c[i] = s ;
   }
}

/*----------------------------------------------------------------------------*/
/* This will transpose a matrix/vector A return in matrix/vector B  //checked//
/* B(n,m) = transpose A(m,n)
/*----------------------------------------------------------------------------*/
static inline void transpose( float *A, float *B, int m, int n )
{
   int  i, j, k, l ;

   k = 0 ; // initialize B index

   /* For each row of B */
   for( i=0 ; i<n ; ++i )
   {
      l = i ; // initialize A index to upper element of colum i

      /* For each colum of B */
      for ( j=0 ; j<m ; ++j )
      {
         B[k] = A[l] ;

         k += 1 ; // increment B index
         l += n ; // increment A row #
      }
   }
}

/*----------------------------------------------------------------------------*/
/* This will add matrix A with matrix B and return matrix C //checked//
/* C(n,m) = A(n,m) + B(n,m)
/*----------------------------------------------------------------------------*/
static inline void MMadd( float *A, float *B, float *C, int m, int n )
{
   int  i, j, k;

   k = 0 ;
   for( i=0 ; i<n ; ++i )
     for( j=0 ; j<m ; ++j ) {C[k] = A[k] + B[k] ; k += 1 ;}
}

/*----------------------------------------------------------------------------*/
/* This will subtract matrix A from matrix B and return matrix C //checked//
/* C(n,m) = A(n,m) - B(n,m)
/*----------------------------------------------------------------------------*/
static inline void MMsub(float *A, float *B, float *C, int m, int n )
{
   int  i, j, k;

   k = 0 ;
   for( i=0 ; i<n ; ++i )
     for( j=0 ; j<m ; ++j ) {C[k] = A[k] - B[k] ; k += 1 ;}
}

/*----------------------------------------------------------------------------*/
/* This will perform LU decomp on matrix A return matrix L and matrix U
/* LU(A(n,n)) => L(n,n) and U(n,n)
/*----------------------------------------------------------------------------*/
static inline void LU( float *A, float *L, float *U, int n )
{
   float  Acopy [MAXSIZE*MAXSIZE] ;
   int     i, j, k ;


   /* Copy A matrix */
   for( i=0 ; i<n ; ++i ) for(j=0; j<n; ++j) Acopy[i*n+j] = A[i*n+j] ;


   /* Decompose */
   for( k=0 ; k<n-1 ; ++k )
   {
      for( i=k+1 ; i<n ; ++i )
      {
         Acopy[i*n+k] = Acopy[i*n+k] / Acopy[k*n+k] ;

         for( j=k+1 ; j<n ; ++j ) Acopy[i*n+j] -= Acopy[i*n+k] * Acopy[k*n+j] ;
      }
   }


   /* Extract the L matrix */
   eye( L, n ) ;
   for( j=0 ; j<n-1 ; ++j )
     for ( i=j+1 ; i<n ; ++i ) L[i*n+j] = Acopy[i*n+j] ;


   /* Extract the U matrix */
   Minit( U, n, n ) ;
   for( i=0; i<n; ++i ) for ( j=i; j<n; ++j ) U[i*n+j] = Acopy[i*n+j] ;
}

/*----------------------------------------------------------------------------*/
/* This will take column c from matrix A(m,n) place it into vector a(m)
/*----------------------------------------------------------------------------*/
static inline void Mcol( float *A, float *a, int c, int m, int n )
{
   int  i ;


   for( i=0; i<m; ++i ) a[i] = A[i*n+c] ;
}

/*----------------------------------------------------------------------------*/
/* This will take vector a(m) and place it into column c of matrix A(m,n)
/*----------------------------------------------------------------------------*/
static inline void Vcol( float *a, float *A, int c, int m, int n )
{
   int  i ;


   for( i=0; i<m; ++i ) A[i*n+c] = a[i] ;
}

/*----------------------------------------------------------------------------*/
/* This will solve A*x = b, where matrix A is upper triangular
/* A(n,n)*x(n,1) = b(n,1)
/*----------------------------------------------------------------------------*/
static inline void solveupper( float *A, float *b, float *x, int n )
{
   int  i, j, p ;


   p = n + 1 ;

   for( i=1; i<=n; ++i )
   {
      x[p-i-1] = b[p-i-1] ;

      for( j=(p+1-i); j<=n; ++j ) x[p-i-1] -= A[(p-i-1)*n+(j-1)]*x[j-1] ;

      x[p-i-1] = x[p-i-1] / A[(p-i-1)*n+(p-i-1)] ;
   }
}

/*----------------------------------------------------------------------------*/
/* This will solve A*x = b, where matrix A is lower triangular
/* A(n,n)*x(n,1) = b(n,1)
/*----------------------------------------------------------------------------*/
static inline void solvelower( float *A, float *b, float *x, int n )
{
   int  i, j ;


   for( i=1; i<=n; ++i )
   {
      x[i-1] = b[i-1] ;

      for(j=1; j<=i-1; ++j ) x[i-1] = x[i-1] - A[(i-1)*n+(j-1)]*x[j-1] ;

      x[i-1] = x[i-1]/A[(i-1)*n+(i-1)] ;
   }
}

/*----------------------------------------------------------------------------*/
/* This will perform the inverse on matrix A return in matrix B
/* inv(A(n,n)) = B(n,n)
/*----------------------------------------------------------------------------*/
static inline void inv( float *A, float *B, int n )
{
   float  identCol [MAXSIZE] ;
   float  ident    [MAXSIZE*MAXSIZE] ;
   float  L        [MAXSIZE*MAXSIZE] ;
   float  U        [MAXSIZE*MAXSIZE] ;
   float  invUcol  [MAXSIZE] ;
   float  invLcol  [MAXSIZE] ;
   float  invU     [MAXSIZE*MAXSIZE] ;
   float  invL     [MAXSIZE*MAXSIZE] ;

   float  detA ;
   int     i ;


   /* Case n = 1 */
   /* ---------- */
   if( n == 1 )
   {
      B[0] = 1.0 / A[0] ;

      return ;
   }


   /* Case n = 2 */
   /* ---------- */
   if( n == 2 )
   {
      detA = A[0]*A[3] - A[1]*A[2] ;

      B[0] =  A[3] / detA ;
      B[1] = -A[1] / detA ;
      B[2] = -A[2] / detA ;
      B[3] =  A[0] / detA ;

      return ;
   }

   /* General case */
   /* ------------ */
   /* Perform LU decomposition on A */
   eye( ident, n ) ; LU( A, L, U, n ) ;

   for (i=0; i<n; ++i)
   {
      /* Separates the ith column */
      Mcol( ident, identCol, i, n, n ) ;

      solveupper( U, identCol, invUcol, n ) ;
      solvelower( L, identCol, invLcol, n ) ;

      /* Place invUcol in ith column of invU */
      Vcol( invUcol, invU, i, n, n ) ;

      /* Place invLcol in ith column of invL */
      Vcol( invLcol, invL, i, n, n ) ;
   }

   /* inv(A) = inv(U)*inv(L) */
   MMmult( invU, invL, B, n, n, n ) ;
}

/*----------------------------------------------------------------------------*/
static inline void MatrixMultiply(float *M1, float *M2)
/*----------------------------------------------------------------------------*/
/* M1 = M1*M2                                                                 */
/*----------------------------------------------------------------------------*/
{
   /* Declarations */
   float  M_buff[9];
   int    i;


   for (i=0;i<9;i++) M_buff[i] = M1[i];

   M1[0] = M_buff[0]*M2[0] + M_buff[1]*M2[3] + M_buff[2]*M2[6];
   M1[3] = M_buff[3]*M2[0] + M_buff[4]*M2[3] + M_buff[5]*M2[6];
   M1[6] = M_buff[6]*M2[0] + M_buff[7]*M2[3] + M_buff[8]*M2[6];

   M1[1] = M_buff[0]*M2[1] + M_buff[1]*M2[4] + M_buff[2]*M2[7];
   M1[4] = M_buff[3]*M2[1] + M_buff[4]*M2[4] + M_buff[5]*M2[7];
   M1[7] = M_buff[6]*M2[1] + M_buff[7]*M2[4] + M_buff[8]*M2[7];

   M1[2] = M_buff[0]*M2[2] + M_buff[1]*M2[5] + M_buff[2]*M2[8];
   M1[5] = M_buff[3]*M2[2] + M_buff[4]*M2[5] + M_buff[5]*M2[8];
   M1[8] = M_buff[6]*M2[2] + M_buff[7]*M2[5] + M_buff[8]*M2[8];
}

/*----------------------------------------------------------------------------*/
static inline void TransposeMatrix(float *M1, float *M2)
/*----------------------------------------------------------------------------*/
/* M2 = Transpose M1                                                          */
/*----------------------------------------------------------------------------*/
{
   M2[0] = M1[0]; M2[1] = M1[3]; M2[2] = M1[6];
   M2[3] = M1[1]; M2[4] = M1[4]; M2[5] = M1[7];
   M2[6] = M1[2]; M2[7] = M1[5]; M2[8] = M1[8];
}

/*----------------------------------------------------------------------------*/
static inline void MatrixVector(float *M, float *V1, float *V2)
/*----------------------------------------------------------------------------*/
/* V2 = M*V1                                                                  */
/*----------------------------------------------------------------------------*/
{
   V2[0] = M[0]*V1[0] + M[1]*V1[1] + M[2]*V1[2];
   V2[1] = M[3]*V1[0] + M[4]*V1[1] + M[5]*V1[2];
   V2[2] = M[6]*V1[0] + M[7]*V1[1] + M[8]*V1[2];
}


/*----------------------------------------------------------------------------*/
static inline void VectorDotProduct(float *V1, float *V2, float *Scalar_Product)
/*----------------------------------------------------------------------------*/
/* Scalar_Product = V1.V2                                                     */
/*----------------------------------------------------------------------------*/
{
   /* Declarations */
   int    i;


   *Scalar_Product = 0.;
   for (i=0;i<3;i++) *Scalar_Product += V1[i]*V2[i];
}

/*----------------------------------------------------------------------------*/
static inline void VectorCross(float *V1, float *V2, float *V3)
/*----------------------------------------------------------------------------*/
/* V3 = V1xV2                                                                 */
/*----------------------------------------------------------------------------*/
{
   V3[0] = V1[1]*V2[2] - V1[2]*V2[1];
   V3[1] = V1[2]*V2[0] - V1[0]*V2[2];
   V3[2] = V1[0]*V2[1] - V1[1]*V2[0];
}

/*----------------------------------------------------------------------------*/
static inline void VectorAdd(float *V1, float *V2, float *V3)
/*----------------------------------------------------------------------------*/
/* V3 = V1+V2                                                                 */
/*----------------------------------------------------------------------------*/
{
   /* Declarations */
   int    i;


   for (i=0;i<3;i++) V3[i] = V1[i]+V2[i];
}

/*----------------------------------------------------------------------------*/
static inline void VectorScale(float *V1, float *K, float *V2)
/*----------------------------------------------------------------------------*/
/* V3 = V1+V2                                                                 */
/*----------------------------------------------------------------------------*/
{
   /* Declarations */
   int    i;


   for (i=0;i<3;i++) V2[i] = V1[i] * (*K);
}
/*

static inline void PrintMatrix(float * A, int C, int R)
{
	int i, j;
	for (i = 0; i < R; i++)
	{
		for (j = 0; j < C; j++)
		{
			printf("%f ", (float)A[i*C+j]);
		}
		printf("\n");
	}
	printf("\n");
}
*/
#endif /* MATRIX_MATH_H_ */
