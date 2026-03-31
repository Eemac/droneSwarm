/* INPUT: A - array of pointers to rows of a square matrix having dimension N
 *        Tol - small tolerance number to detect failure when the matrix is near degenerate
 * OUTPUT: Matrix A is changed, it contains a copy of both matrices L-E and U as A=(L-E)+U such that P*A=L*U.
 *        The permutation matrix is not stored as a matrix, but in an integer vector P of size N+1 
 *        containing column indexes where the permutation matrix has "1". The last element P[N]=S+N, 
 *        where S is the number of row exchanges needed for determinant computation, det(P)=(-1)^S    
 */
bool LUPDecompose(float **A, int N, float Tol, int *P);

/* INPUT: A,P filled in LUPDecompose; b - rhs vector; N - dimension
 * OUTPUT: x - solution vector of A*x=b
 */
void LUPSolve(float **A, int *P, float *b, int N, float *x);

/* INPUT: A,P filled in LUPDecompose; N - dimension
 * OUTPUT: IA is the inverse of the initial matrix
 */
// void LUPInvert(float **A, int *P, int N, float **IA);

// /* INPUT: A,P filled in LUPDecompose; N - dimension. 
//  * OUTPUT: Function returns the determinant of the initial matrix
//  */
// float LUPDeterminant(float **A, int *P, int N);

void qrEigenDecomposition(float A[3][3], int maxIter, float tol, float eigenvalues[3], float eigenvectors[3][3]);

void fit_x1_y3(double *x, double *y, double *z, int n, double *coeff_out);