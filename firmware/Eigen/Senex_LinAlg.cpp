#include <Arduino.h>
#include <math.h>

/* INPUT: A - array of pointers to rows of a square matrix having dimension N
 *        Tol - small tolerance number to detect failure when the matrix is near degenerate
 * OUTPUT: Matrix A is changed, it contains a copy of both matrices L-E and U as A=(L-E)+U such that P*A=L*U.
 *        The permutation matrix is not stored as a matrix, but in an integer vector P of size N+1 
 *        containing column indexes where the permutation matrix has "1". The last element P[N]=S+N, 
 *        where S is the number of row exchanges needed for determinant computation, det(P)=(-1)^S    
 */
bool LUPDecompose(float **A, int N, float Tol, int *P) {

    int i, j, k, imax; 
    float maxA, *ptr, absA;

    for (i = 0; i <= N; i++)
        P[i] = i; //Unit permutation matrix, P[N] initialized with N

    for (i = 0; i < N; i++) {
        maxA = 0.0;
        imax = i;

        for (k = i; k < N; k++)
            if ((absA = abs(A[k][i])) > maxA) { 
                maxA = absA;
                imax = k;
            }

        if (maxA < Tol) return true; //failure, matrix is degenerate

        if (imax != i) {
            //pivoting P
            j = P[i];
            P[i] = P[imax];
            P[imax] = j;

            //pivoting rows of A
            ptr = A[i];
            A[i] = A[imax];
            A[imax] = ptr;

            //counting pivots starting from N (for determinant)
            P[N]++;
        }

        for (j = i + 1; j < N; j++) {
            A[j][i] /= A[i][i];

            for (k = i + 1; k < N; k++)
                A[j][k] -= A[j][i] * A[i][k];
        }
    }

    return false;  //decomposition done 
}

/* INPUT: A,P filled in LUPDecompose; b - rhs vector; N - dimension
 * OUTPUT: x - solution vector of A*x=b
 */
void LUPSolve(float **A, int *P, float *b, int N, float *x) {

    for (int i = 0; i < N; i++) {
        x[i] = b[P[i]];

        for (int k = 0; k < i; k++)
            x[i] -= A[i][k] * x[k];
    }

    for (int i = N - 1; i >= 0; i--) {
        for (int k = i + 1; k < N; k++)
            x[i] -= A[i][k] * x[k];

        x[i] /= A[i][i];
    }
}

/* INPUT: A,P filled in LUPDecompose; N - dimension
 * OUTPUT: IA is the inverse of the initial matrix
 */
// void LUPInvert(float **A, int *P, int N, float **IA) {
  
//     for (int j = 0; j < N; j++) {
//         for (int i = 0; i < N; i++) {
//             IA[i][j] = P[i] == j ? 1.0 : 0.0;

//             for (int k = 0; k < i; k++)
//                 IA[i][j] -= A[i][k] * IA[k][j];
//         }

//         for (int i = N - 1; i >= 0; i--) {
//             for (int k = i + 1; k < N; k++)
//                 IA[i][j] -= A[i][k] * IA[k][j];

//             IA[i][j] /= A[i][i];
//         }
//     }
// }

// /* INPUT: A,P filled in LUPDecompose; N - dimension. 
//  * OUTPUT: Function returns the determinant of the initial matrix
//  */
// float LUPDeterminant(float **A, int *P, int N) {

//     float det = A[0][0];

//     for (int i = 1; i < N; i++)
//         det *= A[i][i];

//     return (P[N] - N) % 2 == 0 ? det : -det;
// }

void qrEigenDecomposition(float A[3][3], int maxIter, float tol, float eigenvalues[3], float eigenvectors[3][3]) {    
    // Start with identity eigenvectors
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            eigenvectors[i][j] = (i == j) ? 1.0 : 0.0;

    for (int iter = 0; iter < maxIter; iter++) {
        // --- QR decomposition (Gram-Schmidt) ---
        float v1[3] = {A[0][0], A[1][0], A[2][0]};
        float v2[3] = {A[0][1], A[1][1], A[2][1]};
        float v3[3] = {A[0][2], A[1][2], A[2][2]};

        // u1 = normalized v1
        float norm1 = sqrt(v1[0]*v1[0] + v1[1]*v1[1] + v1[2]*v1[2]);
        float u1[3] = {v1[0]/norm1, v1[1]/norm1, v1[2]/norm1};

        // u2 = normalized (v2 - proj_u1(v2))
        float dot12 = u1[0]*v2[0] + u1[1]*v2[1] + u1[2]*v2[2];
        float temp2[3] = {v2[0] - dot12*u1[0],
                           v2[1] - dot12*u1[1],
                           v2[2] - dot12*u1[2]};
        float norm2 = sqrt(temp2[0]*temp2[0] + temp2[1]*temp2[1] + temp2[2]*temp2[2]);
        float u2[3] = {temp2[0]/norm2, temp2[1]/norm2, temp2[2]/norm2};

        // u3 = normalized (v3 - proj_u1(v3) - proj_u2(v3))
        float dot13 = u1[0]*v3[0] + u1[1]*v3[1] + u1[2]*v3[2];
        float dot23 = u2[0]*v3[0] + u2[1]*v3[1] + u2[2]*v3[2];
        float temp3[3] = {v3[0] - dot13*u1[0] - dot23*u2[0],
                           v3[1] - dot13*u1[1] - dot23*u2[1],
                           v3[2] - dot13*u1[2] - dot23*u2[2]};
        float norm3 = sqrt(temp3[0]*temp3[0] + temp3[1]*temp3[1] + temp3[2]*temp3[2]);
        float u3[3] = {temp3[0]/norm3, temp3[1]/norm3, temp3[2]/norm3};

        // Q = [u1 u2 u3]
        float Q[3][3] = {{u1[0], u2[0], u3[0]},
                          {u1[1], u2[1], u3[1]},
                          {u1[2], u2[2], u3[2]}};

        // R = Q^T * A
        float QT[3][3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                QT[i][j] = Q[j][i];

        float R[3][3] = {{0}};
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                for (int k = 0; k < 3; k++)
                    R[i][j] += QT[i][k] * A[k][j];

        // A_new = R * Q
        float newA[3][3] = {{0}};
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                for (int k = 0; k < 3; k++)
                    newA[i][j] += R[i][k] * Q[k][j];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                A[i][j] = newA[i][j];

        // Update eigenvectors = eigenvectors * Q
        float temp[3][3] = {{0}};
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                for (int k = 0; k < 3; k++)
                    temp[i][j] += eigenvectors[i][k] * Q[k][j];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                eigenvectors[i][j] = temp[i][j];

        // Check convergence
        float off = fabs(A[0][1]) + fabs(A[0][2]) +
                     fabs(A[1][0]) + fabs(A[1][2]) +
                     fabs(A[2][0]) + fabs(A[2][1]);
        if (off < tol) break;
    }

    // Eigenvalues are diagonal entries
    for (int i = 0; i < 3; i++) eigenvalues[i] = A[i][i];
}








// Fit z = c00 + c10*x + c01*y + c11*x*y + c02*y^2 + c12*x*y^2 + c03*y^3
// degree 1 in x, degree 3 in y (7 coefficients)
void fit_x1_y3(double *x, double *y, double *z, int n, double *coeff_out) {

    const int K = 7;  // number of coefficients

    double ATA[K][K]; // normal matrix
    double ATz[K];    // right-hand side

    // Initialize ATA and ATz
    for (int i = 0; i < K; ++i) {
        ATz[i] = 0.0;
        for (int j = 0; j < K; ++j)
            ATA[i][j] = 0.0;
    }

    // --------------------------------------
    // Subtract reference offset from z[]
    // --------------------------------------
    for(int i = 0; i < 4; i++) {
        double z1 = z[i * 18];
        for (int j = 0; j < n / 4; j++)
            z[i * 18 + j] -= z1;
    }
    
    // --------------------------------------

    // Build normal equations
    for (int i = 0; i < n; ++i) {
        double xx = x[i];
        double yy = y[i];
        double y2 = yy * yy;
        double y3 = y2 * yy;

        double row[K] = {
            1.0,        // c00
            xx,         // c10
            yy,         // c01
            xx * yy,    // c11
            y2,         // c02
            xx * y2,    // c12
            y3          // c03
        };

        for (int a = 0; a < K; ++a) {
            ATz[a] += row[a] * z[i];
            for (int b = 0; b < K; ++b)
                ATA[a][b] += row[a] * row[b];
        }
    }

    // Build augmented matrix A[K][K+1]
    double A[K][K + 1];
    for (int i = 0; i < K; ++i) {
        for (int j = 0; j < K; ++j)
            A[i][j] = ATA[i][j];
        A[i][K] = ATz[i];
    }

    // Gaussian elimination with pivoting
    for (int col = 0; col < K; ++col) {

        // Find pivot row
        int pivot = col;
        for (int r = col + 1; r < K; ++r)
            if (fabs(A[r][col]) > fabs(A[pivot][col]))
                pivot = r;

        // Check singularity
        if (fabs(A[pivot][col]) < 1e-12) {
            Serial.println("SINGULAR");
            return;
        }

        // Swap pivot into place
        if (pivot != col) {
            for (int c = 0; c <= K; ++c) {
                double tmp = A[pivot][c];
                A[pivot][c] = A[col][c];
                A[col][c] = tmp;
            }
        }

        // Normalize pivot row
        double pv = A[col][col];
        for (int c = col; c <= K; ++c)
            A[col][c] /= pv;

        // Eliminate
        for (int r = 0; r < K; ++r) {
            if (r == col) continue;
            double factor = A[r][col];
            for (int c = col; c <= K; ++c)
                A[r][c] -= factor * A[col][c];
        }
    }

    // Write solution
    for (int i = 0; i < K; ++i)
        coeff_out[i] = A[i][K];
}
