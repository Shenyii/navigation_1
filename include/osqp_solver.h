#ifndef OSQP_SOLVER_H
#define OSQP_SOLVER_H

#include <iostream>
#include <Eigen/Dense>
#include "osqp.h"

using namespace std;
using namespace Eigen;

class Osqp_Solver {
public:
    Osqp_Solver() {}

    ~Osqp_Solver() {}

    bool solveQpProblam(Matrix<double, Dynamic, Dynamic> H, Matrix<double, Dynamic, Dynamic> g,
                        Matrix<double, Dynamic, Dynamic> A,Matrix<double, Dynamic, Dynamic> lb, Matrix<double, Dynamic, Dynamic> ub,
                        Matrix<double, Dynamic, 1>& solve) {
        solve.resize(H.rows(), 1);

        c_float* P_x;
        c_int   P_nnz;
        c_int*   P_i;
        c_int*   P_p;
        c_float* q;
        c_float* A_x;
        c_int   A_nnz;
        c_int*   A_i;
        c_int*   A_p;
        c_float* l;
        c_float* u;
        c_int n = H.rows();
        c_int m = A.rows();
        upperMatrixToCsc(&P_x, P_nnz, &P_i, &P_p, H);
        matrixToCsc(&A_x, A_nnz, &A_i, &A_p, A);
        q = new c_float[n];
        for(int i = 0; i < n; i++) q[i] = g(i, 0);
        l = new c_float[m];
        u = new c_float[m];
        for(int i = 0; i < m; i++) {
            l[i] = lb(i, 0);
            u[i] = ub(i, 0);
        }

        // Exitflag
        c_int exitflag = 0;

        // Workspace structures
        OSQPWorkspace *work;
        OSQPSettings  *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
        OSQPData      *data     = (OSQPData *)c_malloc(sizeof(OSQPData));
        OSQPSolution test1;

        // Populate data
        if (data) {
            data->n = n;
            data->m = m;
            data->P = csc_matrix(data->n, data->n, P_nnz, P_x, P_i, P_p);
            data->q = q;
            data->A = csc_matrix(data->m, data->n, A_nnz, A_x, A_i, A_p);
            data->l = l;
            data->u = u;
        }

        // Define solver settings as default
        if (settings) osqp_set_default_settings(settings);

        // Setup workspace
        exitflag = osqp_setup(&work, data, settings);

        // Solve Problem
        osqp_solve(work);

        if(work->info->status_val != 1) {
            cout << "Can't solve the problam." << endl;
            solve.resize(0, 1);
            return false;
        }

        for(int i = 0; i < solve.rows(); i++) {
            solve(i, 0) = work->solution->x[i];
        }

        // Clean workspace
        osqp_cleanup(work);
        if (data) {
            if (data->A) c_free(data->A);
            if (data->P) c_free(data->P);
            c_free(data);
        }
        if (settings)  c_free(settings);

        delete [] P_x;
        delete [] P_i;
        delete [] P_p;
        delete [] q;
        delete [] A_x;
        delete [] A_i;
        delete [] A_p;
        delete [] l;
        delete [] u;

        return true;
    }

private:
    void matrixToCsc(c_float** M_x, c_int& M_nnz, c_int** M_i, c_int** M_p, Matrix<double, Dynamic, Dynamic> M) {
        M_nnz = 0;
        for(int j = 0; j < M.cols(); j++) {
            for(int i = 0; i < M.rows(); i++) {
                if(M(i, j) != 0) {
                    M_nnz++;
                }
            }
        }
        *M_x = new c_float[M_nnz];
        int ptr1 = 0;
        for(int j = 0; j < M.cols(); j++) {
            for(int i = 0; i < M.rows(); i++) {
                if(M(i, j) != 0) {
                    (*M_x)[ptr1] = M(i, j);
                    ptr1++;
                }
            }
        }
        int ptr2 = 0;
        *M_i = new c_int[M_nnz];
        *M_p = new c_int[M.cols() + 1];
        (*M_p)[0] = 0;
        for(int j = 0; j < M.cols(); j++) {
            int ptr3 = 0;
            for(int i = 0; i < M.rows(); i++) {
                if(M(i, j) != 0) {
                    (*M_i)[ptr2] = i;
                    ptr2++;
                    ptr3++;
                }
            }
            (*M_p)[j + 1] = (*M_p)[j] + ptr3;
        }
    }

    int upperMatrixToCsc(c_float** M_x, c_int& M_nnz, c_int** M_i, c_int** M_p, Matrix<double, Dynamic, Dynamic> M) {
        if(M.cols() != M.rows()) {
            cout << "The matrix isn't symmetry." << endl << "The program will error!" << endl;
            return 0;
        }
        for(int j = 0; j < M.cols(); j++) {
            for(int i = 0; i < M.rows(); i++) {
                if(i > j) {
                    M(i, j) = 0;
                }
            }
        }
        matrixToCsc(M_x, M_nnz, M_i, M_p, M);
        return 1;
    }
};

#endif