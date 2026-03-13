
/*******************************************
 * Ownership: Most code copied from Zhongqiang Richard Ren @ RAP-lab@SJTU
 *******************************************/


#ifndef UTILS_H_
#define UTILS_H_

#include <vector>
#include "vec_type.hpp"
#include <Eigen/Dense>

namespace grasp{

#define DEBUG_RIGID_BODY_MOTION 0
#define EPSILON 1e-14
#define THRESH 1e-4

double Min(const double& a, const double& b) ;

double Max(const double& a, const double& b) ;

double Sigmoid(const double& w) ;

double InvSigmoid(const double& y) ;

void Relu(const double& x, double& y) ;

double Relu(const double& x) ;

void Relu (const Eigen::MatrixXd& x, Eigen::MatrixXd& y) ;

/**
 * @brief Return a SO(3), rotation about x-axis for theta rad.
 */
std::vector<std::vector<double> > Rot3D_X(double theta) ;

std::vector<std::vector<double> > Rot3D_Y(double theta) ;

std::vector<std::vector<double> > Rot3D_Z(double theta) ;

/**
 * @brief return a matrix of size rxc, and all entries are init to val.
 */
std::vector<std::vector<double> > MatInit(int r, int c, double val = 0.0);
/**
 * @brief return an identitiy matrix of size rxr.
 */
std::vector<std::vector<double> > MatEye(int r);

/**
 * @brief Matrix addition, subtraction, scaling.
 */
std::vector< std::vector<double> > operator+(
	const std::vector< std::vector<double> >& a, const std::vector< std::vector<double> >& b) ;
std::vector< std::vector<double> > operator-(
	const std::vector< std::vector<double> >& a, const std::vector< std::vector<double> >& b) ;
std::vector< std::vector<double> > operator*(
	const std::vector< std::vector<double> >& a, const double& b) ;
std::vector< std::vector<double> > operator*(
	const double& b, const std::vector< std::vector<double> >& a) ;

/**
 * @brief Matrix absolute values
 */
std::vector< std::vector<double> > Abs(const std::vector< std::vector<double> >& a);

/**
 * @brief Matrix sum
 */
double Sum(
	const std::vector< std::vector<double> >& a, int r1=-1, int r2=-1, int c1=-1, int c2=-1);

/**
 * @brief Matrix trace
 */
double Trace(const std::vector< std::vector<double> >& a);


/**
 * @brief Copy the mat1[r11:r12, c11:c12] to mat2[r21:r22, c21:c22].
 */
bool CopySubMat(const std::vector<std::vector<double> >& mat1, int r11, int r12, int c11, int c12,
	std::vector<std::vector<double> >& mat2, int r21, int r22, int c21, int c22);

/**
 * @brief Get mat1[r11:r12, c11:c12].
 */
std::vector<std::vector<double> > GetSubMat(
	const std::vector<std::vector<double> >& mat1, int r11, int r12, int c11, int c12);
/**
 * @brief Get sub-vector vec1[a:b].
 */
std::vector<double> GetSubVec(const std::vector<double>& vec1, int a, int b);

/**
 * @brief Convert a vector of length n to a nx1 matrix (i.e., column vector).
 */
std::vector< std::vector<double> > Vec2Mat(const std::vector<double>& vec1);

/**
 * @brief Convert nx1 matrix (i.e., column vector) to a vector of length n.
 */
std::vector<double> Mat2Vec(const std::vector< std::vector<double> >& m);

/**
 * @brief Return a SE(3) given SO(3) and the translation vector.
 */
std::vector<std::vector<double> > MatSE3(const std::vector<std::vector<double> >& rmat, const std::vector<double>& p);

/**
 * @brief Return the position vector from the given SE(3) matrix.
 */
std::vector<double> GetPvecFromSE3(const std::vector<std::vector<double> >& q);

/**
 * @brief Return the rotation matrix SO(3) from the given SE(3) matrix.
 */
std::vector<std::vector<double> > GetSO3FromSE3(const std::vector<std::vector<double> >& q);

/**
 * @brief Matrix multiplication.
 */
std::vector< std::vector<double> > MatMul(const std::vector<std::vector<double> >& m1, const std::vector<std::vector<double> >& m2);
std::vector< std::vector<double> > operator*(const std::vector< std::vector<double> >& m1, const std::vector< std::vector<double> >& m2) ;

/**
 * @brief Matrix times scalar.
 */
// std::vector<std::vector<double> > MatScale(const std::vector<std::vector<double> >& m1, double a);

/**
 * @brief Matrix transpose.
 */
std::vector<std::vector<double> > MatTranspose(const std::vector<std::vector<double> >& m1);

/**
 * @brief Inverse a SE(3) matrix.
 */
std::vector<std::vector<double> > InvSE3(const std::vector<std::vector<double> >& mat) ;

/**
 * @brief Convert a vector of length 3 to so(3), a skew-symmetric matrix of size 3x3.
 */
std::vector<std::vector<double> > HatW(const std::vector<double>& w) ;
std::vector<double> UnhatW(const std::vector<std::vector<double> >& m);

/**
 * @brief Convert a twist vector of size 6 to a se(3).
 */
std::vector<std::vector<double> > HatS(const std::vector<double>& s) ;
std::vector<double> UnhatS(const std::vector<std::vector<double> >& m);

/**
 * @brief Given a 4x4 SE(3).
 * Return a 6x6 Adjoint matrix 
 */
std::vector<std::vector<double> > AdjMat(const std::vector<std::vector<double> >& tf) ;

/**
 * @brief Exponential map of so(3) to SO(3). Rodrigues' formula
 */
std::vector<std::vector<double> > ExpW(const std::vector<double>& w, double theta) ;

/**
 * @brief Log map of SO(3) to so(3).
 */
bool LogW(const std::vector<std::vector<double> > m, std::vector<double>* w, double* theta) ;

/**
 * @brief Exponential map of se(3) to SE(3). s is a normalized twist, and theta is the amount of motion.
 */
std::vector<std::vector<double> > ExpS(const std::vector<double>& s0, double theta) ;

/**
 * @brief Log map of SE(3) to se(3).
 */
bool LogS(const std::vector<std::vector<double> > m, std::vector<double>* w, double* theta) ;


/**
 * @brief Added by Yu: Elementwise product of two matrices.
 */
std::vector<std::vector<double>> cwiseMatMul(const std::vector<std::vector<double>>& m1, const std::vector<std::vector<double>>& m2) ;
/**
 * @brief Added by Yu: Vector sum
 */
double Sum(const std::vector<double>& a, int r1=-1, int r2=-1);

/**
 * @brief Added by Yu: Save a matrix to a file.
 */
void saveToFile(const std::vector<std::vector<double>>& mat, const std::string& filename) ;
/**
 * @brief Added by Yu: Save a vector to a file.
 */
void saveToFile(const std::vector<double>& vec, const std::string& filename) ;
/**
 * @brief Added by Yu: Read a vector from a file.
 */
std::vector<double> readFromFile(const std::string& filename) ;
/**
 * @brief Added by Yu: Read a matrix from a file.
 */
std::vector<std::vector<double>> readFromFile2D(const std::string& filename) ;
/**
 * @brief Added by Yu: Interpolate between two matrices.
 */
std::vector<std::vector<double>> Interpolate(const std::vector<std::vector<double>>& m1, 
											 const std::vector<std::vector<double>>& m2, 
											 const double alpha,
											 const std::string & method = "linear") ;
/**
 * @brief Added by Yu: Compute distance matrix between two matrices.
 */
std::vector<std::vector<double>> DistMat(const std::vector<std::vector<double>>& A, 
											const std::vector<std::vector<double>>& B) ;
/**
 * @brief Added by Yu: Compute distance matrix between two matrices.
 */
Eigen::MatrixXd DistMat(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B);

void DistMat(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, 
             Eigen::MatrixXd& D, Eigen::MatrixXd& dx, 
             Eigen::MatrixXd& dy, Eigen::MatrixXd& dz);

std::vector<double> Eigen2Vec(const Eigen::MatrixXd& matrix1d) ;
std::vector<std::vector<double>> Eigen2Vec2d(const Eigen::MatrixXd& matrix) ;

Eigen::MatrixXd Vec2d2Eigen(const std::vector<std::vector<double>>& vec2d) ;

std::vector<double> flatten_vector(const std::vector<std::vector<double>>& vec2d) ;

void smoothRowwiseMin(const Eigen::MatrixXd& A, Eigen::MatrixXd& result, const double alpha);

void smoothRowwiseMin(const Eigen::MatrixXd& A, Eigen::MatrixXd& result, 
						Eigen::MatrixXd& grad, const double alpha);

void smoothRowwiseMinDerivative(const Eigen::MatrixXd& A, Eigen::MatrixXd& grad, double alpha);


void Euler2Mat(const std::vector<double>& euler6d, std::vector<std::vector<double>>& tr_mat) ;

std::vector<double> transformMatrixToQuaternion(const std::vector<std::vector<double>>& mat) ;

std::vector<double> generateRandomVector(const int& dim) ;

Eigen::MatrixXd sampleMatrix(const Eigen::MatrixXd& mat, const int& sample_rate) ;

std::string getExecutableDirectory() ;

} // end namespace yu

#endif  // UTILS_H_
