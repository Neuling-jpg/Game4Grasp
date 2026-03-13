
/*******************************************
 * Ownership: Most code copied from Zhongqiang Richard Ren @ RAP-lab@SJTU
 *******************************************/

#include "utils/utils.hpp"
#include <iostream>
#include <filesystem>
#include <random>
#include <fstream>
#include <math.h>
#include <Eigen/Dense>


#ifdef _WIN32
    #include <windows.h>
#else
    #include <unistd.h>     // Required for readlink()
    #include <limits.h>     // Required for PATH_MAX
#endif
namespace fs = std::filesystem;

namespace grasp{

double Min(const double& a, const double& b) {
    if (a>b) {return b;}
    else {return a;}
}

double Max(const double& a, const double& b) {
    if (a>b) {return a;}
    else {return b;}
}

double Sigmoid(const double& w) {
    return 1. / (1. + exp(-Min(7., Max(w, -7.))));
}

double InvSigmoid(const double& y) {
    return Min(7., Max(-7., std::log(y/(1-y+EPSILON))));
}

void Relu (const double& x, double& y) {
	y= (x >= 0) ? x : 0.0;
}

double Relu (const double& x) { 
	return (x >= 0) ? x : 0.0; 
}

void Relu (const Eigen::MatrixXd& x, Eigen::MatrixXd& y) {
	y = x.cwiseMax(0.0);
}

std::vector<std::vector<double> > Rot3D_X(double theta){
	std::vector<std::vector<double> > out;
	out.resize(3);
	for (int i = 0; i < 3; i++){
		out[i].resize(3);
	}
	out[0][0] = 1;
	out[0][1] = 0;
	out[0][2] = 0;

	out[1][0] = 0;
	out[1][1] = cos(theta);
	out[1][2] = -sin(theta);

	out[2][0] = 0;
	out[2][1] = sin(theta);
	out[2][2] = cos(theta);
	return out;
};


std::vector<std::vector<double> > Rot3D_Y(double theta){
	std::vector<std::vector<double> > out;
	out.resize(3);
	for (int i = 0; i < 3; i++){
		out[i].resize(3);
	}
	out[0][0] = cos(theta);
	out[0][1] = 0;
	out[0][2] = sin(theta);

	out[1][0] = 0;
	out[1][1] = 1;
	out[1][2] = 0;

	out[2][0] = -sin(theta);
	out[2][1] = 0;
	out[2][2] = cos(theta);
	return out;
};

std::vector<std::vector<double> > Rot3D_Z(double theta){
	std::vector<std::vector<double> > out;
	out.resize(3);
	for (int i = 0; i < 3; i++){
		out[i].resize(3);
	}
	out[0][0] = cos(theta);
	out[0][1] = -sin(theta);
	out[0][2] = 0;

	out[1][0] = sin(theta);
	out[1][1] = cos(theta);
	out[1][2] = 0;

	out[2][0] = 0;
	out[2][1] = 0;
	out[2][2] = 1;
	return out;
};


std::vector<std::vector<double> > MatInit(int r, int c, double val){
	std::vector<std::vector<double> >  out;
	out.resize(r);
	for (int i = 0; i < r; i++){
		out[i].resize(c, val);
	}
	return out;
};

std::vector<std::vector<double> > MatEye(int r){
	auto out = MatInit(r,r,0);
	for (int i = 0; i < r; i++){
		out[i][i] = 1;
	}
	return out;
};

std::vector< std::vector<double> > operator+(
	const std::vector< std::vector<double> >& a, const std::vector< std::vector<double> >& b) 
{
	auto out = MatInit(a.size(), a[0].size(), 0);
	for (int i = 0; i < a.size(); i++){
		for (int j = 0; j < a[0].size(); j++){
			out[i][j] = a[i][j] + b[i][j];
		}
	}
	return out;
};

std::vector< std::vector<double> > operator-(
	const std::vector< std::vector<double> >& a, const std::vector< std::vector<double> >& b) 
{
	auto out = MatInit(a.size(), a[0].size(), 0);
	for (int i = 0; i < a.size(); i++){
		for (int j = 0; j < a[0].size(); j++){
			out[i][j] = a[i][j] - b[i][j];
		}
	}
	return out;
};

std::vector< std::vector<double> > operator*(
	const std::vector< std::vector<double> >& a, const double& b) 
{
	auto out = MatInit(a.size(), a[0].size(), 0);
	for (int i = 0; i < a.size(); i++){
		for (int j = 0; j < a[0].size(); j++){
			out[i][j] = a[i][j]*b;
		}
	}
	return out;
};


std::vector< std::vector<double> > operator*(
	const double& b, const std::vector< std::vector<double> >& a)
{
	auto out = MatInit(a.size(), a[0].size(), 0);
	for (int i = 0; i < a.size(); i++){
		for (int j = 0; j < a[0].size(); j++){
			out[i][j] = a[i][j]*b;
		}
	}
	return out;
};

std::vector< std::vector<double> > Abs(const std::vector< std::vector<double> >& a){
	auto out = MatInit(a.size(), a[0].size(), 0);
	for (int i = 0; i < a.size(); i++){
		for (int j = 0; j < a[0].size(); j++){
			out[i][j] = abs(a[i][j]);
		}
	}
	return out;
};

double Sum(const std::vector< std::vector<double> >& a, int r1, int r2, int c1, int c2)
{
	double out = 0;
	if ((r1==-1) && (r2==-1) && (c1==-1) && (c2==-1)){
		r1 = 0; c1 = 0;
		r2 = a.size(); c2 = a[0].size();
	}
	for (int i = r1; i < r2; i++){
		for (int j = c1; j < c2; j++){
			out += a[i][j];
		}
	}
	return out;
};

double Sum(const std::vector<double>& a, int r1, int r2)
{
	double out = 0;
	if ((r1==-1) && (r2==-1)){
		r1 = 0; r2 = a.size();
	}
	for (int i = r1; i < r2; i++){
		out += a[i];
	}
	return out;
};

double Trace(const std::vector< std::vector<double> >& a){
	double out;
	for (int i = 0; i < a.size(); i++){
		out += a[i][i];
	}
	return out;
};

bool CopySubMat(const std::vector<std::vector<double> >& mat1, int r11, int r12, int c11, int c12,
	std::vector<std::vector<double> >& mat2, int r21, int r22, int c21, int c22)
{
	if ((r22-r21) != (r12-r11)){
		std::cout << "[ERROR] CopySubMat row size mismatch!" << std::endl;
		throw std::runtime_error("[ERROR]");
		return false;
	}
	if ((c22-c21) != (c12-c11)){
		std::cout << "[ERROR] CopySubMat col size mismatch!" << std::endl;
		throw std::runtime_error("[ERROR]");
		return false;
	}
	for (int i = 0; i < (r22-r21); i++){
		for (int j = 0; j < (c22-c21); j++){
			(mat2)[r21+i][c21+j] = (mat1)[r11+i][c11+j];
		}
	}
	return true;
};

std::vector<std::vector<double> > GetSubMat(
	const std::vector<std::vector<double> >& mat1, int r11, int r12, int c11, int c12)
{
	std::vector<std::vector<double> > out;
	if (r12 < r11 || c12 < c11){
		std::cout << "[ERROR] GetSubMat input block indices are wrong!" << std::endl;
		throw std::runtime_error("[ERROR]");
		return out;
	}
	out = MatInit(r12-r11, c12-c11, 0);
	for (int i = 0; i < r12-r11; i++){
		for (int j = 0; j < c12-c11; j++){
			out[i][j] = mat1[i+r11][j+c11];
		}
	}
	return out;
};


std::vector<double> GetSubVec(const std::vector<double>& vec1, int a, int b){
	std::vector<double> out;
	out.resize(b-a,0);
	for (int i = a; i < b; i++){
		out[i-a] = vec1[i];
	}
	return out;
};

std::vector< std::vector<double> > Vec2Mat(const std::vector<double>& vec1){
	auto out = MatInit(vec1.size(), 1, 0);
	for (int i = 0; i < vec1.size(); i++){
		out[i][0] = vec1[i];
	}
	return out;
};

std::vector<double> Mat2Vec(const std::vector< std::vector<double> >& m){
	std::vector<double> out;
	out.resize(m.size(), 0);
	for (int i = 0; i < m.size(); i++){
		out[i] = m[i][0];
	}
	return out;
};

std::vector<std::vector<double> > MatSE3(
	const std::vector<std::vector<double> >& rmat, const std::vector<double>& p)
{
	auto out = MatEye(4);
	CopySubMat(rmat, 0,3,0,3, out, 0,3,0,3);
	for (int i = 0; i < 3; i++){
		out[i][3] = p.at(i);
	}
	return out;
};

std::vector<double> GetPvecFromSE3(const std::vector<std::vector<double> >& q){
	std::vector<double> out;
	out.resize(3);
	out[0] = (q[0][3]);
	out[1] = (q[1][3]);
	out[2] = (q[2][3]);
	return out;
};

std::vector<std::vector<double> > GetSO3FromSE3(const std::vector<std::vector<double> >& q){
	return GetSubMat(q,0,2,0,2);
};

std::vector<std::vector<double> > MatMul(
	const std::vector<std::vector<double> >& m1, const std::vector<std::vector<double> >& m2)
{
	std::vector<std::vector<double> > out;
	if (m1.size() == 0 || m2.size() == 0){
		std::cout << "[ERROR] MatMul input matrix is of size 0!" << std::endl;
		return out;
	}
	if (m1[0].size() != m2.size()){
		std::cout << "[ERROR] MatMul matrix dimension mismatch!" << std::endl;
		throw std::runtime_error("[ERROR]");
		return out;
	}
	out = MatInit(m1.size(), m2[0].size(), 0);
	for (int i = 0; i < out.size(); i++){
		for (int j = 0; j < out[0].size(); j++){
			out[i][j] = 0;
			for (int k = 0; k < m1[0].size(); k++){
				out[i][j] += m1[i][k] * m2[k][j];
			}
		}
	}
	return out;
};

std::vector< std::vector<double> > operator*(
	const std::vector< std::vector<double> >& m1, const std::vector< std::vector<double> >& m2)
{
	return MatMul(m1,m2);
};


// std::vector<std::vector<double> > MatScale(const std::vector<std::vector<double> >& m1, double a){
// 	std::vector<std::vector<double> > out = m1;
// 	for (int i = 0; i < m1.size(); i++){
// 		for (int j = 0; j < m1[0].size(); j++){
// 			out[i][j] *= a;
// 		}
// 	}
// 	return out;
// };

std::vector<std::vector<double> > MatTranspose(const std::vector<std::vector<double> >& m1){
	std::vector<std::vector<double> > out;
	if (m1.size() == 0){
		std::cout << "[ERROR] MatTranspose input matrix is of size 0!" << std::endl;
		return out;
	}
	out = MatInit(m1[0].size(), m1.size(), 0);
	for (int i = 0; i < out.size(); i++){
		for (int j = 0; j < out[0].size(); j++){
			out[i][j] = m1[j][i];
		}
	}
	return out;
};

std::vector<std::vector<double> > InvSE3(const std::vector<std::vector<double> >& mat) {
	auto out = MatEye(4);
	auto r = GetSubMat(mat,0,3,0,3);
	auto p = GetSubMat(mat,0,3,3,4);
	// std::cout << " r = " << r << " p = " << p << std::endl;
	auto r_inv = MatTranspose(r);
	CopySubMat(r_inv, 0,3,0,3, out, 0,3,0,3);
	auto p2 = (r_inv * p) * (-1);
	// MatScale(MatMul(r_inv, p),-1);
	// std::cout << " p2 = " << p << std::endl;
	CopySubMat(p2, 0,3,0,1, out, 0,3,3,4);
	return out;
};

std::vector<std::vector<double> > HatW(const std::vector<double>& w) {
	auto out = MatInit(3,3,0);
	out[0][1] = -w[2];
	out[0][2] = w[1];
	out[1][0] = w[2];
	out[1][2] = -w[0];
	out[2][0] = -w[1];
	out[2][1] = w[0];
	return out;
};

std::vector<double> UnhatW(const std::vector<std::vector<double> >& m) {
	std::vector<double> out;
	out.resize(3,0);
	out[0] = m[2][1];
	out[1] = m[0][2];
	out[2] = m[1][0];
	return out;
};

std::vector<std::vector<double> > HatS(const std::vector<double>& s) {
	auto out = MatInit(4,4,0);
	CopySubMat(HatW(GetSubVec(s,0,3)),0,3,0,3, out, 0,3,0,3);
	CopySubMat(Vec2Mat(GetSubVec(s,3,6)),0,3,0,1, out, 0,3,3,4);
	return out;
};

std::vector<double> UnhatS(const std::vector<std::vector<double> >& m) {
	std::vector<double> out;
	out.resize(6,0);
	auto w = UnhatW(GetSubMat(m,0,3,0,3));
	auto v = Mat2Vec(GetSubMat(m,0,3,3,4));
	out[0] = w[0]; out[1] = w[1]; out[2] = w[2];
	out[3] = v[0]; out[4] = v[1]; out[5] = v[2];
	return out;
};

std::vector<std::vector<double> > AdjMat(const std::vector<std::vector<double> >& tf) {
	auto out = MatInit(6,6,0);
	CopySubMat(tf,0,3,0,3, out, 0,3,0,3);
	CopySubMat(tf,0,3,0,3, out, 3,6,3,6);
	auto wh = HatW( Mat2Vec(GetSubMat(tf,0,3,3,4)) );
	CopySubMat( wh*GetSubMat(tf,0,3,0,3), 0,3,0,3, out, 3,6,0,3);
	return out;
};


/**
 * @brief Exponential map of so(3) to SO(3). Rodrigues' formula
 */
std::vector<std::vector<double> > ExpW(const std::vector<double>& w0, double theta) {
	auto wh = HatW(w0);
	return MatEye(3) + (wh*sin(theta)) + MatMul(wh,wh)*(1-cos(theta));
};

bool LogW(const std::vector<std::vector<double> > m, std::vector<double>* out, double* theta) {
	out->clear();
	out->resize(3,0);
	if (Sum(Abs(m - MatEye(3))) < EPSILON){
		out->at(2) = 1;
		*theta = 0;
		return true;
	}
	double tr = Trace(m);
	if ( abs(Trace(m)-(-1)) < EPSILON ) {
		*theta = M_PI;
		double den = 1.0/sqrt(2*(1+m[2][2]));
		out->at(0) = m[0][2] / den;
		out->at(1) = m[1][2] / den;
		out->at(2) = (1.0+m[2][2]) / den;
		if (Sum(Abs(ExpW(*out, *theta) - m)) < EPSILON){
			std::cout << "[DEBUG] LogW, tr=-1, case 1/3" << std::endl;
			return true;
		}
		den = 1.0/sqrt(2*(1+m[1][1]));
		out->at(0) = m[0][1] / den;
		out->at(1) = (1.0+m[1][1]) / den;
		out->at(2) = m[2][1] / den;
		if (Sum(Abs(ExpW(*out, *theta) - m)) < EPSILON){
			std::cout << "[DEBUG] LogW, tr=-1, case 2/3" << std::endl;
			return true;
		}
		den = 1.0/sqrt(2*(1+m[0][0]));
		out->at(0) = (1.0+m[0][0]) / den;
		out->at(1) = m[1][0] / den;
		out->at(2) = m[2][0] / den;
		if (Sum(Abs(ExpW(*out, *theta) - m)) < EPSILON){
			std::cout << "[DEBUG] LogW, tr=-1, case 3/3" << std::endl;
			return true;
		}
		std::cout << "[ERROR] LogW goes into unexpected case!" << std::endl;
		std::cout << "m = " << m << std::endl;
		throw std::runtime_error("[ERROR]");
		return false;
	}
	*theta = acos(0.5*(tr-1));
	*out = UnhatW(0.5 / sin(*theta) * (m - MatTranspose(m)));
	return true;
};


std::vector<std::vector<double> > ExpS(const std::vector<double>& s0, double theta) {
	auto out = MatEye(4);
	if ( abs(theta) < EPSILON ){
		return out;
	}
	auto w0 = GetSubVec(s0,0,3);
	auto v0 = GetSubVec(s0,3,6);
	auto w0norm = NormL2(w0);
	if ( abs(w0norm-1) < EPSILON ){
		CopySubMat(ExpW(w0, theta), 0,3,0,3, out, 0,3,0,3);
		auto wh = HatW(w0);
		auto p = (MatEye(3)*theta + (1-cos(theta))*wh + (theta-sin(theta))*(wh*wh)) * Vec2Mat(v0);
		CopySubMat(p,0,3,0,1,out,0,3,3,4);
		return out;
	}
	if (w0norm > EPSILON ){
		std::cout << "[ERROR] ExpS, input screw axis s0" << s0 << " is not a normalized twist" << std::endl;
		throw std::runtime_error("[ERROR]");
	}
	if ( abs(NormL2(v0)-1) > EPSILON ) {
		std::cout << "[ERROR] ExpS, input screw axis s0" << s0 << " is not a normalized twist" << std::endl;
		throw std::runtime_error("[ERROR]");		
	}
	auto p = Vec2Mat( v0*theta );
	CopySubMat(p,0,3,0,1,out,0,3,3,4);
	return out;
};

bool LogS(const std::vector<std::vector<double> > m, std::vector<double>* s0, double* theta) {
	s0->clear();
	s0->resize(6,0);
	auto matR = GetSubMat(m,0,3,0,3);
	auto matP = GetSubMat(m,0,3,3,4);
	// std::cout << " matP = " << matP << std::endl;
	auto vecP = Mat2Vec(matP);
	if (Sum(Abs(matR - MatEye(3))) < EPSILON){
		*theta = NormL2(vecP);
		auto v = vecP * (1/(*theta));
		s0->at(3) = v[0]; s0->at(4) = v[1]; s0->at(5) = v[2];
		return true;
	}
	std::vector<double> w0;
	if (!LogW(matR, &w0, theta)){
		std::cout << "[ERROR] LogS, invoke LogW returns false!" << std::endl;
		throw std::runtime_error("[ERROR]");
		return false;
	}
	double half_theta = (*theta) / 2.0;
	auto wh = HatW(w0);
	auto invG = 1.0/(*theta) * MatEye(3) - 0.5*wh + (1.0/(*theta) - 0.5*cos(half_theta)/sin(half_theta))*(wh*wh);
	auto v = invG*matP;
	// std::cout << " v renrenren = " << v << std::endl;
	s0->at(0) = w0[0]; s0->at(1) = w0[1]; s0->at(2) = w0[2];
	s0->at(3) = v[0][0]; s0->at(4) = v[1][0]; s0->at(5) = v[2][0];
	return true;
};

// Added by Yu
std::vector<std::vector<double>> cwiseMatMul(const std::vector<std::vector<double>>& m1, const std::vector<std::vector<double>>& m2) 
{
    if (m1.size() != m2.size()) throw std::invalid_argument("Matrices must have the same number of rows.");
    if (m1.empty()) return {};

    const size_t cols_m1 = m1[0].size();
    for (const auto& row : m1) {
        if (row.size() != cols_m1) {
            throw std::invalid_argument("m1 is not a rectangular matrix.");
        }
    }

    const size_t cols_m2 = m2[0].size();
    for (const auto& row : m2) {
        if (row.size() != cols_m2) {
            throw std::invalid_argument("m2 is not a rectangular matrix.");
        }
    }

    if (cols_m1 != cols_m2) {
        throw std::invalid_argument("Matrices must have the same number of columns.");
    }

    std::vector<std::vector<double>> result(m1.size(), std::vector<double>(cols_m1));

    for (size_t i = 0; i < m1.size(); ++i) {
        for (size_t j = 0; j < cols_m1; ++j) {
            result[i][j] = m1[i][j] * m2[i][j];
        }
    }

    return result;
}


void saveToFile(const std::vector<double>& data, const std::string& filename) {
    std::ofstream outFile(filename); // Open file for writing

    if (!outFile) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    // Write each double in the vector to the file
    for (const auto& value : data) {
        outFile << value << "\n"; // Each value on a new line
    }

    outFile.close(); // Close the file
}

void saveToFile(const std::vector<std::vector<double>>& data, const std::string& filename) {
    std::ofstream outFile(filename); // Open file for writing

    if (!outFile) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    // Write each double in the vector to the file
    for (const auto& subvec : data) {
        for (const auto& value : subvec) {
            outFile << value << " "; // Each value on a new line
        }
        outFile << "\n";
    }
    
    outFile.close(); // Close the file
}


std::vector<double> readFromFile(const std::string& filename) {
    std::vector<double> data;
    std::ifstream inFile(filename);
    
    if (!inFile) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return data; // Return empty vector
    }

    std::string line;
    while (std::getline(inFile, line)) {
        if (line.empty()) continue; // Skip empty lines
        std::istringstream iss(line);
        double value;
        if (iss >> value) { // Read the double value
            data.push_back(value);
        }
    }

    return data;
}

std::vector<std::vector<double>> readFromFile2D(const std::string& filename) {
    std::vector<std::vector<double>> data;
    std::ifstream inFile(filename);
    
    if (!inFile) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return data; // Return empty vector
    }

    std::string line;
    while (std::getline(inFile, line)) {
        if (line.empty()) continue; // Skip empty lines
        std::istringstream iss(line);
        std::vector<double> row;
        double value;
        
        while (iss >> value) { // Read all doubles in the line
            row.push_back(value);
        }
        
        if (!row.empty()) {
            data.push_back(row);
        }
    }

    return data;
}

std::vector<std::vector<double>> Interpolate(const std::vector<std::vector<double>>& m1, 
											 const std::vector<std::vector<double>>& m2, 
											 const double alpha,
											 const std::string & method) {
	if (m1.size() != m2.size() || m1[0].size() != m2[0].size()) {
		std::cerr << "[ERROR] Interpolate: Matrices must have the same dimensions." << std::endl;
		throw std::runtime_error("[ERROR]");
	}
	std::vector<std::vector<double>> out(m1.size(), std::vector<double>(m1[0].size(), 0.0));

	double _alpha;
	if (method == "linear") {
		_alpha = alpha;
	} else if (method == "quadratic") {
		_alpha = alpha * alpha;
	} else if (method == "cubic") {
		_alpha = alpha * alpha * alpha;
	} else if (method == "quartic") {
		_alpha = alpha * alpha * alpha * alpha;
	} else if (method == "quintic") {
		_alpha = alpha * alpha * alpha * alpha * alpha;
	} else if (method == "sinusoidal") {
		_alpha = 0.5 - 0.5 * cos(M_PI * alpha);
	} else if (method == "exponential") {
		if (alpha < 0 || alpha > 1) {
			std::cerr << "[ERROR] Interpolate: Exponential method requires 0 <= alpha <= 1." << std::endl;
			throw std::runtime_error("[ERROR]");
		}
		_alpha = exp(alpha) / (exp(1) - 1);
	} else {
		std::cerr << "[ERROR] Interpolate: Unknown method '" << method << "'." << std::endl;
		throw std::runtime_error("[ERROR]");
	}
	for (size_t i = 0; i < m1.size(); ++i) {
		for (size_t j = 0; j < m1[0].size(); ++j) {
			out[i][j] = m1[i][j] * (1 - _alpha) + m2[i][j] * _alpha;
		}
	}
	return out;
}

Eigen::MatrixXd DistMat(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B) {
    // Validate input dimensions
    if (A.cols() != 3 || B.cols() != 3) {
        throw std::invalid_argument("Both matrices must have 3 columns");
    }
    
    const long N = A.rows();
    const long M = B.rows();
    
    // Squared norms of each row in A and B
    Eigen::VectorXd A_sq_norms = A.rowwise().squaredNorm();
    Eigen::VectorXd B_sq_norms = B.rowwise().squaredNorm();
    
    // Compute squared distance matrix
    Eigen::MatrixXd D_sq = A_sq_norms.replicate(1, M);
    D_sq.noalias() += B_sq_norms.transpose().replicate(N, 1);
    D_sq.noalias() -= 2 * (A * B.transpose());
    
    // Handle numerical precision issues and compute Euclidean distance
    return D_sq.cwiseMax(0.0).cwiseSqrt();
}

void DistMat(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
			Eigen::MatrixXd& D) {
    // Validate input dimensions
    if (A.cols() != 3 || B.cols() != 3) {
        throw std::invalid_argument("Both matrices must have 3 columns");
    }
    
    const long N = A.rows();
    const long M = B.rows();
    
    // Squared norms of each row in A and B
    Eigen::VectorXd A_sq_norms = A.rowwise().squaredNorm();
    Eigen::VectorXd B_sq_norms = B.rowwise().squaredNorm();
    
    // Compute squared distance matrix
    Eigen::MatrixXd D_sq = A_sq_norms.replicate(1, M);
    D_sq.noalias() += B_sq_norms.transpose().replicate(N, 1);
    D_sq.noalias() -= 2 * (A * B.transpose());
    
	// update 
	D = D_sq.cwiseMax(0.0).cwiseSqrt();
    // Handle numerical precision issues and compute Euclidean distance
    return ;
}

void DistMat(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, 
             Eigen::MatrixXd& D, Eigen::MatrixXd& dx, 
             Eigen::MatrixXd& dy, Eigen::MatrixXd& dz) {
		
    // Validate input dimensions
    if (A.cols() != 3 || B.cols() != 3) {
        throw std::invalid_argument("Both matrices must have 3 columns");
    }
    
    const long N = A.rows();
    const long M = B.rows();
    
    // Compute component-wise differences (required outputs)
    dx = A.col(0) * Eigen::RowVectorXd::Ones(M) - 
         Eigen::VectorXd::Ones(N) * B.col(0).transpose();
    
    dy = A.col(1) * Eigen::RowVectorXd::Ones(M) - 
         Eigen::VectorXd::Ones(N) * B.col(1).transpose();
    
    dz = A.col(2) * Eigen::RowVectorXd::Ones(M) - 
         Eigen::VectorXd::Ones(N) * B.col(2).transpose();
    
    // Compute squared Euclidean distance directly from components
    D = dx.cwiseProduct(dx) + dy.cwiseProduct(dy) + dz.cwiseProduct(dz);
    
    // Finalize Euclidean distance with numerical stability
    D = D.cwiseMax(0.0).cwiseSqrt();
}

std::vector<double> Eigen2Vec(const Eigen::MatrixXd& matrix1d) {
    const size_t rows = matrix1d.rows();
    const size_t cols = matrix1d.cols();

	if (cols != 1) {
		throw std::invalid_argument("Input is not a 1d matrix");
	}

    std::vector<double> result;
    
    for (size_t i = 0; i < rows; ++i) {
		result.emplace_back(matrix1d(i));  // Move to avoid copy
    }
    
    return result;
}


std::vector<std::vector<double>> Eigen2Vec2d(const Eigen::MatrixXd& matrix) {
    const size_t rows = matrix.rows();
    const size_t cols = matrix.cols();
    
    std::vector<std::vector<double>> result;
    result.reserve(rows);  // Preallocate outer vector
    
    for (size_t i = 0; i < rows; ++i) {
        std::vector<double> row_vector(cols);
		for (size_t j = 0; j < cols; ++j) {
			row_vector[j] = matrix(i, j);
		}
		result.push_back(std::move(row_vector));  // Move to avoid copy
    }
    
    return result;
}

Eigen::MatrixXd Vec2d2Eigen(const std::vector<std::vector<double>>& vec2d) {
    if (vec2d.empty()) {
        return Eigen::MatrixXd(0, 0);  // Return empty matrix
    }

    const size_t rows = vec2d.size();
    const size_t cols = vec2d[0].size();

    // Validate rectangular structure
    for (size_t i = 0; i < rows; ++i) {
        if (vec2d[i].size() != cols) {
            throw std::invalid_argument("Input is not a rectangular matrix");
        }
    }

    // Create Eigen matrix and copy data
    Eigen::MatrixXd mat(rows, cols);
    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < cols; ++j) {
			mat(i, j) = vec2d[i][j];
		}
    }
    
    return mat;
}

std::vector<std::vector<double>> DistMat(const std::vector<std::vector<double>>& A, 
											const std::vector<std::vector<double>>& B) {
	Eigen::MatrixXd A_ = Vec2d2Eigen(A);
	Eigen::MatrixXd B_ = Vec2d2Eigen(B);
	Eigen::MatrixXd D = DistMat(A_, B_);
	std::vector<std::vector<double>> D_ = Eigen2Vec2d(D);
	return D_;
}


std::vector<double> flatten_vector(const std::vector<std::vector<double>>& vec2d) {
    // Calculate total size
    size_t total_size = 0;
    for (const auto& inner_vec : vec2d) {
        total_size += inner_vec.size();
    }
    
    // Create result vector and reserve space
    std::vector<double> result;
    result.reserve(total_size);
    
    // Append all elements
    for (const auto& inner_vec : vec2d) {
        result.insert(result.end(), inner_vec.begin(), inner_vec.end());
    }
    
    return result;
}

void smoothRowwiseMin(const Eigen::MatrixXd& A, Eigen::MatrixXd& result, const double alpha) {
	if (A.rows() != result.rows()) {
		std::cout << "[ERROR] A.rows() != result.rows()." << std::endl;
		throw std::runtime_error("[ERROR]");
	}
	if (result.cols() != 1) {
		std::cout << "[ERROR] result.cols() != 1." << std::endl;
		throw std::runtime_error("[ERROR]");
	}
    for (int i = 0; i < A.rows(); ++i) {
        // log-sum-exp over negative values
        result(i, 0) = -1.0 / alpha * log(((-alpha * A.row(i)).array().exp()).sum());
    }
}

void smoothRowwiseMin(const Eigen::MatrixXd& A, Eigen::MatrixXd& result, 
						Eigen::MatrixXd& grad, const double alpha) {
	if (A.rows() != result.rows()) {
		std::cout << "[ERROR] A.rows() != result.rows()." << std::endl;
		throw std::runtime_error("[ERROR]");
	}
	if (result.cols() != 1) {
		std::cout << "[ERROR] result.cols() != 1." << std::endl;
		throw std::runtime_error("[ERROR]");
	}
	if (A.rows() != grad.rows()) {
		std::cout << "[ERROR] A.rows() != grad.rows()." << std::endl;
		throw std::runtime_error("[ERROR]");
	}
	if (A.cols() != grad.cols()) {
		std::cout << "[ERROR] A.cols() != grad.cols()." << std::endl;
		throw std::runtime_error("[ERROR]");
	}

	////////////////////////////////////////////////////////////////////////
    
	for (int i = 0; i < A.rows(); ++i) {
        // log-sum-exp over negative values
		grad.row(i) = (-alpha * A.row(i).array()).exp();
        result(i, 0) = -1.0 / alpha * log(grad.row(i).sum());
        grad.row(i) /= grad.row(i).sum();  // ∂min_approx/∂x_ij = α * softmax(-α * x_ij), but we already have sign from negative exponent
    }
}

// Derivative of smoothed row-wise min w.r.t. each element of A
void smoothRowwiseMinDerivative(const Eigen::MatrixXd& A, Eigen::MatrixXd& grad, double alpha) {
	if (A.rows() != grad.rows()) {
		std::cout << "[ERROR] A.rows() != grad.rows()." << std::endl;
		throw std::runtime_error("[ERROR]");
	}
	if (A.cols() != grad.cols()) {
		std::cout << "[ERROR] A.cols() != grad.cols()." << std::endl;
		throw std::runtime_error("[ERROR]");
	}
    for (int i = 0; i < A.rows(); ++i) {
        Eigen::RowVectorXd expVals = (-alpha * A.row(i).array()).exp();
        double Z = expVals.sum();  // normalization constant
        grad.row(i) = (expVals / Z);  // ∂min_approx/∂x_ij = α * softmax(-α * x_ij), but we already have sign from negative exponent
    }
}

void Euler2Mat(const std::vector<double>& euler6d, std::vector<std::vector<double>>& tr_mat) {

    // Precompute trig terms
    double cr = cos(euler6d[0]), sr = sin(euler6d[0]);
    double cy = cos(euler6d[1]), sy = sin(euler6d[1]);
    double cp = cos(euler6d[2]), sp = sin(euler6d[2]);

    // Rotation matrix R = Rz(yaw) * Ry(pitch) * Rx(roll)
    Eigen::Matrix3d R;
    R << cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr,
         sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr,
         -sp,   cp*sr,            cp*cr;

    // Fill the 4x4 transformation matrix
	for (int i=0; i<3; ++i) {
		for (int j=0; j<3; ++j) {
	    	tr_mat[i][j] = R(i,j);
		}
	}
    tr_mat[0][3] = euler6d[3]; tr_mat[1][3] = euler6d[4]; tr_mat[2][3] = euler6d[5];
    tr_mat[3][0] = 0; tr_mat[3][1] = 0; tr_mat[3][2] = 0; tr_mat[3][3] = 1;

}

void Euler2Mat(const std::vector<double>& euler6d, 
				std::vector<std::vector<double>>& tr_mat, 
				std::vector<double>& grad6d) {

    // Precompute trig terms
    double cr = cos(euler6d[0]), sr = sin(euler6d[0]);
    double cp = cos(euler6d[1]), sp = sin(euler6d[1]);
    double cy = cos(euler6d[2]), sy = sin(euler6d[2]);

    Eigen::Matrix3d slack_var;

	// We here use slack_var as Rotation matrix 
    // Rotation matrix R = Rz(yaw) * Ry(pitch) * Rx(roll)
    slack_var << cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr,
				sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr,
				-sp,   cp*sr,            cp*cr;

    // Fill the 4x4 transformation matrix
	for (int i=0; i<3; ++i) {
		for (int j=0; j<3; ++j) {
	    	tr_mat[i][j] = slack_var(i,j);
		}
	}
    tr_mat[0][3] = euler6d[3]; tr_mat[1][3] = euler6d[4]; tr_mat[2][3] = euler6d[5];
    tr_mat[3][0] = 0; tr_mat[3][1] = 0; tr_mat[3][2] = 0; tr_mat[3][3] = 1;

	// Now let's compute derivatives of Mat w.r.t. euler angles
	// ∂R/∂roll
    slack_var << 
				0, cy*sp*cr + sy*sr, -cy*sp*sr + sy*cr,
				0, sy*sp*cr - cy*sr, -sy*sp*sr - cy*cr,
				0, cp*cr,           -cp*sr;
	grad6d[0] = slack_var.sum();

    // ∂R/∂pitch
    slack_var << 
				-cy*sp, cy*cp*sr, cy*cp*cr,
				-sy*sp, sy*cp*sr, sy*cp*cr,
				-cp,    -sp*sr,   -sp*cr;
	grad6d[1] = slack_var.sum();

    // ∂R/∂yaw
    slack_var << 
				-sy*cp, -sy*sp*sr - cy*cr, -sy*sp*cr + cy*sr,
				cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr,
				0,      0,                0;
	grad6d[2] = slack_var.sum();

    // Translation derivatives: identity
    grad6d[3] = 1.0;  // dx
    grad6d[4] = 1.0;  // dy
    grad6d[5] = 1.0; // dz
}


std::vector<double> transformMatrixToQuaternion(const std::vector<std::vector<double>>& mat) {
    // Validate input matrix size
    if (mat.size() != 4 || mat[0].size() != 4) {
        throw std::invalid_argument("Input matrix must be 4x4");
    }

    // Extract rotation components (3x3 submatrix)
    const double& m00 = mat[0][0];
    const double& m01 = mat[0][1];
    const double& m02 = mat[0][2];
    const double& m10 = mat[1][0];
    const double& m11 = mat[1][1];
    const double& m12 = mat[1][2];
    const double& m20 = mat[2][0];
    const double& m21 = mat[2][1];
    const double& m22 = mat[2][2];

    // Compute quaternion components
    double x, y, z, w;
    const double trace = m00 + m11 + m22;

    if (trace > 0) {
        const double s = 0.5 / std::sqrt(trace + 1.0);
        w = 0.25 / s;
        x = (m21 - m12) * s;
        y = (m02 - m20) * s;
        z = (m10 - m01) * s;
    } else if (m00 > m11 && m00 > m22) {
        const double s = 0.5 / std::sqrt(1.0 + m00 - m11 - m22);
        w = (m21 - m12) * s;
        x = 0.25 / s;
        y = (m01 + m10) * s;
        z = (m02 + m20) * s;
    } else if (m11 > m22) {
        const double s = 0.5 / std::sqrt(1.0 + m11 - m00 - m22);
        w = (m02 - m20) * s;
        x = (m01 + m10) * s;
        y = 0.25 / s;
        z = (m12 + m21) * s;
    } else {
        const double s = 0.5 / std::sqrt(1.0 + m22 - m00 - m11);
        w = (m10 - m01) * s;
        x = (m02 + m20) * s;
        y = (m12 + m21) * s;
        z = 0.25 / s;
    }

    // Normalize and return quaternion
    const double norm = std::sqrt(w*w + x*x + y*y + z*z);
    return {w/norm, x/norm, y/norm, z/norm};
}

void TransMatGrad(const std::vector<double>& euler6d, const Eigen::MatrixXd& tr_mat, std::vector<double>& grad6d) {
    
	double roll = euler6d[0], pitch = euler6d[1], yaw = euler6d[2];

    double cr = cos(roll), sr = sin(roll);
    double cp = cos(pitch), sp = sin(pitch);
    double cy = cos(yaw), sy = sin(yaw);

    // Partial derivatives of rotation matrix
    Eigen::Matrix3d dR_dangle;

    // ∂R/∂roll
    dR_dangle << 
				0, cy*sp*cr + sy*sr, -cy*sp*sr + sy*cr,
				0, sy*sp*cr - cy*sr, -sy*sp*sr - cy*cr,
				0, cp*cr,           -cp*sr;
	grad6d[0] = dR_dangle.sum();

    // ∂R/∂pitch
    dR_dangle << 
				-cy*sp, cy*cp*sr, cy*cp*cr,
				-sy*sp, sy*cp*sr, sy*cp*cr,
				-cp,    -sp*sr,   -sp*cr;
	grad6d[1] = dR_dangle.sum();

    // ∂R/∂yaw
    dR_dangle << 
				-sy*cp, -sy*sp*sr - cy*cr, -sy*sp*cr + cy*sr,
				cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr,
				0,      0,                0;
	grad6d[2] = dR_dangle.sum();

    // Translation derivatives: identity
    grad6d[3] = 1.0;  // dx
    grad6d[4] = 1.0;  // dy
    grad6d[5] = 1.0; // dz
}


std::vector<double> generateRandomVector(const int& dim) {
    // Thread-local RNG engine (seeded once per thread)
    thread_local std::mt19937 gen(std::random_device{}());
    
    // Uniform distribution for (-1, 1)
    thread_local std::uniform_real_distribution<double> dist(-1.0, 1.0);

    // Generate 6 random values
    std::vector<double> result;
    result.reserve(dim);
    for (int i = 0; i < dim; ++i) {
        result.push_back(dist(gen));
    }
    return result;
}

Eigen::MatrixXd sampleMatrix(const Eigen::MatrixXd& mat, const int& sample_rate) {
    int N = mat.rows();
    int sampled_rows = N / sample_rate;  // Integer division (floor)
    Eigen::MatrixXd sampled_mat(sampled_rows, 3);
    
    for (int i = 0; i < sampled_rows; ++i) {
        sampled_mat.row(i) = mat.row(i * 10);
    }
    return sampled_mat;
}

std::string getExecutableDirectory() {
    #ifdef _WIN32
        char path[MAX_PATH];
        GetModuleFileNameA(NULL, path, MAX_PATH);
        return fs::path(path).parent_path().string();
    #else
        char path[PATH_MAX];
        ssize_t count = readlink("/proc/self/exe", path, PATH_MAX);
        return (count != -1) 
            ? fs::path(std::string(path, count)).parent_path().string() 
            : fs::current_path().string();
    #endif
}

} // end namespace grasp
