/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See https://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Provide some simple operation on column vectors.
 */

/*!
 * \file vpColVector.cpp
 * \brief  Class that provides a data structure for the column vectors as well
 * as a set of operations on these vectors
 */

#include <assert.h>
#include <cmath>  // std::fabs
#include <limits> // numeric_limits
#include <math.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <visp3/core/vpCPUFeatures.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpRotationVector.h>

#include <Simd/SimdLib.hpp>

vpColVector vpColVector::operator+(const vpColVector &v) const
{
  if (getRows() != v.getRows()) {
    throw(vpException(vpException::dimensionError, "Cannot add (%dx1) column vector to (%dx1) column vector", getRows(),
                      v.getRows()));
  }
  vpColVector r(rowNum);

  for (unsigned int i = 0; i < rowNum; i++)
    r[i] = (*this)[i] + v[i];
  return r;
}

vpTranslationVector vpColVector::operator+(const vpTranslationVector &t) const
{
  if (getRows() != 3) {
    throw(vpException(vpException::dimensionError, "Cannot add %d-dimension column vector to a translation vector",
                      getRows()));
  }
  vpTranslationVector s;

  for (unsigned int i = 0; i < 3; i++)
    s[i] = (*this)[i] + t[i];

  return s;
}

vpColVector &vpColVector::operator+=(vpColVector v)
{
  if (getRows() != v.getRows()) {
    throw(vpException(vpException::dimensionError, "Cannot add (%dx1) column vector to (%dx1) column vector", getRows(),
                      v.getRows()));
  }

  for (unsigned int i = 0; i < rowNum; i++)
    (*this)[i] += v[i];
  return (*this);
}

vpColVector &vpColVector::operator-=(vpColVector v)
{
  if (getRows() != v.getRows()) {
    throw(vpException(vpException::dimensionError, "Cannot subtract (%dx1) column vector to (%dx1) column vector",
                      getRows(), v.getRows()));
  }

  for (unsigned int i = 0; i < rowNum; i++)
    (*this)[i] -= v[i];
  return (*this);
}

double vpColVector::operator*(const vpColVector &v) const
{
  if (size() != v.size()) {
    throw(vpException(vpException::dimensionError,
                      "Cannot compute the dot product between column vectors "
                      "with different dimensions (%d) and (%d)",
                      size(), v.size()));
  }
  double r = 0;

  for (unsigned int i = 0; i < rowNum; i++)
    r += (*this)[i] * v[i];
  return r;
}

vpMatrix vpColVector::operator*(const vpRowVector &v) const
{
  vpMatrix M(rowNum, v.getCols());
  for (unsigned int i = 0; i < rowNum; i++) {
    for (unsigned int j = 0; j < v.getCols(); j++) {
      M[i][j] = (*this)[i] * v[j];
    }
  }
  return M;
}

vpColVector vpColVector::operator-(const vpColVector &m) const
{
  if (getRows() != m.getRows()) {
    throw(vpException(vpException::dimensionError,
                      "Bad size during vpColVector (%dx1) and vpColVector "
                      "(%dx1) subtraction",
                      getRows(), m.getRows()));
  }
  vpColVector v(rowNum);

  for (unsigned int i = 0; i < rowNum; i++)
    v[i] = (*this)[i] - m[i];
  return v;
}

vpColVector::vpColVector(const vpColVector &v, unsigned int r, unsigned int nrows) : vpArray2D<double>(nrows, 1)
{
  init(v, r, nrows);
}

void vpColVector::init(const vpColVector &v, unsigned int r, unsigned int nrows)
{
  unsigned int rnrows = r + nrows;

  if (rnrows > v.getRows())
    throw(vpException(vpException::dimensionError, "Bad row dimension (%d > %d) used to initialize vpColVector", rnrows,
                      v.getRows()));
  resize(nrows, false);

  if (this->rowPtrs == NULL) // Fix coverity scan: explicit null dereferenced
    return;                  // Nothing to do
  for (unsigned int i = r; i < rnrows; i++)
    (*this)[i - r] = v[i];
}

vpColVector::vpColVector(const vpRotationVector &v) : vpArray2D<double>(v.size(), 1)
{
  for (unsigned int i = 0; i < v.size(); i++)
    (*this)[i] = v[i];
}

vpColVector::vpColVector(const vpPoseVector &p) : vpArray2D<double>(p.size(), 1)
{
  for (unsigned int i = 0; i < p.size(); i++)
    (*this)[i] = p[i];
}

vpColVector::vpColVector(const vpTranslationVector &v) : vpArray2D<double>(v.size(), 1)
{
  for (unsigned int i = 0; i < v.size(); i++)
    (*this)[i] = v[i];
}

vpColVector::vpColVector(const vpMatrix &M, unsigned int j) : vpArray2D<double>(M.getRows(), 1)
{
  for (unsigned int i = 0; i < M.getCols(); i++)
    (*this)[i] = M[i][j];
}

vpColVector::vpColVector(const vpMatrix &M) : vpArray2D<double>(M.getRows(), 1)
{
  if (M.getCols() != 1) {
    throw(vpException(vpException::dimensionError, "Cannot construct a (%dx1) row vector from a (%dx%d) matrix",
                      M.getRows(), M.getRows(), M.getCols()));
  }

  for (unsigned int i = 0; i < M.getRows(); i++)
    (*this)[i] = M[i][0];
}

vpColVector::vpColVector(const std::vector<double> &v) : vpArray2D<double>((unsigned int)v.size(), 1)
{
  for (unsigned int i = 0; i < v.size(); i++)
    (*this)[i] = v[i];
}

vpColVector::vpColVector(const std::vector<float> &v) : vpArray2D<double>((unsigned int)v.size(), 1)
{
  for (unsigned int i = 0; i < v.size(); i++)
    (*this)[i] = (double)(v[i]);
}

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
vpColVector::vpColVector(vpColVector &&v) : vpArray2D<double>()
{
  rowNum = v.rowNum;
  colNum = v.colNum;
  rowPtrs = v.rowPtrs;
  dsize = v.dsize;
  data = v.data;

  v.rowNum = 0;
  v.colNum = 0;
  v.rowPtrs = NULL;
  v.dsize = 0;
  v.data = NULL;
}
#endif

vpColVector vpColVector::operator-() const
{
  vpColVector A;
  A.resize(rowNum, false);

  double *vd = A.data;
  double *d = data;

  for (unsigned int i = 0; i < rowNum; i++)
    *(vd++) = -(*d++);

  return A;
}

vpColVector vpColVector::operator*(double x) const
{
  vpColVector v(rowNum);

  double *vd = v.data;
  double *d = data;

  for (unsigned int i = 0; i < rowNum; i++)
    *(vd++) = (*d++) * x;
  return v;
}

vpColVector &vpColVector::operator*=(double x)
{
  for (unsigned int i = 0; i < rowNum; i++)
    (*this)[i] *= x;
  return (*this);
}

vpColVector &vpColVector::operator/=(double x)
{
  for (unsigned int i = 0; i < rowNum; i++)
    (*this)[i] /= x;
  return (*this);
}

vpColVector vpColVector::operator/(double x) const
{
  vpColVector v(rowNum);

  double *vd = v.data;
  double *d = data;

  for (unsigned int i = 0; i < rowNum; i++)
    *(vd++) = (*d++) / x;
  return v;
}

vpColVector &vpColVector::operator=(const vpMatrix &M)
{
  if (M.getCols() != 1) {
    throw(vpException(vpException::dimensionError, "Cannot transform a (%dx%d) matrix into a column vector",
                      M.getRows(), M.getCols()));
  }

  resize(M.getRows(), false);
  memcpy(data, M.data, rowNum * sizeof(double));

  return (*this);
}

vpColVector &vpColVector::operator=(const std::vector<double> &v)
{
  resize((unsigned int)v.size(), false);
  for (unsigned int i = 0; i < v.size(); i++)
    (*this)[i] = v[i];
  return *this;
}

vpColVector &vpColVector::operator=(const std::vector<float> &v)
{
  resize((unsigned int)v.size(), false);
  for (unsigned int i = 0; i < v.size(); i++)
    (*this)[i] = (float)v[i];
  return *this;
}

vpColVector &vpColVector::operator=(const vpColVector &v)
{
  unsigned int k = v.rowNum;
  if (rowNum != k) {
    resize(k, false);
  }

  memcpy(data, v.data, rowNum * sizeof(double));
  return *this;
}

vpColVector &vpColVector::operator=(const vpTranslationVector &tv)
{
  unsigned int k = tv.getRows();
  if (rowNum != k) {
    resize(k, false);
  }

  memcpy(data, tv.data, rowNum * sizeof(double));
  return *this;
}

vpColVector &vpColVector::operator=(const vpRotationVector &rv)
{
  unsigned int k = rv.getRows();
  if (rowNum != k) {
    resize(k, false);
  }

  memcpy(data, rv.data, rowNum * sizeof(double));
  return *this;
}

vpColVector &vpColVector::operator=(const vpPoseVector &p)
{
  unsigned int k = p.getRows();
  if (rowNum != k) {
    resize(k, false);
  }

  memcpy(data, p.data, rowNum * sizeof(double));
  return *this;
}

vpColVector &vpColVector::operator<<(const vpColVector &v)
{
  *this = v;
  return *this;
}

vpColVector &vpColVector::operator<<(double *x)
{
  for (unsigned int i = 0; i < rowNum; i++) {
    for (unsigned int j = 0; j < colNum; j++) {
      rowPtrs[i][j] = *x++;
    }
  }
  return *this;
}

vpColVector &vpColVector::operator<<(double val)
{
  resize(1, false);
  data[0] = val;
  return *this;
}

vpColVector &vpColVector::operator,(double val)
{
  resize(rowNum + 1, false);
  data[rowNum - 1] = val;
  return *this;
}

vpColVector &vpColVector::operator=(double x)
{
  double *d = data;

  for (unsigned int i = 0; i < rowNum; i++)
    *(d++) = x;
  return *this;
}

std::vector<double> vpColVector::toStdVector() const
{
  std::vector<double> v(this->size());

  for (unsigned int i = 0; i < this->size(); i++)
    v[i] = data[i];
  return v;
}

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)

vpColVector &vpColVector::operator=(vpColVector &&other)
{
  if (this != &other) {
    free(data);
    free(rowPtrs);

    rowNum = other.rowNum;
    colNum = other.colNum;
    rowPtrs = other.rowPtrs;
    dsize = other.dsize;
    data = other.data;

    other.rowNum = 0;
    other.colNum = 0;
    other.rowPtrs = NULL;
    other.dsize = 0;
    other.data = NULL;
  }

  return *this;
}

vpColVector &vpColVector::operator=(const std::initializer_list<double> &list)
{
  resize(static_cast<unsigned int>(list.size()), false);
  std::copy(list.begin(), list.end(), data);
  return *this;
}
#endif

bool vpColVector::operator==(const vpColVector &v) const
{
  if (rowNum != v.rowNum || colNum != v.colNum /* should not happen */)
    return false;

  for (unsigned int i = 0; i < rowNum; i++) {
    if (!vpMath::equal(data[i], v.data[i], std::numeric_limits<double>::epsilon()))
      return false;
  }

  return true;
}

bool vpColVector::operator==(double v) const
{
  for (unsigned int i = 0; i < rowNum; i++) {
    if (!vpMath::equal(data[i], v, std::numeric_limits<double>::epsilon()))
      return false;
  }

  return true;
}

bool vpColVector::operator!=(const vpColVector &v) const { return !(*this == v); }

bool vpColVector::operator!=(double v) const { return !(*this == v); }

vpRowVector vpColVector::t() const
{
  vpRowVector v(rowNum);
  memcpy(v.data, data, rowNum * sizeof(double));
  return v;
}

vpRowVector vpColVector::transpose() const { return t(); }

void vpColVector::transpose(vpRowVector &v) const { v = t(); }

vpColVector operator*(const double &x, const vpColVector &v)
{
  vpColVector vout;
  vout = v * x;
  return vout;
}

double vpColVector::dotProd(const vpColVector &a, const vpColVector &b)
{
  if (a.data == NULL) {
    throw(vpException(vpException::fatalError, "Cannot compute the dot product: first vector empty"));
  }
  if (b.data == NULL) {
    throw(vpException(vpException::fatalError, "Cannot compute the dot product: second vector empty"));
  }
  if (a.size() != b.size()) {
    throw(vpException(vpException::dimensionError,
                      "Cannot compute the dot product between column vectors "
                      "with different dimensions (%d) and (%d)",
                      a.size(), b.size()));
  }

  double *ad = a.data;
  double *bd = b.data;

  double c = 0;
  for (unsigned int i = 0; i < a.getRows(); i++)
    c += *(ad++) * *(bd++);

  return c;
}


vpColVector &vpColVector::normalize(vpColVector &x) const
{
  x /= sqrt(x.sumSquare());

  return x;
}

vpColVector &vpColVector::normalize()
{

  double sum_square = sumSquare();

  // if (sum != 0.0)
  if (std::fabs(sum_square) > std::numeric_limits<double>::epsilon())
    *this /= sqrt(sum_square);

  // If sum = 0, we have a nul vector. So we return just.
  return *this;
}

vpColVector vpColVector::invSort(const vpColVector &v)
{
  if (v.data == NULL) {
    throw(vpException(vpException::fatalError, "Cannot sort content of column vector: vector empty"));
  }
  vpColVector tab;
  tab = v;
  unsigned int nb_permutation = 1;
  unsigned int i = 0;
  while (nb_permutation != 0) {
    nb_permutation = 0;
    for (unsigned int j = v.getRows() - 1; j >= i + 1; j--) {
      if ((tab[j] > tab[j - 1])) {
        double tmp = tab[j];
        tab[j] = tab[j - 1];
        tab[j - 1] = tmp;
        nb_permutation++;
      }
    }
    i++;
  }

  return tab;
}

vpColVector vpColVector::sort(const vpColVector &v)
{
  if (v.data == NULL) {
    throw(vpException(vpException::fatalError, "Cannot sort content of column vector: vector empty"));
  }
  vpColVector tab;
  tab = v;
  unsigned int nb_permutation = 1;
  unsigned int i = 0;
  while (nb_permutation != 0) {
    nb_permutation = 0;
    for (unsigned int j = v.getRows() - 1; j >= i + 1; j--) {
      if ((tab[j] < tab[j - 1])) {
        double tmp = tab[j];
        tab[j] = tab[j - 1];
        tab[j - 1] = tmp;
        nb_permutation++;
      }
    }
    i++;
  }

  return tab;
}

void vpColVector::stack(double d)
{
  this->resize(rowNum + 1, false);
  (*this)[rowNum - 1] = d;
}

void vpColVector::stack(const vpColVector &v) { *this = vpColVector::stack(*this, v); }

vpColVector vpColVector::stack(const vpColVector &A, const vpColVector &B)
{
  vpColVector C;
  vpColVector::stack(A, B, C);
  return C;
}

void vpColVector::stack(const vpColVector &A, const vpColVector &B, vpColVector &C)
{
  unsigned int nrA = A.getRows();
  unsigned int nrB = B.getRows();

  if (nrA == 0 && nrB == 0) {
    C.resize(0);
    return;
  }

  if (nrB == 0) {
    C = A;
    return;
  }

  if (nrA == 0) {
    C = B;
    return;
  }

  // General case
  C.resize(nrA + nrB, false);

  for (unsigned int i = 0; i < nrA; i++)
    C[i] = A[i];

  for (unsigned int i = 0; i < nrB; i++)
    C[nrA + i] = B[i];
}

double vpColVector::mean(const vpColVector &v)
{
  if (v.data == NULL || v.size() == 0) {
    throw(vpException(vpException::dimensionError, "Cannot compute column vector mean: vector empty"));
  }

  // Use directly sum() function
  double mean = v.sum();

  return mean / v.getRows();
}

double vpColVector::median(const vpColVector &v)
{
  if (v.data == NULL || v.size() == 0) {
    throw(vpException(vpException::dimensionError, "Cannot compute column vector median: vector empty"));
  }

  std::vector<double> vectorOfDoubles(v.data, v.data + v.rowNum);

  return vpMath::getMedian(vectorOfDoubles);
}

double vpColVector::stdev(const vpColVector &v, bool useBesselCorrection)
{
  if (v.data == NULL || v.size() == 0) {
    throw(vpException(vpException::dimensionError, "Cannot compute column vector stdev: vector empty"));
  }

  return SimdVectorStdev(v.data, v.rowNum, useBesselCorrection);
}

vpMatrix vpColVector::skew(const vpColVector &v)
{
  vpMatrix M;
  if (v.getRows() != 3) {
    throw(vpException(vpException::dimensionError, "Cannot compute skew vector of a non 3-dimention vector (%d)",
                      v.getRows()));
  }

  M.resize(3, 3, false, false);
  M[0][0] = 0;
  M[0][1] = -v[2];
  M[0][2] = v[1];
  M[1][0] = v[2];
  M[1][1] = 0;
  M[1][2] = -v[0];
  M[2][0] = -v[1];
  M[2][1] = v[0];
  M[2][2] = 0;

  return M;
}

vpColVector vpColVector::crossProd(const vpColVector &a, const vpColVector &b)
{
  if (a.getRows() != 3 || b.getRows() != 3) {
    throw(vpException(vpException::dimensionError,
                      "Cannot compute the cross product between column "
                      "vector with dimension %d and %d",
                      a.getRows(), b.getRows()));
  }

  return vpColVector::skew(a) * b;
}

vpMatrix vpColVector::reshape(unsigned int nrows, unsigned int ncols)
{
  vpMatrix M(nrows, ncols);
  reshape(M, nrows, ncols);
  return M;
}

void vpColVector::reshape(vpMatrix &M, const unsigned int &nrows, const unsigned int &ncols)
{
  if (dsize != nrows * ncols) {
    throw(vpException(vpException::dimensionError, "Cannot reshape (%dx1) column vector in (%dx%d) matrix", rowNum,
                      M.getRows(), M.getCols()));
  }
  if ((M.getRows() != nrows) || (M.getCols() != ncols))
    M.resize(nrows, ncols, false, false);

  for (unsigned int j = 0; j < ncols; j++)
    for (unsigned int i = 0; i < nrows; i++)
      M[i][j] = data[j * nrows + i];
}

void vpColVector::insert(unsigned int i, const vpColVector &v)
{
  if (i + v.size() > this->size())
    throw(vpException(vpException::dimensionError, "Unable to insert a column vector"));

  if (data != NULL && v.data != NULL && v.rowNum > 0) {
    memcpy(data + i, v.data, sizeof(double) * v.rowNum);
  }
}
void vpColVector::insert(const vpColVector &v, unsigned int i)
{
  insert(i, v);
}

int vpColVector::print(std::ostream &s, unsigned int length, char const *intro) const
{
  typedef std::string::size_type size_type;

  unsigned int m = getRows();
  unsigned int n = 1;

  std::vector<std::string> values(m * n);
  std::ostringstream oss;
  std::ostringstream ossFixed;
  std::ios_base::fmtflags original_flags = oss.flags();

  // ossFixed <<std::fixed;
  ossFixed.setf(std::ios::fixed, std::ios::floatfield);

  size_type maxBefore = 0; // the length of the integral part
  size_type maxAfter = 0;  // number of decimals plus
  // one place for the decimal point
  for (unsigned int i = 0; i < m; ++i) {
    oss.str("");
    oss << (*this)[i];
    if (oss.str().find("e") != std::string::npos) {
      ossFixed.str("");
      ossFixed << (*this)[i];
      oss.str(ossFixed.str());
    }

    values[i] = oss.str();
    size_type thislen = values[i].size();
    size_type p = values[i].find('.');

    if (p == std::string::npos) {
      maxBefore = vpMath::maximum(maxBefore, thislen);
      // maxAfter remains the same
    }
    else {
      maxBefore = vpMath::maximum(maxBefore, p);
      maxAfter = vpMath::maximum(maxAfter, thislen - p - 1);
    }
  }

  size_type totalLength = length;
  // increase totalLength according to maxBefore
  totalLength = vpMath::maximum(totalLength, maxBefore);
  // decrease maxAfter according to totalLength
  maxAfter = (std::min)(maxAfter, totalLength - maxBefore);
  if (maxAfter == 1)
    maxAfter = 0;

  // the following line is useful for debugging
  // std::cerr <<totalLength <<" " <<maxBefore <<" " <<maxAfter <<"\n";

  if (intro)
    s << intro;
  s << "[" << m << "," << n << "]=\n";

  for (unsigned int i = 0; i < m; i++) {
    s << "  ";
    size_type p = values[i].find('.');
    s.setf(std::ios::right, std::ios::adjustfield);
    s.width((std::streamsize)maxBefore);
    s << values[i].substr(0, p).c_str();

    if (maxAfter > 0) {
      s.setf(std::ios::left, std::ios::adjustfield);
      if (p != std::string::npos) {
        s.width((std::streamsize)maxAfter);
        s << values[i].substr(p, maxAfter).c_str();
      }
      else {
        assert(maxAfter > 1);
        s.width((std::streamsize)maxAfter);
        s << ".0";
      }
    }

    s << ' ';

    s << std::endl;
  }

  s.flags(original_flags); // restore s to standard state

  return (int)(maxBefore + maxAfter);
}

double vpColVector::sum() const { return SimdVectorSum(data, rowNum); }

double vpColVector::sumSquare() const { return SimdVectorSumSquare(data, rowNum); }

double vpColVector::frobeniusNorm() const
{
  double norm = sumSquare();

  return sqrt(norm);
}

vpColVector vpColVector::hadamard(const vpColVector &v) const
{
  if (v.getRows() != rowNum || v.getCols() != colNum) {
    throw(vpException(vpException::dimensionError, "Hadamard product: bad dimensions!"));
  }

  vpColVector out;
  out.resize(rowNum, false);

  SimdVectorHadamard(data, v.data, rowNum, out.data);

  return out;
}

double vpColVector::infinityNorm() const
{
  double norm = 0.0;
  for (unsigned int i = 0; i < rowNum; i++) {
    double x = fabs((*this)[i]);
    if (x > norm) {
      norm = x;
    }
  }
  return norm;
}

std::ostream &vpColVector::cppPrint(std::ostream &os, const std::string &matrixName, bool octet) const
{
  os << "vpColVector " << matrixName << " (" << this->getRows() << "); " << std::endl;

  for (unsigned int i = 0; i < this->getRows(); ++i) {

    if (!octet) {
      os << matrixName << "[" << i << "] = " << (*this)[i] << "; " << std::endl;
    }
    else {
      for (unsigned int k = 0; k < sizeof(double); ++k) {
        os << "((unsigned char*)&(" << matrixName << "[" << i << "]) )[" << k << "] = 0x" << std::hex
          << (unsigned int)((unsigned char *)&((*this)[i]))[k] << "; " << std::endl;
      }
    }
  }
  std::cout << std::endl;
  return os;
};

std::ostream &vpColVector::csvPrint(std::ostream &os) const
{
  for (unsigned int i = 0; i < this->getRows(); ++i) {
    os << (*this)[i];

    os << std::endl;
  }
  return os;
};

std::ostream &vpColVector::maplePrint(std::ostream &os) const
{
  os << "([ " << std::endl;
  for (unsigned int i = 0; i < this->getRows(); ++i) {
    os << "[";
    os << (*this)[i] << ", ";
    os << "]," << std::endl;
  }
  os << "])" << std::endl;
  return os;
};

std::ostream &vpColVector::matlabPrint(std::ostream &os) const
{
  os << "[ ";
  for (unsigned int i = 0; i < this->getRows(); ++i) {
    os << (*this)[i] << ", ";
    if (this->getRows() != i + 1) {
      os << ";" << std::endl;
    }
    else {
      os << "]" << std::endl;
    }
  }
  return os;
};

#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)

void vpColVector::insert(const vpColVector &v, unsigned int r, unsigned int c)
{
  (void)c;
  insert(r, v);
}

double vpColVector::euclideanNorm() const { return frobeniusNorm(); }
#endif // defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
