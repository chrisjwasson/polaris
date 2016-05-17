#ifndef MATRIX_H
#define MATRIX_H

#include <assert.h>
#include <stdio.h>
#include <math.h>

// class definition for 2D matrices


template<class T, int R, int C> class Matrix
{
	public:
		Matrix();
		~Matrix();
		Matrix(const Matrix<T,R,C>& inputMatrix);
		Matrix(T arrayInit[]);						// constructor with array initializer
		Matrix<T,R,C> operator+(const Matrix<T,R,C>& rhs) const;		// matrix addition
		Matrix<T,R,C> operator-(const Matrix<T,R,C>& rhs) const;		// matrix subtraction
		Matrix<T,R,C> operator*(const T& factor) const;			// element-by-element multiplication
		Matrix<T,R,C> operator/(const T& divisor) const;			// element-by-element division
		template<int RR, int CR> Matrix<T,R,CR> operator*(const Matrix<T,RR,CR>& rhs) const;		// matrix multiplication
		Matrix<T,C,R> transpose() const;				// transpose
		T& operator()(const int rowIdx, const int colIdx);		// 2D indexing operator (reference)
		T& operator()(const int index);					// 1D indexing operator (reference)
		T operator()(const int rowIdx, const int colIdx) const;		// 2D indexing operator (return copy)
		T operator()(const int index) const;				// 1D indexing operator (return copy)
		Matrix<T,R,C>& rowSwap(int row1Idx, int row2Idx);				// row swap method
		int getNumElements() const;						// get number of elements
		Matrix<int,1,2> size();						// get dimensions of this Matrix
		void printValues();						// print values to command line
		Matrix<T,R,C> inverse() const;
		T norm() const;							// 2-norm of matrix
        Matrix<T,R,C> unit() const;             // returns unit vector
		
		
	private:
		int m_numRows;
		int m_numCols;
		T m_data[R*C];
};

typedef Matrix<double,3,1> Vec3d;


/////// include class implementation file to resolve linking issues for template class




// Constructor
template<class T, int R, int C>
Matrix<T, R, C>::Matrix()
{
	m_numRows = R;
	m_numCols = C;
	for (int ii = 0; ii < m_numRows*m_numCols; ii++)
	{
		m_data[ii] = 0;
	}
}

// Destructor
template<class T, int R, int C>
Matrix<T,R,C>::~Matrix()
{
}

// copy constructor
template<class T, int R, int C>
Matrix<T,R,C>::Matrix(const Matrix<T,R,C>& inputMatrix)
	: m_numRows(inputMatrix.m_numRows), m_numCols(inputMatrix.m_numCols)
{
	for (int ii = 0; ii < m_numRows*m_numCols; ii++)
		m_data[ii] = inputMatrix.m_data[ii];
}

// constructor with array initializer (assign in NON row-traverse-first order so that assignment is intuitive)
template<class T,int R,int C>
Matrix<T,R,C>::Matrix(T arrayInit[])
{
	m_numRows = R;
	m_numCols = C;
	for (int row = 0; row < R; row++)
	{
		for (int col = 0; col < C; col++)
		{
			m_data[col*R + row] = arrayInit[row*C + col];
		}
	}
}

// addition operator
template<class T, int R, int C>
Matrix<T,R,C> Matrix<T,R,C>::operator+(const Matrix<T,R,C>& rhs) const
{
	Matrix<T,R,C> lhs = rhs;
	for (int ii = 0; ii < m_numRows*m_numCols; ii++)
		lhs = this->m_data[ii] + rhs.m_data[ii];
}

// subtraction operator
template<class T, int R, int C>
Matrix<T,R,C> Matrix<T,R,C>::operator-(const Matrix<T,R,C>& rhs) const
{
	Matrix<T,R,C> lhs = rhs;
	for (int ii = 0; ii < m_numRows*m_numCols; ii++)
		lhs = this->m_data[ii] - rhs.m_data[ii];
}

// element-by-element multiplication
template<class T, int R, int C>
Matrix<T,R,C> Matrix<T,R,C>::operator*(const T& factor) const
{
	Matrix<T,R,C> outMatrix = *this;
	for (int ii = 0; ii < outMatrix.getNumElements(); ii++)
	{
		outMatrix(ii) *= factor;
	}
	return outMatrix;
}

// element-by-element multiplication
template<class T, int R, int C>
Matrix<T,R,C> Matrix<T,R,C>::operator/(const T& divisor) const
{
	Matrix<T,R,C> outMatrix = *this;
	for (int ii = 0; ii < outMatrix.getNumElements(); ii++)
	{
		outMatrix(ii) /= divisor;
	}
	return outMatrix;
}

// multiplication operator
template<class T, int R, int C>
template<int RR, int CR>
Matrix<T,R,CR> Matrix<T,R,C>::operator*(const Matrix<T,RR,CR>& rhs) const
{
	// check dimension consistency
	//assert(C == RR);
	if (C != RR)
		printf("ERROR: Incompatible dimensions during matrix multiplication!\n");

	// declare product result
	Matrix<T,R,CR> lhs;
	
	// calculate product for each output matrix element
	for (int rowOut = 0; rowOut < R; rowOut++)
	{
		for (int colOut = 0; colOut < CR; colOut++)
		{
			lhs(rowOut,colOut) = 0;
			for (int ii = 0; ii < C; ii++)
				lhs(rowOut,colOut) += (*this)(rowOut,ii)*rhs(ii,colOut);
		}
	}
	
	// return matrix product
	return lhs;	
		
}

// transpose operation
template<class T, int R, int C>
Matrix<T,C,R> Matrix<T,R,C>::transpose() const
{
	// declare transposed matrix and assign transposed values
	Matrix<T,C,R> outputMatrix;
	for (int row = 0; row < m_numRows; row++)
	{
		for (int col = 0; col < m_numCols; col++)
		{
			outputMatrix(col,row) = (*this)(row,col);
		}
	}
	
	// return transposed matrix
	return outputMatrix;
}

// 2D indexing operator (return reference)
template<class T, int R, int C>
T& Matrix<T,R,C>::operator()(const int rowIdx, const int colIdx)
{
	return m_data[m_numRows*colIdx + rowIdx];
}

// 1D indexing operator (return reference)
template<class T, int R, int C>
T& Matrix<T,R,C>::operator()(const int index)
{
	return m_data[index];
}

// 2D indexing operator (return copy)
template<class T, int R, int C>
T Matrix<T,R,C>::operator()(const int rowIdx, const int colIdx) const
{
	return m_data[m_numRows*colIdx + rowIdx];
}

// 1D indexing operator (return copy)
template<class T, int R, int C>
T Matrix<T,R,C>::operator()(const int index) const
{
	return m_data[index];
}

// get number of elements in this Matrix
template<class T, int R, int C>
int Matrix<T,R,C>::getNumElements() const
{
	return m_numRows*m_numCols;
}

// get R vector containing length of each dimension of this Matrix
template<class T, int R, int C>
Matrix<int,1,2> Matrix<T,R,C>::size()
{
	Matrix<int,1,2> sizeMatrix;
	sizeMatrix(0) = m_numRows;
	sizeMatrix(1) = m_numCols;
	return sizeMatrix;
}

// print values to command line
template<class T, int R, int C>
void Matrix<T,R,C>::printValues()
{
	printf("\n");
	for (int row = 0; row < R; row++)
	{
		for (int col = 0; col < C - 1; col++)
		{
			printf("%16.6f\t",m_data[col*R + row]);
		}
		printf("%16.6f\n",m_data[(C-1)*R + row]);
	}
	printf("\n");
}

// matrix inversion method
//
// This method uses Gauss Jordan elimination with partial pivoting (only row swapping to find largest pivot)
template<class T, int R, int C>
Matrix<T,R,C> Matrix<T,R,C>::inverse() const
{
	// assert that this matrix is square
	//assert(R == C);
	if (R != C)
		printf("ERROR: Attempt to invert a non-square matrix!\n");
	
	// construct new matrix with this matrix and identity side-by-side
	Matrix<T,R,2*C> dualMatrix;
	for (int row = 0; row < R; row++)
	{
		for (int col = 0; col < C; col++)
		{
			// copy this matrix to left half of dual matrix
			dualMatrix(row,col) = (*this)(row,col);
			
			// populate identity matrix
			if (row == col)
			{
				dualMatrix(row,col+C) = 1;
			}
			else
			{
				dualMatrix(row,col+C) = 0;
			}
			
		}
	}
	
	// loop over each column in left side of dual matrix
	for (int col = 0; col < C; col++)
	{
	
		// find pivot in this column (max magnitude element), search only beyond current column
		T maxMag = 0;
		int maxRowIdx = 0;
		for (int row = col; row < R; row++)
		{
			if (fabs(dualMatrix(row,col)) > maxMag)
			{
				maxMag = fabs(dualMatrix(row,col));
				maxRowIdx = row;
			}
		}

		/*
		// check for singularity
		if (maxMag < 100*numeric_limits<T>::epsilon())
		{
			error(1,1,"Inversion of a singular matrix");
		}
		*/
		
		// swap current row (same index as current column) and pivot row
		dualMatrix.rowSwap(col,maxRowIdx);
		
		// divide current column's row by pivot value
		T normalizer = dualMatrix(col,col);
		for (int ii = col; ii < 2*C; ii++)
		{
			dualMatrix(col,ii) /= normalizer;
		}
		
		// null other entries in this column
		for (int row = 0; row < R; row++)
		{
			// skip current column's row
			if (row == col)
			{
				continue;
			}
			else // subtract multiple of pivot row from this row to null out pivot column element
			{
				T multiplier = dualMatrix(row,col);
				for (int ii = col; ii < 2*C; ii++)
				{
					dualMatrix(row,ii) -= dualMatrix(col,ii)*multiplier;
				}
			}
		}

	}
	
	// return right side of dual matrix after inversion
	Matrix<T,R,C> matrixInverse;
	for (int row = 0; row < R; row++)
	{
		for (int col = 0; col < C; col++)
		{
			matrixInverse(row,col) = dualMatrix(row,col+C);
		}
	}
	return matrixInverse;
		
}

// row swap method
template<class T, int R, int C>
Matrix<T,R,C>& Matrix<T,R,C>::rowSwap(int row1Idx, int row2Idx)
{
	// declare temp variable
	T tempVar;
	
	// loop over row elements
	for (int colIdx = 0; colIdx < C; colIdx++)
	{
		// copy row 1 element value
		tempVar = (*this)(row1Idx,colIdx);
		
		// copy row 2 element value to row 1 element
		(*this)(row1Idx,colIdx) = (*this)(row2Idx,colIdx);
		
		// copy original row 1 value to row 2 element
		(*this)(row2Idx,colIdx) = tempVar;
	}
	
	return *this;
}

// 2-norm method
template<class T, int R, int C>
T Matrix<T,R,C>::norm() const
{
	T normVal = 0;
	for (int ii = 0; ii < this->getNumElements(); ii++)
	{
		normVal += m_data[ii]*m_data[ii];
	}
	return sqrt(normVal);
}

template<class T, int R, int C>
Matrix<T,R,C> Matrix<T,R,C>::unit() const
{
	Matrix<T,R,C> outMatrix = *this;
    T mag = norm();
    if (mag > 1e-6)
        outMatrix = outMatrix/mag;
    else
    {
        outMatrix(0) = 1;
        outMatrix(1) = 0;
        outMatrix(2) = 0;
    }

	return outMatrix;
}

// define cross product for 3-vectors
static Vec3d cross(
       const Vec3d & v1,
       const Vec3d & v2)
{
    Vec3d v3;
    v3(0) =   v1(1) * v2(2) - v1(2) * v2(1);
    v3(1) = -(v1(0) * v2(2) - v1(2) * v2(0));
    v3(2) =   v1(0) * v2(1) - v1(1) * v2(0);

	return v3;
}

// define dot product for 3-vectors
static double dot(
       const Vec3d & v1,
       const Vec3d & v2)
{
	return v1(0)*v2(0) + v1(1)*v2(1) + v1(2)*v2(2);
}


// get angle between two vectors in radians
// note: arguments don't have to be unit vectors
static double angle_between(
       const Vec3d & v1,
       const Vec3d & v2)
{
    // handle undefined case
    if (v1.norm() == 0 || v2.norm() == 0)
        return 0;

    // note: for non unit arguments, the magnitudes 
    // appear in both numerator and denominator of atan
    // and cancel out, giving the correct answer.
    return atan2(cross(v1,v2).norm(),
                      dot(v1,v2));
}

#endif // end Matrix header include guard
