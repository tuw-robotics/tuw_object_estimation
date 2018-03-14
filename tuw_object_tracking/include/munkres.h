#ifndef MUNKRES_H
#define MUNKRES_H

#include <eigen3/Eigen/Dense>

using Eigen::Matrix;
using Eigen::MatrixXd;
using Eigen::Dynamic;
using Eigen::Ref;

namespace tuw
{
/**
 * This class implements Munkre's assignment algorithm
 *
 * The implementation itself is based on:
 * @see http://csclab.murraystate.edu/bob.pilgrim/445/munkres.html
 * In this sense, a special thanks goes to the authors and their work!
 */
class Munkres
{
public:
  /**
   * Finds a minimum assignment of rows and columns in the given costmatrix
   *
   * @param costmatrix initial position for the minimum assignment problem (remains unaffected)
   * @return Returns a vector of paired indices (i, j) denoting row i corresponds to column j
   *         in a minimum assignment of the original costmatrix
   */
  static std::vector<std::pair<int, int>> find_minimum_assignment(const MatrixXd costmatrix);

private:
  /**
   * Different kinds of zeros used in Munkre's assignment algorithm
   */
  enum Zero
  {
    UNDEF = 0,
    STAR = 1,
    PRIME = 2,
  };

  /**
   * Create an nxm  matrix called the cost matrix in which each element represents e.g. the cost of
   * assigning one of n workers to one of m jobs. Rotate the matrix so that there are at least as
   * many columns as rows and let k=min(n,m).
   */
  static void step_0(const MatrixXd costmatrix, MatrixXd &C, bool &transpose, int &k, int &step);

  /**
   * For each row of the matrix, find the smallest element and subtract it from every element in its row.
   * Go to Step 2.
   */
  static void step_1(MatrixXd &C, int &step);

  /**
   * Find a zero (Z) in the resulting matrix.  If there is no starred zero in its row or column,
   * star Z. Repeat for each element in the matrix. Go to Step 3.
   */
  static void step_2(const MatrixXd C, Matrix<Zero, Dynamic, Dynamic> &M, std::vector<bool> &row_cover,
                     std::vector<bool> &col_cover, int &step);

  /**
   * Cover each column containing a starred zero.
   * If k columns are covered, the starred zeros describe a complete set of unique assignments.
   * In this case, Go to Step 7, otherwise, Go to Step 4.
   */
  static void step_3(const Matrix<Zero, Dynamic, Dynamic> M, std::vector<bool> &col_cover, const int k, int &step);

  /**
   * Find an uncovered zero and prime it.
   * If there is no starred zero in the row containing this primed zero, Go to Step 5.
   * Otherwise, cover this row and uncover the column containing the starred zero.
   * Continue in this manner until there are no uncovered zeros left and Go to Step 6.
   */
  static void step_4(const MatrixXd C, Matrix<Zero, Dynamic, Dynamic> &M, std::vector<bool> &row_cover,
                     std::vector<bool> &col_cover, std::vector<std::pair<int, int>> &path, int &step);

  /**
   * Construct a series of alternating primed and starred zeros as follows.
   * Let Z0 represent the uncovered primed zero found in Step 4.
   * Let Z1 denote the starred zero in the column of Z0 (if any).
   * Let Z2 denote the primed zero in the row of Z1 (there will always be one).
   * Continue until the series terminates at a primed zero that has no starred zero in its column.
   * Unstar each starred zero of the series, star each primed zero of the series,
   * erase all primes and uncover every line in the matrix.
   * Return to Step 3.
   */
  static void step_5(Matrix<Zero, Dynamic, Dynamic> &M, std::vector<bool> &row_cover, std::vector<bool> &col_cover,
                     std::vector<std::pair<int, int>> &path, int &step);

  /**
   * Add the smallest uncovered value to every element of each covered row, and
   * subtract it from every element of each uncovered column.
   * Return to Step 4 without altering any stars, primes, or covered lines.
   */
  static void step_6(MatrixXd &C, const std::vector<bool> row_cover, const std::vector<bool> col_cover, int &step);

  /**
   * DONE
   *
   * Assignment pairs are indicated by the positions of the starred zeros in the cost matrix.
   * If C(i,j) is a starred zero, then the element associated with row i is assigned to the
   * element associated with column j.
   */
  static void step_7(const Matrix<Zero, Dynamic, Dynamic> M, const bool transpose,
                     std::vector<std::pair<int, int>> &result, int &k, int &step);

  /**
   * Find an uncovered zero.
   */
  static void find_uncovered_zero(const MatrixXd C, const std::vector<bool> row_cover,
                                  const std::vector<bool> col_cover, int &row, int &col);

  /**
   * Find a starred zero in a row.
   */
  static int find_zero_in_row(const Matrix<Zero, Dynamic, Dynamic> M, const int row, const Zero type);

  /**
   * Find a starred zero in a row.
   */
  static int find_zero_in_col(const Matrix<Zero, Dynamic, Dynamic> M, const int col, const Zero type);

  /**
   * Prints the current state.
   */
  static void print(const MatrixXd C, const Matrix<Zero, Dynamic, Dynamic> M, const std::vector<bool> row_cover,
                    const std::vector<bool> col_cover, const int step);
};
};

#endif  // MUNKRES_H
