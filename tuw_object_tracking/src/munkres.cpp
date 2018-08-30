#include "munkres.h"

using namespace tuw;

std::vector<std::pair<int, int>> Munkres::find_minimum_assignment(const MatrixXd costmatrix)
{
  MatrixXd C;
  Matrix<Zero, Dynamic, Dynamic> M;
  std::vector<bool> row_cover, col_cover;
  std::vector<std::pair<int, int>> path, result;
  bool done = false;
  bool transpose;
  int k;
  int step = 0;

  while (!done)
  {
    // invariance check
    assert(0 <= step && step <= 7);
    assert(C.cols() >= C.rows());
    assert((C.rows() == M.rows() && C.cols() == M.cols()) || step < 3);
    assert(M.rows() == (int)row_cover.size() && M.cols() == (int)col_cover.size());

    switch (step)
    {
      case 0:
        step_0(costmatrix, C, transpose, k, step);
        break;
      case 1:
        step_1(C, step);
        break;
      case 2:
        step_2(C, M, row_cover, col_cover, step);
        break;
      case 3:
        step_3(M, col_cover, k, step);
        break;
      case 4:
        step_4(C, M, row_cover, col_cover, path, step);
        break;
      case 5:
        step_5(M, row_cover, col_cover, path, step);
        break;
      case 6:
        step_6(C, row_cover, col_cover, step);
        break;
      case 7:
        step_7(M, transpose, result, k, step);
        done = true;
        break;
      default:
        assert(false);
        break;
    }

    //print (C, M, row_cover, col_cover, step);
  }

  return result;
}

void Munkres::step_0(const MatrixXd costmatrix, MatrixXd& C, bool &transpose, int &k, int &step)
{
  assert(step == 0);

  // copy/save original data
  //C.resize(costmatrix.rows(), costmatrix.cols());
  C = costmatrix;

  // C must have at least as many columns as rows
  if (C.rows() > C.cols())
  {
    C.transposeInPlace();
    transpose = true;
  }
  else
  {
    transpose = false;
  }

  k = C.rows();
  step = 1;
}

void Munkres::step_1(MatrixXd &C, int &step)
{
  assert(step == 1);

  for (int r = 0; r < C.rows(); r++)
  {
    // find smallest element in current row
    double min = C(r, 0);
    for (int c = 1; c < C.cols(); c++)
    {
      if (C(r, c) < min)
        min = C(r, c);
    }

    // subtract smallest element from every element in current row
    for (int c = 0; c < C.cols(); c++)
    {
      C(r, c) -= min;
      if (std::abs(C(r, c)) < __DBL_MIN__)
        C(r, c) = 0;
    }
  }

  step = 2;
}

void Munkres::step_2(const MatrixXd C, Matrix<Zero, Dynamic, Dynamic> &M, std::vector<bool> &row_cover,
                     std::vector<bool> &col_cover, int &step)
{
  assert(step == 2);

  /* M[r][c] =
   * UNDEF ... C[r][c] is undefined yet
   * STAR  ... C[r][c] is a starred zero
   * PRIME ... C[r][c] is a primed zero
   */
  M = Matrix<Zero, Dynamic, Dynamic>::Constant(C.rows(), C.cols(), UNDEF);

  /* row_cover[r] =
   * false ... row r contains no starred zero
   * true  ... row r contains a starred zero
   */
  row_cover = std::vector<bool>(C.rows());

  /* col_cover[c] =
   * false ... column c contains no starred zero
   * true  ... column c contains a starred zero
   */
  col_cover = std::vector<bool>(C.cols());

  // find zeros and if applicable mark them starred
  for (int r = 0; r < C.rows(); r++)
  {
    for (int c = 0; c < C.cols(); c++)
    {
      if (C(r, c) == 0 && !row_cover[r] && !col_cover[c])
      {
        M(r, c) = STAR;
        row_cover[r] = true;
        col_cover[c] = true;
      }
    }
  }

  // clear covers
  for (size_t r = 0; r < row_cover.size(); r++)
    row_cover[r] = false;
  for (size_t c = 0; c < col_cover.size(); c++)
    col_cover[c] = false;

  step = 3;
}

void Munkres::step_3(const Matrix<Zero, Dynamic, Dynamic> M, std::vector<bool> &col_cover, const int k, int &step)
{
  assert(step == 3);

  // cover and count all columns containing a starred zero
  int count = 0;
  for (int c = 0; c < M.cols(); c++)
  {
    for (int r = 0; r < M.rows(); r++)
    {
      if (M(r, c) == 1)
      {
        col_cover[c] = true;
        count++;
        break;
      }
    }
  }

  if (count >= k)
    step = 7;
  else
    step = 4;
}

void Munkres::step_4(const MatrixXd C, Matrix<Zero, Dynamic, Dynamic> &M, std::vector<bool> &row_cover,
                     std::vector<bool> &col_cover, std::vector<std::pair<int, int>> &path, int &step)
{
  assert(step == 4);
  
  while (true)
  {
    int row, col;
    find_uncovered_zero(C, row_cover, col_cover, row, col);
    
    if (0 <= row && row < M.rows() && 0 <= col && col < M.cols())
    {
      // prime the found uncovered zero
      M(row, col) = PRIME;
      int col_prime = col;

      col = find_zero_in_row(M, row, STAR);
      if (!(0 <= col && col < M.cols()))
      {
        // no starred zero in the row containing the currently primed zero
        path = std::vector<std::pair<int, int>>(1);
        path[0] = std::pair<int, int>((int)row, (int)col_prime);
        step = 5;
        break;
      }
      else
      {
        // cover the row containing the currently primed zero and
        // uncover the column containing the found starred zero
        row_cover[row] = true;
        col_cover[col] = false;
      }
    }
    else
    {
      // no uncovered zero left
      step = 6;
      break;
    }
  }
}

void Munkres::step_5(Matrix<Zero, Dynamic, Dynamic> &M, std::vector<bool> &row_cover, std::vector<bool> &col_cover,
                     std::vector<std::pair<int, int>> &path, int &step)
{
  assert(step == 5);
  assert(path.size() == 1);

  while (true)
  {
    int row, col;
    std::pair<int, int> Z0, Z1, Z2;

    // Z0...primed zero (of last time)
    Z0 = path[path.size() - 1];

    // Z1...starred zero in the column of Z0 (can exist)
    row = find_zero_in_col(M, Z0.second, STAR);
    if (0 <= row && row < M.rows())
    {
      Z1 = std::pair<int, int>((int)row, Z0.second);
      path.push_back(Z1);
    }
    else
      break;

    // Z2...primed zero in the row of Z1 (must exist).
    col = find_zero_in_row(M, Z1.first, PRIME);
    assert(0 <= col && col < M.cols());
    Z2 = std::pair<int, int>(Z1.first, (int)col);
    path.push_back(Z2);
  }

  // augment path
  // - starred zero gets unstarred
  // - primed zero gets starred
  for (size_t i = 0; i < path.size(); i++)
  {
    assert(M(path[i].first, path[i].second) == PRIME || M(path[i].first, path[i].second) == STAR);

    if (M(path[i].first, path[i].second) == STAR)
      M(path[i].first, path[i].second) = UNDEF;
    else
      M(path[i].first, path[i].second) = STAR;
  }

  // erase primes
  for (int r = 0; r < M.rows(); r++)
  {
    for (int c = 0; c < M.cols(); c++)
    {
      if (M(r, c) == PRIME)
        M(r, c) = UNDEF;
    }
  }

  // clear covers
  for (size_t r = 0; r < row_cover.size(); r++)
    row_cover[r] = false;
  for (size_t c = 0; c < col_cover.size(); c++)
    col_cover[c] = false;

  step = 3;
}

void Munkres::step_6(MatrixXd &C, const std::vector<bool> row_cover, const std::vector<bool> col_cover, int &step)
{
  // find smallest uncovered element
  double min = std::numeric_limits<double>::infinity();
  for (int r = 0; r < C.rows(); r++)
  {
    for (int c = 0; c < C.cols(); c++)
    {      
      if (!row_cover[r] && !col_cover[c] && C(r, c) < min)
      {
        min = C(r, c);
      }
    }
  }

  assert(min < std::numeric_limits<double>::infinity());

  // add found value to every element of each covered row and
  // subtract found value from every element of each uncovered column
  for (int r = 0; r < C.rows(); r++)
  {
    for (int c = 0; c < C.cols(); c++)
    {
      if (row_cover[r])
        C(r, c) += min;
      if (!col_cover[c])
        C(r, c) -= min;
      if (std::abs(C(r, c)) < __DBL_MIN__)
        C(r, c) = 0;
    }
  }

  step = 4;
}

void Munkres::step_7(const Matrix<Zero, Dynamic, Dynamic> M, const bool transpose,
                     std::vector<std::pair<int, int>> &result, int &k, int &step)
{
  assert(step == 7);

  // prepare result
  int i = 0;
  result = std::vector<std::pair<int, int>>(k);
  for (int r = 0; r < M.rows(); r++)
  {
    for (int c = 0; c < M.cols(); c++)
    {
      if (M(r, c) == STAR)
      {
        // restore original arrangement
        if (transpose)
          result[i] = std::pair<int, int>(c, r);
        else
          result[i] = std::pair<int, int>(r, c);
        i++;
      }
    }
  }
  assert(i == k);

  step = 8;
}

void Munkres::find_uncovered_zero(const MatrixXd C, const std::vector<bool> row_cover,
                                  const std::vector<bool> col_cover, int &row, int &col)
{
  assert(C.rows() == (int)row_cover.size() && C.cols() == (int)col_cover.size());
  
  row = -1;
  col = -1;
  for (int r = 0; r < C.rows(); r++)
  {
    for (int c = 0; c < C.cols(); c++)
    {
      if (C(r, c) == 0 && !row_cover[r] && !col_cover[c])
      {
        row = r;
        col = c;

        return;
      }
    }
  }
}

int Munkres::find_zero_in_row(const Matrix<Zero, Dynamic, Dynamic> M, const int row, const Zero type)
{
  for (int c = 0; c < M.cols(); c++)
  {
    if (M(row, c) == type)
      return c;
  }

  return -1;
}

int Munkres::find_zero_in_col(const Matrix<Zero, Dynamic, Dynamic> M, const int col, const Zero type)
{
  for (int r = 0; r < M.rows(); r++)
  {
    if (M(r, col) == type)
      return r;
  }

  return -1;
}

void Munkres::print(const MatrixXd C, const Matrix<Zero, Dynamic, Dynamic> M, const std::vector<bool> row_cover,
                    const std::vector<bool> col_cover, const int step)
{
  printf("next step %d:\n", step);

  for (int r = 0; r < C.rows(); r++)
  {
    if (r == 0)
      printf("[");
    else
      printf(" ");

    for (int c = 0; c < C.cols(); c++)
    {
      if (row_cover[r] || col_cover[c])
        printf("(");
      else
        printf(" ");

      printf("%.2f", C(r, c));

      if (row_cover[r] || col_cover[c])
        printf(")");
      else
        printf(" ");

      if (M.rows() == C.rows() && M.cols() == C.cols())
      {
        switch (M(r, c))
        {
          case UNDEF:
            printf(" ");
            break;
          case STAR:
            printf("*");
            break;
          case PRIME:
            printf("'");
            break;
        }
      }

      if (c < C.cols() - 1)
        printf(", ");
    }

    if (r == C.rows() - 1)
      printf("]\n");
    else
      printf(";\n");
  }
}
