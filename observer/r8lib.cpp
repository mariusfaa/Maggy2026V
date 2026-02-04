# include <cfloat>
# include <cmath>
# include <complex>
# include <cstdlib>
# include <cstring>
# include <fstream>
# include <iomanip>
# include <iostream>

using namespace std;

# include "r8lib.hpp"

//****************************************************************************80

int i4_log_10 ( int i )

//****************************************************************************80
//
//  Purpose:
//
//    i4_log_10() returns the integer part of the logarithm base 10 of an I4.
//
//  Example:
//
//        I  I4_LOG_10
//    -----  --------
//        0    0
//        1    0
//        2    0
//        9    0
//       10    1
//       11    1
//       99    1
//      100    2
//      101    2
//      999    2
//     1000    3
//     1001    3
//     9999    3
//    10000    4
//
//  Discussion:
//
//    I4_LOG_10 ( I ) + 1 is the number of decimal digits in I.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    04 January 2004
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int I, the number whose logarithm base 10 is desired.
//
//  Output:
//
//    int I4_LOG_10, the integer part of the logarithm base 10 of
//    the absolute value of X.
//
{
  int i_abs;
  int ten_pow;
  int value;

  if ( i == 0 )
  {
    value = 0;
  }
  else
  {
    value = 0;
    ten_pow = 10;

    i_abs = abs ( i );

    while ( ten_pow <= i_abs )
    {
      value = value + 1;
      ten_pow = ten_pow * 10;
    }

  }

  return value;
}
//****************************************************************************80

int i4_modp ( int i, int j )

//****************************************************************************80
//
//  Purpose:
//
//    i4_modp() returns the nonnegative remainder of I4 division.
//
//  Discussion:
//
//    If
//      NREM = I4_MODP ( I, J )
//      NMULT = ( I - NREM ) / J
//    then
//      I = J * NMULT + NREM
//    where NREM is always nonnegative.
//
//    The MOD function computes a result with the same sign as the
//    quantity being divided.  Thus, suppose you had an angle A,
//    and you wanted to ensure that it was between 0 and 360.
//    Then mod(A,360) would do, if A was positive, but if A
//    was negative, your result would be between -360 and 0.
//
//    On the other hand, I4_MODP(A,360) is between 0 and 360, always.
//
//        I         J     MOD  I4_MODP   I4_MODP Factorization
//
//      107        50       7       7    107 =  2 *  50 + 7
//      107       -50       7       7    107 = -2 * -50 + 7
//     -107        50      -7      43   -107 = -3 *  50 + 43
//     -107       -50      -7      43   -107 =  3 * -50 + 43
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    26 May 1999
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int I, the number to be divided.
//
//    int J, the number that divides I.
//
//  Output:
//
//    int I4_MODP, the nonnegative remainder when I is
//    divided by J.
//
{
  int value;

  if ( j == 0 )
  {
    cerr << "\n";
    cerr << "I4_MODP - Fatal error!\n";
    cerr << "  I4_MODP ( I, J ) called with J = " << j << "\n";
    exit ( 1 );
  }

  value = i % j;

  if ( value < 0 )
  {
    value = value + abs ( j );
  }

  return value;
}
//****************************************************************************80

int i4_power ( int i, int j )

//****************************************************************************80
//
//  Purpose:
//
//    i4_power() returns the value of I^J.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    01 April 2004
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int I, J, the base and the power.  J should be nonnegative.
//
//  Output:
//
//    int I4_POWER, the value of I^J.
//
{
  int k;
  int value;

  if ( j < 0 )
  {
    if ( i == 1 )
    {
      value = 1;
    }
    else if ( i == 0 )
    {
      cerr << "\n";
      cerr << "I4_POWER - Fatal error!\n";
      cerr << "  I^J requested, with I = 0 and J negative.\n";
      exit ( 1 );
    }
    else
    {
      value = 0;
    }
  }
  else if ( j == 0 )
  {
    if ( i == 0 )
    {
      cerr << "\n";
      cerr << "I4_POWER - Fatal error!\n";
      cerr << "  I^J requested, with I = 0 and J = 0.\n";
      exit ( 1 );
    }
    else
    {
      value = 1;
    }
  }
  else if ( j == 1 )
  {
    value = i;
  }
  else
  {
    value = 1;
    for ( k = 1; k <= j; k++ )
    {
      value = value * i;
    }
  }
  return value;
}
//****************************************************************************80

int i4_sign ( int i )

//****************************************************************************80
//
//  Purpose:
//
//    i4_sign() returns the sign of an I4.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    27 March 2004
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int I, the integer whose sign is desired.
//
//  Output:
//
//    int I4_SIGN, the sign of I.
{
  int value;

  if ( i < 0 )
  {
    value = -1;
  }
  else
  {
    value = 1;
  }
  return value;
}
//****************************************************************************80

int i4_uniform_ab ( int a, int b )

//****************************************************************************80
//
//  Purpose:
//
//    i4_uniform_ab() returns a scaled pseudorandom I4 between A and B.
//
//  Discussion:
//
//    The pseudorandom number should be uniformly distributed
//    between A and B.
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    25 June 2024
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int A, B, the limits of the interval.
//
//  Output:
//
//    int I4_UNIFORM_AB, a number between A and B.
//
{
  float r;
  int value;

  r = ( float ) ( rand ( ) ) / ( float ) ( RAND_MAX );
  value = a + ( int ) ( ( b + 1 - a ) * r );

  return value;
}
//****************************************************************************80

int i4_wrap ( int ival, int ilo, int ihi )

//****************************************************************************80
//
//  Purpose:
//
//    i4_wrap() forces an I4 to lie between given limits by wrapping.
//
//  Example:
//
//    ILO = 4, IHI = 8
//
//    I   Value
//
//    -2     8
//    -1     4
//     0     5
//     1     6
//     2     7
//     3     8
//     4     4
//     5     5
//     6     6
//     7     7
//     8     8
//     9     4
//    10     5
//    11     6
//    12     7
//    13     8
//    14     4
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    19 August 2003
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int IVAL, an integer value.
//
//    int ILO, IHI, the desired bounds for the integer value.
//
//  Output:
//
//    int I4_WRAP, a "wrapped" version of IVAL.
//
{
  int jhi;
  int jlo;
  int value;
  int wide;

  jlo = min ( ilo, ihi );
  jhi = max ( ilo, ihi );

  wide = jhi + 1 - jlo;

  if ( wide == 1 )
  {
    value = jlo;
  }
  else
  {
    value = jlo + i4_modp ( ival - jlo, wide );
  }

  return value;
}
//****************************************************************************80

double i4int_to_r8int ( int imin, int imax, int i, double rmin, double rmax )

//****************************************************************************80
//
//  Purpose:
//
//    i4int_to_r8int() maps an I4 interval to an R8 interval.
//
//  Discussion:
//
//    The formula is
//
//      R := RMIN + ( RMAX - RMIN ) * ( I - IMIN ) / ( IMAX - IMIN )
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    18 October 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int IMIN, IMAX, the range.
//
//    int I, the integer to be converted.
//
//    double RMIN, RMAX, the range.
//
//  Output:
//
//    double R, the corresponding value in [RMIN,RMAX].
//
{
  double r;

  if ( imax == imin )
  {
    r = 0.5 * ( rmin + rmax );
  }
  else
  {
    r = ( ( double ) ( imax - i        ) * rmin
        + ( double ) (        i - imin ) * rmax )
        / ( double ) ( imax     - imin );
  }

  return r;
}
//****************************************************************************80

void i4mat_print ( int m, int n, int a[], string title )

//****************************************************************************80
//
//  Purpose:
//
//    i4mat_print() prints an I4MAT.
//
//  Discussion:
//
//    An I4MAT is an MxN array of I4's, stored by (I,J) -> [I+J*M].
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    10 September 2009
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, the number of rows in A.
//
//    int N, the number of columns in A.
//
//    int A[M*N], the M by N matrix.
//
//    string TITLE, a title.
//
{
  i4mat_print_some ( m, n, a, 1, 1, m, n, title );

  return;
}
//****************************************************************************80

void i4mat_print_some ( int m, int n, int a[], int ilo, int jlo, int ihi,
  int jhi, string title )

//****************************************************************************80
//
//  Purpose:
//
//    i4mat_print_some() prints some of an I4MAT.
//
//  Discussion:
//
//    An I4MAT is an MxN array of I4's, stored by (I,J) -> [I+J*M].
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    20 August 2010
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, the number of rows of the matrix.
//    M must be positive.
//
//    int N, the number of columns of the matrix.
//    N must be positive.
//
//    int A[M*N], the matrix.
//
//    int ILO, JLO, IHI, JHI, designate the first row and
//    column, and the last row and column to be printed.
//
//    string TITLE, a title.
//
{
# define INCX 10

  int i;
  int i2hi;
  int i2lo;
  int j;
  int j2hi;
  int j2lo;

  cout << "\n";
  cout << title << "\n";

  if ( m <= 0 || n <= 0 )
  {
    cout << "\n";
    cout << "  (None)\n";
    return;
  }
//
//  Print the columns of the matrix, in strips of INCX.
//
  for ( j2lo = jlo; j2lo <= jhi; j2lo = j2lo + INCX )
  {
    j2hi = j2lo + INCX - 1;
    if ( n < j2hi )
    {
      j2hi = n;
    }
    if ( jhi < j2hi )
    {
      j2hi = jhi;
    }

    cout << "\n";
//
//  For each column J in the current range...
//
//  Write the header.
//
    cout << "  Col:";
    for ( j = j2lo; j <= j2hi; j++ )
    {
      cout << "  " << setw(6) << j - 1;
    }
    cout << "\n";
    cout << "  Row\n";
    cout << "\n";
//
//  Determine the range of the rows in this strip.
//
    if ( 1 < ilo )
    {
      i2lo = ilo;
    }
    else
    {
      i2lo = 1;
    }
    if ( ihi < m )
    {
      i2hi = ihi;
    }
    else
    {
      i2hi = m;
    }


    for ( i = i2lo; i <= i2hi; i++ )
    {
//
//  Print out (up to INCX) entries in row I, that lie in the current strip.
//
      cout << setw(5) << i - 1 << ":";
      for ( j = j2lo; j <= j2hi; j++ )
      {
        cout << "  " << setw(6) << a[i-1+(j-1)*m];
      }
      cout << "\n";
    }
  }

  return;
# undef INCX
}
//****************************************************************************80

void i4vec_copy ( int n, int a1[], int a2[] )

//****************************************************************************80
//
//  Purpose:
//
//    i4vec_copy() copies an I4VEC.
//
//  Discussion:
//
//    An I4VEC is a vector of I4's.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    25 April 2007
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of entries in the vectors.
//
//    int A1[N], the vector to be copied.
//
//  Output:
//
//    int A2[N], the copy of A1.
//
{
  int i;

  for ( i = 0; i < n; i++ )
  {
    a2[i] = a1[i];
  }
  return;
}
//****************************************************************************80

int *i4vec_indicator0_new ( int n )

//****************************************************************************80
//
//  Purpose:
//
//    i4vec_indicator0_new() sets an I4VEC to the indicator vector (0,1,2,...).
//
//  Discussion:
//
//    An I4VEC is a vector of I4's.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    27 September 2014
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of elements of A.
//
//  Output:
//
//    int I4VEC_INDICATOR0_NEW[N], the array.
//
{
  int *a;
  int i;

  a = new int[n];

  for ( i = 0; i < n; i++ )
  {
    a[i] = i;
  }
  return a;
}
//****************************************************************************80

int *i4vec_indicator1_new ( int n )

//****************************************************************************80
//
//  Purpose:
//
//    i4vec_indicator1_new() sets an I4VEC to the indicator vector (1,2,3,...).
//
//  Discussion:
//
//    An I4VEC is a vector of I4's.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    27 September 2014
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of elements of A.
//
//  Output:
//
//    int I4VEC_INDICATOR1_NEW[N], the array.
//
{
  int *a;
  int i;

  a = new int[n];

  for ( i = 0; i < n; i++ )
  {
    a[i] = i + 1;
  }
  return a;
}
//****************************************************************************80

void i4vec_permute ( int n, int p[], int a[] )

//****************************************************************************80
//
//  Purpose:
//
//    i4vec_permute() permutes an I4VEC in place.
//
//  Discussion:
//
//    An I4VEC is a vector of I4's.
//
//    This routine permutes an array of integer "objects", but the same
//    logic can be used to permute an array of objects of any arithmetic
//    type, or an array of objects of any complexity.  The only temporary
//    storage required is enough to store a single object.  The number
//    of data movements made is N + the number of cycles of order 2 or more,
//    which is never more than N + N/2.
//
//  Example:
//
//    Input:
//
//      N = 5
//      P = (   1,   3,   4,   0,   2 )
//      A = (   1,   2,   3,   4,   5 )
//
//    Output:
//
//      A    = (   2,   4,   5,   1,   3 ).
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    30 October 2008
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of objects.
//
//    int P[N], the permutation.  P(I) = J means
//    that the I-th element of the output array should be the J-th
//    element of the input array.
//
//    int A[N], the array to be permuted.
//
//  Output:
//
//    int A[N]: the permuted array.
{
  int a_temp;
  int i;
  int iget;
  int iput;
  int istart;

  if ( !perm0_check ( n, p ) )
  {
    cerr << "\n";
    cerr << "I4VEC_PERMUTE - Fatal error!\n";
    cerr << "  PERM0_CHECK rejects permutation.\n";
    exit ( 1 );
  }
//
//  In order for the sign negation trick to work, we need to assume that the
//  entries of P are strictly positive.  Presumably, the lowest number is 0.
//  So temporarily add 1 to each entry to force positivity.
//
  for ( i = 0; i < n; i++ )
  {
    p[i] = p[i] + 1;
  }
//
//  Search for the next element of the permutation that has not been used.
//
  for ( istart = 1; istart <= n; istart++ )
  {
    if ( p[istart-1] < 0 )
    {
      continue;
    }
    else if ( p[istart-1] == istart )
    {
      p[istart-1] = - p[istart-1];
      continue;
    }
    else
    {
      a_temp = a[istart-1];
      iget = istart;
//
//  Copy the new value into the vacated entry.
//
      for ( ; ; )
      {
        iput = iget;
        iget = p[iget-1];

        p[iput-1] = - p[iput-1];

        if ( iget < 1 || n < iget )
        {
          cerr << "\n";
          cerr << "I4VEC_PERMUTE - Fatal error!\n";
          cerr << "  Entry IPUT = " << iput << " of the permutation has\n";
          cerr << "  an illegal value IGET = " << iget << ".\n";
          exit ( 1 );
        }

        if ( iget == istart )
        {
          a[iput-1] = a_temp;
          break;
        }
        a[iput-1] = a[iget-1];
      }
    }
  }
//
//  Restore the signs of the entries.
//
  for ( i = 0; i < n; i++ )
  {
    p[i] = - p[i];
  }
//
//  Restore the entries.
//
  for ( i = 0; i < n; i++ )
  {
    p[i] = p[i] - 1;
  }

  return;
}
//****************************************************************************80

void i4vec_print ( int n, int a[], string title )

//****************************************************************************80
//
//  Purpose:
//
//    i4vec_print() prints an I4VEC.
//
//  Discussion:
//
//    An I4VEC is a vector of I4's.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    14 November 2003
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of components of the vector.
//
//    int A[N], the vector to be printed.
//
//    string TITLE, a title.
//
{
  int i;

  cout << "\n";
  cout << title << "\n";
  cout << "\n";
  for ( i = 0; i < n; i++ )
  {
    cout << "  " << setw(8) << i
         << ": " << setw(8) << a[i]  << "\n";
  }
  return;
}
//****************************************************************************80

void i4vec_transpose_print ( int n, int a[], string title )

//****************************************************************************80
//
//  Purpose:
//
//    i4vec_transpose_print() prints an I4VEC "transposed".
//
//  Discussion:
//
//    An I4VEC is a vector of I4's.
//
//  Example:
//
//    A = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 }
//    TITLE = "My vector:  "
//
//    My vector:      1    2    3    4    5
//                    6    7    8    9   10
//                   11
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    03 July 2004
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of components of the vector.
//
//    int A[N], the vector to be printed.
//
//    string TITLE, a title.
//
{
  int i;
  int ihi;
  int ilo;
  int title_len;

  title_len = title.length ( );

  for ( ilo = 1; ilo <= n; ilo = ilo + 5 )
  {
    ihi = ilo + 5 - 1;
    if ( n < ihi )
    {
      ihi = n;
    }

    if ( ilo == 1 )
    {
      cout << title;
    }
    else
    {
      for ( i = 1; i <= title_len; i++ )
      {
        cout << " ";
      }
    }
    for ( i = ilo; i <= ihi; i++ )
    {
      cout << setw(12) << a[i-1];
    }
    cout << "\n";
  }

  return;
}
//****************************************************************************80

void i4vec_zeros ( int n, int a[] )

//****************************************************************************80
//
//  Purpose:
//
//    i4vec_zeros() zeroes an I4VEC.
//
//  Discussion:
//
//    An I4VEC is a vector of I4's.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    01 August 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of entries in the vector.
//
//  Output:
//
//    int A[N], a vector of zeroes.
//
{
  int i;

  for ( i = 0; i < n; i++ )
  {
    a[i] = 0;
  }
  return;
}
//****************************************************************************80

int *i4vec_zeros_new ( int n )

//****************************************************************************80
//
//  Purpose:
//
//    i4vec_zeros_new() creates and zeroes an I4VEC.
//
//  Discussion:
//
//    An I4VEC is a vector of I4's.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    11 July 2008
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of entries in the vector.
//
//  Output:
//
//    int I4VEC_ZEROS_NEW[N], a vector of zeroes.
//
{
  int *a;
  int i;

  a = new int[n];

  for ( i = 0; i < n; i++ )
  {
    a[i] = 0;
  }
  return a;
}
//****************************************************************************80

double *legendre_zeros ( int order )

//****************************************************************************80
//
//  Purpose:
//
//    legendre_zeros() returns the zeros of the Legendre polynomial of degree N.
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    17 June 2011
//
//  Author:
//
//    Original FORTRAN77 version by Philip Davis, Philip Rabinowitz.
//    This version by John Burkardt.
//
//  Reference:
//
//    Philip Davis, Philip Rabinowitz,
//    Methods of Numerical Integration,
//    Second Edition,
//    Dover, 2007,
//    ISBN: 0486453391,
//    LC: QA299.3.D28.
//
//  Input:
//
//    int ORDER, the order.
//    ORDER must be greater than 0.
//
//  Output:
//
//    double LEGENDRE_ZEROS[ORDER], the zeros.
//
{
  double d1;
  double d2pn;
  double d3pn;
  double d4pn;
  double dp;
  double dpn;
  double e1;
//double fx;
  double h;
  int i;
  int iback;
  int k;
  int m;
  int mp1mi;
  int ncopy;
  int nmove;
  double p;
  double pk;
  double pkm1;
  double pkp1;
  const double r8_pi = 3.141592653589793;
  double t;
  double u;
  double v;
  double x0;
  double *xtab;
  double xtemp;

  xtab = new double[order];

  e1 = ( double ) ( order * ( order + 1 ) );

  m = ( order + 1 ) / 2;

  for ( i = 1; i <= m; i++ )
  {
    mp1mi = m + 1 - i;

    t = ( double ) ( 4 * i - 1 ) * r8_pi / ( double ) ( 4 * order + 2 );

    x0 = cos ( t ) * ( 1.0 - ( 1.0 - 1.0 / ( double ) ( order ) ) 
      / ( double ) ( 8 * order * order ) );

    pkm1 = 1.0;
    pk = x0;

    for ( k = 2; k <= order; k++ )
    {
      pkp1 = 2.0 * x0 * pk - pkm1 - ( x0 * pk - pkm1 ) / ( double ) ( k );
      pkm1 = pk;
      pk = pkp1;
    }

    d1 = ( double ) ( order ) * ( pkm1 - x0 * pk );

    dpn = d1 / ( 1.0 - x0 * x0 );

    d2pn = ( 2.0 * x0 * dpn - e1 * pk ) / ( 1.0 - x0 * x0 );

    d3pn = ( 4.0 * x0 * d2pn + ( 2.0 - e1 ) * dpn ) / ( 1.0 - x0 * x0 );

    d4pn = ( 6.0 * x0 * d3pn + ( 6.0 - e1 ) * d2pn ) / ( 1.0 - x0 * x0 );

    u = pk / dpn;
    v = d2pn / dpn;
//
//  Initial approximation H:
//
    h = -u * ( 1.0 + 0.5 * u * ( v + u * ( v * v - d3pn / ( 3.0 * dpn ) ) ) );
//
//  Refine H using one step of Newton's method:
//
    p = pk + h * ( dpn + 0.5 * h * ( d2pn + h / 3.0 
      * ( d3pn + 0.25 * h * d4pn ) ) );

    dp = dpn + h * ( d2pn + 0.5 * h * ( d3pn + h * d4pn / 3.0 ) );

    h = h - p / dp;

    xtemp = x0 + h;

    xtab[mp1mi-1] = xtemp;

//  fx = d1 - h * e1 * ( pk + 0.5 * h * ( dpn + h / 3.0 
//    * ( d2pn + 0.25 * h * ( d3pn + 0.2 * h * d4pn ) ) ) );
  }

  if ( ( order % 2 ) == 1 )
  {
    xtab[0] = 0.0;
  }
//
//  Shift the data up.
//
  nmove = ( order + 1 ) / 2;
  ncopy = order - nmove;

  for ( i = 1; i <= nmove; i++ )
  {
    iback = order + 1 - i;
    xtab[iback-1] = xtab[iback-ncopy-1];
  }
//
//  Reflect values for the negative abscissas.
//
  for ( i = 1; i <= order - nmove; i++ )
  {
    xtab[i-1] = - xtab[order-i];
  }

  return xtab;
}
//****************************************************************************80

bool perm0_check ( int n, int p[] )

//****************************************************************************80
//
//  Purpose:
//
//    perm0_check() checks a permutation of ( 0, ..., N-1 ).
//
//  Discussion:
//
//    The routine verifies that each of the integers from 0 to
//    to N-1 occurs among the N entries of the permutation.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    24 May 2015
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of entries.
//
//    int P[N], the array to check.
//
//  Output:
//
//    bool PERM0_CHECK, is 
//    TRUE if P is a legal permutation of 0,...,N-1.
//    FALSE if P is not a legal permuation of 0,...,N-1.
//
{
  bool check;
  int location;
  int value;

  check = true;

  for ( value = 0; value < n; value++ )
  {
    check = false;

    for ( location = 0; location < n; location++ )
    {
      if ( p[location] == value )
      {
        check = true;
        break;
      }
    }

    if ( ! check )
    {
      cout << "\n";
      cout << "PERM0_CHECK - Fatal error!\n";
      cout << "  Permutation is missing value " << value << "\n";
      break;
    }

  }

  return check;
}
//****************************************************************************80

int *perm0_uniform_new ( int n, int &seed )

//****************************************************************************80
//
//  Purpose:
//
//    perm0_uniform_new() selects a random permutation of 0,...,N-1.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    23 May 2015
//
//  Author:
//
//    John Burkardt
//
//  Reference:
//
//    Albert Nijenhuis, Herbert Wilf,
//    Combinatorial Algorithms,
//    Academic Press, 1978, second edition,
//    ISBN 0-12-519260-6.
//
//  Input:
//
//    int N, the number of objects to be permuted.
//
//    int &SEED, a seed for the random number generator.
//
//  Output:
//
//    int PERM0_UNIFORM_NEW[N], a permutation of
//    (0, 1, ..., N-1).
//
//    int &SEED: an updated seed.
//
{
  int i;
  int j;
  int k;
  int *p;

  p = new int[n];

  for ( i = 0; i < n; i++ )
  {
    p[i] = i;
  }

  for ( i = 0; i < n; i++ )
  {
    j = i4_uniform_ab ( i, n - 1 );
    k    = p[i];
    p[i] = p[j];
    p[j] = k;
  }

  return p;
}
//****************************************************************************80

bool perm1_check ( int n, int p[] )

//****************************************************************************80
//
//  Purpose:
//
//    perm1_check() checks a permutation of (1, ..., N ).
//
//  Discussion:
//
//    The routine verifies that each of the integers from 0 to
//    to N-1 occurs among the N entries of the permutation.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    24 May 2015
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of entries.
//
//    int P[N], the array to check.
//
//  Output:
//
//    bool PERM1_CHECK, is 
//    TRUE if P is a legal permutation of 1,...,N.
//    FALSE if P is not a legal permuation of 1,...,N.
//
{
  bool check;
  int location;
  int value;

  check = true;

  for ( value = 1; value <= n; value++ )
  {
    check = false;

    for ( location = 0; location < n; location++ )
    {
      if ( p[location] == value )
      {
        check = true;
        break;
      }
    }

    if ( ! check )
    {
      cout << "\n";
      cout << "PERM1_CHECK - Fatal error!\n";
      cout << "  Permutation is missing value " << value << "\n";
      break;
    }

  }

  return check;
}
//****************************************************************************80

int *perm1_uniform_new ( int n, int &seed )

//****************************************************************************80
//
//  Purpose:
//
//    perm1_uniform_new() selects a random permutation of 1,...,N.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    23 May 2015
//
//  Author:
//
//    John Burkardt
//
//  Reference:
//
//    Albert Nijenhuis, Herbert Wilf,
//    Combinatorial Algorithms,
//    Academic Press, 1978, second edition,
//    ISBN 0-12-519260-6.
//
//  Input:
//
//    int N, the number of objects to be permuted.
//
//    int &SEED, a seed for the random number generator.
//
//  Output:
//
//    int PERM1_UNIFORM_NEW[N], a permutation of
//    (1, ..., N).
//
//    int &SEED: an updated seed.
//
{
  int i;
  int j;
  int k;
  int *p;

  p = new int[n];

  for ( i = 0; i < n; i++ )
  {
    p[i] = i + 1;
  }

  for ( i = 0; i < n; i++ )
  {
    j = i4_uniform_ab ( i, n - 1 );
    k    = p[i];
    p[i] = p[j];
    p[j] = k;
  }

  return p;
}
//****************************************************************************80

double r8_abs ( double x )

//****************************************************************************80
//
//  Purpose:
//
//    r8_abs() returns the absolute value of an R8.
//
//  Discussion:
//
//    The C++ math library provides the function fabs() which is preferred.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    14 November 2006
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double X, the quantity whose absolute value is desired.
//
//  Output:
//
//    double R8_ABS, the absolute value of X.
//
{
  double value;

  if ( 0.0 <= x )
  {
    value = + x;
  }
  else
  {
    value = - x;
  }
  return value;
}
//****************************************************************************80

double r8_acos ( double c )

//****************************************************************************80
//
//  Purpose:
//
//    r8_acos() computes the arc cosine function, with argument truncation.
//
//  Discussion:
//
//    If you call your system ACOS routine with an input argument that is
//    outside the range [-1.0, 1.0 ], you may get an unpleasant surprise.
//    This routine truncates arguments outside the range.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    13 June 2002
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double C, the argument, the cosine of an angle.
//
//  Output:
//
//    double R8_ACOS, an angle whose cosine is C.
//
{
  const double r8_pi = 3.141592653589793;
  double value;

  if ( c <= -1.0 )
  {
    value = r8_pi;
  }
  else if ( 1.0 <= c )
  {
    value = 0.0;
  }
  else
  {
    value = acos ( c );
  }
  return value;
}
//****************************************************************************80

double r8_acosh ( double x )

//****************************************************************************80
//
//  Purpose:
//
//    r8_acosh() returns the inverse hyperbolic cosine of a number.
//
//  Discussion:
//
//    Applying the inverse function
//
//      Y = R8_ACOSH(X)
//
//    implies that
//
//      X = COSH(Y) = 0.5 * ( EXP(Y) + EXP(-Y) ).
//
//    For every X greater than or equal to 1, there are two possible
//    choices Y such that X = COSH(Y), differing only in sign.  It
//    is usual to resolve this choice by taking the value of ACOSH(X)
//    to be nonnegative.
//
//  Method:
//
//    One formula is:
//
//      R8_ACOSH = LOG ( X + SQRT ( X^2 - 1.0 ) )
//
//    but this formula suffers from roundoff and overflow problems.
//    The formula used here was recommended by W Kahan, as discussed
//    by Moler.
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    09 May 2003
//
//  Author:
//
//    John Burkardt
//
//  Reference:
//
//    Cleve Moler,
//    Trigonometry is a Complex Subject,
//    MATLAB News and Notes,
//    Summer 1998.
//
//  Input:
//
//    double X, the number whose inverse hyperbolic cosine is desired.
//    X should be greater than or equal to 1.
//
//  Output:
//
//    double R8_ACOSH, the inverse hyperbolic cosine of X.  The
//    principal value (that is, the positive value of the two ) is returned.
//
{
  double value;

  if ( x < 1.0 )
  {
    cerr << "\n";
    cerr << "R8_ACOSH - Fatal error!\n";
    cerr << "  Argument X must satisfy 1 <= X.\n";
    cerr << "  The input X = " << x << "\n";
    exit ( 1 );
  }

  value = 2.0 * log ( 
    sqrt ( 0.5 * ( x + 1.0 ) ) + sqrt ( 0.5 * ( x - 1.0 ) ) );

  return value;
}
//****************************************************************************80

double r8_add ( double x, double y )

//****************************************************************************80
//
//  Purpose:
//
//    r8_add() adds two R8's.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    11 August 2010
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double X, Y, the numbers to be added.
//
//  Output:
//
//    double R8_ADD, the sum of X and Y.
//
{
  double value;

  value = x + y;

  return value;
}
//****************************************************************************80

double r8_agm ( double a, double b )

//****************************************************************************80
//
//  Purpose:
//
//    r8_agm() computes the arithmetic-geometric mean of A and B.
//
//  Discussion:
//
//    The AGM is defined for nonnegative A and B.
//
//    The AGM of numbers A and B is defined by setting
//
//      A(0) = A,
//      B(0) = B
//
//      A(N+1) = ( A(N) + B(N) ) / 2
//      B(N+1) = sqrt ( A(N) * B(N) )
//
//    The two sequences both converge to AGM(A,B).
//
//    In Mathematica, the AGM can be evaluated by
//
//      ArithmeticGeometricMean [ a, b ]
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    27 July 2014
//
//  Author:
//
//    John Burkardt
//
//  Reference:
//
//    Stephen Wolfram,
//    The Mathematica Book,
//    Fourth Edition,
//    Cambridge University Press, 1999,
//    ISBN: 0-521-64314-7,
//    LC: QA76.95.W65.
//
//  Input:
//
//    double A, B, the arguments whose AGM is to be computed.
//    0 <= A, 0 <= B.
//
//  Output:
//
//    double R8_AGM, the arithmetic-geometric mean of A and B.
//
{
  double a1;
  double a2;
  double b1;
  double b2;
  int it;
  int it_max = 1000;
  double tol;
  double value;

  if ( a < 0.0 )
  {
    cerr << "\n";
    cerr << "R8_AGM - Fatal error!\n";
    cerr << "  A < 0.\n";
    exit ( 1 );
  }

  if ( b < 0.0 )
  {
    cerr << "\n";
    cerr << "R8_AGM - Fatal error!\n";
    cerr << "  B < 0.\n";
    exit ( 1 );
  }

  if ( a == 0.0 || b == 0.0 )
  {
    value = 0.0;
    return value;
  }

  if ( a == b )
  {
    value = a;
    return value;
  }

  a1 = a;
  b1 = b;

  it = 0;
  tol = 100.0 * DBL_EPSILON;

  for ( ; ; )
  {
    it = it + 1;

    a2 = ( a1 + b1 ) / 2.0;
    b2 = sqrt ( a1 * b1 );

    if ( fabs ( a2 - b2 ) <= tol * ( a2 + b2 ) )
    {
      break;
    }

    if ( it_max < it )
    {
      break;
    }

    a1 = a2;
    b1 = b2;
  }
  value = a2;

  return value;
}
//****************************************************************************80

double r8_aint ( double x )

//****************************************************************************80
//
//  Purpose:
//
//    r8_aint() truncates an R8 argument to an integer.
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    1 September 2011
//
//  Author:
//
//    John Burkardt.
//
//  Input:
//
//    double X, the argument.
//
//  Output:
//
//    double R8_AINT, the truncated version of X.
//
{
  double value;

  if ( x < 0.0 )
  {
    value = - ( double ) ( ( int ) ( fabs ( x ) ) );
  }
  else
  {
    value =   ( double ) ( ( int ) ( fabs ( x ) ) );
  }

  return value;
}
//****************************************************************************80

double r8_asin ( double s )

//****************************************************************************80
//
//  Purpose:
//
//    r8_asin() computes the arc sine function, with argument truncation.
//
//  Discussion:
//
//    If you call your system ASIN routine with an input argument that is
//    outside the range [-1.0, 1.0 ], you may get an unpleasant surprise.
//    This routine truncates arguments outside the range.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    13 June 2002
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double S, the argument, the sine of an angle.
//
//  Output:
//
//    double R8_ASIN, an angle whose sine is S.
//
{
  double angle;
  const double r8_pi = 3.141592653589793;

  if ( s <= -1.0 )
  {
    angle = - r8_pi / 2.0;
  }
  else if ( 1.0 <= s )
  {
    angle = r8_pi / 2.0;
  }
  else
  {
    angle = asin ( s );
  }
  return angle;
}
//****************************************************************************80

double r8_asinh ( double x )

//****************************************************************************80
//
//  Purpose:
//
//    r8_asinh() returns the inverse hyperbolic sine of a number.
//
//  Discussion:
//
//    The assertion that:
//
//      Y = R8_ASINH ( X )
//
//    implies that
//
//      X = SINH(Y) = 0.5 * ( EXP(Y) - EXP(-Y) ).
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    29 November 2007
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double X, the number whose inverse hyperbolic 
//    sine is desired.
//
//  Output:
//
//    double R8_ASINH, the inverse hyperbolic sine of X.
//
{
  double value;

  value = log ( x + sqrt ( x * x + 1.0 ) );

  return value;
}
//****************************************************************************80

double r8_atan ( double y, double x )

//****************************************************************************80
//
//  Purpose:
//
//    r8_atan() computes the inverse tangent of the ratio Y / X.
//
//  Discussion:
//
//    R8_ATAN returns an angle whose tangent is ( Y / X ), a job which
//    the built in functions ATAN and ATAN2 already do.
//
//    However:
//
//    * R8_ATAN always returns a positive angle, between 0 and 2 PI,
//      while ATAN and ATAN2 return angles in the interval [-PI/2,+PI/2]
//      and [-PI,+PI] respectively;
//
//    * R8_ATAN accounts for the signs of X and Y, (as does ATAN2).  The ATAN
//     function by contrast always returns an angle in the first or fourth
//     quadrants.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    15 August 2008
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double Y, X, two quantities which represent the tangent of
//    an angle.  If Y is not zero, then the tangent is (Y/X).
//
//  Output:
//
//    double R8_ATAN, an angle between 0 and 2 * PI, whose tangent is
//    (Y/X), and which lies in the appropriate quadrant so that the signs
//    of its cosine and sine match those of X and Y.
//
{
  double abs_x;
  double abs_y;
  const double r8_pi = 3.141592653589793;
  double theta = 0.0;
  double theta_0;
//
//  Special cases:
//
  if ( x == 0.0 )
  {
    if ( 0.0 < y )
    {
      theta = r8_pi / 2.0;
    }
    else if ( y < 0.0 )
    {
      theta = 3.0 * r8_pi / 2.0;
    }
    else if ( y == 0.0 )
    {
      theta = 0.0;
    }
  }
  else if ( y == 0.0 )
  {
    if ( 0.0 < x )
    {
      theta = 0.0;
    }
    else if ( x < 0.0 )
    {
      theta = r8_pi;
    }
  }
//
//  We assume that ATAN2 is correct when both arguments are positive.
//
  else
  {
    abs_y = fabs ( y );
    abs_x = fabs ( x );

    theta_0 = atan2 ( abs_y, abs_x );

    if ( 0.0 < x && 0.0 < y )
    {
      theta = theta_0;
    }
    else if ( x < 0.0 && 0.0 < y )
    {
      theta = r8_pi - theta_0;
    }
    else if ( x < 0.0 && y < 0.0 )
    {
      theta = r8_pi + theta_0;
    }
    else if ( 0.0 < x && y < 0.0 )
    {
      theta = 2.0 * r8_pi - theta_0;
    }
  }

  return theta;
}
//****************************************************************************80

double r8_atanh ( double x )

//****************************************************************************80
//
//  Purpose:
//
//    r8_atanh() returns the inverse hyperbolic tangent of a number.
//
//  Discussion:
//
//    Y = R8_ATANH ( X )
//
//    implies that
//
//    X = TANH(Y) = ( EXP(Y) - EXP(-Y) ) / ( EXP(Y) + EXP(-Y) )
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    29 November 2007
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double X, the number whose inverse hyperbolic 
//    tangent is desired.  The absolute value of X should be less than 
//    or equal to 1.
//
//  Output:
//
//    double R8_ATANH, the inverse hyperbolic tangent of X.
//
{
  double value;

  if ( x <= -1.0 )
  {
    value = - HUGE_VAL;
  }
  else if ( 1.0 <= x )
  {
    value = + HUGE_VAL;
  }
  else
  {
    value = 0.5 * log ( ( 1.0 + x ) / ( 1.0 - x ) );
  }

  return value;
}
//****************************************************************************80

double r8_big ( )

//****************************************************************************80
//
//  Purpose:
//
//    r8_big() returns a "big" R8.
//
//  Discussion:
//
//    The value returned by this function is NOT required to be the
//    maximum representable R8.
//    We simply want a "very large" but non-infinite number.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    27 September 2014
//
//  Author:
//
//    John Burkardt
//
//  Output:
//
//    double R8_BIG, a "big" R8 value.
//
{
  double value;

  value = 1.0E+30;

  return value;
}
//****************************************************************************80

double r8_cas ( double x )

//****************************************************************************80
//
//  Purpose:
//
//    r8_cas() returns the "casine" of an R8.
//
//  Discussion:
//
//    The "casine", used in the discrete Hartley transform, is abbreviated
//    CAS(X), and defined by:
//
//      CAS(X) = cos ( X ) + sin( X )
//             = sqrt ( 2 ) * sin ( X + pi/4 )
//             = sqrt ( 2 ) * cos ( X - pi/4 )
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    01 April 2004
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double X, the number whose casine is desired.
//
//  Output:
//
//    double R8_CAS, the casine of X, which will be between
//    plus or minus the square root of 2.
//
{
  double value;

  value = cos ( x ) + sin ( x );

  return value;
}
//****************************************************************************80

double r8_ceiling ( double x )

//****************************************************************************80
//
//  Purpose:
//
//    r8_ceiling() rounds an R8 up to the nearest integral R8.
//
//  Example:
//
//    X        R8_CEILING(X)
//
//   -1.1      -1.0
//   -1.0      -1.0
//   -0.9       0.0
//   -0.1       0.0
//    0.0       0.0
//    0.1       1.0
//    0.9       1.0
//    1.0       1.0
//    1.1       2.0
//    2.9       3.0
//    3.0       3.0
//    3.14159   4.0
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    10 November 2011
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double X, the number whose ceiling is desired.
//
//  Output:
//
//    double R8_CEILING, the ceiling of X.
//
{
  double value;

  value = ( double ) ( ( int ) x );

  if ( value < x )
  {
    value = value + 1.0;
  }

  return value;
}
//****************************************************************************80

double r8_choose ( int n, int k )

//****************************************************************************80
// 
//  Purpose:
//
//    r8_choose() computes the combinatorial coefficient C(N,K).
// 
//  Discussion:
// 
//    Real arithmetic is used, and C(N,K) is computed directly, via
//    Gamma functions, rather than recursively.
// 
//    C(N,K) is the number of distinct combinations of K objects
//    chosen from a set of N distinct objects.  A combination is
//    like a set, in that order does not matter.
// 
//    C(N,K) = N! / ( (N-K)! * K! )
// 
//  Example:
// 
//    The number of combinations of 2 things chosen from 5 is 10.
// 
//    C(5,2) = ( 5 * 4 * 3 * 2 * 1 ) / ( ( 3 * 2 * 1 ) * ( 2 * 1 ) ) = 10.
// 
//    The actual combinations may be represented as:
// 
//      (1,2), (1,3), (1,4), (1,5), (2,3),
//      (2,4), (2,5), (3,4), (3,5), (4,5).
// 
//  Licensing:
// 
//    This code is distributed under the MIT license.
// 
//  Modified:
// 
//    25 July 2011
// 
//  Author:
// 
//    John Burkardt
// 
//  Input:
// 
//    int N, the value of N.
// 
//    int K, the value of K.
//
//  Output:
//
//    double R8_CHOOSE, the value of C(N,K)
// 
{
  double arg;
  double fack;
  double facn;
  double facnmk;
  double value;

  if ( n < 0 )
  {
    value = 0.0;
  }
  else if ( k == 0 )
  {
    value = 1.0;
  }
  else if ( k == 1 )
  {
    value = ( double ) ( n );
  }
  else if ( 1 < k && k < n - 1 )
  {
    arg = ( double ) ( n + 1 );
    facn = lgamma ( arg );

    arg = ( double ) ( k + 1 );
    fack = lgamma ( arg );

    arg = ( double ) ( n - k + 1 );
    facnmk = lgamma ( arg );

    value = round ( exp ( facn - fack - facnmk ) );
  }
  else if ( k == n - 1 )
  {
    value = ( double ) ( n );
  }
  else if ( k == n )
  {
    value = 1.0;
  }
  else
  {
    value = 0.0;
  }

  return value;
}
//****************************************************************************80

double r8_chop ( int place, double x )

//****************************************************************************80
//
//  Purpose:
//
//    r8_chop() chops an R8 to a given number of binary places.
//
//  Example:
//
//    3.875 = 2 + 1 + 1/2 + 1/4 + 1/8.
//
//    The following values would be returned for the 'chopped' value of
//    3.875:
//
//    PLACE  Value
//
//       1      2
//       2      3     = 2 + 1
//       3      3.5   = 2 + 1 + 1/2
//       4      3.75  = 2 + 1 + 1/2 + 1/4
//       5+     3.875 = 2 + 1 + 1/2 + 1/4 + 1/8
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    01 April 2004
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int PLACE, the number of binary places to preserve.
//    PLACE = 0 means return the integer part of X.
//    PLACE = 1 means return the value of X, correct to 1/2.
//    PLACE = 2 means return the value of X, correct to 1/4.
//    PLACE = -1 means return the value of X, correct to 2.
//
//    double X, the number to be chopped.
//
//  Output:
//
//    double R8_CHOP, the chopped number.
//
{
  double fac;
  int temp;
  double value;

  temp = ( int ) ( r8_log_2 ( x ) );
  fac = pow ( 2.0, ( temp - place + 1 ) );
  value = ( double ) ( ( int ) ( x / fac ) ) * fac;

  return value;
}
//****************************************************************************80

double r8_cosd ( double degrees )

//****************************************************************************80
//
//  Purpose:
//
//    r8_cosd() returns the cosine of an angle given in degrees.
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    27 July 2014
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double DEGREES, the angle in degrees.
//
//  Output:
//
//    double R8_COSD, the cosine of the angle.
//
{
  const double r8_pi = 3.141592653589793;
  double radians;
  double value;

  radians = r8_pi * ( degrees / 180.0 );

  value = cos ( radians );

  return value;
}
//****************************************************************************80

double r8_cot ( double angle )

//****************************************************************************80
//
//  Purpose:
//
//    r8_cot() returns the cotangent of an angle.
//
//  Discussion:
//
//    R8_COT ( THETA ) = COS ( THETA ) / SIN ( THETA )
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    12 May 2003
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double ANGLE, the angle, in radians.
//
//  Output:
//
//    double R8_COT, the cotangent of the angle.
//
{
  double value;

  value = cos ( angle ) / sin ( angle );

  return value;
}
//****************************************************************************80

double r8_cotd ( double degrees )

//****************************************************************************80
//
//  Purpose:
//
//    r8_cotd() returns the cotangent of an angle given in degrees.
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    27 July 2014
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double DEGREES, the angle in degrees.
//
//  Output:
//
//    double R8_COTD, the cotangent of the angle.
//
{
  const double r8_pi = 3.141592653589793;
  double radians;
  double value;

  radians = r8_pi * ( degrees / 180.0 );

  value = cos ( radians ) / sin ( radians );

  return value;
}
//****************************************************************************80

double r8_csc ( double theta )

//****************************************************************************80
//
//  Purpose:
//
//    r8_csc() returns the cosecant of X.
//
//  Discussion:
//
//    R8_CSC ( THETA ) = 1.0 / SIN ( THETA )
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    05 March 2012
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double THETA, the angle, in radians, whose cosecant is desired.
//    It must be the case that SIN ( THETA ) is not zero.
//
//  Output:
//
//    double R8_CSC, the cosecant of THETA.
//
{
  double value;

  value = sin ( theta );

  if ( value == 0.0 )
  {
    cerr << " \n";
    cerr << "R8_CSC - Fatal error!\n";
    cerr << "  Cosecant undefined for THETA = " << theta << "\n";
    exit ( 1 );
  }

  value = 1.0 / value;

  return value;
}
//****************************************************************************80

double r8_cscd ( double degrees )

//****************************************************************************80
//
//  Purpose:
//
//    r8_cscd() returns the cosecant of an angle given in degrees.
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    27 July 2014
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double DEGREES, the angle in degrees.
//
//  Output:
//
//    double R8_CSCD, the cosecant of the angle.
//
{
  const double r8_pi = 3.141592653589793;
  double radians;
  double value;

  radians = r8_pi * ( degrees / 180.0 );

  value = 1.0 / sin ( radians );

  return value;
}
//****************************************************************************80

double r8_cube_root ( double x )

//****************************************************************************80
//
//  Purpose:
//
//    r8_cube_root() returns the cube root of an R8.
//
//  Discussion:
//
//    This routine is designed to avoid the possible problems that can occur
//    when formulas like 0.0^(1/3) or (-1.0)^(1/3) are to be evaluated.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    01 April 2004
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double X, the number whose cube root is desired.
//
//  Output:
//
//    double R8_CUBE_ROOT, the cube root of X.
//
{
  double value;

  if ( 0.0 < x )
  {
    value = pow ( ( double ) x, ( 1.0 / 3.0 ) );
  }
  else if ( x == 0.0 )
  {
    value = 0.0;
  }
  else
  {
    value = - pow ( ( double ) ( fabs ( x ) ), ( 1.0 / 3.0 ) );
  }

  return value;
}
//****************************************************************************80

double r8_degrees ( double radians )

//****************************************************************************80
//
//  Purpose:
//
//    r8_degrees() converts an angle from radian to degree measure.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    15 May 2013
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double RADIANS, the angle measurement in radians.
//
//  Output:
//
//    double R8_DEGREES, the angle measurement in degrees.
//
{
  const double r8_pi = 3.1415926535897932384626434;
  double value;

  value = radians * 180.0 / r8_pi;

  return value;
}
//****************************************************************************80

double r8_diff ( double x, double y, int n )

//****************************************************************************80
//
//  Purpose:
//
//    r8_diff() computes (X-Y) to a specified accuracy.
//
//  Discussion:
//
//    The user controls how many binary digits of accuracy
//    are to be used.
//
//    N determines the accuracy of the value.  If N = 10,
//    for example, only 11 binary places will be used in the arithmetic.
//    In general, only N+1 binary places will be used.
//
//    N may be zero.  However, a negative value of N should
//    not be used, since this will cause both X and Y to look like 0.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    15 April 2004
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double X, Y, the two values whose difference is desired.
//
//    int N, the number of binary digits to use.
//
//  Output:
//
//    double R8_DIFF, the value of X-Y.
//
{
  double cx;
  double cy;
  double pow2;
  double size;
  double value;

  if ( x == y )
  {
    value = 0.0;
    return value;
  }

  pow2 = pow ( 2.0, n );
//
//  Compute the magnitude of X and Y, and take the larger of the
//  two.  At least one of the two values is not zero//
//
  size = fmax ( fabs ( x ), fabs ( y ) );
//
//  Make normalized copies of X and Y.  One of the two values will
//  actually be equal to 1.
//
  cx = x / size;
  cy = y / size;
//
//  Here's where rounding comes in.  We know that the larger of the
//  the two values equals 1.  We multiply both values by 2^N,
//  where N+1 is the number of binary digits of accuracy we want
//  to use, truncate the values, and divide back by 2^N.
//
  cx = ( double ) ( ( int ) ( cx * pow2 + 0.5 * r8_sign ( cx ) ) ) / pow2;
  cy = ( double ) ( ( int ) ( cy * pow2 + 0.5 * r8_sign ( cy ) ) ) / pow2;
//
//  Take the difference now.
//
  value = cx - cy;
//
//  Undo the scaling.
//
  value = value * size;

  return value;
}
//****************************************************************************80

int r8_digit ( double x, int idigit )

//****************************************************************************80
//
//  Purpose:
//
//    r8_digit() returns a particular decimal digit of an R8.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    11 April 2004
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double X, the number whose IDIGIT-th decimal digit is desired.
//    Note that if X is zero, all digits will be returned as 0.
//
//    int IDIGIT, the position of the desired decimal digit.
//    A value of 1 means the leading digit, a value of 2 the second digit
//    and so on.
//
//  Output:
//
//    int R8_DIGIT, the value of the IDIGIT-th decimal digit of X.
//
{
  int digit;
  int i;
  int ival;

  if ( x == 0.0 )
  {
    digit = 0;
    return digit;
  }

  if ( idigit <= 0 )
  {
    digit = 0;
    return digit;
  }
//
//  Force X to lie between 1 and 10.
//
  x = fabs ( x );

  while ( x < 1.0 )
  {
    x = x * 10.0;
  }

  while ( 10.0 <= x )
  {
    x = x / 10.0;
  }

  for ( i = 1; i <= idigit; i++ )
  {
    ival = ( int ) ( x );
    x = ( x - ( double ) ival ) * 10.0;
  }

  digit = ival;

  return digit;
}
//****************************************************************************80

double r8_divide_i4 ( int  i, int j )

//****************************************************************************80
//
//  Purpose:
//
//    r8_divide_i4() returns an I4 fraction as an R8.
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    05 June 2012
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int I, J, the numerator and denominator.
//
//  Output:
//
//    double R8_DIVIDE_I4, the value of (I/J).
//
{
  double value;

  value = ( double ) ( i ) / ( double ) ( j );

  return value;
}
//****************************************************************************80

double r8_e ( )

//****************************************************************************80
//
//  Purpose:
//
//    r8_e() returns the value of the base of the natural logarithm system.
//
//  Definition:
//
//    E = Limit ( N -> +oo ) ( 1 + 1 / N )^N
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    12 May 2003
//
//  Author:
//
//    John Burkardt
//
//  Output:
//
//    double R8_E, the base of the natural logarithm system.
//
{
  const double r8_e_save = 2.718281828459045235360287;
  double value;

  value = r8_e_save;

  return value;
}
//****************************************************************************80

double r8_epsilon ( )

//****************************************************************************80
//
//  Purpose:
//
//    r8_epsilon() returns the R8 roundoff unit.
//
//  Discussion:
//
//    The roundoff unit is a number R which is a power of 2 with the
//    property that, to the precision of the computer's arithmetic,
//      1 < 1 + R
//    but
//      1 = ( 1 + R / 2 )
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    01 September 2012
//
//  Author:
//
//    John Burkardt
//
//  Output:
//
//    double R8_EPSILON, the R8 round-off unit.
//
{
  const double value = 2.220446049250313E-016;

  return value;
}
//****************************************************************************80

double r8_epsilon_compute ( )

//****************************************************************************80
//
//  Purpose:
//
//    r8_epsilon_compute() computes the R8 roundoff unit.
//
//  Discussion:
//
//    The roundoff unit is a number R which is a power of 2 with the
//    property that, to the precision of the computer's arithmetic,
//      1 < 1 + R
//    but
//      1 = ( 1 + R / 2 )
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    01 September 2012
//
//  Author:
//
//    John Burkardt
//
//  Output:
//
//    double R8_EPSILON_COMPUTE, the R8 round-off unit.
//
{
  double one;
  double temp;
  double test;
  static double value = 0.0;

  if ( value == 0.0 )
  {
    one = ( double ) ( 1 );

    value = one;
    temp = value / 2.0;
    test = r8_add ( one, temp );

    while ( one < test )
    {
      value = temp;
      temp = value / 2.0;
      test = r8_add ( one, temp );
    }
  }

  return value;
}
//****************************************************************************80

double r8_exp ( double x )

//****************************************************************************80
//
//  Purpose:
//
//    r8_exp() computes the exponential function, avoiding overflow and underflow.
//
//  Discussion:
//
//    For arguments of very large magnitude, the evaluation of the
//    exponential function can cause computational problems.  Some languages
//    and compilers may return an infinite value or a "Not-a-Number".  
//    An alternative, when dealing with a wide range of inputs, is simply
//    to truncate the calculation for arguments whose magnitude is too large.
//    Whether this is the right or convenient approach depends on the problem
//    you are dealing with, and whether or not you really need accurate
//    results for large magnitude inputs, or you just want your code to
//    stop crashing.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    19 September 2014
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double X, the argument of the exponential function.
//
//  Output:
//
//    double R8_EXP, the value of exp ( X ).
//
{
  const double r8_big = 1.0E+30;
  const double r8_log_max = +69.0776;
  const double r8_log_min = -69.0776;
  double value;

  if ( x <= r8_log_min )
  {
    value = 0.0;
  }
  else if ( x < r8_log_max )
  {
    value = exp ( x );
  }
  else
  {
    value = r8_big;
  }

  return value;
}
//****************************************************************************80

double r8_factorial ( int n )

//****************************************************************************80
//
//  Purpose:
//
//    r8_factorial() computes the factorial of N.
//
//  Discussion:
//
//    factorial ( N ) = product ( 1 <= I <= N ) I
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    16 January 1999
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the argument of the factorial function.
//    If N is less than 1, the function value is returned as 1.
//
//  Output:
//
//    double R8_FACTORIAL, the factorial of N.
//
{
  int i;
  double value;

  value = 1.0;

  for ( i = 1; i <= n; i++ )
  {
    value = value * ( double ) ( i );
  }

  return value;
}
//****************************************************************************80

double r8_factorial_stirling ( int n )

//****************************************************************************80
//
//  Purpose:
//
//    r8_factorial_stirling() computes Stirling's approximation to N!.
//
//  Discussion:
//
//    N! = Product ( 1 <= I <= N ) I
//
//    Stirling ( N ) = sqrt ( 2 * PI * N ) * ( N / E )^N * E^(1/(12*N) )
//
//    This routine returns the raw approximation for all nonnegative
//    values of N.  If N is less than 0, the value is returned as 0,
//    and if N is 0, the value of 1 is returned.  In all other cases,
//    Stirling's formula is used.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    07 April 2016
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the argument of the function.
//
//  Output:
//
//    double R8_FACTORIAL_STIRLING, an approximation to N!.
//
{
  const double r8_e = 2.71828182845904523;
  const double r8_pi = 3.14159265358979323;
  double value;

  if ( n < 0 )
  {
    value = 0.0;
  }
  else if ( n == 0 )
  {
    value = 1.0;
  }
  else
  {
    value = sqrt ( 2.0 * r8_pi * ( double ) ( n ) )
      * pow ( ( double ) ( n ) / r8_e, n )
      * exp ( 1.0 / ( double ) ( 12 * n ) );
  }

  return value;
}
//****************************************************************************80

void r8_factorial_values ( int &n_data, int &n, double &fn )

//****************************************************************************80
//
//  Purpose:
//
//    r8_factorial_values() returns values of the real factorial function.
//
//  Discussion:
//
//    0! = 1
//    I! = Product ( 1 <= J <= I ) J
//
//    Although the factorial is an int *valued function, it quickly
//    becomes too large for an int *to hold.  This routine still accepts
//    an int *as the input argument, but returns the function value
//    as a real number.
//
//    In Mathematica, the function can be evaluated by:
//
//      n!
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    18 August 2004
//
//  Author:
//
//    John Burkardt
//
//  Reference:
//
//    Milton Abramowitz, Irene Stegun,
//    Handbook of Mathematical Functions,
//    National Bureau of Standards, 1964,
//    ISBN: 0-486-61272-4,
//    LC: QA47.A34.
//
//    Stephen Wolfram,
//    The Mathematica Book,
//    Fourth Edition,
//    Cambridge University Press, 1999,
//    ISBN: 0-521-64314-7,
//    LC: QA76.95.W65.
//
//  Input:
//
//    int &N_DATA: the user sets N_DATA to 0 before the first call.
//
//  Output:
//
//    int &N_DATA: the routine increments N_DATA by 1, and
//    returns the corresponding data; when there is no more data, the
//    output value of N_DATA will be 0 again.
//
//    int &N, the argument of the function.
//
//    double &FN, the value of the function.
//
{
# define N_MAX 25

  static double fn_vec[N_MAX] = {
     0.1000000000000000E+01,
     0.1000000000000000E+01,
     0.2000000000000000E+01,
     0.6000000000000000E+01,
     0.2400000000000000E+02,
     0.1200000000000000E+03,
     0.7200000000000000E+03,
     0.5040000000000000E+04,
     0.4032000000000000E+05,
     0.3628800000000000E+06,
     0.3628800000000000E+07,
     0.3991680000000000E+08,
     0.4790016000000000E+09,
     0.6227020800000000E+10,
     0.8717829120000000E+11,
     0.1307674368000000E+13,
     0.2092278988800000E+14,
     0.3556874280960000E+15,
     0.6402373705728000E+16,
     0.1216451004088320E+18,
     0.2432902008176640E+19,
     0.1551121004333099E+26,
     0.3041409320171338E+65,
     0.9332621544394415E+158,
     0.5713383956445855E+263 };

  static int n_vec[N_MAX] = {
       0,
       1,
       2,
       3,
       4,
       5,
       6,
       7,
       8,
       9,
      10,
      11,
      12,
      13,
      14,
      15,
      16,
      17,
      18,
      19,
      20,
      25,
      50,
     100,
     150 };

  if ( n_data < 0 )
  {
    n_data = 0;
  }

  n_data = n_data + 1;

  if ( N_MAX < n_data )
  {
    n_data = 0;
    n = 0;
    fn = 0.0;
  }
  else
  {
    n = n_vec[n_data-1];
    fn = fn_vec[n_data-1];
  }

  return;
# undef N_MAX
}
//****************************************************************************80

double r8_factorial2 ( int n )

//****************************************************************************80
//
//  Purpose:
//
//    r8_factorial2() computes the double factorial function.
//
//  Discussion:
//
//    FACTORIAL2( N ) = Product ( N * (N-2) * (N-4) * ... * 2 )  (N even)
//                    = Product ( N * (N-2) * (N-4) * ... * 1 )  (N odd)
//
//  Example:
//
//     N Value
//
//     0     1
//     1     1
//     2     2
//     3     3
//     4     8
//     5    15
//     6    48
//     7   105
//     8   384
//     9   945
//    10  3840
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    22 January 2008
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the argument of the double factorial
//    function.  If N is less than 1, R8_FACTORIAL2 is returned as 1.0.
//
//  Output:
//
//    double R8_FACTORIAL2, the value of Factorial2(N).
//
{
  int n_copy;
  double value;

  value = 1.0;

  if ( n < 1 )
  {
    return value;
  }

  n_copy = n;

  while ( 1 < n_copy )
  {
    value = value * ( double ) n_copy;
    n_copy = n_copy - 2;
  }

  return value;
}
//****************************************************************************80

void r8_factorial2_values ( int &n_data, int &n, double &f )

//****************************************************************************80
//
//  Purpose:
//
//    r8_factorial2_values() returns values of the double factorial function.
//
//  Formula:
//
//    FACTORIAL2( N ) = Product ( N * (N-2) * (N-4) * ... * 2 )  (N even)
//                    = Product ( N * (N-2) * (N-4) * ... * 1 )  (N odd)
//
//    In Mathematica, the function can be evaluated by:
//
//      n!!
//
//  Example:
//
//     N    N!!
//
//     0     1
//     1     1
//     2     2
//     3     3
//     4     8
//     5    15
//     6    48
//     7   105
//     8   384
//     9   945
//    10  3840
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    07 February 2015
//
//  Author:
//
//    John Burkardt
//
//  Reference:
//
//    Milton Abramowitz, Irene Stegun,
//    Handbook of Mathematical Functions,
//    National Bureau of Standards, 1964,
//    ISBN: 0-486-61272-4,
//    LC: QA47.A34.
//
//    Stephen Wolfram,
//    The Mathematica Book,
//    Fourth Edition,
//    Cambridge University Press, 1999,
//    ISBN: 0-521-64314-7,
//    LC: QA76.95.W65.
//
//    Daniel Zwillinger,
//    CRC Standard Mathematical Tables and Formulae,
//    30th Edition,
//    CRC Press, 1996, page 16.
//
//  Input:
//
//    int &N_DATA: the user sets N_DATA to 0 before the first call.
//
//  Output:
//
//    int &N_DATA: the routine increments N_DATA by 1, and
//    returns the corresponding data; when there is no more data, the
//    output value of N_DATA will be 0 again.
//
//    int &N, the argument of the function.
//
//    double &F, the value of the function.
//
{
# define N_MAX 16

  static double f_vec[N_MAX] = {
          1.0,
          1.0,
          2.0,
          3.0,
          8.0,
         15.0,
         48.0,
        105.0,
        384.0,
        945.0,
       3840.0,
      10395.0,
      46080.0,
     135135.0,
     645120.0,
    2027025.0 };

  static int n_vec[N_MAX] = {
     0,
     1,  2,  3,  4,  5,
     6,  7,  8,  9, 10,
    11, 12, 13, 14, 15 };

  if ( n_data < 0 )
  {
    n_data = 0;
  }

  n_data = n_data + 1;

  if ( N_MAX < n_data )
  {
    n_data = 0;
    n = 0;
    f = 0.0;
  }
  else
  {
    n = n_vec[n_data-1];
    f = f_vec[n_data-1];
  }

  return;
# undef N_MAX
}
//****************************************************************************80

double r8_fall ( double x, int n )

//****************************************************************************80
//
//  Purpose:
//
//    r8_fall() computes the falling factorial function [X]_N.
//
//  Discussion:
//
//    Note that the number of "injections" or 1-to-1 mappings from
//    a set of N elements to a set of M elements is [M]_N.
//
//    The number of permutations of N objects out of M is [M]_N.
//
//    Moreover, the Stirling numbers of the first kind can be used
//    to convert a falling factorial into a polynomial, as follows:
//
//      [X]_N = S^0_N + S^1_N * X + S^2_N * X^2 + ... + S^N_N X^N.
//
//    The formula is:
//
//      [X]_N = X * ( X - 1 ) * ( X - 2 ) * ... * ( X - N + 1 ).
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    08 May 2003
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double X, the argument of the falling factorial function.
//
//    int N, the order of the falling factorial function.
//    If N = 0, FALL = 1, if N = 1, FALL = X.  Note that if N is
//    negative, a "rising" factorial will be computed.
//
//  Output:
//
//    double R8_FALL, the value of the falling factorial function.
//
{
  int i;
  double value;

  value = 1.0;

  if ( 0 < n )
  {
    for ( i = 1; i <= n; i++ )
    {
      value = value * x;
      x = x - 1.0;
    }
  }
  else if ( n < 0 )
  {
    for ( i = -1; n <= i; i-- )
    {
      value = value * x;
      x = x + 1.0;
    }
  }

  return value;
}
//****************************************************************************80

void r8_fall_values ( int &n_data, double &x, int &n, double &f )

//****************************************************************************80
//
//  Purpose:
//
//    r8_fall_values() returns some values of the falling factorial function.
//
//  Discussion:
//
//    In Mathematica, the function can be evaluated by:
//
//      FactorialPower[X,Y]
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    20 December 2014
//
//  Author:
//
//    John Burkardt
//
//  Reference:
//
//    Milton Abramowitz, Irene Stegun,
//    Handbook of Mathematical Functions,
//    National Bureau of Standards, 1964,
//    ISBN: 0-486-61272-4,
//    LC: QA47.A34.
//
//    Stephen Wolfram,
//    The Mathematica Book,
//    Fourth Edition,
//    Cambridge University Press, 1999,
//    ISBN: 0-521-64314-7,
//    LC: QA76.95.W65.
//
//  Input:
//
//    int &N_DATA: the user sets N_DATA to 0 before the first call.
//
//  Output:
//
//    int &N_DATA: the routine increments N_DATA by 1, and
//    returns the corresponding data; when there is no more data, the
//    output value of N_DATA will be 0 again.
//
//    double &X, int &N, the arguments of the function.
//
//    double &F, the value of the function.
//
{
# define N_MAX 15

  static double f_vec[N_MAX] = {
    120.0000000000000,
    163.1601562500000,
    216.5625000000000,
    281.6601562500000,
    360.0000000000000,
    1.000000000000000,
    7.500000000000000,
    48.75000000000000,
    268.1250000000000,
    1206.562500000000,
    4222.968750000000,
    10557.42187500000,
    15836.13281250000,
    7918.066406250000,
    -3959.03320312500 };

  static int n_vec[N_MAX] = {
    4,
    4,
    4,
    4,
    4,
    0,
    1,
    2,
    3,
    4,
    5,
    6,
    7,
    8,
    9 };

  static double x_vec[N_MAX] = {
    5.00,
    5.25,
    5.50,
    5.75,
    6.00,
    7.50,
    7.50,
    7.50,
    7.50,
    7.50,
    7.50,
    7.50,
    7.50,
    7.50,
    7.50 };

  if ( n_data < 0 )
  {
    n_data = 0;
  }

  n_data = n_data + 1;

  if ( N_MAX < n_data )
  {
    n_data = 0;
    x = 0.0;
    n = 0;
    f = 0.0;
  }
  else
  {
    x = x_vec[n_data-1];
    n = n_vec[n_data-1];
    f = f_vec[n_data-1];
  }

  return;
# undef N_MAX
}
//****************************************************************************80

double r8_floor ( double x )

//****************************************************************************80
//
//  Purpose:
//
//    r8_floor() rounds an R8 down to the nearest integral R8.
//
//  Example:
//
//    X        R8_FLOOR(X)
//
//   -1.1      -2.0
//   -1.0      -1.0
//   -0.9      -1.0
//   -0.1      -1.0
//    0.0       0.0
//    0.1       0.0
//    0.9       0.0
//    1.0       1.0
//    1.1       1.0
//    2.9       2.0
//    3.0       3.0
//    3.14159   3.0
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    15 April 2007
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double X, the number whose floor is desired.
//
//  Output:
//
//    double R8_FLOOR, the floor of X.
//
{
  double value;

  value = ( double ) ( ( int ) x );

  if ( x < value )
  {
    value = value - 1.0;
  }

  return value;
}
//****************************************************************************80

double r8_fraction ( int i, int j )

//****************************************************************************80
//
//  Purpose:
//
//    r8_fraction() uses real arithmetic on an integer ratio.
//
//  Discussion:
//
//    Given integer variables I and J, both FORTRAN and C will evaluate
//    an expression such as "I/J" using what is called "integer division",
//    with the result being an integer.  It is often convenient to express
//    the parts of a fraction as integers but expect the result to be computed
//    using real arithmetic.  This function carries out that operation.
//
//  Example:
//
//       I     J   I/J  R8_FRACTION
//
//       1     2     0  0.5
//       7     4     1  1.75
//       8     4     2  2.00
//       9     4     2  2.25
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    05 October 2010
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int I, J, the arguments.
//
//  Output:
//
//    double R8_FRACTION, the value of the ratio.
//
{
  double value;

  value = ( double ) ( i ) / ( double ) ( j );

  return value;
}
//****************************************************************************80

double r8_fractional ( double x )

//****************************************************************************80
//
//  Purpose:
//
//    r8_fractional() returns the fractional part of an R8.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    05 October 2010
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double X, the argument.
//
//  Output:
//
//    double R8_FRACTIONAL, the fractional part of X.
//
{
  double value;

  value = fabs ( x ) - ( double ) ( ( int ) fabs ( x ) );

  return value;
}
//****************************************************************************80

double r8_haversine ( double a )

//****************************************************************************80
//
//  Purpose:
//
//    r8_haversine() computes the haversine of an angle.
//
//  Discussion:
//
//    haversine(A) = ( 1 - cos ( A ) ) / 2
//
//    The haversine is useful in spherical trigonometry.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    30 November 2018
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double A, the angle.
//
//  Output:
//
//    double R8_HAVERSINE, the haversine of the angle.
//
{
  double value;

  value = ( 1.0 - cos ( a ) ) / 2.0;

  return value;
}
//****************************************************************************80

double r8_heaviside ( double x )

//****************************************************************************80
//
//  Purpose:
//
//    r8_heaviside() evaluates the Heaviside function.
//
//  Discussion:
//
//    The Heaviside function is 0 for x < 0, 1 for x > 0, and 1/2 for x = 0.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    30 November 2018
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double X, the argument.
//
//  Output:
//
//    double R8_HEAVISIDE, the value.
//
{
  double value;

  if ( x < 0.0 )
  {
    value = 0.0;
  }
  else if ( x == 0.0 )
  {
    value = 0.5;
  }
  else
  {
    value = 1.0;
  }
  
  return value;
}
//****************************************************************************80

double r8_huge ( )

//****************************************************************************80
//
//  Purpose:
//
//    r8_huge() returns a "huge" R8.
//
//  Discussion:
//
//    The value returned by this function is intended to be the largest
//    representable real value.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    27 September 2014
//
//  Author:
//
//    John Burkardt
//
//  Output:
//
//    double R8_HUGE, a "huge" R8 value.
//
{
  double value;

  value = 1.79769313486231571E+308;

  return value;
}
//****************************************************************************80

double r8_hypot ( double x, double y )

//****************************************************************************80
//
//  Purpose:
//
//    r8_hypot() returns the value of sqrt ( X^2 + Y^2 ).
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    26 March 2012
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double X, Y, the arguments.
//
//  Output:
//
//    double R8_HYPOT, the value of sqrt ( X^2 + Y^2 ).
//
{
  double a;
  double b;
  double value;

  if ( fabs ( x ) < fabs ( y ) )
  {
    a = fabs ( y );
    b = fabs ( x );
  }
  else
  {
    a = fabs ( x );
    b = fabs ( y );
  }
//
//  A contains the larger value.
//
  if ( a == 0.0 )
  {
    value = 0.0;
  }
  else
  {
    value = a * sqrt ( 1.0 + ( b / a ) * ( b / a ) );
  }

  return value;
}
//****************************************************************************80

bool r8_is_in_01 ( double a )

//****************************************************************************80
//
//  Purpose:
//
//    r8_in_01() is TRUE if an R8 is in the range [0,1].
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    13 June 2009
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double A, the value.
//
//  Output:
//
//    bool R8_IS_IN_01, is TRUE if A is between 0 and 1.
//
{
  bool value;

  value = ( 0.0 <= a && a <= 1.0 );

  return value;
}
//****************************************************************************80

bool r8_is_inf ( double r )

//****************************************************************************80
//
//  Purpose:
//
//    r8_is_inf() determines if an R8 represents an infinite value.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    14 May 2016
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double R, the number to be checked.
//
//  Output:
//
//    bool R8_IS_INF, is TRUE if R is an infinite value.
//
{
  bool value;

  if ( r < 0.0 )
  {
    value = ( r < - HUGE_VAL );
  }
  else
  {
    value = ( HUGE_VAL < r );
  }

  return value;
}
//****************************************************************************80

bool r8_is_insignificant ( double r, double s )

//****************************************************************************80
//
//  Purpose:
//
//    r8_is_insignificant() determines if an R8 is insignificant.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    26 November 2011
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double R, the number to be compared against.
//
//    double S, the number to be compared.
//
//  Output:
//
//    bool R8_IS_INSIGNIFICANT, is TRUE if S is insignificant
//    compared to R.
//
{
  double t;
  double tol;
  bool value;

  value = true;

  t = r + s;
  tol = DBL_EPSILON * fabs ( r );

  if ( tol < fabs ( r - t ) )
  {
    value = false;
  }
  
  return value;
}
//****************************************************************************80

bool r8_is_integer ( double r )

//****************************************************************************80
//
//  Purpose:
//
//    r8_is_integer() determines if an R8 represents an integer value.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    15 April 2004
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double R, the number to be checked.
//
//  Output:
//
//    bool R8_IS_INTEGER, is TRUE if R is an integer value.
//
{
  const int i4_huge = 2147483647;
  bool value;

  if ( ( double ) ( i4_huge ) < r )
  {
    value = false;
  }
  else if ( r < - ( double ) ( i4_huge ) )
  {
    value = false;
  }
  else if ( r == ( double ) ( ( int ) ( r ) ) )
  {
    value = true;
  }
  else
  {
    value = false;
  }
  return value;
}
//****************************************************************************80

bool r8_is_nan ( double r )

//****************************************************************************80
//
//  Purpose:
//
//    r8_is_nan() determines if an R8 represents a NaN value.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    14 May 2016
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double R, the number to be checked.
//
//  Output:
//
//    bool R8_IS_NAN, is TRUE if R is a NaN
//
{
  bool value;

  value = ( r != r );

  return value;
}
//****************************************************************************80

double r8_log_10 ( double x )

//****************************************************************************80
//
//  Purpose:
//
//    r8_log_10() returns the logarithm base 10 of the absolute value of an R8.
//
//  Discussion:
//
//    value = Log10 ( |X| )
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    22 March 2004
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double X, the number whose base 2 logarithm is desired.
//    X should not be 0.
//
//  Output:
//
//    double R8_LOG_10, the logarithm base 10 of the absolute
//    value of X.  It should be true that |X| = 10^R_LOG_10.
//
{
  double value;

  if ( x == 0.0 )
  {
    value = - r8_big ( );
  }
  else
  {
    value = log10 ( fabs ( x ) );
  }

  return value;
}
//****************************************************************************80

double r8_log_2 ( double x )

//****************************************************************************80
//
//  Purpose:
//
//    r8_log_2() returns the logarithm base 2 of the absolute value of an R8.
//
//  Discussion:
//
//    value = Log ( |X| ) / Log ( 2.0 )
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    22 March 2004
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double X, the number whose base 2 logarithm is desired.
//    X should not be 0.
//
//  Output:
//
//    double R8_LOG_2, the logarithm base 2 of the absolute
//    value of X.  It should be true that |X| = 2^R_LOG_2.
//
{
  double value;

  if ( x == 0.0 )
  {
    value = - r8_big ( );
  }
  else
  {
    value = log ( fabs ( x ) ) / log ( 2.0 );
  }

  return value;
}
//****************************************************************************80

double r8_log_b ( double x, double b )

//****************************************************************************80
//
//  Purpose:
//
//    r8_log_b() returns the logarithm base B of an R8.
//
//  Discussion:
//
//    value = log ( |X| ) / log ( |B| )
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    30 April 2009
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double X, the number whose base B logarithm is desired.
//    X should not be 0.
//
//    double B, the base, which should not be 0, 1 or -1.
//
//  Output:
//
//    double R8_LOG_B, the logarithm base B of the absolute
//    value of X.  It should be true that |X| = |B|^R_LOG_B.
//
{
  double value;

  if ( b == 0.0 || b == 1.0 || b == -1.0 )
  {
    value = - r8_big ( );
  }
  else if ( fabs ( x ) == 0.0 )
  {
    value = - r8_big ( );
  }
  else
  {
    value = log ( fabs ( x ) ) / log ( fabs ( b ) );
  }

  return value;
}
//****************************************************************************80

void r8_mant ( double x, int &s, double &r, int &l )

//****************************************************************************80
//
//  Purpose:
//
//    r8_mant() computes the "mantissa" or "fraction part" of an R8.
//
//  Discussion:
//
//    X = S * R * 2^L
//
//    S is +1 or -1,
//    R is a real between 1.0 and 2.0,
//    L is an integer.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    06 January 2013
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double X, the real number to be decomposed.
//
//  Output:
//
//    int &S, the "sign" of the number.
//    S will be -1 if X is less than 0, and +1 if X is greater
//    than or equal to zero.
//
//    double &R, the mantissa of X.  R will be greater
//    than or equal to 1, and strictly less than 2.  The one
//    exception occurs if X is zero, in which case R will also
//    be zero.
//
//    int &L, the integer part of the logarithm (base 2) of X.
//
{
//
//  Determine the sign.
//
  if ( x < 0.0 )
  {
    s = -1;
  }
  else
  {
    s = 1;
  }
//
//  Set R to the absolute value of X, and L to zero.
//  Then force R to lie between 1 and 2.
//
  if ( x < 0.0 )
  {
    r = -x;
  }
  else
  {
    r = x;
  }

  l = 0;
//
//  Time to bail out if X is zero.
//
  if ( x == 0.0 )
  {
    return;
  }

  while ( 2.0 <= r )
  {
    r = r / 2.0;
    l = l + 1;
  }

  while ( r < 1.0 )
  {
    r = r * 2.0;
    l = l - 1;
  }

  return;
}
//****************************************************************************80

double r8_max ( double x, double y )

//****************************************************************************80
//
//  Purpose:
//
//    r8_max() returns the maximum of two R8's.
//
//  Discussion:
//
//    The C++ math library provides the function fmax() for reals and
//    doubles, and the generic function max().
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    05 October 2025
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double X, Y, the quantities to compare.
//
//  Output:
//
//    double R8_MAX, the maximum of X and Y.
//
{
  double value;

  if ( y < x )
  {
    value = x;
  }
  else
  {
    value = y;
  }
  return value;
}
//****************************************************************************80

double r8_min ( double x, double y )

//****************************************************************************80
//
//  Purpose:
//
//    r8_min() returns the minimum of two R8's.
//
//  Discussion:
//
//    The C++ math library provides the function fmin() for real or double
//    arguments, and the generic function min().
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    05 October 2025
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double X, Y, the quantities to compare.
//
//  Output:
//
//    double R8_MIN, the minimum of X and Y.
//
{
  double value;

  if ( y < x )
  {
    value = y;
  }
  else
  {
    value = x;
  }
  return value;
}
//****************************************************************************80

double r8_mod ( double x, double y )

//****************************************************************************80
//
//  Purpose:
//
//    r8_mod() returns the remainder of R8 division.
//
//  Discussion:
//
//    If
//      REM = R8_MOD ( X, Y )
//      RMULT = ( X - REM ) / Y
//    then
//      X = Y * RMULT + REM
//    where REM has the same sign as X, and abs ( REM ) < Y.
//
//  Example:
//
//        X         Y     R8_MOD   R8_MOD  Factorization
//
//      107        50       7     107 =  2 *  50 + 7
//      107       -50       7     107 = -2 * -50 + 7
//     -107        50      -7    -107 = -2 *  50 - 7
//     -107       -50      -7    -107 =  2 * -50 - 7
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    14 June 2007
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double X, the number to be divided.
//
//    double Y, the number that divides X.
//
//  Output:
//
//    double R8_MOD, the remainder when X is divided by Y.
//
{
  double value;

  if ( y == 0.0 )
  {
    cerr << "\n";
    cerr << "R8_MOD - Fatal error!\n";
    cerr << "  R8_MOD ( X, Y ) called with Y = " << y << "\n";
    exit ( 1 );
  }

  value = x - ( ( double ) ( ( int ) ( x / y ) ) ) * y;

  if ( x < 0.0 && 0.0 < value )
  {
    value = value - fabs ( y );
  }
  else if ( 0.0 < x && value < 0.0 )
  {
    value = value + fabs ( y );
  }

  return value;
}
//****************************************************************************80

double r8_modp ( double x, double y )

//****************************************************************************80
//
//  Purpose:
//
//    r8_modp() returns the nonnegative remainder of R8 division.
//
//  Discussion:
//
//    The MOD function computes a result with the same sign as the
//    quantity being divided.  Thus, suppose you had an angle A,
//    and you wanted to ensure that it was between 0 and 360.
//    Then mod(A,360.0) would do, if A was positive, but if A
//    was negative, your result would be between -360 and 0.
//
//    On the other hand, R8_MODP(A,360.0) is between 0 and 360, always.
//
//    If
//      REM = R8_MODP ( X, Y )
//      RMULT = ( X - REM ) / Y
//    then
//      X = Y * RMULT + REM
//    where REM is always nonnegative.
//
//  Example:
//
//        I         J     MOD  R8_MODP   R8_MODP Factorization
//
//      107        50       7       7    107 =  2 *  50 + 7
//      107       -50       7       7    107 = -2 * -50 + 7
//     -107        50      -7      43   -107 = -3 *  50 + 43
//     -107       -50      -7      43   -107 =  3 * -50 + 43
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    18 October 2004
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double X, the number to be divided.
//
//    double Y, the number that divides X.
//
//  Output:
//
//    double R8_MODP, the nonnegative remainder when X is divided by Y.
//
{
  double value;

  if ( y == 0.0 )
  {
    cerr << "\n";
    cerr << "R8_MODP - Fatal error!\n";
    cerr << "  R8_MODP ( X, Y ) called with Y = " << y << "\n";
    exit ( 1 );
  }

  value = x - ( ( double ) ( ( int ) ( x / y ) ) ) * y;

  if ( value < 0.0 )
  {
    value = value + fabs ( y );
  }

  return value;
}
//****************************************************************************80

double r8_mop ( int i )

//****************************************************************************80
//
//  Purpose:
//
//    r8_mop() returns the I-th power of -1 as an R8 value.
//
//  Discussion:
//
//    An R8 is an double value.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    16 November 2007
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int I, the power of -1.
//
//  Output:
//
//    double R8_MOP, the I-th power of -1.
//
{
  double value;

  if ( ( i % 2 ) == 0 )
  {
    value = 1.0;
  }
  else
  {
    value = -1.0;
  }

  return value;
}
//****************************************************************************80

int r8_nint ( double x )

//****************************************************************************80
//
//  Purpose:
//
//    r8_nint() returns the nearest integer to an R8.
//
//  Example:
//
//        X         Value
//
//      1.3         1
//      1.4         1
//      1.5         1 or 2
//      1.6         2
//      0.0         0
//     -0.7        -1
//     -1.1        -1
//     -1.6        -2
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    26 August 2004
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double X, the value.
//
//  Output:
//
//    int R8_NINT, the nearest integer to X.
//
{
  int value;

  if ( x < 0.0 )
  {
    value = - ( int ) ( fabs ( x ) + 0.5 );
  }
  else
  {
    value =   ( int ) ( fabs ( x ) + 0.5 );
  }

  return value;
}
//****************************************************************************80

double r8_normal_01 ( )

//****************************************************************************80
//
//  Purpose:
//
//    r8_normal_01() returns a unit pseudonormal R8.
//
//  Discussion:
//
//    The standard normal probability distribution function (PDF) has
//    mean 0 and standard deviation 1.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    13 September 2022
//
//  Author:
//
//    John Burkardt
//
//  Output:
//
//    double R8_NORMAL_01, a normally distributed random value.
//
{
  double r1;
  double r2;
  const double r8_pi = 3.141592653589793;
  double x;

  r1 = drand48 ( );
  r2 = drand48 ( );
  x = sqrt ( - 2.0 * log ( r1 ) ) * cos ( 2.0 * r8_pi * r2 );

  return x;
}
//****************************************************************************80

double r8_normal_ab ( double a, double b )

//****************************************************************************80
//
//  Purpose:
//
//    r8_normal_ab() returns a scaled pseudonormal R8.
//
//  Discussion:
//
//    The normal probability distribution function (PDF) is sampled,
//    with mean A and standard deviation B.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    21 November 2004
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double A, the mean of the PDF.
//
//    double B, the standard deviation of the PDF.
//
//  Output:
//
//    double R8_NORMAL_AB, a sample of the normal PDF.
//
//    int &SEED: an updated seed.
//
{
  double value;

  value = a + b * r8_normal_01 ( );

  return value;
}
//****************************************************************************80

double r8_nth_root ( double x, int n )

//****************************************************************************80
//
//  Purpose:
//
//    r8_nth_root() returns the nth-root of an R8.
//
//  Discussion:
//
//    The nth root of X is x^(1/n)
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    03 August 2016
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    real X, the number whose nth root is desired.
//
//    integer N, the index of the root.
//
//  Output:
//
//    real VALUE, the Nth root of X.
//
{
  double e;
  double value;
//
//  Potential Error 1: 0^0
//  But we will use it as 1.
//
  if ( x == 0.0 && n == 0 )
  {
    value = 1.0;
    return value;
  }
//
//  Error 2: 0^(negative power)
//
  if ( x == 0.0 && n < 0 )
  {
    value = NAN;
    return value;
  }
//
//  Error 3: (negative)^(even strictly positive root)
//
  if ( x < 0.0 && ( n % 2 ) == 0 && 0 < n )
  {
    value = NAN;
    return value;
  }
//
//  X^0 = 1
//
  if ( n == 0 )
  {
    value = 1.0;
  }
//
//  X^1 = X
//
  else if ( n == 1 )
  {
    value = x;
  }
//
//  X^(-1) = 1/X
//
  else if ( n == -1 )
  {
    value = 1.0 / x;
  }
  else
  {
    e = 1.0 / ( double ) ( abs ( n ) );

    if ( 0.0 < x )
    {
      value = pow ( x, e );
    }
    else if ( x == 0.0 )
    {
      value = 0.0;
    }
    else
    {
      value = - pow ( - x, e );
    }

    if ( n < 0 )
    {
      value = 1.0 / value;
    }
  }

  return value;
}
//****************************************************************************80

double r8_pi ( )

//****************************************************************************80
//
//  Purpose:
//
//    r8_pi() returns the value of PI as an R8.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    17 August 2004
//
//  Author:
//
//    John Burkardt
//
//  Output:
//
//    double R8_PI, the value of PI.
//
{
  const double value = 3.141592653589793;

  return value;
}
//****************************************************************************80

double r8_pi_sqrt ( )

//****************************************************************************80
//
//  Purpose:
//
//    r8_pi_sqrt() returns the square root of PI as an R8.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    11 September 2012
//
//  Author:
//
//    John Burkardt
//
//  Output:
//
//    double R8_PI_SQRT, the square root of PI.
//
{
  const double value = 1.7724538509055160273;

  return value;
}
//****************************************************************************80

double r8_power ( double r, int p )

//****************************************************************************80
//
//  Purpose:
//
//    r8_power() computes an integer power of an R8.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    04 October 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double R, the base.
//
//    int P, the power, which may be negative.
//
//  Output:
//
//    double R8_POWER, the value of R^P.
//
{
  double value;
//
//  Special case.  R^0 = 1.
//
  if ( p == 0 )
  {
    value = 1.0;
  }
//
//  Special case.  Positive powers of 0 are 0.
//  We go ahead and compute negative powers, relying on the software to complain.
//
  else if ( r == 0.0 )
  {
    if ( 0 < p )
    {
      value = 0.0;
    }
    else
    {
      value = pow ( r, p );
    }
  }
  else if ( 1 <= p )
  {
    value = pow ( r, p );
  }
  else
  {
    value = pow ( r, p );
  }

  return value;
}
//****************************************************************************80

double r8_power_fast ( double r, int p, int &mults )

//****************************************************************************80
//
//  Purpose:
//
//    r8_power_fast() computes the P-th power of R, for real R and integer P.
//
//  Discussion:
//
//    Obviously, R^P can be computed using P-1 multiplications.
//
//    However, R^P can also be computed using at most 2*LOG2(P) multiplications.
//    To do the calculation this way, let N = LOG2(P).
//    Compute A, A^2, A^4, ..., A^N by N-1 successive squarings.
//    Start the value of R^P at A, and each time that there is a 1 in
//    the binary expansion of P, multiply by the current result of the squarings.
//
//    This algorithm is not optimal.  For small exponents, and for special
//    cases, the result can be computed even more quickly.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    06 January 2013
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double R, the base.
//
//    int P, the power, which may be negative.
//
//  Output:
//
//    int &MULTS, the number of multiplications and divisions.
//
//    double R8_POWER_FAST, the value of R^P.
//
{
  int p_mag;
  int p_sign;
  double r2;
  double value;

  mults = 0;
//
//  Special bases.
//
  if ( r == 1.0 )
  {
    value = 1.0;
    return value;
  }

  if ( r == -1.0 )
  {
    if ( ( p % 2 ) == 1 )
    {
      value = -1.0;
    }
    else
    {
      value = 1.0;
    }
    return value;
  }

  if ( r == 0.0 )
  {
    if ( p <= 0 )
    {
      cerr << "\n";
      cerr << "R8_POWER_FAST - Fatal error!\n";
      cerr << "  Base is zero, and exponent is negative.\n";
      exit ( 1 );
    }

    value = 0.0;
    return value;
  }
//
//  Special powers.
//
  if ( p == -1 )
  {
    value = 1.0 / r;
    mults = mults + 1;
    return value;
  }
  else if ( p == 0 )
  {
    value = 1.0;
    return value;
  }
  else if ( p == 1 )
  {
    value = r;
    return value;
  }
//
//  Some work to do.
//
  p_mag = abs ( p );
  p_sign = i4_sign ( p );

  value = 1.0;
  r2 = r;

  while ( 0 < p_mag )
  {
    if ( ( p_mag % 2 ) == 1 )
    {
      value = value * r2;
      mults = mults + 1;
    }

    p_mag = p_mag / 2;
    r2 = r2 * r2;
    mults = mults + 1;
  }

  if ( p_sign == -1 )
  {
    value = 1.0 / value;
    mults = mults + 1;
  }

  return value;
}
//****************************************************************************80

void r8_print ( double r, string title )

//****************************************************************************80
//
//  Purpose:
//
//    r8_print() prints an R8.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    14 August 2014
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double R, the value to print.
//
//    string TITLE, a title.
//
{
  cout << title << "  "
       << r << "\n";

  return;
}
//****************************************************************************80

double r8_radians ( double degrees )

//****************************************************************************80
//
//  Purpose:
//
//    r8_radians() converts an angle from degree to radian measure.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    15 May 2013
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double DEGREES, the angle measurement in degrees.
//
//  Output:
//
//    double R8_RADIANS, the angle measurement in radians.
//
{
  const double r8_pi = 3.1415926535897932384626434;
  double value;

  value = degrees * r8_pi / 180.0;

  return value;
}
//****************************************************************************80

double r8_relu ( double x )

//****************************************************************************80
//
//  Purpose:
//
//    r8_relu() evaluates the ReLU function of an R8.
//
//  Discussion:
//
//    An R8 is a double precision real value.
//
//    The ReLU function is max(x,0.0).
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    10 January 2019
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double X, the argument.
//
//  Output:
//
//    double VALUE, the function value.
//
{
  double value;

  if ( x <= 0.0 )
  {
    value = 0.0;
  }
  else
  {
    value = x;
  }

  return value;
}
//****************************************************************************80

double r8_reverse_bytes ( double x )

//****************************************************************************80
//
//  Purpose:
//
//    r8_reverse_bytes() reverses the bytes in an R8.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    12 May 2007
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double X, a value whose bytes are to be reversed.
//
//  Output:
//
//    R8_REVERSE_BYTES, a value with bytes in reverse order;
//
{
  char c;
  union
  {
    double ydouble;
    char ychar[8];
  } y;

  y.ydouble = x;

  c = y.ychar[0];
  y.ychar[0] = y.ychar[7];
  y.ychar[7] = c;

  c = y.ychar[1];
  y.ychar[1] = y.ychar[6];
  y.ychar[6] = c;

  c = y.ychar[2];
  y.ychar[2] = y.ychar[5];
  y.ychar[5] = c;

  c = y.ychar[3];
  y.ychar[3] = y.ychar[4];
  y.ychar[4] = c;

  return ( y.ydouble );
}
//****************************************************************************80

double r8_rise ( double x, int n )

//****************************************************************************80
//
//  Purpose:
//
//    r8_rise() computes the rising factorial function [X]^N.
//
//  Discussion:
//
//    [X}^N = X * ( X + 1 ) * ( X + 2 ) * ... * ( X + N - 1 ).
//
//    Note that the number of ways of arranging N objects in M ordered
//    boxes is [M}^N.  (Here, the ordering in each box matters).  Thus,
//    2 objects in 2 boxes have the following 6 possible arrangements:
//
//      -/12, 1/2, 12/-, -/21, 2/1, 21/-.
//
//    Moreover, the number of non-decreasing maps from a set of
//    N to a set of M ordered elements is [M]^N / N!.  Thus the set of
//    nondecreasing maps from (1,2,3) to (a,b,c,d) is the 20 elements:
//
//      aaa, abb, acc, add, aab, abc, acd, aac, abd, aad
//      bbb, bcc, bdd, bbc, bcd, bbd, ccc, cdd, ccd, ddd.
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    08 May 2003
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double X, the argument of the rising factorial function.
//
//    int N, the order of the rising factorial function.
//    If N = 0, RISE = 1, if N = 1, RISE = X.  Note that if N is
//    negative, a "falling" factorial will be computed.
//
//  Output:
//
//    double R8_RISE, the value of the rising factorial function.
//
{
  int i;
  double value;

  value = 1.0;

  if ( 0 < n )
  {
    for ( i = 1; i <= n; i++ )
    {
      value = value * x;
      x = x + 1.0;
    }
  }
  else if ( n < 0 )
  {
    for ( i = -1; n <= i; i-- )
    {
      value = value * x;
      x = x - 1.0;
    }
  }

  return value;
}
//****************************************************************************80

void r8_rise_values ( int &n_data, double &x, int &n, double &f )

//****************************************************************************80
//
//  Purpose:
//
//    r8_rise_values() returns some values of the rising factorial function.
//
//  Discussion:
//
//    Pochhammer(X,Y) = Gamma(X+Y) / Gamma(X)
//
//    For integer arguments, Pochhammer(M,N) = ( M + N - 1 )! / ( N - 1 )!
//
//    In Mathematica, the function can be evaluated by:
//
//      Pochhammer[X,Y]
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    20 December 2014
//
//  Author:
//
//    John Burkardt
//
//  Reference:
//
//    Milton Abramowitz, Irene Stegun,
//    Handbook of Mathematical Functions,
//    National Bureau of Standards, 1964,
//    ISBN: 0-486-61272-4,
//    LC: QA47.A34.
//
//    Stephen Wolfram,
//    The Mathematica Book,
//    Fourth Edition,
//    Cambridge University Press, 1999,
//    ISBN: 0-521-64314-7,
//    LC: QA76.95.W65.
//
//  Input:
//
//    int &N_DATA: the user sets N_DATA to 0 before the first call.
//
//  Output:
//
//    int &N_DATA: the routine increments N_DATA by 1, and
//    returns the corresponding data; when there is no more data, the
//    output value of N_DATA will be 0 again.
//
//    double &X, int &N, the arguments of the function.
//
//    double &F, the value of the function.
//
{
# define N_MAX 15

  static double f_vec[N_MAX] = {
    1680.000000000000,
    1962.597656250000,
    2279.062500000000,
    2631.972656250000,
    3024.000000000000,
    1.000000000000000,
    7.500000000000000,
    63.75000000000000,
    605.6250000000000,
    6359.062500000000,
    73129.21875000000,
    914115.2343750000,
    1.234055566406250E+07,
    1.789380571289063E+08,
    2.773539885498047E+09 };

  static int n_vec[N_MAX] = {
    4,
    4,
    4,
    4,
    4,
    0,
    1,
    2,
    3,
    4,
    5,
    6,
    7,
    8,
    9 };

  static double x_vec[N_MAX] = {
    5.00,
    5.25,
    5.50,
    5.75,
    6.00,
    7.50,
    7.50,
    7.50,
    7.50,
    7.50,
    7.50,
    7.50,
    7.50,
    7.50,
    7.50 };

  if ( n_data < 0 )
  {
    n_data = 0;
  }

  n_data = n_data + 1;

  if ( N_MAX < n_data )
  {
    n_data = 0;
    x = 0.0;
    n = 0;
    f = 0.0;
  }
  else
  {
    x = x_vec[n_data-1];
    n = n_vec[n_data-1];
    f = f_vec[n_data-1];
  }

  return;
# undef N_MAX
}
//****************************************************************************80

double r8_round ( double x )

//****************************************************************************80
//
//  Purpose:
//
//    r8_round() rounds an R8 to the nearest integral value.
//
//  Example:
//
//        X         Value
//
//      1.3         1.0
//      1.4         1.0
//      1.5         1.0 or 2.0
//      1.6         2.0
//      0.0         0.0
//     -0.7        -1.0
//     -1.1        -1.0
//     -1.6        -2.0
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    25 March 2013
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double X, the value.
//
//  Output:
//
//    double R8_ROUND, the rounded value.
//
{
  double value;

  if ( x < 0.0 )
  {
    value = - ( double ) floor ( - x + 0.5 );
  }
  else
  {
    value =   ( double ) floor (   x + 0.5 );
  }

  return value;
}
//****************************************************************************80

int r8_round_i4 ( double x )

//****************************************************************************80
//
//  Purpose:
//
//    r8_round_i4() rounds an R8, returning an I4.
//
//  Example:
//
//        X         Value
//
//      1.3         1
//      1.4         1
//      1.5         1 or 2
//      1.6         2
//      0.0         0
//     -0.7        -1
//     -1.1        -1
//     -1.6        -2
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    25 March 2013
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double X, the value.
//
//  Output:
//
//    int R8_ROUND_I4, the rounded value.
//
{
  int value;

  if ( x < 0.0 )
  {
    value = - floor ( - x + 0.5 );
  }
  else
  {
    value =   floor (   x + 0.5 );
  }

  return value;
}
//****************************************************************************80

double r8_round2 ( int nplace, double x )

//****************************************************************************80
//
//  Purpose:
//
//    r8_round2() rounds an R8 in base 2.
//
//  Discussion:
//
//    Assume that the input quantity X has the form
//
//      X = S * J * 2^L
//
//    where S is plus or minus 1, L is an integer, and J is a binary
//    mantissa which is either exactly zero, or greater than or equal
//    to 0.5 and less than 1.0.
//
//    Then on return, XROUND = R8_ROUND2 ( NPLACE, X ) will satisfy
//
//      XROUND = S * K * 2^L
//
//    where S and L are unchanged, and K is a binary mantissa which
//    agrees with J in the first NPLACE binary digits and is zero
//    thereafter.
//
//    If NPLACE is 0, XROUND will always be zero.
//
//    If NPLACE is 1, the mantissa of XROUND will be 0 or 0.5.
//
//    If NPLACE is 2, the mantissa of XROUND will be 0, 0.25, 0.50,
//    or 0.75.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    16 April 2004
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int NPLACE, the number of binary digits to
//    preserve.  NPLACE should be 0 or positive.
//
//    double X, the real number to be decomposed.
//
//  Output:
//
//    double R8_ROUND2, the rounded value of X.
//
{
  int iplace;
  int l;
  int s;
  double xmant;
  double xtemp;
  double value;

  value = 0.0;
//
//  1: Handle the special case of 0.
//
  if ( x == 0.0 )
  {
    return value;
  }

  if ( nplace <= 0 )
  {
    return value;
  }
//
//  2: Determine the sign S.
//
  if ( 0.0 < x )
  {
    s = 1;
    xtemp = x;
  }
  else
  {
    s = -1;
    xtemp = -x;
  }
//
//  3: Force XTEMP to lie between 1 and 2, and compute the
//  logarithm L.
//
  l = 0;

  while ( 2.0 <= xtemp )
  {
    xtemp = xtemp / 2.0;
    l = l + 1;
  }

  while ( xtemp < 1.0 )
  {
    xtemp = xtemp * 2.0;
    l = l - 1;
  }
//
//  4: Strip out the digits of the mantissa as XMANT, and decrease L.
//
  xmant = 0.0;
  iplace = 0;

  for ( ; ; )
  {
    xmant = 2.0 * xmant;

    if ( 1.0 <= xtemp )
    {
      xmant = xmant + 1.0;
      xtemp = xtemp - 1.0;
    }

    iplace = iplace + 1;

    if ( xtemp == 0.0 || nplace <= iplace )
    {
      value = s * xmant * pow ( 2.0, l );
      break;
    }

    l = l - 1;
    xtemp = xtemp * 2.0;
  }

  return value;
}
//****************************************************************************80

double r8_roundb ( int base, int nplace, double x )

//****************************************************************************80
//
//  Purpose:
//
//    r8_roundb() rounds an R8 in a given base.
//
//  Discussion:
//
//    The code does not seem to do a good job of rounding when
//    the base is negative.
//
//    Assume that the input quantity X has the form
//
//      X = S * J * BASE^L
//
//    where S is plus or minus 1, L is an integer, and J is a
//    mantissa base BASE which is either exactly zero, or greater
//    than or equal to (1/BASE) and less than 1.0.
//
//    Then on return, XROUND will satisfy
//
//      XROUND = S * K * BASE^L
//
//    where S and L are unchanged, and K is a mantissa base BASE
//    which agrees with J in the first NPLACE digits and is zero
//    thereafter.
//
//    Note that because of rounding, for most bases, most numbers
//    with a fractional quantities cannot be stored exactly in the
//    computer, and hence will have trailing "bogus" digits.
//
//    If NPLACE is 0, XROUND will always be zero.
//
//    If NPLACE is 1, the mantissa of XROUND will be 0,
//    1/BASE, 2/BASE, ..., (BASE-1)/BASE.
//
//    If NPLACE is 2, the mantissa of XROUND will be 0,
//    BASE/BASE^2, (BASE+1)/BASE^2, ...,
//    BASE^2-2/BASE^2, BASE^2-1/BASE^2.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    12 November 2010
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int BASE, the base of the arithmetic.
//    BASE must not be zero.  Theoretically, BASE may be negative.
//
//    int NPLACE, the number of digits base BASE to
//    preserve.  NPLACE should be 0 or positive.
//
//    double X, the number to be decomposed.
//
//  Output:
//
//    double R8_ROUNDB, the rounded value of X.
//
{
  int iplace;
  int is;
  int js;
  int l;
  double r8_base;
  double value;
  double xmant;
  double xtemp;

  value = 0.0;
  r8_base = ( double ) base;
//
//  0: Error checks.
//
  if ( base == 0 )
  {
    cerr << "\n";
    cerr << "R8_ROUNDB - Fatal error!\n";
    cerr << "  The base BASE cannot be zero.\n";
    exit ( 1 );
  }
//
//  1: Handle the special case of 0.
//
  if ( x == 0.0 )
  {
    return value;
  }

  if ( nplace <= 0 )
  {
    return value;
  }
//
//  2: Determine the sign IS.
//
  if ( 0.0 < x )
  {
    is = 1;
    xtemp = x;
  }
  else
  {
    is = -1;
    xtemp = -x;
  }
//
//  3: Force XTEMP to lie between 1 and ABS(BASE), and compute the
//  logarithm L.
//
  l = 0;

  while ( fabs ( r8_base ) <= fabs ( xtemp ) )
  {
    xtemp = xtemp / r8_base;

    if ( xtemp < 0.0 )
    {
      is = -is;
      xtemp = -xtemp;
    }
    l = l + 1;
  }

  while ( fabs ( xtemp ) < 1.0 )
  {
    xtemp = xtemp * r8_base;

    if ( xtemp < 0.0 )
    {
      is = -is;
      xtemp = -xtemp;
    }

    l = l - 1;
  }
//
//  4: Now strip out the digits of the mantissa as XMANT, and
//  decrease L.
//
  xmant = 0.0;
  iplace = 0;
  js = is;

  for ( ; ; )
  {
    xmant = r8_base * xmant;

    if ( xmant < 0.0 )
    {
      js = -js;
      xmant = -xmant;
    }

    if ( 1.0 <= xtemp )
    {
      xmant = xmant + ( int ) ( xtemp );
      xtemp = xtemp - ( int ) ( xtemp );
    }

    iplace = iplace + 1;

    if ( xtemp == 0.0 || nplace <= iplace )
    {
      value = ( double ) js * xmant * pow ( r8_base, l );
      break;
    }

    l = l - 1;
    xtemp = xtemp * r8_base;

    if ( xtemp < 0.0 )
    {
      is = -is;
      xtemp = -xtemp;
    }
  }

  return value;
}
//****************************************************************************80

double r8_roundx ( int nplace, double x )

//****************************************************************************80
//
//  Purpose:
//
//    r8_roundx() rounds an R8 in base 10.
//
//  Discussion:
//
//    Assume that the input quantity X has the form
//
//      X = S * J * 10^L
//
//    where S is plus or minus 1, L is an integer, and J is a decimal
//    mantissa which is either exactly zero, or greater than or equal
//    to 0.1 and less than 1.0.
//
//    Then on return, XROUND will satisfy
//
//      XROUND = S * K * 10^L
//
//    where S and L are unchanged, and K is a decimal mantissa which
//    agrees with J in the first NPLACE decimal digits and is zero
//    thereafter.
//
//    Note that because of rounding, most decimal fraction quantities
//    cannot be stored exactly in the computer, and hence will have
//    trailing "bogus" digits.
//
//    If NPLACE is 0, XROUND will always be zero.
//
//    If NPLACE is 1, the mantissa of XROUND will be 0, 0.1,
//    0.2, ..., or 0.9.
//
//    If NPLACE is 2, the mantissa of XROUND will be 0, 0.01, 0.02,
//    0.03, ..., 0.98, 0.99.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    12 September 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int NPLACE, the number of decimal digits to
//    preserve.  NPLACE should be 0 or positive.
//
//    double X, the number to be decomposed.
//
//  Output:
//
//    double R8_ROUNDX, the rounded value of X.
//
{
  int iplace;
  int is;
  int l;
  double xmant;
  double xround;
  double xtemp;

  xround = 0.0;
//
//  1: Handle the special case of 0.
//
  if ( x == 0.0 )
  {
    return xround;
  }

  if ( nplace <= 0 )
  {
    return xround;
  }
//
//  2: Determine the sign IS.
//
  if ( 0.0 < x )
  {
    is = 1;
    xtemp = x;
  }
  else
  {
    is = -1;
    xtemp = -x;
  }
//
//  3: Force XTEMP to lie between 1 and 10, and compute the
//  logarithm L.
//
  l = 0;

  while ( 10.0 <= x )
  {
    xtemp = xtemp / 10.0;
    l = l + 1;
  }

  while ( xtemp < 1.0 )
  {
    xtemp = xtemp * 10.0;
    l = l - 1;
  }
//
//  4: Now strip out the digits of the mantissa as XMANT, and
//  decrease L.
//
  xmant = 0.0;
  iplace = 0;

  for ( ; ; )
  {
    xmant = 10.0 * xmant;

    if ( 1.0 <= xtemp )
    {
      xmant = xmant + ( int ) xtemp;
      xtemp = xtemp - ( int ) xtemp;
    }

    iplace = iplace + 1;

    if ( xtemp == 0.0 || nplace <= iplace )
    {
      xround = is * xmant * pow ( 10.0, l );
      break;
    }

    l = l - 1;
    xtemp = xtemp * 10.0;
  }

  return xround;
}
//****************************************************************************80

double r8_secd ( double degrees )

//****************************************************************************80
//
//  Purpose:
//
//    r8_secd() returns the secant of an angle given in degrees.
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    27 July 2014
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double DEGREES, the angle in degrees.
//
//  Output:
//
//    double R8_SECD, the secant of the angle.
//
{
  const double r8_pi = 3.141592653589793;
  double radians;
  double value;

  radians = r8_pi * ( degrees / 180.0 );

  value = 1.0 / cos ( radians );

  return value;
}
//****************************************************************************80

double r8_sech ( double x )

//****************************************************************************80
//
//  Purpose:
//
//    r8_sech() evaluates the hyperbolic secant, while avoiding COSH overflow.
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    21 December 2011
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double X, the argument of the function.
//
//  Output:
//
//    double R8_SECH, the value of the function.
//
{
  const double log_huge = 80.0;
  double value;

  if ( log_huge < fabs ( x ) )
  {
    value = 0.0;
  }
  else
  {
    value = 1.0 / cosh ( x );
  }
  return value;
}
//****************************************************************************80

double r8_sigmoid ( double l, double b, double m, double x )

//****************************************************************************80
//
//  Purpose:
//
//    r8_sigmoid() evaluates the sigmoid or logistic function.
//
//  Discussion:
//
//    An R8 is a double value.
//
//    The sigmoid function is useful for classification problems in
//    machine learning.  Its value is always between 0 and 1.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    11 October 2019
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double l, the maximum value of the function.  This is often 1.
//
//    double b, the cutoff value, where the function equals l/2.
//    This is often 0.
//
//    double m, the slope, which determines the steepness of the curve
//    and the width of the uncertainty interval.  This is often 1.
//
//    double x, the argument.
//
//  Output:
//
//    double r8_sigmoid, the value.
//
{
  double value;

  value = l / ( 1.0 + exp ( - m * ( x - b ) ) );

  return value;
}
//****************************************************************************80

double r8_sign ( double x )

//****************************************************************************80
//
//  Purpose:
//
//    r8_sign() returns the sign of an R8.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    18 October 2004
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double X, the number whose sign is desired.
//
//  Output:
//
//    double R8_SIGN, the sign of X.
//
{
  double value;

  if ( x < 0.0 )
  {
    value = -1.0;
  }
  else
  {
    value = 1.0;
  }
  return value;
}
//****************************************************************************80

double r8_sign3 ( double x )

//****************************************************************************80
//
//  Purpose:
//
//    r8_sign3() returns the three-way sign of an R8.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    28 September 2014
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double X, the number whose sign is desired.
//
//  Output:
//
//    double R8_SIGN3, the sign of X.
//
{
  double value;

  if ( x < 0.0 )
  {
    value = -1.0;
  }
  else if ( x == 0.0 )
  {
    value = 0.0;
  }
  else
  {
    value = 1.0;
  }
  return value;
}
//****************************************************************************80

char r8_sign_char ( double x )

//****************************************************************************80
//
//  Purpose:
//
//    r8_sign_char() returns a character indicating the sign of an R8.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    28 April 2012
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double X, the number whose sign is desired.
//
//  Output:
//
//    char R8_SIGN_CHAR, the sign of X, '-', '0' or '+'.
//
{
  char value;

  if ( x < 0.0 )
  {
    value = '-';
  }
  else if ( x == 0.0 )
  {
    value = '0';
  }
  else
  {
    value = '+';
  }
  return value;
}
//****************************************************************************80

bool r8_sign_match ( bool r1, bool r2 )

//****************************************************************************80
//
//  Purpose:
//
//    r8_sign_match() is TRUE if two R8's are of the same sign.
//
//  Discussion:
//
//    This test could be coded numerically as
//
//      if ( 0 <= r1 * r2 ) then ...
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    26 April 2012
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double R1, R2, the values to check.
//
//  Output:
//
//    bool R8_SIGN_MATCH, is TRUE if ( R1 <= 0 and R2 <= 0 )
//    or ( 0 <= R1 and 0 <= R2 ).
//
{
  bool value;

  value = ( r1 <= 0.0 && r2 <= 0.0 ) || ( 0.0 <= r1 && 0.0 <= r2 );

  return value;
}
//****************************************************************************80

bool r8_sign_match_strict ( bool r1, bool r2 )

//****************************************************************************80
//
//  Purpose:
//
//    r8_sign_match_strict() is TRUE if two R8's are of the same strict sign.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    28 April 2012
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double R1, R2, the values to check.
//
//  Output:
//
//    bool R8_SIGN_MATCH_STRICT, is TRUE if the signs match.
//
{
  bool value;

  value = ( r1 < 0.0 && r2 < 0.0 ) || 
          ( r1 == 0.0 && r2 == 0.0 ) || 
          ( 0.0 < r1 && 0.0 < r2 );

  return value;
}
//****************************************************************************80

bool r8_sign_opposite ( double r1, double r2 )

//****************************************************************************80
//
//  Purpose:
//
//    r8_sign_opposite() is TRUE if two R8's are not of the same sign.
//
//  Discussion:
//
//    This test could be coded numerically as
//
//      if ( r1 * r2 <= 0.0 ) ...
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    23 June 2010
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double R1, R2, the values to check.
//
//  Output:
//
//    bool R8_SIGN_OPPOSITE, is TRUE if ( R1 <= 0 and 0 <= R2 )
//    or ( R2 <= 0 and 0 <= R1 ).
//
{
  bool value;

  value = ( r1 <= 0.0 && 0.0 <= r2 ) || ( r2 <= 0.0 && 0.0 <= r1 );

  return value;
}
//****************************************************************************80

bool r8_sign_opposite_strict ( double r1, double r2 )

//****************************************************************************80
//
//  Purpose:
//
//    r8_sign_opposite_strict() is TRUE if two R8's are strictly of opposite sign.
//
//  Discussion:
//
//    This test could be coded numerically as
//
//      if ( r1 * r2 < 0.0 ) ...
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    23 June 2010
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double R1, R2, the values to check.
//
//  Output:
//
//    bool R8_SIGN_OPPOSITE_STRICT, is TRUE if ( R1 < 0 and 0 < R2 )
//    or ( R2 < 0 and 0 < R1 ).
//
{
  bool value;

  value = ( r1 < 0.0 && 0.0 < r2 ) || ( r2 < 0.0 && 0.0 < r1 );

  return value;
}
//****************************************************************************80

double r8_sign2 ( double x, double y )

//****************************************************************************80
//
//  Purpose:
//
//    r8_sign2() returns the first argument with the sign of the second.
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    08 January 2002
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double X, Y, the input arguments.
//
//  Output:
//
//    double R8_SIGN2, is equal to the absolute value of X, and
//    has the sign of Y.
//
{
  double value;

  if ( 0.0 <= y )
  {
    value = fabs ( x );
  } 
  else
  {
    value = - fabs ( x );
  }
  return value;
}
//****************************************************************************80

void r8_sincos_sum ( double a, double b, double &d, double &e, double &f )

//****************************************************************************80
//
//  Purpose:
//
//    r8_sincos_sum() simplifies a*sin(cx)+b*cos(cx).
//
//  Discussion:
//
//    The expression
//      a * sin ( c * x ) + b * cos ( c * x )
//    can be rewritten as
//      d * sin ( c * x + e )
//    or
//      d * cos ( c * x + f ) 
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    19 January 2016
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double A, B, the coefficients in the linear combination.
//
//  Output:
//
//    double &D, &E, &F, the new coefficient, and the shift for
//    sine or for cosine.
//
{
  const double r8_pi = 3.141592653589793E+00;

  d = sqrt ( a * a + b * b );
  e = atan2 ( b, a );
  f = atan2 ( b, a ) - r8_pi / 2.0E+00;
  if ( f < - r8_pi )
  {
    f = f + 2.0E+00 * r8_pi;
  }

  return;
}
//****************************************************************************80

double r8_sind ( double degrees )

//****************************************************************************80
//
//  Purpose:
//
//    r8_sind() returns the sine of an angle given in degrees.
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    27 July 2014
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double DEGREES, the angle in degrees.
//
//  Output:
//
//    double R8_SIND, the sine of the angle.
//
{
  const double r8_pi = 3.141592653589793;
  double radians;
  double value;

  radians = r8_pi * ( degrees / 180.0 );

  value = sin ( radians );

  return value;
}
//****************************************************************************80

double r8_softplus ( double x )

//****************************************************************************80
//
//  Purpose:
//
//    r8_softplus() evaluates the softplus function of an R8.
//
//  Discussion:
//
//    An R8 is a double precision real value.
//
//    The softplus function is a smoothed (differentiable) version of max(x,0.0).
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    07 September 2018
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double X, the argument.
//
//  Output:
//
//    double VALUE, the function value.
//
{
  double value;

  if ( x <= -36.841 )
  {
    value = 0.0;
  }
  else if ( +36.841 <= x )
  {
    value = x;
  }
  else
  {
    value = log ( 1.0 + exp ( x ) );
  }

  return value;
}
//****************************************************************************80

double r8_sqrt_i4 ( int i )

//****************************************************************************80
//
//  Purpose:
//
//    r8_sqrt_i4() returns the square root of an I4 as an R8.
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    05 June 2012
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int I, the number whose square root is desired.
//
//  Output:
//
//    double R8_SQRT_I4, the value of sqrt(I).
//
{
  double value;

  value = sqrt ( ( double ) ( i ) );

  return value;
}
//****************************************************************************80

double r8_square ( double x )

//****************************************************************************80
//
//  Purpose:
//
//    r8_square() returns the square of an R8.
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    04 August 2019
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double x: the argument.
//
//  Output:
//
//    double r8_square: the square of x.
//
{
  double value;

  value = x * x;

  return value;
}
//****************************************************************************80

double r8_sum ( double x, double y )

//****************************************************************************80
//
//  Purpose:
//
//    r8_sum() returns the sum of two R8's.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    20 April 2009
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double X, Y, the quantities to add.
//
//  Output:
//
//    double R8_SUM, the sum of X and Y.
//
{
  double value;

  value = x + y;

  return value;
}
//****************************************************************************80

void r8_swap ( double &x, double &y )

//****************************************************************************80
//
//  Purpose:
//
//    r8_swap() switches two R8's.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    29 August 2004
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double &X, &Y: values to be interchanged.
//
//  Output:
//
//    double &X, &Y: the interchanged values.
{
  double z;

  z = x;
  x = y;
  y = z;

  return;
}
//****************************************************************************80

void r8_swap3 ( double &x, double &y, double &z )

//****************************************************************************80
//
//  Purpose:
//
//    r8_swap3() swaps three R8's.
//
//  Example:
//
//    Input:
//
//      X = 1, Y = 2, Z = 3
//
//    Output:
//
//      X = 2, Y = 3, Z = 1
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    06 January 2013
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double &X, &Y, &Z, three values to be swapped.
//
//  Output:
//
//    double &X, &Y, &Z: the swapped values.
{
  double w;

  w = x;
  x = y;
  y = z;
  z =  w;

  return;
}
//****************************************************************************80

double r8_tand ( double degrees )

//****************************************************************************80
//
//  Purpose:
//
//    r8_tand() returns the tangent of an angle given in degrees.
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    27 July 2014
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double DEGREES, the angle in degrees.
//
//  Output:
//
//    double R8_TAND, the tangent of the angle.
//
{
  const double r8_pi = 3.141592653589793;
  double radians;
  double value;

  radians = r8_pi * ( degrees / 180.0 );

  value = sin ( radians ) / cos ( radians );

  return value;
}
//****************************************************************************80

double r8_tiny ( )

//****************************************************************************80
//
//  Purpose:
//
//    r8_tiny() returns a "tiny" R8.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    08 March 2007
//
//  Author:
//
//    John Burkardt
//
//  Output:
//
//    double R8_TINY, a "tiny" R8 value.
//
{
  const double value = 0.4450147717014E-307;

  return value;
}
//****************************************************************************80

void r8_to_dhms ( double r, int &d, int &h, int &m, int &s )

//****************************************************************************80
//
//  Purpose:
//
//    r8_to_dhms() converts an R8 day value into days, hours, minutes, seconds.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    17 April 2004
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double R, a real number representing a time period measured in days.
//
//  Output:
//
//    int &D, &H, &M, &S, the equivalent number of days, hours,
//    minutes and seconds.
//
{
  int sign;

  if ( 0.0 <= r )
  {
    sign = 1;
  }
  else
  {
    sign = - 1;
    r = - r;
  }

  d = ( int ) r;

  r = r - ( double ) d;
  r = 24.0 * r;
  h = ( int ) r;

  r = r - ( double ) h;
  r = 60.0 * r;
  m = ( int ) r;

  r = r - ( double ) m;
  r = 60.0 * r;
  s = ( int ) r;

  if ( sign == -1 )
  {
    d = -d;
    h = -h;
    m = -m;
    s = -s;
  }

  return;
}
//****************************************************************************80

int r8_to_i4 ( double xmin, double xmax, double x, int ixmin, int ixmax )

//****************************************************************************80
//
//  Purpose:
//
//    r8_to_i4() maps real X in [XMIN, XMAX] to integer IX in [IXMIN, IXMAX].
//
//  Discussion:
//
//    IX := IXMIN + ( IXMAX - IXMIN ) * ( X - XMIN ) / ( XMAX - XMIN )
//    IX := min ( IX, max ( IXMIN, IXMAX ) )
//    IX := max ( IX, min ( IXMIN, IXMAX ) )
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    19 April 2014
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double XMIN, XMAX, the real range.  XMAX and XMIN must not be
//    equal.  It is not necessary that XMIN be less than XMAX.
//
//    double X, the real number to be converted.
//
//    int IXMIN, IXMAX, the allowed range of the output
//    variable.  IXMAX corresponds to XMAX, and IXMIN to XMIN.
//    It is not necessary that IXMIN be less than IXMAX.
//
//  Output:
//
//    int R8_TO_I4, the value in the range [IXMIN,IXMAX] that
//    corresponds to X.
//
{
  int ix;
  double temp;

  if ( xmax == xmin )
  {
    cerr << "\n";
    cerr << "R8_TO_I4 - Fatal error!\n";
    cerr << "  XMAX = XMIN, making a zero divisor.\n";
    cerr << "  XMAX = " << xmax << "\n";
    cerr << "  XMIN = " << xmin << "\n";
    exit ( 1 );
  }

  temp =
      ( ( xmax - x        ) * ( double ) ixmin
      + (        x - xmin ) * ( double ) ixmax )
      / ( xmax     - xmin );

  if ( 0.0 <= temp )
  {
    temp = temp + 0.5;
  }
  else
  {
    temp = temp - 0.5;
  }

  ix = ( int ) temp;

  return ix;
}
//****************************************************************************80

double r8_to_r8_discrete ( double r, double rmin, double rmax, int nr )

//****************************************************************************80
//
//  Purpose:
//
//    r8_to_r8_discrete() maps R to RD in [RMIN, RMAX] with NR possible values.
//
//  Discussion:
//
//    if ( R < RMIN ) then
//      RD = RMIN
//    else if ( RMAX < R ) then
//      RD = RMAX
//    else
//      T = nint ( ( NR - 1 ) * ( R - RMIN ) / ( RMAX - RMIN ) )
//      RD = RMIN + T * ( RMAX - RMIN ) / real ( NR - 1 )
//
//    In the special case where NR = 1, when
//
//      XD = 0.5 * ( RMAX + RMIN )
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    11 September 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double R, the number to be converted.
//
//    double RMAX, RMIN, the maximum and minimum
//    values for RD.
//
//    int NR, the number of allowed values for XD.
//    NR should be at least 1.
//
//  Output:
//
//    double RD, the corresponding discrete value.
//
{
  int f;
  double rd;
//
//  Check for errors.
//
  if ( nr < 1 )
  {
    cerr << "\n";
    cerr << "R8_TO_R8_DISCRETE - Fatal error!\n";
    cerr << "  NR = " << nr << "\n";
    cerr << "  but NR must be at least 1.\n";
    exit ( 1 );
  }

  if ( nr == 1 )
  {
    rd = 0.5 * ( rmin + rmax );
    return rd;
  }

  if ( rmax == rmin )
  {
    rd = rmax;
    return rd;
  }

  f = round ( ( double ) ( nr ) * ( rmax - r ) / ( rmax - rmin ) );
  f = max ( f, 0 );
  f = min ( f, nr );

  rd = ( ( double ) (      f ) * rmin
       + ( double ) ( nr - f ) * rmax )
       / ( double ) ( nr     );

  return rd;
}
//****************************************************************************80

double r8_uniform_01 ( int &seed )

//****************************************************************************80
//
//  Purpose:
//
//    r8_uniform_01() returns a unit pseudorandom R8.
//
//  Discussion:
//
//    This routine implements the recursion
//
//      seed = ( 16807 * seed ) mod ( 2^31 - 1 )
//      u = seed / ( 2^31 - 1 )
//
//    The integer arithmetic never requires more than 32 bits,
//    including a sign bit.
//
//    If the initial seed is 12345, then the first three computations are
//
//      Input     Output      R8_UNIFORM_01
//      SEED      SEED
//
//         12345   207482415  0.096616
//     207482415  1790989824  0.833995
//    1790989824  2035175616  0.947702
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    09 April 2012
//
//  Author:
//
//    John Burkardt
//
//  Reference:
//
//    Paul Bratley, Bennett Fox, Linus Schrage,
//    A Guide to Simulation,
//    Second Edition,
//    Springer, 1987,
//    ISBN: 0387964673,
//    LC: QA76.9.C65.B73.
//
//    Bennett Fox,
//    Algorithm 647:
//    Implementation and Relative Efficiency of Quasirandom
//    Sequence Generators,
//    ACM Transactions on Mathematical Software,
//    Volume 12, Number 4, December 1986, pages 362-376.
//
//    Pierre L'Ecuyer,
//    Random Number Generation,
//    in Handbook of Simulation,
//    edited by Jerry Banks,
//    Wiley, 1998,
//    ISBN: 0471134031,
//    LC: T57.62.H37.
//
//    Peter Lewis, Allen Goodman, James Miller,
//    A Pseudo-Random Number Generator for the System/360,
//    IBM Systems Journal,
//    Volume 8, Number 2, 1969, pages 136-143.
//
//  Input:
//
//    int &SEED, the "seed" value.  Normally, this
//    value should not be 0.  On output, SEED has been updated.
//
//  Output:
//
//    double R8_UNIFORM_01, a new pseudorandom variate, 
//    strictly between 0 and 1.
//
//    int &SEED: an updated seed.
//
{
  const int i4_huge = 2147483647;
  int k;
  double r;

  if ( seed == 0 )
  {
    cerr << "\n";
    cerr << "R8_UNIFORM_01 - Fatal error!\n";
    cerr << "  Input value of SEED = 0.\n";
    exit ( 1 );
  }

  k = seed / 127773;

  seed = 16807 * ( seed - k * 127773 ) - k * 2836;

  if ( seed < 0 )
  {
    seed = seed + i4_huge;
  }
  r = ( double ) ( seed ) * 4.656612875E-10;

  return r;
}
//****************************************************************************80

double r8_uniform_ab ( double a, double b, int &seed )

//****************************************************************************80
//
//  Purpose:
//
//    r8_uniform_ab() returns a scaled pseudorandom R8.
//
//  Discussion:
//
//    The pseudorandom number should be uniformly distributed
//    between A and B.
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    09 April 2012
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double A, B, the limits of the interval.
//
//    int &SEED, the "seed" value, which should NOT be 0.
//
//  Output:
//
//    double R8_UNIFORM_AB, a number strictly between A and B.
//
//    int &SEED: an updated seed.
//
{
  const int i4_huge = 2147483647;
  int k;
  double value;

  if ( seed == 0 )
  {
    cerr << "\n";
    cerr << "R8_UNIFORM_AB - Fatal error!\n";
    cerr << "  Input value of SEED = 0.\n";
    exit ( 1 );
  }

  k = seed / 127773;

  seed = 16807 * ( seed - k * 127773 ) - k * 2836;

  if ( seed < 0 )
  {
    seed = seed + i4_huge;
  }

  value = ( double ) ( seed ) * 4.656612875E-10;

  value = a + ( b - a ) * value;

  return value;
}
//****************************************************************************80

void r8_unswap3 ( double &x, double &y, double &z )

//****************************************************************************80
//
//  Purpose:
//
//    r8_unswap3() unswaps three R8's.
//
//  Example:
//
//    Input:
//
//      X = 2, Y = 3, Z = 1
//
//    Output:
//
//      X = 1, Y = 2, Z = 3
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    17 April 2004
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double &X, &Y, &Z, three values to be swapped.
//
//  Output:
//
//    double &X, &Y, &Z: the swapped values.
{
  double w;

  w = z;
  z = y;
  y = x;
  x = w;

  return;
}
//****************************************************************************80

double r8_walsh_1d ( double x, int digit )

//****************************************************************************80
//
//  Purpose:
//
//    r8_walsh_1d() evaluates the Walsh function of a real scalar argument.
//
//  Discussion:
//
//    Consider the binary representation of X, and number the digits
//    in descending order, from leading to lowest, with the units digit
//    being numbered 0.
//
//    The Walsh function W(J)(X) is equal to the J-th binary digit of X.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    17 April 2004
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double X, the argument of the Walsh function.
//
//    int DIGIT, the index of the Walsh function.
//
//  Output:
//
//    double R8_WALSH_1D, the value of the Walsh function.
//
{
  int n;
  double value;
//
//  Hide the effect of the sign of X.
//
  x = fabs ( x );
//
//  If DIGIT is positive, divide by 2 DIGIT times.
//  If DIGIT is negative, multiply by 2 (-DIGIT) times.
//
  x = x / pow ( 2.0, digit );
//
//  Make it an integer.
//  Because it's positive, and we're using INT, we don't change the
//  units digit.
//
  n = ( int ) x;
//
//  Is the units digit odd or even?
//
  if ( ( n % 2 ) == 0 )
  {
    value = 0.0;
  }
  else
  {
    value = 1.0;
  }

  return value;
}
//****************************************************************************80

double r8_wrap ( double r, double rlo, double rhi )

//****************************************************************************80
//
//  Purpose:
//
//    r8_wrap() forces an R8 to lie between given limits by wrapping.
//
//  Discussion:
//
//    An R8 is a double value.
//
//  Example:
//
//    RLO = 4.0, RHI = 8.0
//
//     R  Value
//
//    -2     8
//    -1     4
//     0     5
//     1     6
//     2     7
//     3     8
//     4     4
//     5     5
//     6     6
//     7     7
//     8     8
//     9     4
//    10     5
//    11     6
//    12     7
//    13     8
//    14     4
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    24 June 2020
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double R, a value.
//
//    double RLO, RHI, the desired bounds.
//
//  Output:
//
//    double R8_WRAP, a "wrapped" version of the value.
//
{
  double value;

  if ( r < rlo )
  {
    value = rhi - fmod ( rlo - r, rhi - rlo );
  }
  else
  {
    value = rlo + fmod ( r - rlo, rhi - rlo );
  }

  return value;
}
//****************************************************************************80

double r82_dist_l2 ( double a1[2], double a2[2] )

//****************************************************************************80
//
//  Purpose:
//
//    r82_dist_l2() returns the L2 distance between a pair of R82's.
//
//  Discussion:
//
//    An R82 is a vector of type R8, with two entries.
//
//    The vector L2 norm is defined as:
//
//      sqrt ( sum ( 1 <= I <= N ) A(I) * A(I) ).
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    12 November 2011
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double A1[2], A2[2], the vectors.
//
//  Output:
//
//    double R82_DIST_L2, the L2 norm of A1 - A2.
//
{
  double value;

  value = sqrt ( pow ( a1[0] - a2[0], 2 )
               + pow ( a1[1] - a2[1], 2 ) );

  return value;
}
//****************************************************************************80

void r82_print ( double a[2], string title )

//****************************************************************************80
//
//  Purpose:
//
//    r82_print() prints an R82.
//
//  Discussion:
//
//    An R82 is an R8VEC with two entries.
//
//    A format is used which suggests a coordinate pair:
//
//  Example:
//
//    Center : ( 1.23, 7.45 )
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    30 July 2009
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double A[2], the coordinates of the vector.
//
//    string TITLE, a title.
//
{
  cout << "  " << title << " : ";
  cout << ": ( " << setw(12) << a[0]
       << ", "   << setw(12) << a[1] << " )\n";

  return;
}
//****************************************************************************80

void r82_uniform_ab ( double b, double c, int &seed, double r[] )

//****************************************************************************80
//
//  Purpose:
//
//    r82_uniform_ab() returns a random R82 value in a given range.
//
//  Discussion:
//
//    An R82 is an R8VEC with two entries.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    09 September 2003
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double B, C, the minimum and maximum values.
//
//    int &SEED, a seed for the random number generator.
//
//  Output:
//
//    double R[2], the randomly chosen value.
//
//    int &SEED: an updated seed.
//
{
  int i;

  for ( i = 0; i < 2; i++ )
  {
    r[i] = r8_uniform_ab ( b, c, seed );
  }

  return;
}
//****************************************************************************80

void r82col_print_part ( int n, double a[], int max_print, string title )

//****************************************************************************80
//
//  Purpose:
//
//    r82col_print_part() prints "part" of an R82COL.
//
//  Discussion:
//
//    An R82COL is an (N,2) array of R8's.
//
//    The user specifies MAX_PRINT, the maximum number of lines to print.
//
//    If N, the size of the vector, is no more than MAX_PRINT, then
//    the entire vector is printed, one entry per line.
//
//    Otherwise, if possible, the first MAX_PRINT-2 entries are printed,
//    followed by a line of periods suggesting an omission,
//    and the last entry.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    10 April 2015
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of entries of the vector.
//
//    double A[N*2], the vector to be printed.
//
//    int MAX_PRINT, the maximum number of lines
//    to print.
//
//    string TITLE, a title.
//
{
  int i;

  if ( max_print <= 0 )
  {
    return;
  }

  if ( n <= 0 )
  {
    return;
  }

  cout << "\n";
  cout << title << "\n";
  cout << "\n";

  if ( n <= max_print )
  {
    for ( i = 0; i < n; i++ )
    {
      cout << "  " << setw(8) << i
           << "  " << setw(14) << a[i+0*n]
           << "  " << setw(14) << a[i+1*n] << "\n";
    }
  }
  else if ( 3 <= max_print )
  {
    for ( i = 0; i < max_print - 2; i++ )
    {
      cout << "  " << setw(8) << i
           << ": " << setw(14) << a[i+0*n]
           << "  " << setw(14) << a[i+1*n]  << "\n";
    }
    cout << "  ........  ..............  ..............\n";
    i = n - 1;
    cout << "  " << setw(8) << i
         << ": " << setw(14) << a[i+0*n]
         << "  " << setw(14) << a[i+1*n]  << "\n";
  }
  else
  {
    for ( i = 0; i < max_print - 1; i++ )
    {
      cout << "  " << setw(8) << i
           << ": " << setw(14) << a[i+0*n]
           << "  " << setw(14) << a[i+1*n]  << "\n";
    }
    i = max_print - 1;
    cout << "  " << setw(8) << i
         << ": " << setw(14) << a[i+0*n]
         << "  " << setw(14) << a[i+1*n] 
         << "  " << "...more entries...\n";
  }

  return;
}
//****************************************************************************80

double *r82row_max ( int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r82row_max() returns the maximum value in an R82ROW.
//
//  Discussion:
//
//    An R82ROW is a (2,N) array of R8's.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    17 July 2006
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of entries in the array.
//
//    double A[2*N], the array.
//
//  Output:
//
//    double R82ROW_MAX[2]; the largest entries in each row.
//
{
# define DIM_NUM 2

  double *amax = NULL;
  int i;
  int j;

  if ( n <= 0 )
  {
    return NULL;
  }

  amax = new double[DIM_NUM];

  for ( i = 0; i < DIM_NUM; i++ )
  {
    amax[i] = a[i+0*DIM_NUM];
    for ( j = 1; j < n; j++ )
    {
      if ( amax[i] < a[0+j*DIM_NUM] )
      {
        amax[i] = a[0+j*DIM_NUM];
      }
    }
  }
  return amax;
# undef DIM_NUM
}
//****************************************************************************80

double *r82row_min ( int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r82row_min() returns the minimum value in an R82ROW.
//
//  Discussion:
//
//    An R82ROW is a (2,N) array of R8's.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    17 July 2006
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of entries in the array.
//
//    double A[2*N], the array.
//
//  Output:
//
//    double R82ROW_MIN[2]; the smallest entries in each row.
//
{
# define DIM_NUM 2

  double *amin = NULL;
  int i;
  int j;

  if ( n <= 0 )
  {
    return NULL;
  }

  amin = new double[DIM_NUM];

  for ( i = 0; i < DIM_NUM; i++ )
  {
    amin[i] = a[i+0*DIM_NUM];
    for ( j = 1; j < n; j++ )
    {
      if ( a[0+j*DIM_NUM] < amin[i] )
      {
        amin[i] = a[0+j*DIM_NUM];
      }
    }
  }
  return amin;
# undef DIM_NUM
}
//****************************************************************************80

int r82row_order_type ( int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r82row_order_type() finds if an R82ROW is (non)strictly ascending/descending.
//
//  Discussion:
//
//    An R82ROW is a (2,N) array of R8's.
//
//    The dictionary or lexicographic ordering is used.
//
//    (X1,Y1) < (X2,Y2)  <=>  X1 < X2 or ( X1 = X2 and Y1 < Y2).
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    13 September 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of entries of the array.
//
//    double A[2*N], the array to be checked.
//
//  Output:
//
//    int R82ROW_ORDER_TYPE, order indicator:
//    -1, no discernable order;
//    0, all entries are equal;
//    1, ascending order;
//    2, strictly ascending order;
//    3, descending order;
//    4, strictly descending order.
//
{
  int i;
  int order;
//
//  Search for the first value not equal to A(1,1).
//
  i = 0;

  for ( ; ; )
  {
    i = i + 1;

    if ( n <= i )
    {
      order = 0;
      return order;
    }

    if ( a[0+0*2] < a[0+i*2] || ( a[0+0*2] == a[0+i*2] && a[1+0*2] < a[1+i*2] ) )
    {
      if ( i == 2 )
      {
        order = 2;
      }
      else
      {
        order = 1;
      }
      break;
    }
    else if ( a[0+i*2] < a[0+0*2] || 
      ( a[0+i*2] == a[0+0*2] && a[1+i*2] < a[1+0*2] ) )
    {
      if ( i == 2 )
      {
        order = 4;
      }
      else
      {
        order = 3;
      }
      break;
    }
  }
//
//  Now we have a "direction".  Examine subsequent entries.
//
  for ( ; ; )
  {
    i = i + 1;
    if ( n <= i )
    {
      break;
    }

    if ( order == 1 )
    {
      if ( a[0+i*2] < a[0+(i-1)*2] ||
        ( a[0+i*2] == a[0+(i-1)*2] && a[1+i*2] < a[1+(i-1)*2] ) )
      {
        order = -1;
        break;
      }
    }
    else if ( order == 2 )
    {
      if ( a[0+i*2] < a[0+(i-1)*2] ||
        ( a[0+i*2] == a[0+(i-1)*2] && a[1+i*2] < a[1+(i-1)*2] ) )
      {
        order = -1;
        break;
      }
      else if ( a[0+i*2] == a[0+(i-1)*2] && a[1+i*2] == a[1+(i-1)*2] )
      {
        order = 1;
      }
    }
    else if ( order == 3 )
    {
      if ( a[0+(i-1)*2] < a[0+i*2] ||
        ( a[0+(i-1)*2] == a[0+i*2] && a[1+(i-1)*2] < a[1+i*2] ) )
      {
        order = -1;
        break;
      }
    }
    else if ( order == 4 )
    {
      if ( a[0+(i-1)*2] < a[0+i*2] ||
        ( a[0+(i-1)*2] == a[0+i*2] && a[1+(i-1)*2] < a[1+i*2] ) )
      {
        order = -1;
        break;
      }
      else if ( a[0+i*2] == a[0+(i-1)*2] && a[1+i*2] == a[1+(i-1)*2] )
      {
        order = 3;
      }
    }
  }
  return order;
}
//****************************************************************************80

void r82row_part_quick_a ( int n, double a[], int &l, int &r )

//****************************************************************************80
//
//  Purpose:
//
//    r82row_part_quick_a() reorders an R82ROW as part of a quick sort.
//
//  Discussion:
//
//    An R82ROW is a (2,N) array of R8's.
//
//    The routine reorders the entries of A.  Using A(1:2,1) as a
//    key, all entries of A that are less than or equal to the key will
//    precede the key, which precedes all entries that are greater than the key.
//
//  Example:
//
//    Input:
//
//      N = 8
//
//      A = ( (2,4), (8,8), (6,2), (0,2), (10,6), (10,0), (0,6), (4,8) )
//
//    Output:
//
//      L = 2, R = 4
//
//      A = ( (0,2), (0,6), (2,4), (8,8), (6,2), (10,6), (10,0), (4,8) )
//             -----------          ----------------------------------
//             LEFT          KEY    RIGHT
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    01 September 2003
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of entries of A.
//
//    double A[N*2]: the array to be checked.
//
//  Output:
//
//    double A[N*2]: A has been reordered as described above.
//
//    int &L, &R, the indices of A that define the three segments.
//    Let KEY = the input value of A(1:2,1).  Then
//    I <= L                 A(1:2,I) < KEY;
//         L < I < R         A(1:2,I) = KEY;
//                 R <= I    A(1:2,I) > KEY.
//
{
  int i;
  int j;
  double key[2];
  int ll;
  int m;
  int rr;
//
  if ( n < 1 )
  {
    cerr << "\n";
    cerr << "R82ROW_PART_QUICK_A - Fatal error!\n";
    cerr << "  N < 1.\n";
    exit ( 1 );
  }

  if ( n == 1 )
  {
    l = 0;
    r = 2;
    return;
  }

  key[0] = a[2*0+0];
  key[1] = a[2*0+1];
  m = 1;
//
//  The elements of unknown size have indices between L+1 and R-1.
//
  ll = 1;
  rr = n + 1;

  for ( i = 2; i <= n; i++ )
  {
    if ( r8vec_gt ( 2, a+2*ll, key ) )
    {
      rr = rr - 1;
      r8vec_swap ( 2, a+2*(rr-1), a+2*ll );
    }
    else if ( r8vec_eq ( 2, a+2*ll, key ) )
    {
      m = m + 1;
      r8vec_swap ( 2, a+2*(m-1), a+2*ll );
      ll = ll + 1;
    }
    else if ( r8vec_lt ( 2, a+2*ll, key ) )
    {
      ll = ll + 1;
    }

  }
//
//  Now shift small elements to the left, and KEY elements to center.
//
  for ( i = 0; i < ll - m; i++ )
  {
    for ( j = 0; j < 2; j++ )
    {
      a[2*i+j] = a[2*(i+m)+j];
    }
  }

  ll = ll - m;

  for ( i = ll; i < ll+m; i++ )
  {
    for ( j = 0; j < 2; j++ )
    {
      a[2*i+j] = key[j];
    }
  }

  l = ll;
  r = rr;

  return;
}
//****************************************************************************80

void r82row_permute ( int n, int p[], double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r82row_permute() permutes an R82ROW in place.
//
//  Discussion:
//
//    An R82ROW is a (2,N) array of R8's.
//
//    This routine permutes an array of real "objects", but the same
//    logic can be used to permute an array of objects of any arithmetic
//    type, or an array of objects of any complexity.  The only temporary
//    storage required is enough to store a single object.  The number
//    of data movements made is N + the number of cycles of order 2 or more,
//    which is never more than N + N/2.
//
//  Example:
//
//    Input:
//
//      N = 5
//      P = (   2,    4,    5,    1,    3 )
//      A = ( 1.0,  2.0,  3.0,  4.0,  5.0 )
//          (11.0, 22.0, 33.0, 44.0, 55.0 )
//
//    Output:
//
//      A    = (  2.0,  4.0,  5.0,  1.0,  3.0 )
//             ( 22.0, 44.0, 55.0, 11.0, 33.0 ).
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    30 October 2008
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of objects.
//
//    int P[N], the permutation.  P(I) = J means
//    that the I-th element of the output array should be the J-th
//    element of the input array.
//
//    double A[2*N], the array to be permuted.
//
//  Output:
//
//    double A[2*N]: the permuted array.
{
  double a_temp[2];
  int i;
  int iget;
  int iput;
  int istart;

  if ( !perm0_check ( n, p ) )
  {
    cerr << "\n";
    cerr << "R82ROW_PERMUTE - Fatal error!\n";
    cerr << "  PERM0_CHECK rejects permutation.\n";
    exit ( 1 );
  }
//
//  In order for the sign negation trick to work, we need to assume that the
//  entries of P are strictly positive.  Presumably, the lowest number is 0.
//  So temporarily add 1 to each entry to force positivity.
//
  for ( i = 0; i < n; i++ )
  {
    p[i] = p[i] + 1;
  }
//
//  Search for the next element of the permutation that has not been used.
//
  for ( istart = 1; istart <= n; istart++ )
  {
    if ( p[istart-1] < 0 )
    {
      continue;
    }
    else if ( p[istart-1] == istart )
    {
      p[istart-1] = - p[istart-1];
      continue;
    }
    else
    {
      a_temp[0] = a[0+(istart-1)*2];
      a_temp[1] = a[1+(istart-1)*2];
      iget = istart;
//
//  Copy the new value into the vacated entry.
//
      for ( ; ; )
      {
        iput = iget;
        iget = p[iget-1];

        p[iput-1] = - p[iput-1];

        if ( iget < 1 || n < iget )
        {
          cerr << "\n";
          cerr << "R82ROW_PERMUTE - Fatal error!\n";
          cerr << "  Entry IPUT = " << iput << " of the permutation has\n";
          cerr << "  an illegal value IGET = " << iget << ".\n";
          exit ( 1 );
        }

        if ( iget == istart )
        {
          a[0+(iput-1)*2] = a_temp[0];
          a[1+(iput-1)*2] = a_temp[1];
          break;
        }
        a[0+(iput-1)*2] = a[0+(iget-1)*2];
        a[1+(iput-1)*2] = a[1+(iget-1)*2];
      }
    }
  }
//
//  Restore the signs of the entries.
//
  for ( i = 0; i < n; i++ )
  {
    p[i] = - p[i];
  }
//
//  Restore the entries.
//
  for ( i = 0; i < n; i++ )
  {
    p[i] = p[i] - 1;
  }
  return;
}
//****************************************************************************80

void r82row_print ( int n, double a[], string title )

//****************************************************************************80
//
//  Purpose:
//
//    r82row_print() prints an R82ROW.
//
//  Discussion:
//
//    An R82ROW is a (2,N) array of R8's.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    14 November 2003
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of components of the vector.
//
//    double A[2*N], the vector to be printed.
//
//    string TITLE, a title.
//
{
  int j;

  cout << "\n";
  cout << title << "\n";
  cout << "\n";
  for ( j = 0; j < n; j++ )
  {
    cout << "  " << setw(8)  << j
         << ": " << setw(14) << a[0+j*2]
         << "  " << setw(14) << a[1+j*2] << "\n";
  }

  return;
}
//****************************************************************************80

void r82row_print_part ( int n, double a[], int max_print, string title )

//****************************************************************************80
//
//  Purpose:
//
//    r82row_print_part() prints "part" of an R82ROW.
//
//  Discussion:
//
//    An R82ROW is a (2,N) array of R8's.
//
//    The user specifies MAX_PRINT, the maximum number of lines to print.
//
//    If N, the size of the vector, is no more than MAX_PRINT, then
//    the entire vector is printed, one entry per line.
//
//    Otherwise, if possible, the first MAX_PRINT-2 entries are printed,
//    followed by a line of periods suggesting an omission,
//    and the last entry.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    09 November 2011
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of entries of the vector.
//
//    double A[2*N], the vector to be printed.
//
//    int MAX_PRINT, the maximum number of lines
//    to print.
//
//    string TITLE, a title.
//
{
  int i;

  if ( max_print <= 0 )
  {
    return;
  }

  if ( n <= 0 )
  {
    return;
  }

  cout << "\n";
  cout << title << "\n";
  cout << "\n";

  if ( n <= max_print )
  {
    for ( i = 0; i < n; i++ )
    {
      cout << "  " << setw(8) << i
           << "  " << setw(14) << a[0+i*2]
           << "  " << setw(14) << a[1+i*2] << "\n";
    }
  }
  else if ( 3 <= max_print )
  {
    for ( i = 0; i < max_print - 2; i++ )
    {
      cout << "  " << setw(8) << i
           << ": " << setw(14) << a[0+i*2]
           << "  " << setw(14) << a[1+i*2]  << "\n";
    }
    cout << "  ........  ..............  ..............\n";
    i = n - 1;
    cout << "  " << setw(8) << i
         << ": " << setw(14) << a[0+i*2]
         << "  " << setw(14) << a[1+i*2]  << "\n";
  }
  else
  {
    for ( i = 0; i < max_print - 1; i++ )
    {
      cout << "  " << setw(8) << i
           << ": " << setw(14) << a[0+i*2]
           << "  " << setw(14) << a[1+i*2]  << "\n";
    }
    i = max_print - 1;
    cout << "  " << setw(8) << i
         << ": " << setw(14) << a[0+i*2]
         << "  " << setw(14) << a[1+i*2] 
         << "  " << "...more entries...\n";
  }

  return;
}
//****************************************************************************80

int *r82row_sort_heap_index_a ( int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r82row_sort_heap_index_a() does an indexed heap ascending sort of an R82ROW.
//
//  Discussion:
//
//    An R82ROW is a (2,N) array of R8's.
//
//    The sorting is not actually carried out.  Rather an index array is
//    created which defines the sorting.  This array may be used to sort
//    or index the array, or to sort or index related arrays keyed on the
//    original array.
//
//    Once the index array is computed, the sorting can be carried out
//    "implicitly:
//
//      a(*,indx(*))
//
//    or explicitly, by the call
//
//      r82row_permute ( n, indx, a )
//
//    after which a(*,*) is sorted.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    03 June 2009
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of entries in the array.
//
//    double A[2*N], an array to be index-sorted.
//
//  Output:
//
//    int R82ROW_SORT_HEAP_INDEX_A[N], the sort index.  The
//    I-th element of the sorted array is A(0:1,R82ROW_SORT_HEAP_INDEX_A(I)).
//
{
  double aval[2];
  int i;
  int *indx;
  int indxt;
  int ir;
  int j;
  int l;

  if ( n < 1 )
  {
    return NULL;
  }

  indx = new int[n];

  for ( i = 0; i < n; i++ )
  {
    indx[i] = i;
  }

  if ( n == 1 )
  {
    indx[0] = indx[0];
    return indx;
  }

  l = n / 2 + 1;
  ir = n;

  for ( ; ; )
  {
    if ( 1 < l )
    {
      l = l - 1;
      indxt = indx[l-1];
      aval[0] = a[0+indxt*2];
      aval[1] = a[1+indxt*2];
    }
    else
    {
      indxt = indx[ir-1];
      aval[0] = a[0+indxt*2];
      aval[1] = a[1+indxt*2];
      indx[ir-1] = indx[0];
      ir = ir - 1;

      if ( ir == 1 )
      {
        indx[0] = indxt;
        break;
      }
    }
    i = l;
    j = l + l;

    while ( j <= ir )
    {
      if ( j < ir )
      {
        if (   a[0+indx[j-1]*2] <  a[0+indx[j]*2] ||
             ( a[0+indx[j-1]*2] == a[0+indx[j]*2] &&
               a[1+indx[j-1]*2] <  a[1+indx[j]*2] ) )
        {
          j = j + 1;
        }
      }

      if (   aval[0] <  a[0+indx[j-1]*2] ||
           ( aval[0] == a[0+indx[j-1]*2] &&
             aval[1] <  a[1+indx[j-1]*2] ) )
      {
        indx[i-1] = indx[j-1];
        i = j;
        j = j + j;
      }
      else
      {
        j = ir + 1;
      }
    }
    indx[i-1] = indxt;
  }

  return indx;
}
//****************************************************************************80

void r82row_sort_quick_a ( int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r82row_sort_quick_a() ascending sorts an R82ROW using quick sort.
//
//  Discussion:
//
//    An R82ROW is a (2,N) array of R8's.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    01 September 2003
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of entries in the array.
//
//    double A[2*N]: the array to be sorted.
//
//  Output:
//
//    double A[2*N]: the sorted array.
{
# define LEVEL_MAX 30

  int base;
  int l_segment;
  int level;
  int n_segment;
  int rsave[LEVEL_MAX];
  int r_segment;

  if ( n < 1 )
  {
    cerr << "\n";
    cerr << "R82ROW_SORT_QUICK_A - Fatal error!\n";
    cerr << "  N < 1.\n";
    exit ( 1 );
  }

  if ( n == 1 )
  {
    return;
  }

  level = 1;
  rsave[level-1] = n + 1;
  base = 1;
  n_segment = n;

  while ( 0 < n_segment )
  {
//
//  Partition the segment.
//
    r82row_part_quick_a ( n_segment, a+2*(base-1)+0, l_segment, r_segment );
//
//  If the left segment has more than one element, we need to partition it.
//
    if ( 1 < l_segment )
    {
      if ( LEVEL_MAX < level )
      {
        cerr << "\n";
        cerr<< "R82ROW_SORT_QUICK_A - Fatal error!\n";
        cerr << "  Exceeding recursion maximum of " << LEVEL_MAX << "\n";
        exit ( 1 );
      }

      level = level + 1;
      n_segment = l_segment;
      rsave[level-1] = r_segment + base - 1;
    }
//
//  The left segment and the middle segment are sorted.
//  Must the right segment be partitioned?
//
    else if ( r_segment < n_segment )
    {
      n_segment = n_segment + 1 - r_segment;
      base = base + r_segment - 1;
    }
//
//  Otherwise, we back up a level if there is an earlier one.
//
    else
    {
      for ( ; ; )
      {
        if ( level <= 1 )
        {
          n_segment = 0;
          break;
        }

        base = rsave[level-1];
        n_segment = rsave[level-2] - rsave[level-1];
        level = level - 1;

        if ( 0 < n_segment )
        {
          break;
        }
      }
    }
  }
  return;
# undef LEVEL_MAX
}
//****************************************************************************80

double r83_norm ( double x, double y, double z )

//****************************************************************************80
//
//  Purpose:
//
//    r83_norm() returns the Euclidean norm of an R83.
//
//  Discussion:
//
//    An R83 is a vector of 3 R8's.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    10 November 2011
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double X, Y, Z, the vector.
//
//  Output:
//
//    double R83_NORM, the norm of the vector.
//
{
  double value;

  value = sqrt ( x * x + y * y + z * z );

  return value;
}
//****************************************************************************80

void r83col_print_part ( int n, double a[], int max_print, string title )

//****************************************************************************80
//
//  Purpose:
//
//    r83col_print_part() prints "part" of an R83COL.
//
//  Discussion:
//
//    An R83COL is an (N,3) array of R8's.
//
//    The user specifies MAX_PRINT, the maximum number of lines to print.
//
//    If N, the size of the vector, is no more than MAX_PRINT, then
//    the entire vector is printed, one entry per line.
//
//    Otherwise, if possible, the first MAX_PRINT-2 entries are printed,
//    followed by a line of periods suggesting an omission,
//    and the last entry.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    12 April 2015
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of entries of the vector.
//
//    double A[N*3], the vector to be printed.
//
//    int MAX_PRINT, the maximum number of lines
//    to print.
//
//    string TITLE, a title.
//
{
  int i;

  if ( max_print <= 0 )
  {
    return;
  }

  if ( n <= 0 )
  {
    return;
  }

  cout << "\n";
  cout << title << "\n";
  cout << "\n";

  if ( n <= max_print )
  {
    for ( i = 0; i < n; i++ )
    {
      cout << "  " << setw(8) << i
           << "  " << setw(14) << a[i+0*n]
           << "  " << setw(14) << a[i+1*n] 
           << "  " << setw(14) << a[i+2*n] << "\n";
    }
  }
  else if ( 3 <= max_print )
  {
    for ( i = 0; i < max_print - 2; i++ )
    {
      cout << "  " << setw(8) << i
           << ": " << setw(14) << a[i+0*n]
           << "  " << setw(14) << a[i+1*n] 
           << "  " << setw(14) << a[i+2*n]  << "\n";
    }
    cout << "  ........  ..............  ..............  ..............\n";
    i = n - 1;
    cout << "  " << setw(8) << i
         << ": " << setw(14) << a[i+0*n]
         << "  " << setw(14) << a[i+1*n] 
         << "  " << setw(14) << a[i+2*n]  << "\n";
  }
  else
  {
    for ( i = 0; i < max_print - 1; i++ )
    {
      cout << "  " << setw(8) << i
           << ": " << setw(14) << a[i+0*n]
           << "  " << setw(14) << a[i+1*n] 
           << "  " << setw(14) << a[i+2*n]  << "\n";
    }
    i = max_print - 1;
    cout << "  " << setw(8) << i
         << ": " << setw(14) << a[i+0*n]
         << "  " << setw(14) << a[i+1*n] 
         << "  " << setw(14) << a[i+2*n] 
         << "  " << "...more entries...\n";
  }

  return;
}
//****************************************************************************80

double *r83row_max ( int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r83row_max() returns the maximum value in an R83ROW.
//
//  Discussion:
//
//    An R83ROW is a (3,N) array of R8's.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    04 January 2009
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of entries in the array.
//
//    double A[3*N], the array.
//
//  Output:
//
//    double R83ROW_MAX[3]; the largest entries in each row.
//
{
# define DIM_NUM 3

  double *amax = NULL;
  int i;
  int j;

  if ( n <= 0 )
  {
    return NULL;
  }

  amax = new double[DIM_NUM];

  for ( i = 0; i < DIM_NUM; i++ )
  {
    amax[i] = a[i+0*DIM_NUM];
    for ( j = 1; j < n; j++ )
    {
      if ( amax[i] < a[i+j*DIM_NUM] )
      {
        amax[i] = a[i+j*DIM_NUM];
      }
    }
  }
  return amax;
# undef DIM_NUM
}
//****************************************************************************80

double *r83row_min ( int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r83row_min() returns the minimum value in an R83ROW.
//
//  Discussion:
//
//    An R83ROW is a (3,N) array of R8's.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    04 January 2009
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of entries in the array.
//
//    double A[3*N], the array.
//
//  Output:
//
//    double R83ROW_MIN[3]; the smallest entries in each row.
//
{
# define DIM_NUM 3

  double *amin = NULL;
  int i;
  int j;

  if ( n <= 0 )
  {
    return NULL;
  }

  amin = new double[DIM_NUM];

  for ( i = 0; i < DIM_NUM; i++ )
  {
    amin[i] = a[i+0*DIM_NUM];
    for ( j = 1; j < n; j++ )
    {
      if ( a[i+j*DIM_NUM] < amin[i] )
      {
        amin[i] = a[i+j*DIM_NUM];
      }
    }
  }
  return amin;
# undef DIM_NUM
}
//****************************************************************************80

void r83row_part_quick_a ( int n, double a[], int &l, int &r )

//****************************************************************************80
//
//  Purpose:
//
//    r83row_part_quick_a() reorders an R83ROW as part of a quick sort.
//
//  Discussion:
//
//    An R83ROW is a (3,N) array of R8's.
//
//    The routine reorders the entries of A.  Using A(1:3,1) as a
//    key, all entries of A that are less than or equal to the key will
//    precede the key, which precedes all entries that are greater than the key.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    12 September 2003
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of entries of A.
//
//    double A[3*N]: the array to be checked.
//
//  Output:
//
//    double A[3*N]: reordered as described above.
//
//    int &L, &R, the indices of A that define the three segments.
//    Let KEY = the input value of A(1:3,1).  Then
//    I <= L                 A(1:3,I) < KEY;
//         L < I < R         A(1:3,I) = KEY;
//                 R <= I    A(1:3,I) > KEY.
//
{
  int i;
  int j;
  double key[3];
  int ll;
  int m;
  int rr;

  if ( n < 1 )
  {
    cerr << "\n";
    cerr << "R83ROW_PART_QUICK_A - Fatal error!\n";
    cerr << "  N < 1.\n";
    exit ( 1 );
  }

  if ( n == 1 )
  {
    l = 0;
    r = 2;
    return;
  }

  key[0] = a[3*0+0];
  key[1] = a[3*0+1];
  key[2] = a[3*0+2];
  m = 1;
//
//  The elements of unknown size have indices between L+1 and R-1.
//
  ll = 1;
  rr = n + 1;

  for ( i = 2; i <= n; i++ )
  {
    if ( r8vec_gt ( 3, a+3*ll, key ) )
    {
      rr = rr - 1;
      r8vec_swap ( 3, a+3*(rr-1), a+3*ll );
    }
    else if ( r8vec_eq ( 3, a+3*ll, key ) )
    {
      m = m + 1;
      r8vec_swap ( 3, a+3*(m-1), a+3*ll );
      ll = ll + 1;
    }
    else if ( r8vec_lt ( 3, a+3*ll, key ) )
    {
      ll = ll + 1;
    }
  }
//
//  Now shift small elements to the left, and KEY elements to center.
//
  for ( i = 0; i < ll - m; i++ )
  {
    for ( j = 0; j < 3; j++ )
    {
      a[3*i+j] = a[3*(i+m)+j];
    }
  }

  ll = ll - m;

  for ( i = ll; i < ll+m; i++ )
  {
    for ( j = 0; j < 3; j++ )
    {
      a[3*i+j] = key[j];
    }
  }

  l = ll;
  r = rr;

  return;
}
//****************************************************************************80

void r83row_print_part ( int n, double a[], int max_print, string title )

//****************************************************************************80
//
//  Purpose:
//
//    r83row_print_part() prints "part" of an R83ROW.
//
//  Discussion:
//
//    An R83ROW is a (3,N) array of R8's.
//
//    The user specifies MAX_PRINT, the maximum number of lines to print.
//
//    If N, the size of the vector, is no more than MAX_PRINT, then
//    the entire vector is printed, one entry per line.
//
//    Otherwise, if possible, the first MAX_PRINT-2 entries are printed,
//    followed by a line of periods suggesting an omission,
//    and the last entry.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    11 November 2011
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of entries of the vector.
//
//    double A[3*N], the vector to be printed.
//
//    int MAX_PRINT, the maximum number of lines
//    to print.
//
//    string TITLE, a title.
//
{
  int i;

  if ( max_print <= 0 )
  {
    return;
  }

  if ( n <= 0 )
  {
    return;
  }

  cout << "\n";
  cout << title << "\n";
  cout << "\n";

  if ( n <= max_print )
  {
    for ( i = 0; i < n; i++ )
    {
      cout << "  " << setw(8) << i
           << "  " << setw(14) << a[0+i*3]
           << "  " << setw(14) << a[1+i*3] 
           << "  " << setw(14) << a[2+i*3] << "\n";
    }
  }
  else if ( 3 <= max_print )
  {
    for ( i = 0; i < max_print - 2; i++ )
    {
      cout << "  " << setw(8) << i
           << ": " << setw(14) << a[0+i*3]
           << "  " << setw(14) << a[1+i*3] 
           << "  " << setw(14) << a[2+i*3]  << "\n";
    }
    cout << "  ........  ..............  ..............  ..............\n";
    i = n - 1;
    cout << "  " << setw(8) << i
         << ": " << setw(14) << a[0+i*3]
         << "  " << setw(14) << a[1+i*3] 
         << "  " << setw(14) << a[2+i*3]  << "\n";
  }
  else
  {
    for ( i = 0; i < max_print - 1; i++ )
    {
      cout << "  " << setw(8) << i
           << ": " << setw(14) << a[0+i*3]
           << "  " << setw(14) << a[1+i*3] 
           << "  " << setw(14) << a[2+i*3]  << "\n";
    }
    i = max_print - 1;
    cout << "  " << setw(8) << i
         << ": " << setw(14) << a[0+i*3]
         << "  " << setw(14) << a[1+i*3] 
         << "  " << setw(14) << a[2+i*3] 
         << "  " << "...more entries...\n";
  }

  return;
}
//****************************************************************************80

void r83row_sort_quick_a ( int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r83row_sort_quick_a() ascending sorts an R83ROW using quick sort.
//
//  Discussion:
//
//    An R83ROW is a (3,N) array of R8's.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    12 September 2003
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of entries in the array.
//
//    double A[3*N]: the array to be sorted.
//
//  Output:
//
//    double A[3*N]: the sorted array.
{
# define LEVEL_MAX 30

  int base;
  int l_segment;
  int level;
  int n_segment;
  int rsave[LEVEL_MAX];
  int r_segment;

  if ( n < 1 )
  {
    cerr << "\n";
    cerr << "R83ROW_SORT_QUICK_A - Fatal error!\n";
    cerr << "  N < 1.\n";
    exit ( 1 );
  }

  if ( n == 1 )
  {
    return;
  }

  level = 1;
  rsave[level-1] = n + 1;
  base = 1;
  n_segment = n;

  while ( 0 < n_segment )
  {
//
//  Partition the segment.
//
    r83row_part_quick_a ( n_segment, a+3*(base-1)+0, l_segment, r_segment );
//
//  If the left segment has more than one element, we need to partition it.
//
    if ( 1 < l_segment )
    {
      if ( LEVEL_MAX < level )
      {
        cerr << "\n";
        cerr << "R83ROW_SORT_QUICK_A - Fatal error!\n";
        cerr << "  Exceeding recursion maximum of " << LEVEL_MAX << "\n";
        exit ( 1 );
      }
      level = level + 1;
      n_segment = l_segment;
      rsave[level-1] = r_segment + base - 1;
    }
//
//  The left segment and the middle segment are sorted.
//  Must the right segment be partitioned?
//
    else if ( r_segment < n_segment )
    {
      n_segment = n_segment + 1 - r_segment;
      base = base + r_segment - 1;
    }
//
//  Otherwise, we back up a level if there is an earlier one.
//
    else
    {
      for ( ; ; )
      {
        if ( level <= 1 )
        {
          n_segment = 0;
          break;
        }

        base = rsave[level-1];
        n_segment = rsave[level-2] - rsave[level-1];
        level = level - 1;

        if ( 0 < n_segment )
        {
          break;
        }
      }
    }
  }
  return;
# undef LEVEL_MAX
}
//****************************************************************************80

void r8block_delete ( int l, int m, int n, double ***a )

//****************************************************************************80
//
//  Purpose:
//
//    r8block_delete() frees memory associated with an R8BLOCK.
//
//  Discussion:
//
//    This function releases the memory associated with an array that was 
//    created by a command like
//      double ***a;
//      a = r8block_new ( l, m, n );
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    01 March 2012
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int L, M, N, the number of rows, columns, and layers in the array.
//
//    double ***A, the pointer to the data.
//
{
  int i;
  int j;

  for ( i = 0; i < l; i++ )
  {
    for ( j = 0; j < m; j++ )
    {
      delete [] a[i][j];
    }
  }

  for ( i = 0; i < l; i++ )
  {
    delete [] a[i];
  }

  delete [] a;

  return;
}
//****************************************************************************80

double *r8block_expand_linear ( int l, int m, int n, double x[], int lfat,
  int mfat, int nfat )

//****************************************************************************80
//
//  Purpose:
//
//    r8block_expand_linear() linearly interpolates new data into a 3D block.
//
//  Discussion:
//
//    In this routine, the expansion is specified by giving the number
//    of intermediate values to generate between each pair of original
//    data rows and columns.
//
//    The interpolation is not actually linear.  It uses the functions
//
//      1, x, y, z, xy, xz, yz, xyz.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    19 October 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int L, M, N, the dimensions of the input data.
//
//    double X[L*M*N], the original data.
//
//    int LFAT, MFAT, NFAT, the number of data values to interpolate
//    original data values in the first, second and third dimensions.
//
//  Output:
//
//    double XFAT[L2*M2*N2], the fattened data, where
//    L2 = (L-1)*(LFAT+1)+1,
//    M2 = (M-1)*(MFAT+1)+1,
//    N2 = (N-1)*(NFAT+1)+1.
//
{
  int i;
  int ihi;
  int ii;
  int iii;
  int ip1;
  int j;
  int jhi;
  int jj;
  int jjj;
  int jp1;
  int k;
  int khi;
  int kk;
  int kkk;
  int kp1;
  int l2;
  int m2;
  int n2;
  double r;
  double s;
  double t;
  double x000;
  double x001;
  double x010;
  double x011;
  double x100;
  double x101;
  double x110;
  double x111;
  double *xfat;

  l2 = ( l - 1 ) * ( lfat + 1 ) + 1;
  m2 = ( m - 1 ) * ( mfat + 1 ) + 1;
  n2 = ( n - 1 ) * ( nfat + 1 ) + 1;

  xfat = new double[l2*m2*n2];

  for ( i = 1; i <= l; i++ )
  {
    if ( i < l )
    {
      ihi = lfat;
    }
    else
    {
      ihi = 0;
    }

    for ( j = 1; j <= m; j++ )
    {
      if ( j < m )
      {
        jhi = mfat;
      }
      else
      {
        jhi = 0;
      }

      for ( k = 1; k <= n; k++ )
      {
        if ( k < n )
        {
          khi = nfat;
        }
        else
        {
          khi = 0;
        }

        if ( i < l )
        {
          ip1 = i + 1;
        }
        else
        {
          ip1 = i;
        }

        if ( j < m )
        {
          jp1 = j + 1;
        }
        else
        {
          jp1 = j;
        }

        if ( k < n )
        {
          kp1 = k + 1;
        }
        else
        {
          kp1 = k;
        }

        x000 = x[i-1+(j-1)*l+(k-1)*l*m];
        x001 = x[i-1+(j-1)*l+(kp1-1)*l*m];
        x100 = x[ip1-1+(j-1)*l+(k-1)*l*m];
        x101 = x[ip1-1+(j-1)*l+(kp1-1)*l*m];
        x010 = x[i-1+(jp1-1)*l+(k-1)*l*m];
        x011 = x[i-1+(jp1-1)*l+(kp1-1)*l*m];
        x110 = x[ip1-1+(jp1-1)*l+(k-1)*l*m];
        x111 = x[ip1-1+(jp1-1)*l+(kp1-1)*l*m];

        for ( ii = 0; ii <= ihi; ii++ )
        {
          r = ( double ) ( ii ) / ( double ) ( ihi + 1 );

          for ( jj = 0; jj <= jhi; jj++ )
          {
            s = ( double ) ( jj ) / ( double ) ( jhi + 1 );

            for ( kk = 0; kk <= khi; kk++ )
            {
              t = ( double ) ( kk ) / ( double ) ( khi + 1 );

              iii = 1 + ( i - 1 ) * ( lfat + 1 ) + ii;
              jjj = 1 + ( j - 1 ) * ( mfat + 1 ) + jj;
              kkk = 1 + ( k - 1 ) * ( nfat + 1 ) + kk;

              xfat[iii-1+(jjj-1)*l2+(kkk-1)*l2*m2] =
                  x000 * ( 1.0 - r ) * ( 1.0 - s ) * ( 1.0 - t )
                + x001 * ( 1.0 - r ) * ( 1.0 - s ) * (       t )
                + x010 * ( 1.0 - r ) * (       s ) * ( 1.0 - t )
                + x011 * ( 1.0 - r ) * (       s ) * (       t )
                + x100 * (       r ) * ( 1.0 - s ) * ( 1.0 - t )
                + x101 * (       r ) * ( 1.0 - s ) * (       t )
                + x110 * (       r ) * (       s ) * ( 1.0 - t )
                + x111 * (       r ) * (       s ) * (       t );
            }
          }
        }
      }
    }
  }

  return xfat;
}
//****************************************************************************80

double ***r8block_new ( int l, int m, int n )

//****************************************************************************80
//
//  Purpose:
//
//    r8block_new() allocates a new R8BLOCK.
//
//  Discussion:
//
//    A declaration of the form
//      double ***a;
//    is necesary.  Then an assignment of the form:
//      a = r8block_new ( l, m, n );
//    allows the user to assign entries to the matrix using typical
//    3D array notation:
//      a[2][3][4] = 17.0;
//      y = a[1][0][3];
//    and so on.
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    01 March 2012
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int L, M, N, the number of rows, columns and layers.
//
//  Output:
//
//    double R8BLOCK_NEW[L][M][N], a new block.
//
{
  double ***a;
  int i;
  int j;

  a = new double **[l];

  if ( a == NULL )
  {
    cerr << "\n";
    cerr << "R8BLOCK_NEW - Fatal error!\n";
    cerr << "  Unable to allocate row pointer array.\n";
    exit ( 1 );
  }

  for ( i = 0; i < l; i++ )
  {
    a[i] = new double *[m];
    if ( a[i] == NULL )
    {
      cerr << "\n";
      cerr << "R8BLOCK_NEW - Fatal error!\n";
      cerr << "  Unable to allocate column pointer array.\n";
      exit ( 1 );
    }
  }

  for ( i = 0; i < l; i++ )
  {
    for ( j = 0; j < m; j++ )
    {
      a[i][j] = new double[n];
      if ( a[i][j] == NULL )
      {
        cerr << "\n";
        cerr << "R8BLOCK_NEW - Fatal error!\n";
        cerr << "  Unable to allocate layer array.\n";
        exit ( 1 );
      }
    }
  }
  return a;
}
//****************************************************************************80

void r8block_print ( int l, int m, int n, double a[], string title )

//****************************************************************************80
//
//  Purpose:
//
//    r8block_print() prints an R8BLOCK block (a 3D matrix).
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    13 October 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int L, M, N, the dimensions of the block.
//
//    double A[L*M*N], the matrix to be printed.
//
//    string TITLE, a title.
//
{
  int i;
  int j;
  int jhi;
  int jlo;
  int k;

  cout << "\n";
  cout << title << "\n";

  for ( k = 1; k <= n; k++ )
  {
    cout << "\n";
    cout << "  K = " << k << "\n";
    cout << "\n";
    for ( jlo = 1; jlo <= m; jlo = jlo + 5 )
    {
      jhi = min ( jlo + 4, m );
      cout << "\n";
      cout << "      ";
      for ( j = jlo; j <= jhi; j++ )
      {
        cout << setw(7) << j << "       ";
      }
      cout << "\n";
      cout << "\n";
      for ( i = 1; i <= l; i++ )
      {
        cout << setw(5) << i << ":";
        for ( j = jlo; j <= jhi; j++ )
        {
          cout << "  " << setw(12) << a[i-1+(j-1)*l+(k-1)*l*m];
        }
        cout << "\n";
      }
    }
  }

  return;
}
//****************************************************************************80

double *r8block_zeros_new ( int l, int m, int n )

//****************************************************************************80
//
//  Purpose:
//
//    r8block_zeros_new() returns a new zeroed R8BLOCK.
//
//  Discussion:
//
//    An R8BLOCK is a triple dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    13 April 2013
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int L, M, N, the number of rows and columns.
//
//  Output:
//
//    double R8BLOCK_ZEROS_NEW[L*M*N], the new zeroed matrix.
//
{
  double *a;
  int i;
  int j;
  int k;

  a = new double[l*m*n];

  for ( k = 0; k < n; k++ )
  {
    for ( j = 0; j < m; j++ )
    {
      for ( i = 0; i < l; i++ )
      {
        a[i+j*l+k*l*m] = 0.0;
      }
    }
  }
  return a;
}
//****************************************************************************80

void r8cmat_delete ( int m, int n, double **a )

//****************************************************************************80
//
//  Purpose:
//
//    r8cmat_delete() frees memory associated with an R8CMAT.
//
//  Discussion:
//
//    This function releases the memory associated with an R8CMAT.
//
//    An R8CMAT is a column-major array, storing element (I,J)
//    as A[J][I], and can be created by a command like:
//      double **a;
//      a = r8cmat_new ( m, n );
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    09 September 2013
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, N, the number of rows and columns in the array.
//
//    double **A, the pointer to the array.
//
{
  int j;

  for ( j = 0; j < n; j++ )
  {
    delete [] a[j];
  }

  delete [] a;

  return;
}
//****************************************************************************80

double **r8cmat_new ( int m, int n )

//****************************************************************************80
//
//  Purpose:
//
//    r8cmat_new() allocates a new R8CMAT.
//
//  Discussion:
//
//    An R8CMAT is a column-major array, storing element (I,J)
//    as A[J][I], and can be created by a command like:
//      double **a;
//      a = r8cmat_new ( m, n );
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    09 September 2013
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, N, the number of rows and columns in the matrix.
//
//  Output:
//
//    double **R8CMAT_NEW, a new matrix.
//
{
  double **a;
  int j;

  a = new double *[n];

  if ( a == NULL )
  {
    cerr << "\n";
    cerr << "R8CMAT_NEW - Fatal error!\n";
    cerr << "  Unable to allocate row pointer array.\n";
    exit ( 1 );
  }

  for ( j = 0; j < n; j++ )
  {
    a[j] = new double[m];
    if ( a[j] == NULL )
    {
      cerr << "\n";
      cerr << "R8CMAT_NEW - Fatal error!\n";
      cerr << "  Unable to allocate row array.\n";
      exit ( 1 );
    }
  }

  return a;
}
//****************************************************************************80

void r8cmat_print ( int m, int n, double **a, string title )

//****************************************************************************80
//
//  Purpose:
//
//    r8cmat_print() prints an R8CMAT.
//
//  Discussion:
//
//    An R8CMAT is a column-major array, storing element (I,J)
//    as A[J][I], and can be created by a command like:
//      double **a;
//      a = r8cmat_new ( m, n );
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    10 September 2009
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, the number of rows in A.
//
//    int N, the number of columns in A.
//
//    double **A = A[M][N], the M by N matrix.
//
//    string TITLE, a title.
//
{
  r8cmat_print_some ( m, n, a, 1, 1, m, n, title );

  return;
}
//****************************************************************************80

void r8cmat_print_some ( int m, int n, double **a, int ilo, int jlo, int ihi,
  int jhi, string title )

//****************************************************************************80
//
//  Purpose:
//
//    r8cmat_print_some() prints some of an R8CMAT.
//
//  Discussion:
//
//    An R8CMAT is a column-major array, storing element (I,J)
//    as A[J][I], and can be created by a command like:
//      double **a;
//      a = r8cmat_new ( m, n );
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    26 June 2013
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, the number of rows of the matrix.
//    M must be positive.
//
//    int N, the number of columns of the matrix.
//    N must be positive.
//
//    double **A = A[M][N], the matrix.
//
//    int ILO, JLO, IHI, JHI, designate the first row and
//    column, and the last row and column to be printed.
//
//    string TITLE, a title.
//
{
# define INCX 5

  int i;
  int i2hi;
  int i2lo;
  int j;
  int j2hi;
  int j2lo;

  cout << "\n";
  cout << title << "\n";

  if ( m <= 0 || n <= 0 )
  {
    cout << "\n";
    cout << "  (None)\n";
    return;
  }
//
//  Print the columns of the matrix, in strips of 5.
//
  for ( j2lo = jlo; j2lo <= jhi; j2lo = j2lo + INCX )
  {
    j2hi = j2lo + INCX - 1;
    if ( n < j2hi )
    {
      j2hi = n;
    }
    if ( jhi < j2hi )
    {
      j2hi = jhi;
    }
    cout << "\n";
//
//  For each column J in the current range...
//
//  Write the header.
//
    cout << "  Col:    ";
    for ( j = j2lo; j <= j2hi; j++ )
    {
      cout << setw(7) << j - 1 << "       ";
    }
    cout << "\n";
    cout << "  Row\n";
    cout << "\n";
//
//  Determine the range of the rows in this strip.
//
    if ( 1 < ilo )
    {
      i2lo = ilo;
    }
    else
    {
      i2lo = 1;
    }
    if ( ihi < m )
    {
      i2hi = ihi;
    }
    else
    {
      i2hi = m;
    }

    for ( i = i2lo; i <= i2hi; i++ )
    {
//
//  Print out (up to) 5 entries in row I, that lie in the current strip.
//
      cout << setw(5) << i - 1 << ": ";
      for ( j = j2lo; j <= j2hi; j++ )
      {
        cout << setw(12) << a[j-1][i-1] << "  ";
      }
      cout << "\n";
    }
  }

  return;
# undef INCX
}
//****************************************************************************80

double *r8cmat_to_r8mat_new ( int m, int n, double **a )

//****************************************************************************80
//
//  Purpose:
//
//    r8cmat_to_r8mat_new() copies data from an R8CMAT to an R8MAT.
//
//  Discussion:
//
//    An R8CMAT is a column-major array, storing element (I,J)
//    as A[J][I], and can be created by a command like:
//      double **a;
//      a = r8cmat_new ( m, n );
//
//    An R8MAT is a column-major array stored as a vector, so
//    that element (I,J) of the M by N array is stored in location
//    I+J*M.
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    07 January 2014
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, N, the number of rows and columns.
//
//    double **A = double A[M][N], the data, stored as an R8CMAT.
//
//  Output:
//
//    double R8CMAT_TO_R8MAT_NEW[M*N], the data, stored as an R8MAT.
//
{
  double *b;
  int i;
  int j;

  b = new double[m*n];

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      b[i+j*m] = a[j][i];
    }
  }

  return b;
}
//****************************************************************************80

double **r8cmat_zeros_new ( int m, int n )

//****************************************************************************80
//
//  Purpose:
//
//    r8cmat_zeros_new() allocates and zeros a new R8CMAT.
//
//  Discussion:
//
//    An R8CMAT is a column-major array, storing element (I,J)
//    as A[J][I], and can be created by a command like:
//      double **a;
//      a = r8cmat_new ( m, n );
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    09 September 2013
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, N, the number of rows and columns in the matrix.
//
//  Output:
//
//    double **R8CMAT_ZEROS_NEW, a new matrix.
//
{
  double **a;
  int i;
  int j;

  a = new double *[n];

  if ( a == NULL )
  {
    cerr << "\n";
    cerr << "R8CMAT_ZEROS_NEW - Fatal error!\n";
    cerr << "  Unable to allocate row pointer array.\n";
    exit ( 1 );
  }

  for ( j = 0; j < n; j++ )
  {
    a[j] = new double[m];
    if ( a[j] == NULL )
    {
      cerr << "\n";
      cerr << "R8CMAT_ZEROS_NEW - Fatal error!\n";
      cerr << "  Unable to allocate row array.\n";
      exit ( 1 );
    }
  }

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      a[j][i] = 0.0;
    }
  }
  return a;
}
//****************************************************************************80

double r8int_to_r8int ( double rmin, double rmax, double r, double r2min,
  double r2max )

//****************************************************************************80
//
//  Purpose:
//
//    r8int_to_r8int() maps one R8 interval to another.
//
//  Discussion:
//
//    The formula used is
//
//      R2 := R2MIN + ( R2MAX - R2MIN ) * ( R - RMIN ) / ( RMAX - RMIN )
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    01 January 2001
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double RMIN, RMAX, the first range.
//
//    double R, the number to be converted.
//
//    double R2MAX, R2MIN, the second range.
//
//  Output:
//
//    double R8INT_TO_R8INT, the corresponding value in
//    the range [R2MIN,R2MAX].
//
{
  double  r2;

  if ( rmax == rmin )
  {
    r2 = ( r2max + r2min ) / 2.0;
  }
  else
  {
    r2 = ( ( ( rmax - r        ) * r2min
           + (        r - rmin ) * r2max )
           / ( rmax     - rmin ) );
  }

  return r2;
}
//****************************************************************************80

int r8int_to_i4int ( double rmin, double rmax, double r, int imin, int imax )

//****************************************************************************80
//
//  Purpose:
//
//    r8int_to_i4int() maps an R8 interval to an integer interval.
//
//  Discussion:
//
//    The formula used is
//
//      I := IMIN + ( IMAX - IMIN ) * ( R - RMIN ) / ( RMAX - RMIN )
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    01 January 2001
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double RMIN, RMAX, the range.
//
//    double R, the number to be converted.
//
//    int IMAX, IMIN, the integer range.
//
//  Output:
//
//    int R8INT_TO_I4INT, the corresponding value in the range [IMIN,IMAX].
//
{
  int i;

  if ( rmax == rmin )
  {
    i = ( imax + imin ) / 2;
  }
  else
  {
    i = round (
      ( ( rmax - r        ) * ( double ) ( imin )
      + (        r - rmin ) * ( double ) ( imax ) )
      / ( rmax     - rmin ) );
  }

  return i;
}
//****************************************************************************80

void r8mat_add ( int m, int n, double alpha, double a[], double beta, 
  double b[], double c[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_add() computes C = alpha * A + beta * B for R8MAT's.
//
//  Discussion:
//
//    An R8MAT is an array of R8 values.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    09 November 2012
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, N, the number of rows and columns.
//
//    double ALPHA, the multiplier for A.
//
//    double A[M*N], the first matrix.
//
//    double BETA, the multiplier for A.
//
//    double B[M*N], the second matrix.
//
//  Output:
//
//    double C[M*N], the sum of alpha*A+beta*B.
//
{
  int i;
  int j;

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      c[i+j*m] = alpha * a[i+j*m] + beta * b[i+j*m];
    }
  }
  return;
}
//****************************************************************************80

double *r8mat_add_new ( int m, int n, double alpha, double a[], double beta, 
  double b[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_add_new() computes C = alpha * A + beta * B for R8MAT's.
//
//  Discussion:
//
//    An R8MAT is an array of R8 values.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    01 December 2011
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, N, the number of rows and columns.
//
//    double ALPHA, the multiplier for A.
//
//    double A[M*N], the first matrix.
//
//    double BETA, the multiplier for A.
//
//    double B[M*N], the second matrix.
//
//  Output:
//
//    double R8MAT_ADD_NEW[M*N], the sum of alpha*A+beta*B.
//
{
  double *c;
  int i;
  int j;

  c = new double[m*n];

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      c[i+j*m] = alpha * a[i+j*m] + beta * b[i+j*m];
    }
  }
  return c;
}
//****************************************************************************80

double r8mat_amax ( int m, int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_amax() returns the maximum absolute value entry of an R8MAT.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    21 April 2012
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, the number of rows in A.
//
//    int N, the number of columns in A.
//
//    double A[M*N], the M by N matrix.
//
//  Output:
//
//    double R8MAT_AMAX, the maximum absolute value entry of A.
//
{
  int i;
  int j;
  double value;

  value = fabs ( a[0+0*m] );

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      value = fmax ( value, fabs ( a[i+j*m] ) );
    }
  }
  return value;
}
//****************************************************************************80

double *r8mat_border_add ( int m, int n, double table[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_border_add() adds a "border" to an R8MAT.
//
//  Discussion:
//
//    We suppose the input data gives values of a quantity on nodes
//    in the interior of a 2D grid, and we wish to create a new table
//    with additional positions for the nodes that would be on the
//    border of the 2D grid.
//
//                  0 0 0 0 0 0
//      * * * *     0 * * * * 0
//      * * * * --> 0 * * * * 0
//      * * * *     0 * * * * 0
//                  0 0 0 0 0 0
//
//    The illustration suggests the situation in which a 3 by 4 array
//    is input, and a 5 by 6 array is to be output.
//
//    The old data is shifted to its correct positions in the new array.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    25 January 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, the spatial dimension.
//
//    int N, the number of points.
//
//    double TABLE[M*N], the table data.
//
//  Output:
//
//    double TABLE2[(M+2)*(N+2)], the augmented table data.
//
{
  int i;
  int j;
  double *table2;

  table2 = new double[(m+2)*(n+2)];

  for ( j = 0; j < n+2; j++ )
  {
    for ( i = 0; i < m+2; i++ )
    {
      if ( i == 0 || i == m+1 || j == 0 || j == n+1 )
      {
        table2[i+j*(m+2)] = 0.0;
      }
      else
      {
        table2[i+j*(m+2)] = table[(i-1)+(j-1)*m];
      }
    }
  }
  return table2;
}
//****************************************************************************80

double *r8mat_border_cut ( int m, int n, double table[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_border_cut() cuts the "border" of an R8MAT.
//
//  Discussion:
//
//    We suppose the input data gives values of a quantity on nodes
//    on a 2D grid, and we wish to create a new table corresponding only
//    to those nodes in the interior of the 2D grid.
//
//      0 0 0 0 0 0
//      0 * * * * 0    * * * *
//      0 * * * * 0 -> * * * *
//      0 * * * * 0    * * * *
//      0 0 0 0 0 0
//
//    The illustration suggests the situation in which a 5 by 6 array
//    is input, and a 3 by 4 array is to be output.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    25 January 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, the spatial dimension.
//
//    int N, the number of points.
//
//    double TABLE[M*N], the table data.
//
//  Output:
//
//    double TABLE2[(M-2)*(N-2)], the "interior" table data.
//
{
  int i;
  int j;
  double *table2;

  if ( m <= 2 || n <= 2 )
  {
    return NULL;
  }

  table2 = new double[(m-2)*(n-2)];

  for ( j = 0; j < n-2; j++ )
  {
    for ( i = 0; i < m-2; i++ )
    {
      table2[i+j*(m-2)] = table[(i+1)+(j+1)*m];
    }
  }

  return table2;
}
//****************************************************************************80

double *r8mat_cholesky_factor ( int n, double a[], int &flag )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_cholesky_factor() computes the Cholesky factor of a symmetric R8MAT.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    The matrix must be symmetric and positive semidefinite.
//
//    For a positive semidefinite symmetric matrix A, the Cholesky factorization
//    is a lower triangular matrix L such that:
//
//      A = L * L'
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    11 November 2012
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of rows and columns of the matrix A.
//
//    double A[N*N], the N by N matrix.
//
//  Output:
//
//    int &FLAG, an error flag.
//    0, no error occurred.
//    1, the matrix is not positive definite.
//    2, the matrix is not nonnegative definite.
//
//    double R8MAT_CHOLESKY_FACTOR[N*N], the N by N lower triangular
//    Cholesky factor.
//
{
  double *c;
  int i;
  int j;
  int k;
  double sum2;
  double tol;

  flag = 0;
  tol = sqrt ( DBL_EPSILON );

  c = r8mat_copy_new ( n, n, a );

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < j; i++ )
    {
      c[i+j*n] = 0.0;
    }
    for ( i = j; i < n; i++ )
    {
      sum2 = c[j+i*n];
      for ( k = 0; k < j; k++ )
      {
        sum2 = sum2 - c[j+k*n] * c[i+k*n];
      }
      if ( i == j )
      {
        if ( 0.0 < sum2 )
        {
          c[i+j*n] = sqrt ( sum2 );
        }
        else if ( sum2 < - tol )
        {
          flag = 2;
          cerr << "\n";
          cerr << "R8MAT_CHOLESKY_FACTOR - Fatal error!\n";
          cerr << "  Matrix is not nonnegative definite.\n";
          cerr << "  Diagonal I = " << i << "\n";
          cerr << "  SUM2 = " << sum2 << "\n";
          exit ( 1 );
        }
        else
        {
          flag = 1;
          c[i+j*n] = 0.0;
        }
      }
      else
      {

        if ( c[j+j*n] != 0.0 )
        {
          c[i+j*n] = sum2 / c[j+j*n];
        }
        else
        {
          c[i+j*n] = 0.0;
        }
      }
    }
  }

  return c;
}
//****************************************************************************80

double *r8mat_cholesky_factor_upper ( int n, double a[], int &flag )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_cholesky_factor_upper(): upper Cholesky factor of a symmetric R8MAT.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    The matrix must be symmetric and positive semidefinite.
//
//    For a positive semidefinite symmetric matrix A, the Cholesky factorization
//    is an upper triangular matrix R such that:
//
//      A = R' * R
//
//    Note that the usual Cholesky factor is a LOWER triangular matrix L
//    such that
//
//      A = L * L'
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    03 August 2013
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of rows and columns of the matrix A.
//
//    double A[N*N], the N by N matrix.
//
//  Output:
//
//    int &FLAG, an error flag.
//    0, no error occurred.
//    1, the matrix is not positive definite.  A NULL factor is returned.
//
//    double R8MAT_CHOLESKY_FACTOR[N*N], the N by N upper triangular
//    Cholesky factor.
//
{
  double *c;
  int i;
  int j;
  int k;
  double sum2;

  flag = 0;

  c = r8mat_copy_new ( n, n, a );

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < j; i++ )
    {
      c[j+i*n] = 0.0;
    }
    for ( i = j; i < n; i++ )
    {
      sum2 = c[i+j*n];
      for ( k = 0; k < j; k++ )
      {
        sum2 = sum2 - c[k+j*n] * c[k+i*n];
      }
      if ( i == j )
      {
        if ( sum2 <= 0.0 )
        {
          flag = 1;
          return NULL;
        }
        c[j+i*n] = sqrt ( sum2 );
      }
      else
      {
        if ( c[j+j*n] != 0.0 )
        {
          c[j+i*n] = sum2 / c[j+j*n];
        }
        else
        {
          c[j+i*n] = 0.0;
        }
      }
    }
  }

  return c;
}
//****************************************************************************80

void r8mat_cholesky_inverse ( int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_cholesky_inverse() computes the inverse of a symmetric matrix.
//
//  Discussion:
//
//    The matrix must be symmetric and positive semidefinite.
//
//    The upper triangular Cholesky factorization R is computed, so that:
//
//      A = R' * R
//
//    Then the inverse B is computed by
//
//      B = inv ( A ) = inv ( R ) * inv ( R' )
//
//    An R8MAT is an MxN array of R8's, stored by (I,J) -> [I+J*M].
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    22 October 2013
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of rows and columns of
//    the matrix A.
//
//    double A[N*N]: the matrix.
//
//  Output:
//
//    double A[N*N]: the inverse matrix.
{
  int i;
  int j;
  int k;
  double s;
  double t;

  for ( j = 0; j < n; j++ )
  {
    s = 0.0;

    for ( k = 0; k < j; k++ )
    {
      t = a[k+j*n];
      for ( i = 0; i < k; i++ )
      {
        t = t - a[i+k*n] * a[i+j*n];
      }
      t = t / a[k+k*n];
      a[k+j*n] = t;
      s = s + t * t;
    }

    s = a[j+j*n] - s;

    if ( s <= 0.0 )
    {
      cerr << "\n";
      cerr << "R8MAT_CHOLESKY_INVERSE - Fatal error!\n";
      cerr << "  The matrix is singular.\n";
      exit ( 1 );
    }

    a[j+j*n] = sqrt ( s );

    for ( i = j + 1; i < n; i++ )
    {
      a[i+j*n] = 0.0;
    }
  }
//
//  Compute inverse(R).
//
  for ( k = 0; k < n; k++ )
  {
    a[k+k*n] = 1.0 / a[k+k*n];
    for ( i = 0; i < k; i++ )
    {
      a[i+k*n] = - a[i+k*n] * a[k+k*n];
    }

    for ( j = k + 1; j < n; j++ )
    {
      t = a[k+j*n];
      a[k+j*n] = 0.0;
      for ( i = 0; i <= k; i++ )
      {
        a[i+j*n] = a[i+j*n] + t * a[i+k*n];
      }
    }
  }
//
//  Form inverse(R) * (inverse(R))'.
//
  for ( j = 0; j < n; j++ )
  {
    for ( k = 0; k < j; k++ )
    {
      t = a[k+j*n];
      for ( i = 0; i <= k; i++ )
      {
        a[i+k*n] = a[i+k*n] + t * a[i+j*n];
      }
    }
    t = a[j+j*n];
    for ( i = 0; i <= j; i++ )
    {
      a[i+j*n] = a[i+j*n] * t;
    }
  }
//
//  Use reflection.
//
  for ( i = 0; i < n; i++ )
  {
    for ( j = 0; j < i; j++ )
    {
      a[i+j*n] = a[j+i*n];
    }
  }

  return;
}
//****************************************************************************80

double *r8mat_cholesky_solve ( int n, double l[], double b[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_cholesky_solve() solves a Cholesky factored linear system A * x = b.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    07 October 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of rows and columns of the matrix A.
//
//    double L[N*N], the N by N Cholesky factor of the
//    system matrix A.
//
//    double B[N], the right hand side of the linear system.
//
//  Output:
//
//    double R8MAT_CHOLESKY_SOLVE[N], the solution of the linear system.
//
{
  double *x;
  double *y;
//
//  Solve L * y = b.
//
  y = r8mat_l_solve ( n, l, b );
//
//  Solve L' * x = y.
//
  x = r8mat_lt_solve ( n, l, y );

  delete [] y;

  return x;
}
//****************************************************************************80

double *r8mat_cholesky_solve_upper ( int n, double r[], double b[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_cholesky_solve_upper() solves Cholesky factored linear system A * x = b.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    21 October 2013
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of rows and columns of the matrix A.
//
//    double R[N*N], the N by N Cholesky factor of the
//    system matrix A.
//
//    double B[N], the right hand side of the linear system.
//
//  Output:
//
//    double R8MAT_CHOLESKY_SOLVE_UPPER[N], the solution of the linear system.
//
{
  double *x;
  double *y;
//
//  Solve U' * y = b.
//
  y = r8mat_ut_solve ( n, r, b );
//
//  Solve U * x = y.
//
  x = r8mat_u_solve ( n, r, y );

  delete [] y;

  return x;
}
//****************************************************************************80

void r8mat_copy ( int m, int n, double a1[], double a2[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_copy() copies one R8MAT to another.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    16 October 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, N, the number of rows and columns.
//
//    double A1[M*N], the matrix to be copied.
//
//  Output:
//
//    double A2[M*N], the copy of A1.
//
{
  int i;
  int j;

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      a2[i+j*m] = a1[i+j*m];
    }
  }
  return;
}
//****************************************************************************80

double *r8mat_copy_new ( int m, int n, double a1[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_copy_new() copies one R8MAT to a "new" R8MAT.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8's, which
//    may be stored as a vector in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    03 July 2008
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, N, the number of rows and columns.
//
//    double A1[M*N], the matrix to be copied.
//
//  Output:
//
//    double R8MAT_COPY_NEW[M*N], the copy of A1.
//
{
  double *a2;
  int i;
  int j;

  a2 = new double[m*n];

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      a2[i+j*m] = a1[i+j*m];
    }
  }
  return a2;
}
//****************************************************************************80

double *r8mat_covariance ( int m, int n, double x[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_covariance() computes the sample covariance of a set of vector data.
//
//  Discussion:
//
//    An R8MAT is an MxN array of R8's, stored by (I,J) -> [I+J*M].
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    26 June 2013
//
//  Author:
//
//    John Burkardt.
//
//  Input:
//
//    int M, the size of a single data vectors.
//
//    int N, the number of data vectors.
//    N should be greater than 1.
//
//    double X[M*N], an array of N data vectors, each
//    of length M.
//
//  Output:
//
//    double C[M*M], the covariance matrix for the data.
//
{
  double *c;
  int i;
  int j;
  int k;
  double *x_mean;

  c = new double[m*m];
  for ( j = 0; j < m; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      c[i+j*m] = 0.0;
    }
  }
//
//  Special case of N = 1.
//
  if ( n == 1 )
  {
    for ( i = 0; i < m; i++ )
    {
      c[i+i*m] = 1.0;
    }
    return c;
  }
//
//  Determine the sample means.
//
  x_mean = new double[m];
  for ( i = 0; i < m; i++ )
  {
    x_mean[i] = 0.0;
    for ( j = 0; j < n; j++ )
    {
      x_mean[i] = x_mean[i] + x[i+j*m];
    }
    x_mean[i] = x_mean[i] / ( double ) ( n );
  }
//
//  Determine the sample covariance.
//
  for ( j = 0; j < m; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      for ( k = 0; k < n; k++ )
      {
        c[i+j*m] = c[i+j*m] 
          + ( x[i+k*m] - x_mean[i] ) * ( x[j+k*m] - x_mean[j] );
      }
    }
  }

  for ( j = 0; j < m; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      c[i+j*m] = c[i+j*m] / ( double ) ( n - 1 );
    }
  }

  delete [] x_mean;

  return c;
}
//****************************************************************************80

double r8mat_det ( int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_det() computes the determinant of an R8MAT.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    08 October 2005
//
//  Author:
//
//    Original FORTRAN77 version by Helmut Spaeth
//    This version by John Burkardt
//
//  Reference:
//
//    Helmut Spaeth,
//    Cluster Analysis Algorithms
//    for Data Reduction and Classification of Objects,
//    Ellis Horwood, 1980, page 125-127.
//
//  Input:
//
//    int N, the order of the matrix.
//
//    double A[N*N], the matrix whose determinant is desired.
//
//  Output:
//
//    double R8MAT_DET, the determinant of the matrix.
//
{
  double *b;
  double det;
  int i;
  int j;
  int k;
  int kk;
  int m;
  double temp;

  b = new double[n*n];

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < n; i++ )
    {
      b[i+j*n] = a[i+j*n];
    }
  }

  det = 1.0;

  for ( k = 1; k <= n; k++ )
  {
    m = k;
    for ( kk = k+1; kk <= n; kk++ )
    {
      if ( fabs ( b[m-1+(k-1)*n] ) < fabs ( b[kk-1+(k-1)*n] ) )
      {
        m = kk;
      }
    }

    if ( m != k )
    {
      det = -det;

      temp = b[m-1+(k-1)*n];
      b[m-1+(k-1)*n] = b[k-1+(k-1)*n];
      b[k-1+(k-1)*n] = temp;
    }

    det = det * b[k-1+(k-1)*n];

    if ( b[k-1+(k-1)*n] != 0.0 )
    {
      for ( i = k+1; i <= n; i++ )
      {
        b[i-1+(k-1)*n] = -b[i-1+(k-1)*n] / b[k-1+(k-1)*n];
      }

      for ( j = k+1; j <= n; j++ )
      {
        if ( m != k )
        {
          temp = b[m-1+(j-1)*n];
          b[m-1+(j-1)*n] = b[k-1+(j-1)*n];
          b[k-1+(j-1)*n] = temp;
        }
        for ( i = k+1; i <= n; i++ )
        {
          b[i-1+(j-1)*n] = b[i-1+(j-1)*n] + b[i-1+(k-1)*n] * b[k-1+(j-1)*n];
        }
      }
    }
  }

  delete [] b;

  return det;
}
//****************************************************************************80

double r8mat_det_2d ( double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_det_2d() computes the determinant of a 2 by 2 R8MAT.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Discussion:
//
//    The determinant of a 2 by 2 matrix is
//
//      a11 * a22 - a12 * a21.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    10 September 2003
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double A[2*2], the matrix whose determinant is desired.
//
//  Output:
//
//    double R8MAT_DET_2D, the determinant of the matrix.
//
{
  double det;

  det = a[0+0*2] * a[1+1*2] - a[0+1*2] * a[1+0*2];

  return det;
}
//****************************************************************************80

double r8mat_det_3d ( double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_det_3d() computes the determinant of a 3 by 3 R8MAT.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    The determinant of a 3 by 3 matrix is
//
//        a11 * a22 * a33 - a11 * a23 * a32
//      + a12 * a23 * a31 - a12 * a21 * a33
//      + a13 * a21 * a32 - a13 * a22 * a31
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    10 September 2003
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double A[3*3], the matrix whose determinant is desired.
//
//  Output:
//
//    double R8MAT_DET_3D, the determinant of the matrix.
//
{
  double det;

  det =
      a[0+0*3] * ( a[1+1*3] * a[2+2*3] - a[1+2*3] * a[2+1*3] )
    + a[0+1*3] * ( a[1+2*3] * a[2+0*3] - a[1+0*3] * a[2+2*3] )
    + a[0+2*3] * ( a[1+0*3] * a[2+1*3] - a[1+1*3] * a[2+0*3] );

  return det;
}
//****************************************************************************80

double r8mat_det_4d ( double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_det_4d() computes the determinant of a 4 by 4 R8MAT.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    10 September 2003
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double A[4*4], the matrix whose determinant is desired.
//
//  Output:
//
//    double R8MAT_DET_4D, the determinant of the matrix.
//
{
  double det;

  det =
      a[0+0*4] * (
          a[1+1*4] * ( a[2+2*4] * a[3+3*4] - a[2+3*4] * a[3+2*4] )
        - a[1+2*4] * ( a[2+1*4] * a[3+3*4] - a[2+3*4] * a[3+1*4] )
        + a[1+3*4] * ( a[2+1*4] * a[3+2*4] - a[2+2*4] * a[3+1*4] ) )
    - a[0+1*4] * (
          a[1+0*4] * ( a[2+2*4] * a[3+3*4] - a[2+3*4] * a[3+2*4] )
        - a[1+2*4] * ( a[2+0*4] * a[3+3*4] - a[2+3*4] * a[3+0*4] )
        + a[1+3*4] * ( a[2+0*4] * a[3+2*4] - a[2+2*4] * a[3+0*4] ) )
    + a[0+2*4] * (
          a[1+0*4] * ( a[2+1*4] * a[3+3*4] - a[2+3*4] * a[3+1*4] )
        - a[1+1*4] * ( a[2+0*4] * a[3+3*4] - a[2+3*4] * a[3+0*4] )
        + a[1+3*4] * ( a[2+0*4] * a[3+1*4] - a[2+1*4] * a[3+0*4] ) )
    - a[0+3*4] * (
          a[1+0*4] * ( a[2+1*4] * a[3+2*4] - a[2+2*4] * a[3+1*4] )
        - a[1+1*4] * ( a[2+0*4] * a[3+2*4] - a[2+2*4] * a[3+0*4] )
        + a[1+2*4] * ( a[2+0*4] * a[3+1*4] - a[2+1*4] * a[3+0*4] ) );

  return det;
}
//****************************************************************************80

double r8mat_det_5d ( double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_det_5d() computes the determinant of a 5 by 5 R8MAT.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    10 September 2003
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double A[5*5], the matrix whose determinant is desired.
//
//  Output:
//
//    double R8MAT_DET_5D, the determinant of the matrix.
//
{
  double b[4*4];
  double det;
  int i;
  int inc;
  int j;
  int k;
  double sign;
//
//  Expand the determinant into the sum of the determinants of the
//  five 4 by 4 matrices created by dropping row 1, and column k.
//
  det = 0.0;
  sign = 1.0;

  for ( k = 0; k < 5; k++ )
  {
    for ( i = 0; i < 4; i++ )
    {
      for ( j = 0; j < 4; j++ )
      {
        if ( j < k )
        {
          inc = 0;
        }
        else
        {
          inc = 1;
        }
        b[i+j*4] = a[i+1+(j+inc)*5];
      }
    }

    det = det + sign * a[0+k*5] * r8mat_det_4d ( b );

    sign = - sign;
  }

  return det;
}
//****************************************************************************80

void r8mat_diag_add_scalar ( int n, double a[], double s )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_diag_add_scalar() adds a scalar to the diagonal of an R8MAT.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    07 October 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of rows and columns of the matrix.
//
//    double A[N*N], the N by N matrix to be modified.
//
//    double S, the value to be added to the diagonal.
//
//  Output:
//
//    double A[N*N]: the modified matrix.
{
  int i;

  for ( i = 0; i < n; i++ )
  {
    a[i+i*n] = a[i+i*n] + s;
  }

  return;
}
//****************************************************************************80

void r8mat_diag_add_vector ( int n, double a[], double v[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_diag_add_vector() adds a vector to the diagonal of an R8MAT.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    07 October 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of rows and columns of the matrix.
//
//    double A[N*N], the N by N matrix.
//
//    double V[N], the vector to be added to the diagonal of A.
//
//  Output:
//
//    double A[N*N]: the modified matrix.
{
  int i;

  for ( i = 0; i < n; i++ )
  {
    a[i+i*n] = a[i+i*n] + v[i];
  }

  return;
}
//****************************************************************************80

void r8mat_diag_get_vector ( int n, double a[], double v[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_diag_get_vector() gets the value of the diagonal of an R8MAT.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    15 July 2013
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of rows and columns of the matrix.
//
//    double A[N*N], the N by N matrix.
//
//  Output:
//
//    double V[N], the diagonal entries
//    of the matrix.
//
{
  int i;

  for ( i = 0; i < n; i++ )
  {
    v[i] = a[i+i*n];
  }

  return;
}
//****************************************************************************80

double *r8mat_diag_get_vector_new ( int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_diag_get_vector_new() gets the value of the diagonal of an R8MAT.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    15 July 2013
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of rows and columns of the matrix.
//
//    double A[N*N], the N by N matrix.
//
//  Output:
//
//    double R8MAT_DIAG_GET_VECTOR_NEW[N], the diagonal entries
//    of the matrix.
//
{
  int i;
  double *v;

  v = new double[n];

  for ( i = 0; i < n; i++ )
  {
    v[i] = a[i+i*n];
  }

  return v;
}
//****************************************************************************80

void r8mat_diag_set_scalar ( int n, double a[], double s )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_diag_set_scalar() sets the diagonal of an R8MAT to a scalar value.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    07 October 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of rows and columns of the matrix.
//
//    double A[N*N], the N by N matrix to be modified.
//
//    double S, the value to be assigned to the diagonal
//    of the matrix.
//
//  Output:
//
//    double A[N*N]: the modified matrix.
{
  int i;

  for ( i = 0; i < n; i++ )
  {
    a[i+i*n] = s;
  }

  return;
}
//****************************************************************************80

void r8mat_diag_set_vector ( int n, double a[], double v[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_diag_set_vector() sets the diagonal of an R8MAT to a vector.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    07 October 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of rows and columns of the matrix.
//
//    double A[N*N], the N by N matrix.
//
//    double V[N], the vector to be assigned to the diagonal of A.
//
//  Output:
//
//    double A[N*N}: the modified matrix.
//
{
  int i;

  for ( i = 0; i < n; i++ )
  {
    a[i+i*n] = v[i];
  }

  return;
}
//****************************************************************************80

double *r8mat_diagonal_new ( int n, double diag[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_diagonal_new() returns a diagonal matrix.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    31 July 2013
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the order of A.
//
//    double DIAG[N], the diagonal entries.
//
//  Output:
//
//    double R8MAT_DIAGONAL_NEW[N*N], the N by N identity matrix.
//
{
  double *a;
  int i;
  int j;

  a = new double[n*n];

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < n; i++ )
    {
      if ( i == j )
      {
        a[i+j*n] = diag[i];
      }
      else
      {
        a[i+j*n] = 0.0;
      }
    }
  }

  return a;
}
//****************************************************************************80

double r8mat_diff_frobenius ( int m, int n, double a[], double b[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_diff_frobenius() returns the Frobenius norm of the difference of R8MAT's.
//
//  Discussion: 							    
//
//    An R8MAT is a doubly dimensioned array of double precision values, which
//    may be stored as a vector in column-major order.
//
//    The Frobenius norm is defined as
//
//      R8MAT_NORM_FRO = sqrt (
//        sum ( 1 <= I <= M ) sum ( 1 <= j <= N ) A(I,J)^2 )
//
//    The matrix Frobenius norm is not derived from a vector norm, but
//    is compatible with the vector L2 norm, so that:
//
//      r8vec_norm_l2 ( A * x ) <= r8mat_norm_fro ( A ) * r8vec_norm_l2 ( x ).
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    14 September 2006
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, the number of rows in A.
//
//    int N, the number of columns in A.
//
//    double A[M*N], double B[M*N], the matrices for which we
//    want the Frobenius norm of the difference.
//
//  Output:
//
//    double R8MAT_DIFF_FROBENIUS, the Frobenius norm of ( A - B ).
//
{
  int i;
  int j;
  double value;

  value = 0.0;
  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      value = value + pow ( a[i+j*m] - b[i+j*m], 2 );
    }
  }
  value = sqrt ( value );

  return value;
}
//****************************************************************************80

double *r8mat_expand_linear ( int m, int n, double x[], int mfat, int nfat )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_expand_linear() linearly interpolates new data into an R8MAT.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    In this routine, the expansion is specified by giving the number
//    of intermediate values to generate between each pair of original
//    data rows and columns.
//
//    The interpolation is not actually linear.  It uses the functions
//
//      1, x, y, and xy.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    19 October 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, N, the number of rows and columns of input data.
//
//    double X[M*N], the original data.
//
//    int MFAT, NFAT, the number of data values to interpolate
//    between each row, and each column, of original data values.
//
//  Output:
//
//    double XFAT[M2*N2], the fattened data, where
//    M2 = (M-1)*(MFAT+1)+1,
//    N2 = (N-1)*(NFAT+1)+1.
//
{
  int i;
  int ihi;
  int ii;
  int iii;
  int ip1;
  int j;
  int jhi;
  int jj;
  int jjj;
  int jp1;
  int m2;
  int n2;
  double s;
  double t;
  double x00;
  double x01;
  double x10;
  double x11;
  double *xfat;

  m2 = ( m - 1 ) * ( mfat + 1 ) + 1;
  n2 = ( n - 1 ) * ( nfat + 1 ) + 1;

  xfat = new double[m2*n2];

  for ( i = 1; i <= m; i++ )
  {
    if ( i < m )
    {
      ihi = mfat;
    }
    else
    {
      ihi = 0;
    }

    for ( j = 1; j <= n; j++ )
    {
      if ( j < n )
      {
        jhi = nfat;
      }
      else
      {
        jhi = 0;
      }

      if ( i < m )
      {
        ip1 = i + 1;
      }
      else
      {
        ip1 = i;
      }

      if ( j < n )
      {
        jp1 = j + 1;
      }
      else
      {
        jp1 = j;
      }

      x00 = x[i-1+(j-1)*m];
      x10 = x[ip1-1+(j-1)*m];
      x01 = x[i-1+(jp1-1)*m];
      x11 = x[ip1-1+(jp1-1)*m];

      for ( ii = 0; ii <= ihi; ii++ )
      {
        s = ( double ) ( ii ) / ( double ) ( ihi + 1 );

        for ( jj = 0; jj <= jhi; jj++ )
        {
          t = ( double ) ( jj ) / ( double ) ( jhi + 1 );

          iii = 1 + ( i - 1 ) * ( mfat + 1 ) + ii;
          jjj = 1 + ( j - 1 ) * ( nfat + 1 ) + jj;

          xfat[iii-1+(jjj-1)*m2] =
                                            x00
              + s     * (       x10       - x00 )
              + t     * (             x01 - x00 )
              + s * t * ( x11 - x10 - x01 + x00 );
        }
      }
    }
  }

  return xfat;
}
//****************************************************************************80

double *r8mat_expand_linear2 ( int m, int n, double a[], int m2, int n2 )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_expand_linear2() expands an R8MAT by linear interpolation.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    In this version of the routine, the expansion is indicated
//    by specifying the dimensions of the expanded array.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    19 October 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, N, the number of rows and columns in A.
//
//    double A(M,N), a "small" M by N array.
//
//    int M2, N2, the number of rows and columns in A2.
//
//  Output:
//
//    double R8MAT_EXPAND_LINEAR2[M2*N2], the expanded array,
//    which contains an interpolated version of the data in A.
//
{
  double *a2;
  int i;
  int i1;
  int i2;
  int j;
  int j1;
  int j2;
  double r;
  double r1;
  double r2;
  double s;
  double s1;
  double s2;

  a2 = new double[m2*n2];

  for ( i = 1; i <= m2; i++ )
  {
    if ( m2 == 1 )
    {
      r = 0.5;
    }
    else
    {
      r = ( double ) ( i - 1 ) / ( double ) ( m2 - 1 );
    }

    i1 = 1 + ( int ) ( r * ( double ) ( m - 1 ) );
    i2 = i1 + 1;

    if ( m < i2 )
    {
      i1 = m - 1;
      i2 = m;
    }

    r1 = ( double ) ( i1 - 1 ) / ( double ) ( m - 1 );
    r2 = ( double ) ( i2 - 1 ) / ( double ) ( m - 1 );

    for ( j = 1; j <= n2; j++ )
    {
      if ( n2 == 1 )
      {
        s = 0.5;
      }
      else
      {
        s = ( double ) ( j - 1 ) / ( double ) ( n2 - 1 );
      }

      j1 = 1 + ( int ) ( s * ( double ) ( n - 1 ) );
      j2 = j1 + 1;

      if ( n < j2 )
      {
        j1 = n - 1;
        j2 = n;
      }

      s1 = ( double ) ( j1 - 1 ) / ( double ) ( n - 1 );
      s2 = ( double ) ( j2 - 1 ) / ( double ) ( n - 1 );

      a2[i-1+(j-1)*m2] =
        ( ( r2 - r ) * ( s2 - s ) * a[i1-1+(j1-1)*m]
        + ( r - r1 ) * ( s2 - s ) * a[i2-1+(j1-1)*m]
        + ( r2 - r ) * ( s - s1 ) * a[i1-1+(j2-1)*m]
        + ( r - r1 ) * ( s - s1 ) * a[i2-1+(j2-1)*m] )
        / ( ( r2 - r1 ) * ( s2 - s1 ) );
    }
  }

  return a2;
}
//****************************************************************************80

double *r8mat_flip_cols_new ( int m, int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_flip_cols_new() makes a new copy of an R8MAT with reversed column order.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    01 November 2013
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, N, the number of rows and columns.
//
//    double A[M*N], the matrix to be copied.
//
//  Output:
//
//    double R8MAT_FLIP_COLS_NEW[M*N], the reversed-column-order copy.
//
{
  double *b;
  int i;
  int j;

  b = new double[m*n];

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      b[i+(n-1-j)*m] = a[i+j*m];
    }
  }

  return b;
}
//****************************************************************************80

double *r8mat_flip_rows_new ( int m, int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_flip_rows_new() makes a new copy of an R8MAT with reversed row order.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    01 November 2013
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, N, the number of rows and columns.
//
//    double A[M*N], the matrix to be copied.
//
//  Output:
//
//    double R8MAT_FLIP_ROWS_NEW[M*N], the reversed-rows-order copy.
//
{
  double *b;
  int i;
  int j;

  b = new double[m*n];

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      b[(m-1-i)+j*m] = a[i+j*m];
    }
  }

  return b;
}
//****************************************************************************80

void r8mat_fs ( int n, double a[], double x[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_fs() factors and solves a system with one right hand side.
//
//  Discussion:
//
//    This routine differs from R8MAT_FSS in two ways:
//    * only one right hand side is allowed;
//    * the input matrix A is not modified.
//
//    This routine uses partial pivoting, but no pivot vector is required.
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    21 January 2013
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the order of the matrix.
//    N must be positive.
//
//    double A[N*N], the coefficient matrix of the linear system.
//
//    double X[N]: the right hand side of the linear system.
//
//  Output:
//
//    double X[N]: the solution of the linear system.
{
  double *a2;
  int i;
  int ipiv;
  int j;
  int jcol;
  double piv;
  double t;

  a2 = new double[n*n];

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < n; i++ )
    {
      a2[i+j*n] = a[i+j*n];
    }
  }

  for ( jcol = 1; jcol <= n; jcol++ )
  {
//
//  Find the maximum element in column I.
//
    piv = fabs ( a2[jcol-1+(jcol-1)*n] );
    ipiv = jcol;
    for ( i = jcol+1; i <= n; i++ )
    {
      if ( piv < fabs ( a2[i-1+(jcol-1)*n] ) )
      {
        piv = fabs ( a2[i-1+(jcol-1)*n] );
        ipiv = i;
      }
    }

    if ( piv == 0.0 )
    {
      cerr << "\n";
      cerr << "R8MAT_FS - Fatal error!\n";
      cerr << "  Zero pivot on step " << jcol << "\n";
      exit ( 1 );
    }
//
//  Switch rows JCOL and IPIV, and X.
//
    if ( jcol != ipiv )
    {
      for ( j = 1; j <= n; j++ )
      {
        t                  = a2[jcol-1+(j-1)*n];
        a2[jcol-1+(j-1)*n] = a2[ipiv-1+(j-1)*n];
        a2[ipiv-1+(j-1)*n] = t;
      }
      t         = x[jcol-1];
      x[jcol-1] = x[ipiv-1];
      x[ipiv-1] = t;
    }
//
//  Scale the pivot row.
//
    t = a2[jcol-1+(jcol-1)*n];
    a2[jcol-1+(jcol-1)*n] = 1.0;
    for ( j = jcol+1; j <= n; j++ )
    {
      a2[jcol-1+(j-1)*n] = a2[jcol-1+(j-1)*n] / t;
    }
    x[jcol-1] = x[jcol-1] / t;
//
//  Use the pivot row to eliminate lower entries in that column.
//
    for ( i = jcol+1; i <= n; i++ )
    {
      if ( a2[i-1+(jcol-1)*n] != 0.0 )
      {
        t = - a2[i-1+(jcol-1)*n];
        a2[i-1+(jcol-1)*n] = 0.0;
        for ( j = jcol+1; j <= n; j++ )
        {
          a2[i-1+(j-1)*n] = a2[i-1+(j-1)*n] + t * a2[jcol-1+(j-1)*n];
        }
        x[i-1] = x[i-1] + t * x[jcol-1];
      }
    }
  }
//
//  Back solve.
//
  for ( jcol = n; 2 <= jcol; jcol-- )
  {
    for ( i = 1; i < jcol; i++ )
    {
      x[i-1] = x[i-1] - a2[i-1+(jcol-1)*n] * x[jcol-1];
    }
  }

  delete [] a2;

  return;
}
//****************************************************************************80

double *r8mat_fs_new ( int n, double a[], double b[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_fs_new() factors and solves a system with one right hand side.
//
//  Discussion:
//
//    This routine differs from R8MAT_FSS_NEW in two ways:
//    * only one right hand side is allowed;
//    * the input matrix A is not modified.
//
//    This routine uses partial pivoting, but no pivot vector is required.
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    21 January 2013
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the order of the matrix.
//    N must be positive.
//
//    double A[N*N], the coefficient matrix of the linear system.
//    On output, A is in unit upper triangular form, and
//    represents the U factor of an LU factorization of the
//    original coefficient matrix.
//
//    double B[N], the right hand side of the linear system.
//
//  Output:
//
//    double X[N], the solution of the linear system.
//
{
  double *a2;
  int i;
  int ipiv;
  int j;
  int jcol;
  double piv;
  double t;
  double *x;

  a2 = new double[n*n];

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < n; i++ )
    {
      a2[i+j*n] = a[i+j*n];
    }
  }

  x = new double[n];
  for ( i = 0; i < n; i++ )
  {
    x[i] = b[i];
  }

  for ( jcol = 1; jcol <= n; jcol++ )
  {
//
//  Find the maximum element in column I.
//
    piv = fabs ( a2[jcol-1+(jcol-1)*n] );
    ipiv = jcol;
    for ( i = jcol+1; i <= n; i++ )
    {
      if ( piv < fabs ( a2[i-1+(jcol-1)*n] ) )
      {
        piv = fabs ( a2[i-1+(jcol-1)*n] );
        ipiv = i;
      }
    }

    if ( piv == 0.0 )
    {
      cerr << "\n";
      cerr << "R8MAT_FS_NEW - Fatal error!\n";
      cerr << "  Zero pivot on step " << jcol << "\n";
      exit ( 1 );
    }
//
//  Switch rows JCOL and IPIV, and X.
//
    if ( jcol != ipiv )
    {
      for ( j = 1; j <= n; j++ )
      {
        t                  = a2[jcol-1+(j-1)*n];
        a2[jcol-1+(j-1)*n] = a2[ipiv-1+(j-1)*n];
        a2[ipiv-1+(j-1)*n] = t;
      }
      t         = x[jcol-1];
      x[jcol-1] = x[ipiv-1];
      x[ipiv-1] = t;
    }
//
//  Scale the pivot row.
//
    t = a2[jcol-1+(jcol-1)*n];
    a2[jcol-1+(jcol-1)*n] = 1.0;
    for ( j = jcol+1; j <= n; j++ )
    {
      a2[jcol-1+(j-1)*n] = a2[jcol-1+(j-1)*n] / t;
    }
    x[jcol-1] = x[jcol-1] / t;
//
//  Use the pivot row to eliminate lower entries in that column.
//
    for ( i = jcol+1; i <= n; i++ )
    {
      if ( a2[i-1+(jcol-1)*n] != 0.0 )
      {
        t = - a2[i-1+(jcol-1)*n];
        a2[i-1+(jcol-1)*n] = 0.0;
        for ( j = jcol+1; j <= n; j++ )
        {
          a2[i-1+(j-1)*n] = a2[i-1+(j-1)*n] + t * a2[jcol-1+(j-1)*n];
        }
        x[i-1] = x[i-1] + t * x[jcol-1];
      }
    }
  }
//
//  Back solve.
//
  for ( jcol = n; 2 <= jcol; jcol-- )
  {
    for ( i = 1; i < jcol; i++ )
    {
      x[i-1] = x[i-1] - a2[i-1+(jcol-1)*n] * x[jcol-1];
    }
  }

  delete [] a2;

  return x;
}
//****************************************************************************80

void r8mat_fss ( int n, double a[], int nb, double x[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_fss() factors and solves a system with multiple right hand sides.
//
//  Discussion:
//
//    This routine uses partial pivoting, but no pivot vector is required.
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    28 November 2011
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the order of the matrix.
//    N must be positive.
//
//    double A[N*N]: the coefficient matrix of the linear system.
//
//    int NB, the number of right hand sides.
//
//    double X[N*NB]: the right hand sides of the linear systems.
//
//  Output:
//
//    double A[N*N]: A is in unit upper triangular form, and
//    represents the U factor of an LU factorization of the
//    original coefficient matrix.
//
//    double X[N*NB]: the solutions of the linear systems.
//
{
  int i;
  int ipiv;
  int j;
  int jcol;
  double piv;
  double t;

  for ( jcol = 1; jcol <= n; jcol++ )
  {
//
//  Find the maximum element in column I.
//
    piv = fabs ( a[jcol-1+(jcol-1)*n] );
    ipiv = jcol;
    for ( i = jcol+1; i <= n; i++ )
    {
      if ( piv < fabs ( a[i-1+(jcol-1)*n] ) )
      {
        piv = fabs ( a[i-1+(jcol-1)*n] );
        ipiv = i;
      }
    }

    if ( piv == 0.0 )
    {
      cerr << "\n";
      cerr << "R8MAT_FSS - Fatal error!\n";
      cerr << "  Zero pivot on step " << jcol << "\n";
      exit ( 1 );
    }
//
//  Switch rows JCOL and IPIV, and X.
//
    if ( jcol != ipiv )
    {
      for ( j = 1; j <= n; j++ )
      {
        t                 = a[jcol-1+(j-1)*n];
        a[jcol-1+(j-1)*n] = a[ipiv-1+(j-1)*n];
        a[ipiv-1+(j-1)*n] = t;
      }
      for ( j = 0; j < nb; j++ )
      {
        t            = x[jcol-1+j*n];
        x[jcol-1+j*n] = x[ipiv-1+j*n];
        x[ipiv-1+j*n] = t;
      }
    }
//
//  Scale the pivot row.
//
    t = a[jcol-1+(jcol-1)*n];
    a[jcol-1+(jcol-1)*n] = 1.0;
    for ( j = jcol+1; j <= n; j++ )
    {
      a[jcol-1+(j-1)*n] = a[jcol-1+(j-1)*n] / t;
    }
    for ( j = 0; j < nb; j++ )
    {
      x[jcol-1+j*n] = x[jcol-1+j*n] / t;
    }
//
//  Use the pivot row to eliminate lower entries in that column.
//
    for ( i = jcol+1; i <= n; i++ )
    {
      if ( a[i-1+(jcol-1)*n] != 0.0 )
      {
        t = - a[i-1+(jcol-1)*n];
        a[i-1+(jcol-1)*n] = 0.0;
        for ( j = jcol+1; j <= n; j++ )
        {
          a[i-1+(j-1)*n] = a[i-1+(j-1)*n] + t * a[jcol-1+(j-1)*n];
        }
        for ( j = 0; j < nb; j++ )
        {
          x[i-1+j*n] = x[i-1+j*n] + t * x[jcol-1+j*n];
        }
      }
    }
  }
//
//  Back solve.
//
  for ( jcol = n; 2 <= jcol; jcol-- )
  {
    for ( i = 1; i < jcol; i++ )
    {
      for ( j = 0; j < nb; j++ )
      {
        x[i-1+j*n] = x[i-1+j*n] - a[i-1+(jcol-1)*n] * x[jcol-1+j*n];
      }
    }
  }

  return;
}
//****************************************************************************80

double *r8mat_fss_new ( int n, double a[], int nb, double b[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_fss_new() factors and solves a system with multiple right hand sides.
//
//  Discussion:
//
//    This routine uses partial pivoting, but no pivot vector is required.
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    28 November 2011
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the order of the matrix.
//    N must be positive.
//
//    double A[N*N]: the coefficient matrix of the linear system.
//
//    int NB, the number of right hand sides.
//
//    double B[N*NB], the right hand sides of the linear systems.
//
//  Output:
//
//    double A[N*N]: A is in unit upper triangular form, and
//    represents the U factor of an LU factorization of the
//    original coefficient matrix.
//
//    double R8MAT_FSS_NEW[N*NB], the solutions of the linear systems.
//
{
  int i;
  int ipiv;
  int j;
  int jcol;
  double piv;
  double t;
  double *x;

  x = new double[n*nb];

  for ( j = 0; j < nb; j++ )
  {
    for ( i = 0; i < n; i++ )
    {
      x[i+j*n] = b[i+j*n];
    }
  }
  for ( jcol = 1; jcol <= n; jcol++ )
  {
//
//  Find the maximum element in column I.
//
    piv = fabs ( a[jcol-1+(jcol-1)*n] );
    ipiv = jcol;
    for ( i = jcol + 1; i <= n; i++ )
    {
      if ( piv < fabs ( a[i-1+(jcol-1)*n] ) )
      {
        piv = fabs ( a[i-1+(jcol-1)*n] );
        ipiv = i;
      }
    }

    if ( piv == 0.0 )
    {
      cerr << "\n";
      cerr << "R8MAT_FSS_NEW - Fatal error!\n";
      cerr << "  Zero pivot on step " << jcol << "\n";
      exit ( 1 );
    }
//
//  Switch rows JCOL and IPIV, and X.
//
    if ( jcol != ipiv )
    {
      for ( j = 1; j <= n; j++ )
      {
        t                 = a[jcol-1+(j-1)*n];
        a[jcol-1+(j-1)*n] = a[ipiv-1+(j-1)*n];
        a[ipiv-1+(j-1)*n] = t;
      }
      for ( j = 0; j < nb; j++ )
      {
        t            = x[jcol-1+j*n];
        x[jcol-1+j*n] = x[ipiv-1+j*n];
        x[ipiv-1+j*n] = t;
      }
    }
//
//  Scale the pivot row.
//
    t = a[jcol-1+(jcol-1)*n];
    a[jcol-1+(jcol-1)*n] = 1.0;
    for ( j = jcol+1; j <= n; j++ )
    {
      a[jcol-1+(j-1)*n] = a[jcol-1+(j-1)*n] / t;
    }
    for ( j = 0; j < nb; j++ )
    {
      x[jcol-1+j*n] = x[jcol-1+j*n] / t;
    }
//
//  Use the pivot row to eliminate lower entries in that column.
//
    for ( i = jcol+1; i <= n; i++ )
    {
      if ( a[i-1+(jcol-1)*n] != 0.0 )
      {
        t = - a[i-1+(jcol-1)*n];
        a[i-1+(jcol-1)*n] = 0.0;
        for ( j = jcol+1; j <= n; j++ )
        {
          a[i-1+(j-1)*n] = a[i-1+(j-1)*n] + t * a[jcol-1+(j-1)*n];
        }
        for ( j = 0; j < nb; j++ )
        {
          x[i-1+j*n] = x[i-1+j*n] + t * x[jcol-1+j*n];
        }
      }
    }
  }
//
//  Back solve.
//
  for ( jcol = n; 2 <= jcol; jcol-- )
  {
    for ( i = 1; i < jcol; i++ )
    {
      for ( j = 0; j < nb; j++ )
      {
        x[i-1+j*n] = x[i-1+j*n] - a[i-1+(jcol-1)*n] * x[jcol-1+j*n];
      }
    }
  }

  return x;
}
//****************************************************************************80

double *r8mat_givens_post ( int n, double a[], int row, int col )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_givens_post() computes the Givens postmultiplier rotation matrix.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    The Givens post-multiplier matrix G(ROW,COL) has the property that
//    the (ROW,COL)-th entry of A*G is zero.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    23 October 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the order of the matrices A and G.
//
//    double A[N*N], the matrix to be operated upon.
//
//    int ROW, COL, the row and column of the
//    entry of A*G which is to be zeroed out.
//
//  Output:
//
//    double R8MAT_GIVENS_POST[N*N], the Givens rotation matrix.
//    G is an orthogonal matrix, that is, the inverse of
//    G is the transpose of G.
//
{
  double *g;
  double theta;

  g = r8mat_identity_new ( n );

  theta = atan2 ( a[row-1+(col-1)*n], a[row-1+(row-1)*n] );

  g[row-1+(row-1)*n] =  cos ( theta );
  g[row-1+(col-1)*n] = -sin ( theta );
  g[col-1+(row-1)*n] =  sin ( theta );
  g[col-1+(col-1)*n] =  cos ( theta );

  return g;
}
//****************************************************************************80

double *r8mat_givens_pre ( int n, double a[], int row, int col )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_givens_pre() computes the Givens premultiplier rotation matrix.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    The Givens premultiplier rotation matrix G(ROW,COL) has the
//    property that the (ROW,COL)-th entry of G*A is zero.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    26 September 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the order of the matrices A and G.
//
//    double A[N*N], the matrix to be operated upon.
//
//    int ROW, COL, the row and column of the
//    entry of the G*A which is to be zeroed out.
//
//  Output:
//
//    double R8MAT_GIVENS_PRE[N*N], the Givens rotation matrix.
//    G is an orthogonal matrix, that is, the inverse of
//    G is the transpose of G.
//
{
  double *g;
  double theta;

  g = r8mat_identity_new ( n );

  theta = atan2 ( a[row-1+(col-1)*n], a[col-1+(col-1)*n] );

  g[row-1+(row-1)*n] =  cos ( theta );
  g[row-1+(col-1)*n] = -sin ( theta );
  g[col-1+(row-1)*n] =  sin ( theta );
  g[col-1+(col-1)*n] =  cos ( theta );

  return g;
}
//****************************************************************************80

double *r8mat_hess ( double (*fx) ( int n, double x[] ), int n, double x[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_hess() approximates a Hessian matrix via finite differences.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    H(I,J) = d2 F / d X(I) d X(J)
//
//    The values returned by this routine will be only approximate.
//    In some cases, they will be so poor that they are useless.
//    However, one of the best applications of this routine is for
//    checking your own Hessian calculations, since as Heraclitus
//    said, you'll never get the same result twice when you differentiate
//    a complicated expression by hand.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    26 August 2011
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double *FX ( int N, double X[] ), the name of the user
//    function routine.
//
//    int N, the number of variables.
//
//    double X[N], the values of the variables.
//
//  Output:
//
//    double H[N*N], the approximated N by N Hessian matrix.
//
{
  double eps;
  double f00;
  double fmm;
  double fmp;
  double fpm;
  double fpp;
  double *h;
  int i;
  int j;
  double *s;
  double xi;
  double xj;
//
//  Choose the stepsizes.
//
  s = new double[n];

  eps = pow ( DBL_EPSILON, 0.33 );

  for ( i = 0; i < n; i++ )
  {
    s[i] = eps * fmax ( fabs ( x[i] ), 1.0 );
  }
//
//  Calculate the diagonal elements.
//
  h = new double[n*n];

  for ( i = 0; i < n; i++ )
  {
    xi = x[i];

    f00 = fx ( n, x );

    x[i] = xi + s[i];
    fpp = fx ( n, x );

    x[i] = xi - s[i];
    fmm = fx ( n, x );

    h[i+i*n] = ( ( fpp - f00 ) + ( fmm - f00 ) ) / s[i] / s[i];

    x[i] = xi;
  }
//
//  Calculate the off diagonal elements.
//
  for ( i = 0; i < n; i++ )
  {
    xi = x[i];

    for ( j = i+1; j < n; j++ )
    {
      xj = x[j];

      x[i] = xi + s[i];
      x[j] = xj + s[j];
      fpp = fx ( n, x );

      x[i] = xi + s[i];
      x[j] = xj - s[j];
      fpm = fx ( n, x );

      x[i] = xi - s[i];
      x[j] = xj + s[j];
      fmp = fx ( n, x );

      x[i] = xi - s[i];
      x[j] = xj - s[j];
      fmm = fx ( n, x );

      h[j+i*n] = ( ( fpp - fpm ) + ( fmm - fmp ) ) / ( 4.0 * s[i] * s[j] );

      h[i+j*n] = h[j+i*n];

      x[j] = xj;
    }
    x[i] = xi;
  }

  delete [] s;

  return h;
}
//****************************************************************************80

void r8mat_house_axh ( int n, double a[], double v[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_house_axh() computes A*H where H is a compact Householder matrix.
//
//  Discussion: 							    
//
//    An R8MAT is a doubly dimensioned array of double precision values, which
//    may be stored as a vector in column-major order.
//
//    The Householder matrix H(V) is defined by
//
//      H(V) = I - 2 * v * v' / ( v' * v )
//
//    This routine is not particularly efficient.
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    07 July 2011
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the order of A.
//
//    double A[N*N]: the matrix to be postmultiplied by H.
//
//    double V[N], a vector defining a Householder matrix.
//
//  Output:
//
//    double A[N*N]: the value of A*H.
{
  double *ah;
  int i;
  int j;
  int k;
  double v_normsq;

  v_normsq = 0.0;
  for ( i = 0; i < n; i++ )
  {
    v_normsq = v_normsq + v[i] * v[i];
  }
//
//  Compute A*H' = A*H
//
  ah = new double[n*n];

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < n; i++ )
    {
      ah[i+j*n] = a[i+j*n];
      for ( k = 0; k < n; k++ )
      {
        ah[i+j*n] = ah[i+j*n] - 2.0 * a[i+k*n] * v[k] * v[j] / v_normsq;
      }
    }
  }
//
//  Copy A = AH;
//
  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < n; i++ )
    {
      a[i+j*n] = ah[i+j*n];
    }
  }
  delete [] ah;

  return;
}
//****************************************************************************80

double *r8mat_house_axh_new ( int n, double a[], double v[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_house_axh_new() computes A*H where H is a compact Householder matrix.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    The Householder matrix H(V) is defined by
//
//      H(V) = I - 2 * v * v' / ( v' * v )
//
//    This routine is not particularly efficient.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    11 July 2008
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the order of A.
//
//    double A[N*N], the matrix to be postmultiplied.
//
//    double V[N], a vector defining a Householder matrix.
//
//  Output:
//
//    double R8MAT_HOUSE_AXH[N*N], the product A*H.
//
{
  double *ah;
  int i;
  int j;
  int k;
  double v_normsq;

  v_normsq = 0.0;
  for ( i = 0; i < n; i++ )
  {
    v_normsq = v_normsq + v[i] * v[i];
  }
//
//  Compute A*H' = A*H
//
  ah = new double[n*n];

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < n; i++ )
    {
      ah[i+j*n] = a[i+j*n];
      for ( k = 0; k < n; k++ )
      {
        ah[i+j*n] = ah[i+j*n] - 2.0 * a[i+k*n] * v[k] * v[j] / v_normsq;
      }
    }
  }

  return ah;
}
//****************************************************************************80

double *r8mat_house_form ( int n, double v[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_house_form() constructs a Householder matrix from its compact form.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    H(v) = I - 2 * v * v' / ( v' * v )
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    07 October 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the order of the matrix.
//
//    double V[N], the vector defining the Householder matrix.
//
//  Output:
//
//    double R8MAT_HOUSE_FORM[N*N], the Householder matrix.
//
{
  double beta;
  double *h;
  int i;
  int j;
//
//  Compute the L2 norm of V.
//
  beta = 0.0;
  for ( i = 0; i < n; i++ )
  {
    beta = beta + v[i] * v[i];
  }
//
//  Form the matrix H.
//
  h = r8mat_identity_new ( n );

  for ( i = 0; i < n; i++ )
  {
    for ( j = 0; j < n; j++ )
    {
      h[i+j*n] = h[i+j*n] - 2.0 * v[i] * v[j] / beta;
    }
  }

  return h;
}
//****************************************************************************80

double *r8mat_house_hxa ( int n, double a[], double v[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_house_hxa() computes H*A where H is a compact Householder matrix.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    The Householder matrix H(V) is defined by
//
//      H(V) = I - 2 * v * v' / ( v' * v )
//
//    This routine is not particularly efficient.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    07 October 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the order of A.
//
//    double A[N*N], the matrix to be premultiplied.
//
//    double V[N], a vector defining a Householder matrix.
//
//  Output:
//
//    double R8MAT_HOUSE_HXA[N*N], the product H*A.
//
{
  double *ha;
  int i;
  int j;
  int k;
  double v_normsq;

  v_normsq = 0.0;
  for ( i = 0; i < n; i++ )
  {
    v_normsq = v_normsq + v[i] * v[i];
  }
//
//  Compute A*H' = A*H
//
  ha = new double[n*n];

  for ( i = 0; i < n; i++ )
  {
    for ( j = 0; j < n; j++ )
    {
      ha[i+j*n] = a[i+j*n];
      for ( k = 0; k < n; k++ )
      {
        ha[i+j*n] = ha[i+j*n] - 2.0 * v[i] * v[k] * a[k+j*n] / v_normsq;
      }
    }
  }

  return ha;
}
//****************************************************************************80

double *r8mat_house_post ( int n, double a[], int row, int col )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_house_post() computes a Householder post-multiplier matrix.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    H(ROW,COL) has the property that the ROW-th column of
//    A*H(ROW,COL) is zero from entry COL+1 to the end.
//
//    In the most common case, where a QR factorization is being computed,
//    ROW = COL.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    07 October 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the order of the matrices.
//
//    double A[N*N], the matrix whose Householder matrix
//    is to be computed.
//
//    int ROW, COL, specify the location of the
//    entry of the matrix A which is to be preserved.  The entries in
//    the same row, but higher column, will be zeroed out if
//    A is postmultiplied by H.
//
//  Output:
//
//    double R8MAT_HOUSE_POST[N*N], the Householder matrix.
//
{
  double *a_row;
  double *h;
  int j;
  double *v;
//
//  Extract the ROW-th row of A.
//
  a_row = new double[n];

  for ( j = 0; j < col-1; j++ )
  {
    a_row[j] = 0.0;
  }
  for ( j = col - 1; j < n; j++ )
  {
    a_row[j] = a[row+j*n];
  }
//
//  Set up the vector V.
//
  v = r8vec_house_column ( n, a_row, col );
//
//  Form the matrix H(V).
//
  h = r8mat_house_form ( n, v );
//
//  Release memory.
//
  delete [] a_row;
  delete [] v;

  return h;
}
//****************************************************************************80

double *r8mat_house_pre ( int n, double a[], int row, int col )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_house_pre() computes a Householder pre-multiplier matrix.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    H(ROW,COL) has the property that the COL-th column of
//    H(ROW,COL)*A is zero from entry ROW+1 to the end.
//
//    In the most common case, where a QR factorization is being computed,
//    ROW = COL.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    07 October 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the order of the matrices.
//
//    double A[N*N], the matrix whose Householder matrix
//    is to be computed.
//
//    int ROW, COL, specify the location of the
//    entry of the matrix A which is to be preserved.  The entries in
//    the same column, but higher rows, will be zeroed out if A is
//    premultiplied by H.
//
//  Output:
//
//    double R8MAT_HOUSE_PRE[N*N], the Householder matrix.
//
{
  double *a_col;
  double *h;
  int i;
  double *v;
//
//  Extract the COL-th column of A.
//
  a_col = new double[n];

  for ( i = 0; i < row-1; i++ )
  {
    a_col[i] = 0.0;
  }
  for ( i = row-1; i < n; i++ )
  {
    a_col[i] = a[i+col*n];
  }
//
//  Set up the vector V.
//
  v = r8vec_house_column ( n, a_col, row );
//
//  Form the matrix H(V).
//
  h = r8mat_house_form ( n, v );
//
//  Release memory.
//
  delete [] a_col;
  delete [] v;

  return h;
}
//****************************************************************************80

void r8mat_identity ( int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_identity() sets the square matrix A to the identity.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    01 December 2011
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the order of A.
//
//  Output:
//
//    double A[N*N], the N by N identity matrix.
//
{
  int i;
  int j;
  int k;

  k = 0;
  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < n; i++ )
    {
      if ( i == j )
      {
        a[k] = 1.0;
      }
      else
      {
        a[k] = 0.0;
      }
      k = k + 1;
    }
  }

  return;
}
//****************************************************************************80

double *r8mat_identity_new ( int n )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_identity_new() returns an identity matrix.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    06 September 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the order of A.
//
//  Output:
//
//    double R8MAT_IDENTITY_NEW[N*N], the N by N identity matrix.
//
{
  double *a;
  int i;
  int j;
  int k;

  a = new double[n*n];

  k = 0;
  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < n; i++ )
    {
      if ( i == j )
      {
        a[k] = 1.0;
      }
      else
      {
        a[k] = 0.0;
      }
      k = k + 1;
    }
  }

  return a;
}
//****************************************************************************80

double *r8mat_indicator_new ( int m, int n )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_indicator_new() sets up an "indicator" R8MAT.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    The value of each entry suggests its location, as in:
//
//      11  12  13  14
//      21  22  23  24
//      31  32  33  34
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    25 January 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, the number of rows of the matrix.
//    M must be positive.
//
//    int N, the number of columns of the matrix.
//    N must be positive.
//
//  Output:
//
//    double R8MAT_INDICATOR_NEW[M*N], the table.
//
{
  double *a;
  int fac;
  int i;
  int j;

  a = new double[m*n];

  fac = i4_power ( 10, i4_log_10 ( n ) + 1 );

  for ( i = 1; i <= m; i++ )
  {
    for ( j = 1; j <= n; j++ )
    {
      a[i-1+(j-1)*m] = ( double ) ( fac * i + j );
    }
  }
  return a;
}
//****************************************************************************80

double *r8mat_inverse_2d ( double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_inverse_2d() inverts a 2 by 2 matrix using Cramer's rule.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    23 September 2003
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double A[2*2], the matrix to be inverted.
//
//  Output:
//
//    double R8MAT_INVERSE_2D[2*2], the inverse of the matrix A.
//
{
  double *b;
  double det;
//
//  Compute the determinant of A.
//
  det = a[0+0*2] * a[1+1*2] - a[0+1*2] * a[1+0*2];
//
//  If the determinant is zero, bail out.
//
  if ( det == 0.0 )
  {
    return NULL;
  }
//
//  Compute the entries of the inverse matrix using an explicit formula.
//
  b = new double[2*2];

  b[0+0*2] = + a[1+1*2] / det;
  b[0+1*2] = - a[0+1*2] / det;
  b[1+0*2] = - a[1+0*2] / det;
  b[1+1*2] = + a[0+0*2] / det;

  return b;
}
//****************************************************************************80

double *r8mat_inverse_3d ( double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_inverse_3d() inverts a 3 by 3 matrix using Cramer's rule.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    If the determinant is zero, A is singular, and does not have an
//    inverse.  In that case, the output is set to NULL.
//
//    If the determinant is nonzero, its value is an estimate
//    of how nonsingular the matrix A is.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    23 September 2003
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double A[3*3], the matrix to be inverted.
//
//  Output:
//
//    double R8MAT_INVERSE_3D[3*3], the inverse of the matrix A.
//
{
  double *b;
  double det;
//
//  Compute the determinant of A.
//
  det =
     a[0+0*3] * ( a[1+1*3] * a[2+2*3] - a[1+2*3] * a[2+1*3] )
   + a[0+1*3] * ( a[1+2*3] * a[2+0*3] - a[1+0*3] * a[2+2*3] )
   + a[0+2*3] * ( a[1+0*3] * a[2+1*3] - a[1+1*3] * a[2+0*3] );

  if ( det == 0.0 )
  {
    return NULL;
  }

  b = new double[3*3];

  b[0+0*3] =   ( a[1+1*3] * a[2+2*3] - a[1+2*3] * a[2+1*3] ) / det;
  b[0+1*3] = - ( a[0+1*3] * a[2+2*3] - a[0+2*3] * a[2+1*3] ) / det;
  b[0+2*3] =   ( a[0+1*3] * a[1+2*3] - a[0+2*3] * a[1+1*3] ) / det;

  b[1+0*3] = - ( a[1+0*3] * a[2+2*3] - a[1+2*3] * a[2+0*3] ) / det;
  b[1+1*3] =   ( a[0+0*3] * a[2+2*3] - a[0+2*3] * a[2+0*3] ) / det;
  b[1+2*3] = - ( a[0+0*3] * a[1+2*3] - a[0+2*3] * a[1+0*3] ) / det;

  b[2+0*3] =   ( a[1+0*3] * a[2+1*3] - a[1+1*3] * a[2+0*3] ) / det;
  b[2+1*3] = - ( a[0+0*3] * a[2+1*3] - a[0+1*3] * a[2+0*3] ) / det;
  b[2+2*3] =   ( a[0+0*3] * a[1+1*3] - a[0+1*3] * a[1+0*3] ) / det;

  return b;
}
//****************************************************************************80

double *r8mat_inverse_4d ( double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_inverse_4d() inverts a 4 by 4 matrix using Cramer's rule.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    18 September 2003
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double A[4][4], the matrix to be inverted.
//
//  Output:
//
//    double R8MAT_INVERSE_4D[4][4], the inverse of the matrix A.
//
{
  double *b;
  double det;
//
//  Compute the determinant of A.
//
  det = r8mat_det_4d ( a );
//
//  If the determinant is zero, bail out.
//
  if ( det == 0.0 )
  {
    return NULL;
  }
//
//  Compute the entries of the inverse matrix using an explicit formula.
//
  b = new double[4*4];

  b[0+0*4] =
    +(
    + a[1+1*4] * ( a[2+2*4] * a[3+3*4] - a[2+3*4] * a[3+2*4] )
    + a[1+2*4] * ( a[2+3*4] * a[3+1*4] - a[2+1*4] * a[3+3*4] )
    + a[1+3*4] * ( a[2+1*4] * a[3+2*4] - a[2+2*4] * a[3+1*4] )
    ) / det;

  b[1+0*4] =
    -(
    + a[1+0*4] * ( a[2+2*4] * a[3+3*4] - a[2+3*4] * a[3+2*4] )
    + a[1+2*4] * ( a[2+3*4] * a[3+0*4] - a[2+0*4] * a[3+3*4] )
    + a[1+3*4] * ( a[2+0*4] * a[3+2*4] - a[2+2*4] * a[3+0*4] )
    ) / det;

  b[2+0*4] =
    +(
    + a[1+0*4] * ( a[2+1*4] * a[3+3*4] - a[2+3*4] * a[3+1*4] )
    + a[1+1*4] * ( a[2+3*4] * a[3+0*4] - a[2+0*4] * a[3+3*4] )
    + a[1+3*4] * ( a[2+0*4] * a[3+1*4] - a[2+1*4] * a[3+0*4] )
    ) / det;

  b[3+0*4] =
    -(
    + a[1+0*4] * ( a[2+1*4] * a[3+2*4] - a[2+2*4] * a[3+1*4] )
    + a[1+1*4] * ( a[2+2*4] * a[3+0*4] - a[2+0*4] * a[3+2*4] )
    + a[1+2*4] * ( a[2+0*4] * a[3+1*4] - a[2+1*4] * a[3+0*4] )
    ) / det;

  b[0+1*4] =
    -(
    + a[0+1*4] * ( a[2+2*4] * a[3+3*4] - a[2+3*4] * a[3+2*4] )
    + a[0+2*4] * ( a[2+3*4] * a[3+1*4] - a[2+1*4] * a[3+3*4] )
    + a[0+3*4] * ( a[2+1*4] * a[3+2*4] - a[2+2*4] * a[3+1*4] )
    ) / det;

  b[1+1*4] =
    +(
    + a[0+0*4] * ( a[2+2*4] * a[3+3*4] - a[2+3*4] * a[3+2*4] )
    + a[0+2*4] * ( a[2+3*4] * a[3+0*4] - a[2+0*4] * a[3+3*4] )
    + a[0+3*4] * ( a[2+0*4] * a[3+2*4] - a[2+2*4] * a[3+0*4] )
    ) / det;

  b[2+1*4] =
    -(
    + a[0+0*4] * ( a[2+1*4] * a[3+3*4] - a[2+3*4] * a[3+1*4] )
    + a[0+1*4] * ( a[2+3*4] * a[3+0*4] - a[2+0*4] * a[3+3*4] )
    + a[0+3*4] * ( a[2+0*4] * a[3+1*4] - a[2+1*4] * a[3+0*4] )
    ) / det;

  b[3+1*4] =
    +(
    + a[0+0*4] * ( a[2+1*4] * a[3+2*4] - a[2+2*4] * a[3+1*4] )
    + a[0+1*4] * ( a[2+2*4] * a[3+0*4] - a[2+0*4] * a[3+2*4] )
    + a[0+2*4] * ( a[2+0*4] * a[3+1*4] - a[2+1*4] * a[3+0*4] )
    ) / det;

  b[0+2*4] =
    +(
    + a[0+1*4] * ( a[1+2*4] * a[3+3*4] - a[1+3*4] * a[3+2*4] )
    + a[0+2*4] * ( a[1+3*4] * a[3+1*4] - a[1+1*4] * a[3+3*4] )
    + a[0+3*4] * ( a[1+1*4] * a[3+2*4] - a[1+2*4] * a[3+1*4] )
    ) / det;

  b[1+2*4] =
    -(
    + a[0+0*4] * ( a[1+2*4] * a[3+3*4] - a[1+3*4] * a[3+2*4] )
    + a[0+2*4] * ( a[1+3*4] * a[3+0*4] - a[1+0*4] * a[3+3*4] )
    + a[0+3*4] * ( a[1+0*4] * a[3+2*4] - a[1+2*4] * a[3+0*4] )
    ) / det;

  b[2+2*4] =
    +(
    + a[0+0*4] * ( a[1+1*4] * a[3+3*4] - a[1+3*4] * a[3+1*4] )
    + a[0+1*4] * ( a[1+3*4] * a[3+0*4] - a[1+0*4] * a[3+3*4] )
    + a[0+3*4] * ( a[1+0*4] * a[3+1*4] - a[1+1*4] * a[3+0*4] )
    ) / det;

  b[3+2*4] =
    -(
    + a[0+0*4] * ( a[1+1*4] * a[3+2*4] - a[1+2*4] * a[3+1*4] )
    + a[0+1*4] * ( a[1+2*4] * a[3+0*4] - a[1+0*4] * a[3+2*4] )
    + a[0+2*4] * ( a[1+0*4] * a[3+1*4] - a[1+1*4] * a[3+0*4] )
    ) / det;

  b[0+3*4] =
    -(
    + a[0+1*4] * ( a[1+2*4] * a[2+3*4] - a[1+3*4] * a[2+2*4] )
    + a[0+2*4] * ( a[1+3*4] * a[2+1*4] - a[1+1*4] * a[2+3*4] )
    + a[0+3*4] * ( a[1+1*4] * a[2+2*4] - a[1+2*4] * a[2+1*4] )
    ) / det;

  b[1+3*4] =
    +(
    + a[0+0*4] * ( a[1+2*4] * a[2+3*4] - a[1+3*4] * a[2+2*4] )
    + a[0+2*4] * ( a[1+3*4] * a[2+0*4] - a[1+0*4] * a[2+3*4] )
    + a[0+3*4] * ( a[1+0*4] * a[2+2*4] - a[1+2*4] * a[2+0*4] )
    ) / det;

  b[2+3*4] =
    -(
    + a[0+0*4] * ( a[1+1*4] * a[2+3*4] - a[1+3*4] * a[2+1*4] )
    + a[0+1*4] * ( a[1+3*4] * a[2+0*4] - a[1+0*4] * a[2+3*4] )
    + a[0+3*4] * ( a[1+0*4] * a[2+1*4] - a[1+1*4] * a[2+0*4] )
    ) / det;

  b[3+3*4] =
    +(
    + a[0+0*4] * ( a[1+1*4] * a[2+2*4] - a[1+2*4] * a[2+1*4] )
    + a[0+1*4] * ( a[1+2*4] * a[2+0*4] - a[1+0*4] * a[2+2*4] )
    + a[0+2*4] * ( a[1+0*4] * a[2+1*4] - a[1+1*4] * a[2+0*4] )
    ) / det;

  return b;
}
//****************************************************************************80

bool r8mat_is_binary ( int m, int n, double x[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_is_binary() is true if the entries in an R8MAT are all 0 or 1.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    24 April 2018
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, N, the dimensions of the array.
//
//    double X[M*N], the array to be checked.
//
//  Output:
//
//    bool R8MAT_IS_BINARY is true if are entries are 0 or 1.
//
{
  int i;
  int j;
  bool value;

  value = true;

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      if ( x[i+j*m] != 0.0 && x[i+j*m] != 1.0 )
      {
        value = false;
        break;
      }
    }
  }
  return value;
}
//****************************************************************************80

double r8mat_is_identity ( int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_is_identity() determines if an R8MAT is the identity.
//
//  Discussion:
//
//    An R8MAT is a matrix of real ( kind = 8 ) values.
//
//    The routine returns the Frobenius norm of A - I.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    29 July 2011
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the order of the matrix.
//
//    double A[N*N], the matrix.
//
//  Output:
//
//    double R8MAT_IS_IDENTITY, the Frobenius norm
//    of the difference matrix A - I, which would be exactly zero
//    if A were the identity matrix.
//
{
  double error_frobenius;
  int i;
  int j;
  double t;

  error_frobenius = 0.0;

  for ( i = 0; i < n; i++ )
  {
    for ( j = 0; j < n; j++ )
    {
      if ( i == j )
      {
        t = a[i+j*n] - 1.0;
      }
      else
      {
        t = a[i+j*n];
      }
      error_frobenius = error_frobenius + t * t;
    }
  }
  error_frobenius = sqrt ( error_frobenius );

  return error_frobenius;
}
//****************************************************************************80

bool r8mat_is_in_01 ( int m, int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_is_in_01() is TRUE if the entries of an R8MAT are in the range [0,1].
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    06 October 2004
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, the number of rows in A.
//
//    int N, the number of columns in A.
//
//    double A[M*N], the matrix.
//
//  Output:
//
//    bool R8MAT_IS_IN_01, is TRUE if every entry of A is
//    between 0 and 1.
//
{
  int i;
  int j;

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      if ( a[i+j*m] < 0.0 || 1.0 < a[i+j*m] )
      {
        return false;
      }
    }
  }

  return true;
}
//****************************************************************************80

bool r8mat_is_insignificant ( int m, int n, double r[], double s[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_is_insignificant() determines if an R8MAT is relatively insignificant.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    26 November 2011
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, N, the dimension of the matrices.
//
//    double R[M*N], the vector to be compared against.
//
//    double S[M*N], the vector to be compared.
//
//  Output:
//
//    bool R8MAT_IS_INSIGNIFICANT, is TRUE if S is insignificant
//    compared to R.
//
{
  int i;
  int j;
  double t;
  double tol;
  bool value;

  value = true;

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      t = r[i+j*m] + s[i+j*m];
      tol = DBL_EPSILON * fabs ( r[i+j*m] );

      if ( tol < fabs ( r[i+j*m] - t ) )
      {
        value = false;
        break;
      }
    }
  }
  return value;
}
//****************************************************************************80

bool r8mat_is_integer ( int m, int n, double x[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_is_integer() is true if an R8MAT only contains integer entries.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    26 August 2018
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, N, the dimensions of the array.
//
//    double X[M*N], the vector to be checked.
//
//  Output:
//
//    bool R8MAT_IS_INTEGER is true if all elements of X
//    are integers.
//
{
  int i;
  int j;
  bool value;

  value = true;

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      if ( ! r8_is_integer ( x[i+j*m] ) )
      {
        value = false;
        break;
      }
    }
  }
  return value;
}
//****************************************************************************80

bool r8mat_is_significant ( int m, int n, double r[], double s[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_is_significant() determines if an R8MAT is relatively significant.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    26 November 2011
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, N, the dimension of the matrices.
//
//    double R[M*N], the vector to be compared against.
//
//    double S[M*N], the vector to be compared.
//
//  Output:
//
//    bool R8MAT_IS_SIGNIFICANT, is TRUE if S is significant
//    compared to R.
//
{
  int i;
  int j;
  double t;
  double tol;
  bool value;

  value = false;

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      t = r[i+j*m] + s[i+j*m];
      tol = DBL_EPSILON * fabs ( r[i+j*m] );

      if ( tol < fabs ( r[i+j*m] - t ) )
      {
        value = true;
        break;
      }
    }
  }
  return value;
}
//****************************************************************************80

double r8mat_is_symmetric ( int m, int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_is_symmetric() checks an R8MAT for symmetry.
//
//  Discussion:
//
//    An R8MAT is a matrix of double precision real values.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    15 July 2008
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, N, the order of the matrix.
//
//    double A[M*N], the matrix.
//
//  Output:
//
//    double RMAT_IS_SYMMETRIC, measures the 
//    Frobenius norm of ( A - A' ), which would be zero if the matrix
//    were exactly symmetric.
//
{
  int i;
  int j;
  double value;

  if ( m != n )
  {
    value = HUGE_VAL;
    return value;
  }

  value = 0.0;
  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      value = value + pow ( a[i+j*m] - a[j+i*m], 2 );
    }
  }
  value = sqrt ( value );

  return value;
}
//****************************************************************************80

double *r8mat_jac ( int m, int n, double eps,
  double *(*fx) ( int m, int n, double x[] ), double x[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_jac() estimates a dense jacobian matrix of the function FX.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    FPRIME(I,J) = d F(I) / d X(J).
//
//    The jacobian is assumed to be dense, and the LINPACK/LAPACK
//    double precision general matrix storage mode ("DGE") is used.
//
//    Forward differences are used, requiring N+1 function evaluations.
//
//    Values of EPS have typically been chosen between
//    sqrt ( EPSMCH ) and sqrt ( sqrt ( EPSMCH ) ) where EPSMCH is the
//    machine tolerance.
//
//    If EPS is too small, then F(X+EPS) will be the same as
//    F(X), and the jacobian will be full of zero entries.
//
//    If EPS is too large, the finite difference estimate will
//    be inaccurate.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    30 October 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, the number of functions.
//
//    int N, the number of variables.
//
//    double EPS, a tolerance to be used for shifting the
//    X values during the finite differencing.  No single value
//    of EPS will be reliable for all vectors X and functions FX.
//
//    double *(*FX) ( int m, int n, double x[] ), the name of
//    the user written routine which evaluates the M-dimensional
//    function at a given N-dimensional point X.
//
//    double X[N], the point where the jacobian
//    is to be estimated.
//
//  Output:
//
//    double R8MAT_JAC[M*N], the estimated jacobian matrix.
//
{
  double del;
  double *fprime;
  int i;
  int j;
  double xsave;
  double *work1;
  double *work2;

  fprime = new double[m*n];
//
//  Evaluate the function at the base point, X.
//
  work2 = fx ( m, n, x );
//
//  Now, one by one, vary each component J of the base point X, and
//  estimate DF(I)/DX(J) = ( F(X+) - F(X) )/ DEL.
//
  for ( j = 0; j < n; j++ )
  {
    xsave = x[j];
    del = eps * ( 1.0 + fabs ( x[j] ) );
    x[j] = x[j] + del;
    work1 = fx ( m, n, x );
    x[j] = xsave;
    for ( i = 0; i < m; i++ )
    {
      fprime[i+j*m] = ( work1[i] - work2[i] ) / del;
    }
    delete [] work1;
  }
  delete [] work2;

  return fprime;
}
//****************************************************************************80

double *r8mat_kronecker ( int m1, int n1, double a[], int m2, int n2, 
  double b[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_kronecker() computes the Kronecker product of two R8MAT's.
//
//  Discussion:
//
//    An R8MAT is an MxN array of R8's, stored by (I,J) -> [I+J*M].
//
//    If A is an M1 by N1 array, and B is an M2 by N2 array, then
//    the Kronecker product of A and B is an M1*M2 by N1*N2 array
//      C(I,J) = A(I1,J1) * B(I2,J2)
//    where
//      I1 =       I   / M2
//      I2 = mod ( I,    M2 )
//      J1 =       J   / N2
//      J2 = mod ( J,    N2 )
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    01 December 2013
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M1, N1, the order of the first matrix.
//
//    double A[M1*N1], the first matrix.
//
//    int M2, N2, the order of the second matrix.
//
//    double B[M2*N2], the second matrix.
//
//  Output:
//
//    double R8MAT_KRONECKER[(M1*M2)*(N1*N2)], the Kronecker product.
//
{
  double *c;
  int i;
  int i0;
  int i1;
  int i2;
  int j;
  int j0;
  int j1;
  int j2;
  int m;
  int n;

  m = m1 * m2;
  n = n1 * n2;
  c = new double[m*n];

  for ( j1 = 0; j1 < n1; j1++ )
  {
    for ( i1 = 0; i1 < m1; i1++ )
    {
      i0 = i1 * m2;
      j0 = j1 * n2;
      j = j0;
      for ( j2 = 0; j2 < n2; j2++ )
      {
        i = i0;
        for ( i2 = 0; i2 < m2; i2++ )
        {
          c[i+j*m] = a[i1+j1*m1] * b[i2+j2*m2];
          i = i + 1;
        }
        j = j + 1;
      }
    }
  }

  return c;
}
//****************************************************************************80

double *r8mat_l_inverse ( int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_l_inverse() inverts a lower triangular R8MAT.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    A lower triangular matrix is a matrix whose only nonzero entries
//    occur on or below the diagonal.
//
//    The inverse of a lower triangular matrix is a lower triangular matrix.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    06 September 2005
//
//  Author:
//
//    John Burkardt
//
//  Reference:
//
//    Albert Nijenhuis, Herbert Wilf,
//    Combinatorial Algorithms,
//    Academic Press, 1978, second edition,
//    ISBN 0-12-519260-6.
//
//  Input:
//
//    int N, number of rows and columns in the matrix.
//
//    double A[N*N], the lower triangular matrix.
//
//  Output:
//
//    double R8MAT_L_INVERSE[N*N], the inverse matrix.
//
{
  double *b;
  int i;
  int j;
  int k;
  double temp;

  b = new double[n*n];

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < n; i++ )
    {
      if ( i < j )
      {
        b[i+j*n] = 0.0;
      }
      else if ( j == i )
      {
        b[i+j*n] = 1.0 / a[i+j*n];
      }
      else
      {
        temp = 0.0;
        for ( k = 0; k < i; k++ )
        {
          temp = temp + a[i+k*n] * b[k+j*n];
        }
        b[i+j*n] = -temp / a[i+i*n];
      }
    }
  }

  return b;
}
//****************************************************************************80

void r8mat_l_print ( int m, int n, double a[], string title )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_l_print() prints a lower triangular R8MAT.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Example:
//
//    M = 5, N = 5
//    A = (/ 11, 21, 31, 41, 51, 22, 32, 42, 52, 33, 43, 53, 44, 54, 55 /)
//
//    11
//    21 22
//    31 32 33
//    41 42 43 44
//    51 52 53 54 55
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    19 January 2016
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, the number of rows in A.
//
//    int N, the number of columns in A.
//
//    double A[*], the M by N matrix.  Only the lower
//    triangular elements are stored, in column major order.
//
//    string TITLE, a title.
//
{
  int i;
  int indx[10];
  int j;
  int jhi;
  int jlo;
  int jmax;
  int nn;
  int size;

  cout << "\n";
  cout << title << "\n";

  jmax = min ( n, m );

  if ( m <= n )
  {
    size = ( m * ( m + 1 ) ) / 2;
  }
  else
  {
    size = ( n * ( n + 1 ) ) / 2 + ( m - n ) * n;
  }

  if ( r8vec_is_integer ( size, a ) )
  {
    nn = 10;
    for ( jlo = 1; jlo <= jmax; jlo = jlo + nn )
    {
      jhi = min ( jlo + nn - 1, min ( m, jmax ) );
      cout << "\n";
      cout << "  Col   ";
      for ( j = jlo; j <= jhi; j++ )
      {
        cout << setw(6) << j;
      }
      cout << "\n";
      cout << "  Row  \n";
      for ( i = jlo; i <= m; i++ )
      {
        jhi = min ( jlo + nn - 1, min ( i, jmax ) );
        for ( j = jlo; j <= jhi; j++ )
        {
          indx[j-jlo] = ( j - 1 ) * m + i - ( j * ( j - 1 ) ) / 2;
        }
        cout << "  " << setw(6) << i;
        for ( j = 0; j <= jhi-jlo; j++ )
        {
          cout << setw(6) << a[indx[j]-1];
        }
        cout << "\n";
      }
    }
  }
  else if ( r8vec_amax ( size, a ) < 1000000.0 )
  {
    nn = 5;
    for ( jlo = 1; jlo <= jmax; jlo = jlo + nn )
    {
      jhi = min ( jlo + nn - 1, min ( m - 1, jmax ) );
      cout << "\n";
      cout << "  Col ";
      for ( j = jlo; j <= jhi; j++ )
      {
        cout << setw(14) << j;
      }
      cout << "\n";
      cout << "  Row  \n";
      for ( i = jlo; i <= m; i++ )
      {
        jhi = min ( jlo + nn - 1, min ( i, jmax ) );
        for ( j = jlo; j <= jhi; j++ )
        {
          indx[j-jlo] = ( j - 1 ) * m + i - ( j * ( j - 1 ) ) / 2;
        }
        cout << "  " << setw(6) << i;
        for ( j = 0; j <= jhi-jlo; j++ )
        {
          cout << setw(14) << a[indx[j]-1];
        }
        cout << "\n";
      }
    }
  }
  else
  {
    nn = 5;

    for ( jlo = 1; jlo <= jmax; jlo = jlo + nn )
    {
      jhi = min ( jlo + nn - 1, min ( m - 1, jmax ) );
      cout << "\n";
      cout << "  Col ";
      for ( j = jlo; j <= jhi; j++ )
      {
        cout << setw(7) << j << "       ";
      }
      cout << "\n";
      cout << "  Row \n";
      for ( i = jlo; i <= m; i++ )
      {
        jhi = min ( jlo + nn - 1, min ( i, jmax ) );
        for ( j = jlo; j <= jhi; j++ )
        {
          indx[j-jlo] = ( j - 1 ) * m + i - ( j * ( j - 1 ) ) / 2;
        }
        cout << setw(6) << i;
        for ( j = 0; j <= jhi-jlo; j++ )
        {
          cout << setw(14) << a[indx[j]-1];
        }
      }
    }
  }

  return;
}
//****************************************************************************80

double *r8mat_l_solve ( int n, double a[], double b[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_l_solve() solves a lower triangular linear system.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    12 October 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of rows and columns of the matrix A.
//
//    double A[N*N], the N by N lower triangular matrix.
//
//    double B[N], the right hand side of the linear system.
//
//  Output:
//
//    double R8MAT_L_SOLVE[N], the solution of the linear system.
//
{
  int i;
  int j;
  double temp;
  double *x;

  x = new double[n];
//
//  Solve L * x = b.
//
  for ( i = 0; i < n; i++ )
  {
    temp = 0.0;
    for ( j = 0; j < i; j++ )
    {
      temp = temp + a[i+j*n] * x[j];
    }
    x[i] = ( b[i] - temp ) / a[i+i*n];
  }

  return x;
}
//****************************************************************************80

double *r8mat_l1_inverse ( int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_l1_inverse() inverts a unit lower triangular R8MAT.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    A unit lower triangular matrix is a matrix with only 1's on the main
//    diagonal, and only 0's above the main diagonal.
//
//    The inverse of a unit lower triangular matrix is also
//    a unit lower triangular matrix.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    14 September 2005
//
//  Author:
//
//    John Burkardt
//
//  Reference:
//
//    Albert Nijenhuis, Herbert Wilf,
//    Combinatorial Algorithms,
//    Academic Press, 1978, second edition,
//    ISBN 0-12-519260-6.
//
//  Input:
//
//    int N, number of rows and columns in the matrix.
//
//    double A[N*N], the unit lower triangular matrix.
//
//  Output:
//
//    double R8MAT_L1_INVERSE[N*N], the inverse matrix.
//
{
  double *b;
  int i;
  int j;
  int k;

  b = new double[n*n];

  for ( i = 0; i < n; i++ )
  {
    for ( j = 0; j < n; j++ )
    {
      if ( i < j )
      {
        b[i+j*n] = 0.0;
      }
      else if ( j == i )
      {
        b[i+j*n] = 1.0;
      }
      else
      {
        b[i+j*n] = 0.0;
        for ( k = 0; k < i; k++ )
        {
          b[i+j*n] = b[i+j*n] - a[i+k*n] * b[k+j*n];
        }
      }
    }
  }

  return b;
}
//****************************************************************************80

double *r8mat_lt_solve ( int n, double a[], double b[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_lt_solve() solves a transposed lower triangular linear system.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    Given the lower triangular matrix A, the linear system to be solved is:
//
//      A' * x = b
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    14 September 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of rows and columns of the matrix A.
//
//    double A[N*N], the N by N lower triangular matrix.
//
//    double B[N], the right hand side of the linear system.
//
//  Output:
//
//    double R8MAT_LT_SOLVE[N], the solution of the linear system.
//
{
  int i;
  int j;
  double *x;

  x = new double[n];

  for ( j = n-1; 0 <= j; j-- )
  {
    x[j] = b[j];
    for ( i = j+1; i < n; i++ )
    {
      x[j] = x[j] - x[i] * a[i+j*n];
    }
    x[j] = x[j] / a[j+j*n];
  }

  return x;
}
//****************************************************************************80

void r8mat_lu ( int m, int n, double a[], double l[], double p[], double u[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_lu() computes the LU factorization of a rectangular R8MAT.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    The routine is given an M by N matrix A, and produces
//
//      L, an M by M unit lower triangular matrix,
//      U, an M by N upper triangular matrix, and
//      P, an M by M permutation matrix P,
//
//    so that
//
//      A = P' * L * U.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    09 November 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, the number of rows in A.
//
//    int N, the number of columns in A.
//
//    double A[M*N], the M by N matrix to be factored.
//
//  Output:
//
//    double L[M*M], the M by M unit lower triangular factor.
//
//    double P[M*M], the M by M permutation matrix.
//
//    double U[M*N], the M by N upper triangular factor.
//
{
  int i;
  int ipiv;
  int j;
  int k;
  double pivot;
  double t;
//
//  Initialize:
//
//    U:=A
//    L:=Identity
//    P:=Identity
//
  r8mat_copy ( m, n, a, u );

  r8mat_zeros ( m, m, l );
  r8mat_zeros ( m, m, p );
  for ( i = 0; i < m; i++ )
  {
    l[i+i*m] = 1.0;
    p[i+i*m] = 1.0;
  }
//
//  On step J, find the pivot row, IPIV, and the pivot value PIVOT.
//
  for ( j = 0; j < min ( m - 1, n ); j++ )
  {
    pivot = 0.0;
    ipiv = -1;

    for ( i = j; i < m; i++ )
    {
      if ( pivot < fabs ( u[i+j*m] ) )
      {
        pivot = fabs ( u[i+j*m] );
        ipiv = i;
      }
    }
//
//  Unless IPIV is zero, swap rows J and IPIV.
//
    if ( ipiv != -1 )
    {
      for ( k = 0; k < n; k++ )
      {
        t = u[j+k*m];
        u[j+k*m] = u[ipiv+j*m];
        u[ipiv+k*m] = t;

        t = l[j+k*m];
        l[j+k*m] = l[ipiv+j*m];
        l[ipiv+k*m] = t;

        t = p[j+k*m];
        p[j+k*m] = p[ipiv+j*m];
        p[ipiv+k*m] = t;
      }
//
//  Zero out the entries in column J, from row J+1 to M.
//
      for ( i = j+1; i < m; i++ )
      {
        if ( u[i+j*m] != 0.0 )
        {
          l[i+j*m] = u[i+j*m] / u[j+j*m];

          u[i+j*m] = 0.0;

          for ( k = j+1; k < n; k++ )
          {
            u[i+k*m] = u[i+k*m] - l[i+j*m] * u[j+k*m];
          }
        }
      }
    }
  }

  return;
}
//****************************************************************************80

double r8mat_max ( int m, int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_max() returns the maximum entry of an R8MAT.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    21 May 2011
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, the number of rows in A.
//
//    int N, the number of columns in A.
//
//    double A[M*N], the M by N matrix.
//
//  Output:
//
//    double R8MAT_MAX, the maximum entry of A.
//
{
  int i;
  int j;
  double value;

  value = a[0+0*m];

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      if ( value < a[i+j*m] )
      {
        value = a[i+j*m];
      }
    }
  }
  return value;
}
//****************************************************************************80

double *r8mat_max_columns ( int m, int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_max_columns() returns the column maximums of an R8MAT.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    04 October 2018
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, the number of rows in A.
//
//    int N, the number of columns in A.
//
//    double A[M*N], the M by N matrix.
//
//  Output:
//
//    double R8MAT_MAX_COLUMNS[N], the column maximums.
//
{
  int i;
  int j;
  double *max_columns;
  double value;

  max_columns = new double[n];

  for ( j = 0; j < n; j++ )
  {
    value = a[0+j*m];
    for ( i = 1; i < m; i++ )
    {
      if ( value < a[i+j*m] )
      {
        value = a[i+j*m];
      }
    }
    max_columns[j] = value;
  }

  return max_columns;
}
//****************************************************************************80

void r8mat_max_index ( int m, int n, double a[], int &i_max, int &j_max )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_max_index() returns the location of the maximum entry of an R8MAT.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    18 September 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, the number of rows in A.
//
//    int N, the number of columns in A.
//
//    double A[M*N], the M by N matrix.
//
//  Output:
//
//    int &I_MAX, &J_MAX, the indices of the maximum entry of A.
//
{
  int i;
  int i2;
  int j;
  int j2;

  i2 = -1;
  j2 = -1;

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      if ( i2 == -1 && j2 == -1 )
      {
        i2 = i;
        j2 = j;
      }
      else if ( a[i2+j2*m] < a[i+j*m] )
      {
        i2 = i;
        j2 = j;
      }
    }
  }

  i_max = i2 + 1;
  j_max = j2 + 1;

  return;
}
//****************************************************************************80

double *r8mat_max_rows ( int m, int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_max_rows() returns the row maximums of an R8MAT.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    04 October 2018
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, the number of rows in A.
//
//    int N, the number of columns in A.
//
//    double A[M*N], the M by N matrix.
//
//  Output:
//
//    double R8MAT_MAX_ROWS[M], the row maximums.
//
{
  int i;
  int j;
  double *max_rows;
  double value;

  max_rows = new double[m];

  for ( i = 0; i < m; i++ )
  {
    value = a[i+0*m];
    for ( j = 1; j < n; j++ )
    {
      if ( value < a[i+j*m] )
      {
        value = a[i+j*m];
      }
    }
    max_rows[i] = value;
  }

  return max_rows;
}
//****************************************************************************80

double r8mat_maxcol_minrow ( int m, int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_maxcol_minrow() gets the maximum column minimum row of an M by N matrix.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    R8MAT_MAXCOL_MINROW = max ( 1 <= I <= N ) ( min ( 1 <= J <= M ) A(I,J) )
//
//    For a given matrix, R8MAT_MAXCOL_MINROW <= R8MAT_MINROW_MAXCOL.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    16 October 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, the number of rows in A.
//
//    int N, the number of columns in A.
//
//    double A[M*N], the matrix.
//
//  Output:
//
//    double R8MAT_MAXCOL_MINROW, the maximum column
//    minimum row entry of A.
//
{
  int i;
  int j;
  double minrow;
  double value;

  value = - HUGE_VAL;

  for ( i = 0; i < m; i++ )
  {
    minrow = HUGE_VAL;

    for ( j = 0; j < n; j++ )
    {
      minrow = fmin ( minrow, a[i+j*m] );
    }
    value = fmax ( value, minrow );
  }

  return value;
}
//****************************************************************************80

double r8mat_maxrow_mincol ( int m, int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_maxrow_mincol() gets the maximum row minimum column of an M by N matrix.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    R8MAT_MAXROW_MINCOL = max ( 1 <= J <= N ) ( min ( 1 <= I <= M ) A(I,J) )
//
//    For a given matrix, R8MAT_MAXROW_MINCOL <= R8MAT_MINCOL_MAXROW.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    17 October 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, the number of rows in A.
//
//    int N, the number of columns in A.
//
//    double A[M*N], the matrix.
//
//  Output:
//
//    double R8MAT_MAXROW_MINCOL, the maximum row
//    minimum column entry of A.
//
{
  int i;
  int j;
  double mincol;
  double value;

  value = - HUGE_VAL;

  for ( j = 0; j < n; j++ )
  {
    mincol = HUGE_VAL;
    for ( i = 0; i < m; i++ )
    {
      mincol = fmin ( mincol, a[i+j*m] );
    }
    value = fmax ( value, mincol );
  }
  return value;
}
//****************************************************************************80

double r8mat_mean ( int m, int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_mean() returns the mean of an R8MAT.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    03 September 2013
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, the number of rows in A.
//
//    int N, the number of columns in A.
//
//    double A[M*N], the M by N matrix.
//
//  Output:
//
//    double R8MAT_MEAN, the mean of A.
//
{
  int i;
  int j;
  double value;

  value = 0.0;

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      value = value + a[i+j*m];
    }
  }
  value = value / ( double ) ( m * n );

  return value;
}
//****************************************************************************80

double *r8mat_mean_columns ( int m, int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_mean_columns() returns the column means of an R8MAT.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    04 October 2018
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, the number of rows in A.
//
//    int N, the number of columns in A.
//
//    double A[M*N], the M by N matrix.
//
//  Output:
//
//    double R8MAT_MEAN_COLUMNS[N], the column means.
//
{
  int i;
  int j;
  double *mean_columns;
  double value;

  mean_columns = new double[n];

  for ( j = 0; j < n; j++ )
  {
    value = 0.0;
    for ( i = 0; i < m; i++ )
    {
      value = value + a[i+j*m];
    }
    mean_columns[j] = value / ( double ) m;
  }

  return mean_columns;
}
//****************************************************************************80

double *r8mat_mean_rows ( int m, int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_mean_rows() returns the row means of an R8MAT.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    04 October 2018
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, the number of rows in A.
//
//    int N, the number of columns in A.
//
//    double A[M*N], the M by N matrix.
//
//  Output:
//
//    double R8MAT_MEAN_ROWS[M], the row means.
//
{
  int i;
  int j;
  double *mean_rows;
  double value;

  mean_rows = new double[m];

  for ( i = 0; i < m; i++ )
  {
    value = 0.0;
    for ( j = 0; j < n; j++ )
    {
      value = value + a[i+j*m];
    }
    mean_rows[i] = value / ( double ) n;
  }

  return mean_rows;
}
//****************************************************************************80

double r8mat_min ( int m, int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_min() returns the minimum entry of an R8MAT.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    21 May 2011
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, the number of rows in A.
//
//    int N, the number of columns in A.
//
//    double A[M*N], the M by N matrix.
//
//  Output:
//
//    double R8MAT_MIN, the minimum entry of A.
//
{
  int i;
  int j;
  double value;

  value = a[0+0*m];

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      if ( a[i+j*m] < value )
      {
        value = a[i+j*m];
      }
    }
  }
  return value;
}
//****************************************************************************80

double *r8mat_min_columns ( int m, int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_min_columns() returns the column minimums of an R8MAT.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    04 October 2018
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, the number of rows in A.
//
//    int N, the number of columns in A.
//
//    double A[M*N], the M by N matrix.
//
//  Output:
//
//    double R8MAT_MIN_COLUMNS[N], the column minimums.
//
{
  int i;
  int j;
  double *min_columns;
  double value;

  min_columns = new double[n];

  for ( j = 0; j < n; j++ )
  {
    value = a[0+j*m];
    for ( i = 1; i < m; i++ )
    {
      if ( a[i+j*m] < value )
      {
        value = a[i+j*m];
      }
    }
    min_columns[j] = value;
  }

  return min_columns;
}
//****************************************************************************80

void r8mat_min_index ( int m, int n, double a[], int &i_min, int &j_min )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_min_index() returns the location of the minimum entry of an R8MAT.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    18 September 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, the number of rows in A.
//
//    int N, the number of columns in A.
//
//    double A[M*N], the M by N matrix.
//
//  Output:
//
//    int &I_MIN, &J_MIN, the indices of the minimum entry of A.
//
{
  int i;
  int i2;
  int j;
  int j2;

  i2 = -1;
  j2 = -1;

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      if ( i2 == -1 && j2 == -1 )
      {
        i2 = i;
        j2 = j;
      }
      else if ( a[i+j*m] < a[i2+j2*m] )
      {
        i2 = i;
        j2 = j;
      }
    }
  }

  i_min = i2 + 1;
  j_min = j2 + 1;

  return;
}
//****************************************************************************80

double *r8mat_min_rows ( int m, int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_min_rows() returns the row minimums of an R8MAT.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    04 October 2018
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, the number of rows in A.
//
//    int N, the number of columns in A.
//
//    double A[M*N], the M by N matrix.
//
//  Output:
//
//    double R8MAT_MIN_ROWS[M], the row minimums.
//
{
  int i;
  int j;
  double *min_rows;
  double value;

  min_rows = new double[m];

  for ( i = 0; i < m; i++ )
  {
    value = a[i+0*m];
    for ( j = 1; j < n; j++ )
    {
      if ( a[i+j*m] < value )
      {
        value = a[i+j*m];
      }
    }
    min_rows[i] = value;
  }

  return min_rows;
}
//****************************************************************************80

double r8mat_mincol_maxrow ( int m, int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_mincol_maxrow() gets the minimum column maximum row of an M by N matrix.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    R8MAT_MINCOL_MAXROW = min ( 1 <= I <= N ) ( max ( 1 <= J <= M ) A(I,J) )
//
//    For a given matrix, R8MAT_MAXROW_MINCOL <= R8MAT_MINCOL_MAXROW.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    17 October 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, the number of rows in A.
//
//    int N, the number of columns in A.
//
//    double A(M,N), the matrix.
//
//  Output:
//
//    double R8MAT_MINCOL_MAXROW, the minimum column
//    maximum row entry of A.
//
{
  int i;
  int j;
  double maxrow;
  double value;

  value = HUGE_VAL;

  for ( i = 0; i < m; i++ )
  {
    maxrow = - HUGE_VAL;
    for ( j = 0; j < n; j++ )
    {
      maxrow = fmax ( maxrow, a[i+j*m] );
    }
    value = fmin ( value, maxrow );
  }

  return value;
}
//****************************************************************************80

double r8mat_minrow_maxcol ( int m, int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_minrow_maxcol() gets the minimum row maximum column of an M by N matrix.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    R8MAT_MINROW_MAXCOL = min ( 1 <= J <= N ) ( max ( 1 <= I <= M ) A(I,J) )
//
//    For a given matrix, R8MAT_MAXCOL_MINROW <= R8MAT_MINROW_MAXCOL.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    17 October 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, the number of rows in A.
//
//    int N, the number of columns in A.
//
//    double A[M*N], the matrix.
//
//  Output:
//
//    double R8MAT_MINROW_MAXCOL, the minimum row
//    maximum column entry of A.
//
{
  int i;
  int j;
  double maxcol;
  double value;

  value = HUGE_VAL;

  for ( j = 0; j < n; j++ )
  {
    maxcol = - HUGE_VAL;
    for ( i = 0; i < m; i++ )
    {
      maxcol = fmax ( maxcol, a[i+j*m] );
    }
    value = fmin ( value, maxcol );
  }

  return value;
}
//****************************************************************************80

void r8mat_minvm ( int n1, int n2, double a[], double b[], double c[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_minvm() computes inverse(A) * B for R8MAT's.
//
//  Discussion:
//
//    An R8MAT is an array of R8 values.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    01 December 2011
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N1, N2, the order of the matrices.
//
//    double A[N1*N1], B[N1*N2], the matrices.
//
//  Output:
//
//    double C[N1*N2], the result, C = inverse(A) * B.
//
{
  double *alu;
  double *d;

  alu = r8mat_copy_new ( n1, n1, a );

  d = r8mat_fss_new ( n1, alu, n2, b );

  r8mat_copy ( n1, n2, d, c );

  delete [] alu;
  delete [] d;

  return;
}
//****************************************************************************80

double *r8mat_minvm_new ( int n1, int n2, double a[], double b[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_minvm_new() returns inverse(A) * B for R8MAT's.
//
//  Discussion:
//
//    An R8MAT is an array of R8 values.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    28 November 2011
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N1, N2, the order of the matrices.
//
//    double A[N1*N1], B[N1*N2], the matrices.
//
//  Output:
//
//    double R8MAT_MINVM_NEW[N1*N2], the result, C = inverse(A) * B.
//
{
  double *alu;
  double *c;

  alu = r8mat_copy_new ( n1, n1, a );
  c = r8mat_fss_new ( n1, alu, n2, b );
 
  delete [] alu;

  return c;
}
//****************************************************************************80

void r8mat_mm ( int n1, int n2, int n3, double a[], double b[], double c[] )

//****************************************************************************80
//
//  Purpose:
//
//    R8MAT_MM multiplies two matrices.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    For this routine, the result is returned as the function value.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    02 December 2011
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N1, N2, N3, the order of the matrices.
//
//    double A[N1*N2], double B[N2*N3], the matrices to multiply.
//
//  Output:
//
//    double C[N1*N3], the product matrix C = A * B.
//
{
  double *c1;
  int i;
  int j;
  int k;

  c1 = new double[n1*n3];

  for ( i = 0; i < n1; i++ )
  {
    for ( j = 0; j < n3; j++ )
    {
      c1[i+j*n1] = 0.0;
      for ( k = 0; k < n2; k++ )
      {
        c1[i+j*n1] = c1[i+j*n1] + a[i+k*n1] * b[k+j*n2];
      }
    }
  }

  r8mat_copy ( n1, n3, c1, c );

  delete [] c1;

  return;
}
//****************************************************************************80

double *r8mat_mm_new ( int n1, int n2, int n3, double a[], double b[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_mm_new() multiplies two matrices.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    For this routine, the result is returned as the function value.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    18 October 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N1, N2, N3, the order of the matrices.
//
//    double A[N1*N2], double B[N2*N3], the matrices to multiply.
//
//  Output:
//
//    double R8MAT_MM_NEW[N1*N3], the product matrix C = A * B.
//
{
  double *c;
  int i;
  int j;
  int k;

  c = new double[n1*n3];

  for ( i = 0; i < n1; i++ )
  {
    for ( j = 0; j < n3; j++ )
    {
      c[i+j*n1] = 0.0;
      for ( k = 0; k < n2; k++ )
      {
        c[i+j*n1] = c[i+j*n1] + a[i+k*n1] * b[k+j*n2];
      }
    }
  }

  return c;
}
//****************************************************************************80

double *r8mat_mmt_new ( int n1, int n2, int n3, double a[], double b[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_mmt_new() computes C = A * B'.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    For this routine, the result is returned as the function value.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    13 November 2012
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N1, N2, N3, the order of the matrices.
//
//    double A[N1*N2], double B[N3*N2], the matrices to multiply.
//
//  Output:
//
//    double R8MAT_MMT_NEW[N1*N3], the product matrix C = A * B'.
//
{
  double *c;
  int i;
  int j;
  int k;

  c = new double[n1*n3];

  for ( i = 0; i < n1; i++ )
  {
    for ( j = 0; j < n3; j++ )
    {
      c[i+j*n1] = 0.0;
      for ( k = 0; k < n2; k++ )
      {
        c[i+j*n1] = c[i+j*n1] + a[i+k*n1] * b[j+k*n3];
      }
    }
  }

  return c;
}
//****************************************************************************80

void r8mat_mtm ( int n1, int n2, int n3, double a[], double b[], double c[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_mtm() computes C = A' * B.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    For this routine, the result is returned as the function value.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    05 June 2024
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N1, N2, N3, the order of the matrices.
//
//    double A[N2*N1], double B[N2*N3], the matrices to multiply.
//
//  Output:
//
//    double C[N1*N3], the product matrix C = A' * B.
//
{
  int i;
  int j;
  int k;

  for ( i = 0; i < n1; i++ )
  {
    for ( j = 0; j < n3; j++ )
    {
      c[i+j*n1] = 0.0;
      for ( k = 0; k < n2; k++ )
      {
        c[i+j*n1] = c[i+j*n1] + a[k+i*n2] * b[k+j*n2];
      }
    }
  }

  return;
}
//****************************************************************************80

double *r8mat_mtm_new ( int n1, int n2, int n3, double a[], double b[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_mtm_new() computes C = A' * B.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    For this routine, the result is returned as the function value.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    07 September 2012
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N1, N2, N3, the order of the matrices.
//
//    double A[N2*N1], double B[N2*N3], the matrices to multiply.
//
//  Output:
//
//    double R8MAT_MTM_NEW[N1*N3], the product matrix C = A' * B.
//
{
  double *c;
  int i;
  int j;
  int k;

  c = new double[n1*n3];

  for ( i = 0; i < n1; i++ )
  {
    for ( j = 0; j < n3; j++ )
    {
      c[i+j*n1] = 0.0;
      for ( k = 0; k < n2; k++ )
      {
        c[i+j*n1] = c[i+j*n1] + a[k+i*n2] * b[k+j*n2];
      }
    }
  }

  return c;
}
//****************************************************************************80

void r8mat_mtv ( int m, int n, double a[], double x[], double atx[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_mtv() multiplies a transposed matrix times a vector.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    For this routine, the result is returned as an argument.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    12 April 2007
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, N, the number of rows and columns of the matrix.
//
//    double A[M,N], the M by N matrix.
//
//    double X[M], the vector to be multiplied by A.
//
//  Output:
//
//    double ATX[N], the product A'*X.
//
{
  int i;
  int j;
  double *y;

  y = new double[n];

  for ( j = 0; j < n; j++ )
  {
    y[j] = 0.0;
    for ( i = 0; i < m; i++ )
    {
      y[j] = y[j] + a[i+j*m] * x[i];
    }
  }

  r8vec_copy ( n, y, atx );

  delete[] y;

  return;
}
//****************************************************************************80

double *r8mat_mtv_new ( int m, int n, double a[], double x[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_mtv_new() multiplies a transposed matrix times a vector.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    For this routine, the result is returned as the function value.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    11 April 2007
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, N, the number of rows and columns of the matrix.
//
//    double A[M,N], the M by N matrix.
//
//    double X[M], the vector to be multiplied by A.
//
//  Output:
//
//    double R8MAT_MTV_NEW[N], the product A'*X.
//
{
  int i;
  int j;
  double *y;

  y = new double[n];

  for ( j = 0; j < n; j++ )
  {
    y[j] = 0.0;
    for ( i = 0; i < m; i++ )
    {
      y[j] = y[j] + a[i+j*m] * x[i];
    }
  }

  return y;
}
//****************************************************************************80

void r8mat_mv ( int m, int n, double a[], double x[], double ax[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_mv() multiplies a matrix times a vector.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    For this routine, the result is returned as an argument.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    11 April 2007
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, N, the number of rows and columns of the matrix.
//
//    double A[M,N], the M by N matrix.
//
//    double X[N], the vector to be multiplied by A.
//
//  Output:
//
//    double AX[M], the product A*X.
//
{
  int i;
  int j;
  double *y;

  y = new double[m];

  for ( i = 0; i < m; i++ )
  {
    y[i] = 0.0;
    for ( j = 0; j < n; j++ )
    {
      y[i] = y[i] + a[i+j*m] * x[j];
    }
  }

  r8vec_copy ( m, y, ax );

  delete [] y;

  return;
}
//****************************************************************************80

double *r8mat_mv_new ( int m, int n, double a[], double x[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_mv_new() multiplies a matrix times a vector.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    For this routine, the result is returned as the function value.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    11 April 2007
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, N, the number of rows and columns of the matrix.
//
//    double A[M,N], the M by N matrix.
//
//    double X[N], the vector to be multiplied by A.
//
//  Output:
//
//    double R8MAT_MV_NEW[M], the product A*X.
//
{
  int i;
  int j;
  double *y;

  y = new double[m];

  for ( i = 0; i < m; i++ )
  {
    y[i] = 0.0;
    for ( j = 0; j < n; j++ )
    {
      y[i] = y[i] + a[i+j*m] * x[j];
    }
  }

  return y;
}
//****************************************************************************80

void r8mat_nint ( int m, int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_nint() rounds the entries of an R8MAT.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    18 August 2014
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, N, the number of rows and columns.
//
//    double A[M*N], the matrix.
//
//  Output:
//
//    double A[M*N]: the modified matrix.
{
  int i;
  int j;
  int s;

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      if ( a[i+j*m] < 0.0 )
      {
        s = -1;
      }
      else
      {
        s = 1;
      }
      a[i+j*m] = s * ( int ) ( fabs ( a[i+j*m] ) + 0.5 );
    }
  }

  return;
}
//****************************************************************************80

int r8mat_nonzeros ( int m, int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_nonzeros() returns the number of nonzeros in an R8MAT.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    31 August 2014
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, N, the number of rows and columns.
//
//    double A[M*N], the matrix.
//
//  Output:
//
//    int R8MAT_NONZEROS, the number of nonzeros.
//
{
  int i;
  int j;
  int value;

  value = 0;

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      if ( a[i+j*m] != 0.0 )
      {
        value = value + 1;
      }
    }
  }

  return value;
}
//****************************************************************************80

double r8mat_norm_eis ( int m, int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_norm_eis() returns the EISPACK norm of an R8MAT.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    The EISPACK norm is defined as:
//
//      R8MAT_NORM_EIS =
//        sum ( 1 <= I <= M ) sum ( 1 <= J <= N ) abs ( A(I,J) )
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    18 September 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, the number of rows in A.
//
//    int N, the number of columns in A.
//
//    double A[M*N], the matrix whose EISPACK norm is desired.
//
//  Output:
//
//    double R8MAT_NORM_EIS, the EISPACK norm of A.
//
{
  int i;
  int j;
  double value;

  value = 0.0;
  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      value = value + fabs ( a[i+j*m] );
    }
  }

  return value;
}
//****************************************************************************80

double r8mat_norm_fro ( int m, int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_norm_fro() returns the Frobenius norm of an R8MAT.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    The Frobenius norm is defined as
//
//      R8MAT_NORM_FRO = sqrt (
//        sum ( 1 <= I <= M ) sum ( 1 <= j <= N ) A(I,J)^2 )
//    The matrix Frobenius norm is not derived from a vector norm, but
//    is compatible with the vector L2 norm, so that:
//
//      r8vec_norm_l2 ( A * x ) <= r8mat_norm_fro ( A ) * r8vec_norm_l2 ( x ).
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    10 October 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, the number of rows in A.
//
//    int N, the number of columns in A.
//
//    double A[M*N], the matrix whose Frobenius
//    norm is desired.
//
//  Output:
//
//    double R8MAT_NORM_FRO, the Frobenius norm of A.
//
{
  int i;
  int j;
  double value;

  value = 0.0;
  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      value = value + pow ( a[i+j*m], 2 );
    }
  }
  value = sqrt ( value );

  return value;
}
//****************************************************************************80

double r8mat_norm_fro_affine ( int m, int n, double a1[], double a2[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_norm_fro_affine() returns the Frobenius norm of an R8MAT difference.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    The Frobenius norm is defined as
//
//      R8MAT_NORM_FRO = sqrt (
//        sum ( 1 <= I <= M ) sum ( 1 <= j <= N ) A(I,J)^2 )
//    The matrix Frobenius norm is not derived from a vector norm, but
//    is compatible with the vector L2 norm, so that:
//
//      r8vec_norm_l2 ( A * x ) <= r8mat_norm_fro ( A ) * r8vec_norm_l2 ( x ).
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    26 September 2012
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, the number of rows.
//
//    int N, the number of columns.
//
//    double A1[M*N], A2[M,N], the matrice for whose difference the 
//    Frobenius norm is desired.
//
//  Output:
//
//    double R8MAT_NORM_FRO_AFFINE, the Frobenius norm of A1 - A2.
//
{
  int i;
  int j;
  double value;

  value = 0.0;
  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      value = value + pow ( a1[i+j*m] - a2[i+j*m], 2 );
    }
  }
  value = sqrt ( value );

  return value;
}
//****************************************************************************80

double r8mat_norm_l1 ( int m, int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_norm_l1() returns the matrix L1 norm of an R8MAT.
//
//  Discussion:
//
//    An R8MAT is an array of R8 values.
//
//    The matrix L1 norm is defined as:
//
//      R8MAT_NORM_L1 = max ( 1 <= J <= N )
//        sum ( 1 <= I <= M ) abs ( A(I,J) ).
//
//    The matrix L1 norm is derived from the vector L1 norm, and
//    satisifies:
//
//      r8vec_norm_l1 ( A * x ) <= r8mat_norm_l1 ( A ) * r8vec_norm_l1 ( x ).
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    01 December 2011
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, the number of rows in A.
//
//    int N, the number of columns in A.
//
//    double A(M,N), the matrix whose L1 norm is desired.
//
//  Output:
//
//    double R8MAT_NORM_L1, the L1 norm of A.
//
{
  double col_sum;
  int i;
  int j;
  double value;

  value = 0.0;

  for ( j = 0; j < n; j++ )
  {
    col_sum = 0.0;
    for ( i = 0; i < m; i++ )
    {
      col_sum = col_sum + fabs ( a[i+j*m] );
    }
    value = fmax ( value, col_sum );
  }
  return value;
}
//****************************************************************************80

double r8mat_norm_l2 ( int m, int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_norm_l2() returns the matrix L2 norm of an R8MAT.
//
//  Discussion:
//
//    An R8MAT is an array of R8 values.
//
//    The matrix L2 norm is defined as:
//
//      R8MAT_NORM_L2 = sqrt ( max ( 1 <= I <= M ) LAMBDA(I) )
//
//    where LAMBDA contains the eigenvalues of A * A'.
//
//    The matrix L2 norm is derived from the vector L2 norm, and
//    satisifies:
//
//      r8vec_norm_l2 ( A * x ) <= r8mat_norm_l2 ( A ) * r8vec_norm_l2 ( x ).
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    01 December 2011
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, the number of rows in A.
//
//    int N, the number of columns in A.
//
//    double A(M,N), the matrix whose L2 norm is desired.
//
//  Output:
//
//    double R8MAT_NORM_L2, the L2 norm of A.
//
{
  double *at;
  double *b;
  double *diag;
  double value;

  at = r8mat_transpose_new ( m, n, a );
//
//  Compute B = A * A'.
//
  b = r8mat_mm_new ( m, n, m, a, at );
//
//  Diagonalize B.
//
  r8mat_symm_jacobi ( m, b );
//
//  Find the maximum eigenvalue, and take its square root.
//
  diag = r8mat_diag_get_vector_new ( m, b );

  value = sqrt ( r8vec_max ( m, diag ) );

  delete [] at;
  delete [] b;
  delete [] diag;

  return value;
}
//****************************************************************************80

double r8mat_norm_li ( int m, int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_norm_li() returns the matrix L-oo norm of an R8MAT.
//
//  Discussion:
//
//    An R8MAT is an array of R8 values.
//
//    The matrix L-oo norm is defined as:
//
//      R8MAT_NORM_LI =  max ( 1 <= I <= M ) sum ( 1 <= J <= N ) abs ( A(I,J) ).
//
//    The matrix L-oo norm is derived from the vector L-oo norm,
//    and satisifies:
//
//      r8vec_norm_li ( A * x ) <= r8mat_norm_li ( A ) * r8vec_norm_li ( x ).
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    01 December 2011
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, the number of rows in A.
//
//    int N, the number of columns in A.
//
//    double A[M*N], the matrix whose L-oo
//    norm is desired.
//
//  Output:
//
//    double R8MAT_NORM_LI, the L-oo norm of A.
//
{
  int i;
  int j;
  double row_sum;
  double value;

  value = 0.0;

  for ( i = 0; i < m; i++ )
  {
    row_sum = 0.0;
    for ( j = 0; j < n; j++ )
    {
      row_sum = row_sum + fabs ( a[i+j*m] );
    }
    value = fmax ( value, row_sum );
  }
  return value;
}
//****************************************************************************80

double *r8mat_normal_01_new ( int m, int n, int &seed )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_normal_01_new() returns a unit pseudonormal R8MAT.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    09 April 2012
//
//  Author:
//
//    John Burkardt
//
//  Reference:
//
//    Paul Bratley, Bennett Fox, Linus Schrage,
//    A Guide to Simulation,
//    Springer Verlag, pages 201-202, 1983.
//
//    Bennett Fox,
//    Algorithm 647:
//    Implementation and Relative Efficiency of Quasirandom
//    Sequence Generators,
//    ACM Transactions on Mathematical Software,
//    Volume 12, Number 4, pages 362-376, 1986.
//
//    Peter Lewis, Allen Goodman, James Miller,
//    A Pseudo-Random Number Generator for the System/360,
//    IBM Systems Journal,
//    Volume 8, pages 136-143, 1969.
//
//  Input:
//
//    int M, N, the number of rows and columns in the array.
//
//    int &SEED, the "seed" value, which should NOT be 0.
//
//  Output:
//
//    double R8MAT_NORMAL_01_NEW[M*N], the array of pseudonormal values.
//
//    int &SEED: an updated seed.
//
{
  double *r;

  r = r8vec_normal_01_new ( m * n, seed );

  return r;
}
//****************************************************************************80

double *r8mat_nullspace ( int m, int n, double a[], int nullspace_size )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_nullspace() computes the nullspace of a matrix.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    Let A be an MxN matrix.
//
//    If X is an N-vector, and A*X = 0, then X is a null vector of A.
//
//    The set of all null vectors of A is called the nullspace of A.
//
//    The 0 vector is always in the null space.
//
//    If the 0 vector is the only vector in the nullspace of A, then A
//    is said to have maximum column rank.  (Because A*X=0 can be regarded
//    as a linear combination of the columns of A).  In particular, if A
//    is square, and has maximum column rank, it is nonsingular.
//
//    The dimension of the nullspace is the number of linearly independent
//    vectors that span the nullspace.  If A has maximum column rank,
//    its nullspace has dimension 0.
//
//    This routine uses the reduced row echelon form of A to determine
//    a set of NULLSPACE_SIZE independent null vectors.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    03 October 2008
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, N, the number of rows and columns of
//    the matrix A.
//
//    double A[M*N], the matrix to be analyzed.
//
//    int NULLSPACE_SIZE, the size of the nullspace.
//
//  Output:
//
//    double R8MAT_NULLSPACE[N*NULLSPACE_SIZE], vectors that
//    span the nullspace.
//
{
  int *col;
  int i;
  int i2;
  int j;
  int j2;
  double *nullspace;
  int *row;
  double *rref;
//
//  Make a copy of A.
//
  rref = r8mat_copy_new ( m, n, a );
//
//  Get the reduced row echelon form of A.
//
  r8mat_rref ( m, n, rref );
//
//  Note in ROW the columns of the leading nonzeros.
//  COL(J) = +J if there is a leading 1 in that column, and -J otherwise.
//
  row = new int[m];
  for ( i = 0; i < m; i++ )
  {
    row[i] = 0;
  }

  col = new int[n];
  for ( j = 0; j < n; j++ )
  {
    col[j] = - ( j + 1 );
  }

  for ( i = 0; i < m; i++ )
  {
    for ( j = 0; j < n; j++ )
    {
      if ( rref[i+j*m] == 1.0 )
      {
        row[i] = ( j + 1 );
        col[j] = ( j + 1 );
        break;
      }
    }
  }

  nullspace = r8mat_zeros_new ( n, nullspace_size );

  j2 = 0;
//
//  If column J does not contain a leading 1, then it contains
//  information about a null vector.
//
  for ( j = 0; j < n; j++ )
  {
    if ( col[j] < 0 )
    {
      for ( i = 0; i < m; i++ )
      {
        if ( rref[i+j*m] != 0.0 )
        {
          i2 = row[i] - 1;
          nullspace[i2+j2*n] = - rref[i+j*m];
        }
      }
      nullspace[j+j2*n] = 1.0;
      j2 = j2 + 1;
    }
  }
  delete [] col;
  delete [] row;
  delete [] rref;

  return nullspace;
}
//****************************************************************************80

int r8mat_nullspace_size ( int m, int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_nullspace_size() computes the size of the nullspace of a matrix.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    Let A be an MxN matrix.
//
//    If X is an N-vector, and A*X = 0, then X is a null vector of A.
//
//    The set of all null vectors of A is called the nullspace of A.
//
//    The 0 vector is always in the null space.
//
//    If the 0 vector is the only vector in the nullspace of A, then A
//    is said to have maximum column rank.  (Because A*X=0 can be regarded
//    as a linear combination of the columns of A).  In particular, if A
//    is square, and has maximum column rank, it is nonsingular.
//
//    The dimension of the nullspace is the number of linearly independent
//    vectors that span the nullspace.  If A has maximum column rank,
//    its nullspace has dimension 0.
//
//    This routine ESTIMATES the dimension of the nullspace.  Cases of
//    singularity that depend on exact arithmetic will probably be missed.
//
//    The nullspace will be estimated by counting the leading 1's in the
//    reduced row echelon form of A, and subtracting this from N.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    26 August 2011
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, N, the number of rows and columns of
//    the matrix A.
//
//    double A[M*N], the matrix to be analyzed.
//
//  Output:
//
//    int R8MAT_NULLSPACE_SIZE, the estimated size
//    of the nullspace.
//
{
  int i;
  int j;
  int leading;
  int nullspace_size;
  double *rref;
//
//  Make a copy of A.
//
  rref = r8mat_copy_new ( m, n, a );
//
//  Get the reduced row echelon form of A.
//
  r8mat_rref ( m, n, rref );
//
//  Count the leading 1's in A.
//
  leading = 0;
  for ( i = 0; i < m; i++ )
  {
    for ( j = 0; j < n; j++ )
    {
      if ( rref[i+j*m] == 1.0 )
      {
        leading = leading + 1;
        break;
      }
    }
  }
  nullspace_size = n - leading;

  delete [] rref;

  return nullspace_size;
}
//****************************************************************************80

double *r8mat_orth_uniform_new ( int n, int &seed )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_orth_uniform_new() returns a random orthogonal matrix.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    The inverse of A is equal to A'.
//
//    A * A'  = A' * A = I.
//
//    Columns and rows of A have unit Euclidean norm.
//
//    Distinct pairs of columns of A are orthogonal.
//
//    Distinct pairs of rows of A are orthogonal.
//
//    The L2 vector norm of A*x = the L2 vector norm of x for any vector x.
//
//    The L2 matrix norm of A*B = the L2 matrix norm of B for any matrix B.
//
//    The determinant of A is +1 or -1.
//
//    All the eigenvalues of A have modulus 1.
//
//    All singular values of A are 1.
//
//    All entries of A are between -1 and 1.
//
//  Discussion:
//
//    Thanks to Eugene Petrov, B I Stepanov Institute of Physics,
//    National Academy of Sciences of Belarus, for convincingly
//    pointing out the severe deficiencies of an earlier version of
//    this routine.
//
//    Essentially, the computation involves saving the Q factor of the
//    QR factorization of a matrix whose entries are normally distributed.
//    However, it is only necessary to generate this matrix a column at
//    a time, since it can be shown that when it comes time to annihilate
//    the subdiagonal elements of column K, these (transformed) elements of
//    column K are still normally distributed random values.  Hence, there
//    is no need to generate them at the beginning of the process and
//    transform them K-1 times.
//
//    For computational efficiency, the individual Householder transformations
//    could be saved, as recommended in the reference, instead of being
//    accumulated into an explicit matrix format.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    26 November 2005
//
//  Author:
//
//    John Burkardt
//
//  Reference:
//
//    Pete Stewart,
//    Efficient Generation of Random Orthogonal Matrices With an Application
//    to Condition Estimators,
//    SIAM Journal on Numerical Analysis,
//    Volume 17, Number 3, June 1980, pages 403-409.
//
//  Input:
//
//    int N, the order of A.
//
//    int &SEED, a seed for the random number generator.
//
//  Output:
//
//    double R8MAT_ORTH_UNIFORM_NEW[N*N], the orthogonal matrix.
//
//    int &SEED: an updated seed.
//
{
  double *a_col;
  double *q;
  double *q2;
  int i;
  int j;
  double *v;
//
//  Start with Q = the identity matrix.
//
  q = r8mat_identity_new ( n );
//
//  Now behave as though we were computing the QR factorization of
//  some other random matrix.  Generate the N elements of the first column,
//  compute the Householder matrix H1 that annihilates the subdiagonal elements,
//  and set Q := Q * H1' = Q * H.
//
//  On the second step, generate the lower N-1 elements of the second column,
//  compute the Householder matrix H2 that annihilates them,
//  and set Q := Q * H2' = Q * H2 = H1 * H2.
//
//  On the N-1 step, generate the lower 2 elements of column N-1,
//  compute the Householder matrix HN-1 that annihilates them, and
//  and set Q := Q * H(N-1)' = Q * H(N-1) = H1 * H2 * ... * H(N-1).
//  This is our random orthogonal matrix.
//
  a_col = new double[n];

  for ( j = 1; j < n; j++ )
  {
//
//  Set the vector that represents the J-th column to be annihilated.
//
    for ( i = 1; i < j; i++ )
    {
      a_col[i-1] = 0.0;
    }
    for ( i = j; i <= n; i++ )
    {
      a_col[i-1] = r8_normal_01 ( );
    }
//
//  Compute the vector V that defines a Householder transformation matrix
//  H(V) that annihilates the subdiagonal elements of A.
//
    v = r8vec_house_column ( n, a_col, j );
//
//  Postmultiply the matrix Q by H'(V) = H(V).
//
    q2 = r8mat_house_axh_new ( n, q, v );

    delete [] v;

    r8mat_copy ( n, n, q2, q );

    delete [] q2;
  }
//
//  Release memory.
//
  delete [] a_col;

  return q;
}
//****************************************************************************80

void r8mat_plot ( int m, int n, double a[], string title )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_plot() "plots" an R8MAT.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    10 September 2009
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, the number of rows in A.
//
//    int N, the number of columns in A.
//
//    double A[M*N], the matrix.
//
//    string TITLE, a title.
//
{
  int i;
  int j;
  int jhi;
  int jlo;

  cout << "\n";
  cout << title << "\n";

  for ( jlo = 1; jlo <= n; jlo = jlo + 70 )
  {
    jhi = min ( jlo + 70-1, n );
    cout << "\n";
    cout << "          ";
    for ( j = jlo; j <= jhi; j++ )
    {
      cout <<  ( j % 10 );
    }
    cout << "\n";
    cout << "\n";

    for ( i = 1; i <= m; i++ )
    {
      cout << setw(6) << i << "    ";
      for ( j = jlo; j <= jhi; j++ )
      {
        cout << r8mat_plot_symbol ( a[i-1+(j-1)*m] );
      }
      cout << "\n";
    }
  }

  return;
}
//****************************************************************************80

char r8mat_plot_symbol ( double r )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_plot_symbol() returns a symbol for entries of an R8MAT.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    16 October 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double R, a value whose symbol is desired.
//
//  Output:
//
//    char R8MAT_PLOT_SYMBOL, is
//    '-' if R is negative,
//    '0' if R is zero,
//    '+' if R is positive.
//
{
  char c;

  if ( r < 0.0 )
  {
    c = '-';
  }
  else if ( r == 0.0 )
  {
    c = '0';
  }
  else
  {
    c = '+';
  }

  return c;
}
//****************************************************************************80

double *r8mat_poly_char ( int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_poly_char() computes the characteristic polynomial of an R8MAT.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    16 October 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the order of the matrix A.
//
//    double A[N*N], the N by N matrix.
//
//  Output:
//
//    double R8MAT_POLY_CHAR[N+1], the coefficients of the characteristic
//    polynomial of A.  P(N) contains the coefficient of X^N
//    (which will be 1), P(I) contains the coefficient of X^I,
//    and P(0) contains the constant term.
//
{
  int i;
  int order;
  double *p;
  double trace;
  double *work1;
  double *work2;

  p = new double[n+1];
//
//  Initialize WORK1 to the identity matrix.
//
  work1 = r8mat_identity_new ( n );

  p[n] = 1.0;

  for ( order = n-1; 0 <= order; order-- )
  {
//
//  Work2 = A * WORK1.
//
    work2 = r8mat_mm_new ( n, n, n, a, work1 );
//
//  Take the trace.
//
    trace = r8mat_trace ( n, work2 );
//
//  P(ORDER) = -Trace ( WORK2 ) / ( N - ORDER )
//
    p[order] = -trace / ( double ) ( n - order );
//
//  WORK1 := WORK2 + P(IORDER) * Identity.
//
    delete [] work1;

    r8mat_copy ( n, n, work2, work1 );

    delete [] work2;

    for ( i = 0; i < n; i++ )
    {
      work1[i+i*n] = work1[i+i*n] + p[order];
    }
  }

  delete [] work1;

  return p;
}
//****************************************************************************80

double *r8mat_power ( int n, double a[], int npow )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_power() computes a nonnegative power of an R8MAT.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    The algorithm is:
//
//      B = I
//      do NPOW times:
//        B = A * B
//      end
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    16 October 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the order of A.
//
//    double A[N*N], the matrix to be raised to a power.
//
//    int NPOW, the power to which A is to be raised.
//    NPOW must be nonnegative.
//
//  Output:
//
//    double B[N*N], the value of A^NPOW.
//
{
  double *b;
  double *c;
  int ipow;

  if ( npow < 0 )
  {
    cerr << "\n";
    cerr << "R8MAT_POWER - Fatal error!\n";
    cerr << "  Input value of NPOW < 0.\n";
    cerr << "  NPOW = " << npow << "\n";
    exit ( 1 );
  }

  b = r8mat_identity_new ( n );

  for ( ipow = 1; ipow <= npow; ipow++ )
  {
    c = r8mat_mm_new ( n, n, n, a, b );
    r8mat_copy ( n, n, c, b );
    delete [] c;
  }

  return b;
}
//****************************************************************************80

void r8mat_power_method ( int n, double a[], double *r, double v[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_power_method() applies the power method to a matrix.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    If the power method has not converged, then calling the routine
//    again immediately with the output from the previous call will
//    continue the iteration.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    21 April 2012
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the order of A.
//
//    double A[N*N], the matrix.
//
//    double V[N]: a starting estimate for the eigenvector.
//
//  Output:
//
//    double *R, the estimated eigenvalue.
//
//    double V[N]: an improved eigenvector estimate.
//
{
  double *av;
  double eps;
  int i;
  int it;
  double it_eps = 0.0001;
  int it_max = 100;
  int it_min = 10;
  int j;
  double r2;
  double r_old;

  eps = sqrt ( DBL_EPSILON );

  *r = r8vec_norm ( n, v );

  if ( *r == 0.0 )
  {
    for ( i = 0; i < n; i++ )
    {
      v[i] = 1.0;
    }
    *r = sqrt ( ( double ) n );
  }

  for ( i = 0; i < n; i++ )
  {
    v[i] = v[i] / *r;
  }

  for ( it = 1; it <= it_max; it++ )
  {
    av = r8mat_mv_new ( n, n, a, v );

    r_old = *r;
    *r = r8vec_norm ( n, av );

    if ( it_min < it )
    {
      if ( fabs ( *r - r_old ) <= it_eps * ( 1.0 + fabs ( *r ) ) )
      {
        break;
      }
    }

    r8vec_copy ( n, av, v );

    delete [] av;

    if ( *r != 0.0 )
    {
      for ( i = 0; i < n; i++ )
      {
        v[i] = v[i] / *r;
      }
    }
//
//  Perturb V a bit, to avoid cases where the initial guess is exactly
//  the eigenvector of a smaller eigenvalue.
//
    if ( it < it_max / 2 )
    {
      j = ( ( it - 1 ) % n );
      v[j] = v[j] + eps * ( 1.0 + fabs ( v[j] ) );
      r2 = r8vec_norm ( n, v );
      for ( i = 0; i < n; i++ )
      {
        v[i] = v[i] / r2;
      }
    }
  }
  return;
}
//****************************************************************************80

void r8mat_print ( int m, int n, double a[], string title )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_print() prints an R8MAT.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    Entry A(I,J) is stored as A[I+J*M]
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    10 September 2009
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, the number of rows in A.
//
//    int N, the number of columns in A.
//
//    double A[M*N], the M by N matrix.
//
//    string TITLE, a title.
//
{
  r8mat_print_some ( m, n, a, 1, 1, m, n, title );

  return;
}
//****************************************************************************80

void r8mat_print_some ( int m, int n, double a[], int ilo, int jlo, int ihi,
  int jhi, string title )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_print_some() prints some of an R8MAT.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    26 June 2013
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, the number of rows of the matrix.
//    M must be positive.
//
//    int N, the number of columns of the matrix.
//    N must be positive.
//
//    double A[M*N], the matrix.
//
//    int ILO, JLO, IHI, JHI, designate the first row and
//    column, and the last row and column to be printed.
//
//    string TITLE, a title.
//
{
# define INCX 5

  int i;
  int i2hi;
  int i2lo;
  int j;
  int j2hi;
  int j2lo;

  cout << "\n";
  cout << title << "\n";

  if ( m <= 0 || n <= 0 )
  {
    cout << "\n";
    cout << "  (None)\n";
    return;
  }
//
//  Print the columns of the matrix, in strips of 5.
//
  for ( j2lo = jlo; j2lo <= jhi; j2lo = j2lo + INCX )
  {
    j2hi = j2lo + INCX - 1;
    if ( n < j2hi )
    {
      j2hi = n;
    }
    if ( jhi < j2hi )
    {
      j2hi = jhi;
    }
    cout << "\n";
//
//  For each column J in the current range...
//
//  Write the header.
//
    cout << "  Col:    ";
    for ( j = j2lo; j <= j2hi; j++ )
    {
      cout << setw(7) << j - 1 << "       ";
    }
    cout << "\n";
    cout << "  Row\n";
    cout << "\n";
//
//  Determine the range of the rows in this strip.
//
    if ( 1 < ilo )
    {
      i2lo = ilo;
    }
    else
    {
      i2lo = 1;
    }
    if ( ihi < m )
    {
      i2hi = ihi;
    }
    else
    {
      i2hi = m;
    }

    for ( i = i2lo; i <= i2hi; i++ )
    {
//
//  Print out (up to) 5 entries in row I, that lie in the current strip.
//
      cout << setw(5) << i - 1 << ": ";
      for ( j = j2lo; j <= j2hi; j++ )
      {
        cout << setw(12) << a[i-1+(j-1)*m] << "  ";
      }
      cout << "\n";
    }
  }

  return;
# undef INCX
}
//****************************************************************************80

double r8mat_product_elementwise ( int m, int n, double a[], double b[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_product_elementwise() returns the elementwise produce to two R8MAT's.
//
//  Example:
//
//    A = [ 1, 2, 3;    B = [ 1, 3, 5;    product = 86
//         4, 5, 6 ]         2, 4, 6 ]
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    31 March 2018
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, the number of rows.
//
//    int N, the number of columns.
//
//    double A[M*N], B[M*N], the two matrices.
//
//  Output:
//
//    double I4MAT_PRODUCT_ELEMENTWISE, the elementwise 
//    product of A and B.
//
{
  int i;
  int j;
  double value;

  value = 0.0;
  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      value = value + a[i+j*m] * b[i+j*m];
    }
  }
  
  return value;
}
//****************************************************************************80

double r8mat_ref ( int m, int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_ref() computes the row echelon form of a matrix.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    A matrix is in row echelon form if:
//
//    * The first nonzero entry in each row is 1.
//
//    * The leading 1 in a given row occurs in a column to
//      the right of the leading 1 in the previous row.
//
//    * Rows which are entirely zero must occur last.
//
//  Example:
//
//    Input matrix:
//
//     1.0  3.0  0.0  2.0  6.0  3.0  1.0
//    -2.0 -6.0  0.0 -2.0 -8.0  3.0  1.0
//     3.0  9.0  0.0  0.0  6.0  6.0  2.0
//    -1.0 -3.0  0.0  1.0  0.0  9.0  3.0
//
//    Output matrix:
//
//     1.0  3.0  0.0  2.0  6.0  3.0  1.0
//     0.0  0.0  0.0  1.0  2.0  4.5  1.5
//     0.0  0.0  0.0  0.0  0.0  1.0  0.3
//     0.0  0.0  0.0  0.0  0.0  0.0  0.0
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    15 April 2018
//
//  Author:
//
//    John Burkardt
//
//  Reference:
//
//    Charles Cullen,
//    An Introduction to Numerical Linear Algebra,
//    PWS Publishing Company, 1994,
//    ISBN: 978-0534936903,
//    LC: QA185.D37.C85.
//
//  Input:
//
//    int M, N, the number of rows and columns of the matrix.
//
//    double A[M*N]: the matrix to be analyzed.
//
//  Output:
//
//    double A[M*N]: the matrix to be analyzed.
//
//    double R8MAT_REF, the pseudo-determinant.
//
{
  double asum;
  double det;
  int i;
  int j;
  int lead;
  int r;
  double temp;
  double tol;

  det = 1.0;
  asum = 0.0;
  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      asum = asum + fabs ( a[i+j*m] );
    }
  }
  tol = DBL_EPSILON * asum;
  lead = 0;

  for ( r = 0; r < m; r++ )
  {
    if ( n - 1 < lead )
    {
      break;
    }

    i = r;

    while ( fabs ( a[i+lead*m] ) <= tol )
    {
      i = i + 1;

      if ( m - 1 < i )
      {
        i = r;
        lead = lead + 1;
        if ( n - 1 < lead )
        {
          lead = -1;
          break;
         }
      }
    }

    if ( lead < 0 )
    {
      break;
    }

    for ( j = 0; j < n; j++ )
    {
      temp     = a[i+j*m];
      a[i+j*m] = a[r+j*m];
      a[r+j*m] = temp;
    }

    det = det * a[r+lead*m];
    temp = a[r+lead*m];

    for ( j = 0; j < n; j++ )
    {
      a[r+j*m] = a[r+j*m] / temp;
    }

    for ( i = r + 1; i < m; i++ )
    {
      temp = a[i+lead*m];
      for ( j = 0; j < n; j++ )
      {
        a[i+j*m] = a[i+j*m] - temp * a[r+j*m];
      }
    }
    lead = lead + 1;
  }
  return det;
}
//****************************************************************************80

double r8mat_rms ( int m, int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_rms() returns the RMS norm of an R8MAT.
//
//  Discussion:
//
//    An R8MAT is an array of R8's.
//
//    The matrix RMS norm is defined as:
//
//      R8MAT_RMS =
//        sqrt ( sum ( 0 <= J < N ) sum ( 0 <= I < M ) A[I,J]^2 / M / N ).
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    14 December 2011
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, N, the dimensions of the array.
//
//    double A[M*N], the array.
//
//  Output:
//
//    double R8MAT_RMS, the RMS norm of A.
//
{
  int i;
  int j;
  double value;

  value = 0.0;

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      value = value + a[i+j*m] * a[i+j*m];
    }
    value = sqrt ( value / ( double ) ( m ) / ( double ) ( n ) );
  }
  return value;
}
//****************************************************************************80

void r8mat_row_copy ( int m, int n, int i, double v[], double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_row_copy() copies a vector into a row of an R8MAT.
//
//  Discussion:
//
//    An R8MAT is an MxN array of R8's, stored by (I,J) -> [I+J*M].
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    30 June 2014
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, N, the order of the matrix.
//
//    int I, the index of the row.
//    0 <= I <= M-1.
//
//    double V[N], the row to be copied.
//
//    double A[M*N], the matrix into which the row is to be copied.
//
//  Output:
//
//    double A[M*N}: the modified matrix.
{
  int j;

  for ( j = 0; j < n; j++ )
  {
    a[i+j*m] = v[j];
  }
  return;
}
//****************************************************************************80

double r8mat_rref ( int m, int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_rref() computes the reduced row echelon form of a matrix.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    A matrix is in row echelon form if:
//
//    * The first nonzero entry in each row is 1.
//
//    * The leading 1 in a given row occurs in a column to
//      the right of the leading 1 in the previous row.
//
//    * Rows which are entirely zero must occur last.
//
//    The matrix is in reduced row echelon form if, in addition to
//    the first three conditions, it also satisfies:
//
//    * Each column containing a leading 1 has no other nonzero entries.
//
//  Example:
//
//    Input matrix:
//
//     1.0  3.0  0.0  2.0  6.0  3.0  1.0
//    -2.0 -6.0  0.0 -2.0 -8.0  3.0  1.0
//     3.0  9.0  0.0  0.0  6.0  6.0  2.0
//    -1.0 -3.0  0.0  1.0  0.0  9.0  3.0
//
//    Output matrix:
//
//     1.0  3.0  0.0  0.0  2.0  0.0  0.0
//     0.0  0.0  0.0  1.0  2.0  0.0  0.0
//     0.0  0.0  0.0  0.0  0.0  1.0  0.3
//     0.0  0.0  0.0  0.0  0.0  0.0  0.0
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    15 April 2018
//
//  Author:
//
//    John Burkardt
//
//  Reference:
//
//    Charles Cullen,
//    An Introduction to Numerical Linear Algebra,
//    PWS Publishing Company, 1994,
//    ISBN: 978-0534936903,
//    LC: QA185.D37.C85.
//
//  Input:
//
//    int M, N, the number of rows and columns of
//    the matrix A.
//
//    double A[M*N]: the matrix to be analyzed.
//
//  Output:
//
//    double A[M*N]: the RREF form of the matrix.
//
//    double R8MAT_RREF, the pseudo-determinant.
//
{
  double asum;
  double det;
  int i;
  int j;
  int lead;
  int r;
  double temp;
  double tol;

  det = 1.0;
  asum = 0.0;
  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      asum = asum + fabs ( a[i+j*m] );
    }
  }
  tol = DBL_EPSILON * asum;
  lead = 0;

  for ( r = 0; r < m; r++ )
  {
    if ( n - 1 < lead )
    {
      break;
    }

    i = r;

    while ( fabs ( a[i+lead*m] ) <= tol )
    {
      i = i + 1;

      if ( m - 1 < i )
      {
        i = r;
        lead = lead + 1;
        if ( n - 1 < lead )
        {
          lead = -1;
          break;
         }
      }
    }

    if ( lead < 0 )
    {
      break;
    }

    for ( j = 0; j < n; j++ )
    {
      temp     = a[i+j*m];
      a[i+j*m] = a[r+j*m];
      a[r+j*m] = temp;
    }

    det = det * a[r+lead*m];
    temp = a[r+lead*m];

    for ( j = 0; j < n; j++ )
    {
      a[r+j*m] = a[r+j*m] / temp;
    }

    for ( i = 0; i < m; i++ )
    {
      if ( i != r )
      {
        temp = a[i+lead*m];
        for ( j = 0; j < n; j++ )
        {
          a[i+j*m] = a[i+j*m] - temp * a[r+j*m];
        }
      }
    }
    lead = lead + 1;

  }
  return det;
}
//****************************************************************************80

void r8mat_scale ( int m, int n, double s, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_scale() multiplies an R8MAT by a scalar.
//
//  Discussion:
//
//    An R8MAT is an array of R8 values.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    01 December 2011
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, N, the number of rows and columns.
//
//    double S, the scale factor.
//
//    double A[M*N], the matrix to be scaled.
//
//  Output:
//
//    double A[M*N]: the scaled matrix.
{
  int i;
  int j;

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      a[i+j*m] = a[i+j*m] * s;
    }
  }
  return;
}
//****************************************************************************80

double *r8mat_scale_01 ( int m, int n, double x[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_scale_01() shifts and scales an R8MAT so columns have min 0 and max 1.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    08 October 2018
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, N, the number of rows and columns.
//
//    double X[M*N], the array to be rescaled.
//
//  Output:
//
//    double R8MAT_SCALE_01[M*N], the rescaled array.
//
{
  int i;
  int j;
  double *xmax;
  double *xmin;
  double *xs;

  xmax = r8mat_max_columns ( m, n, x );
  xmin = r8mat_min_columns ( m, n, x );
  
  xs = new double [ m * n ];

  for ( j = 0; j < n; j++ )
  {
    if ( 0 < xmax[j] - xmin[j] )
    {
      for ( i = 0; i < m; i++ )
      {
        xs[i+j*m] = ( x[i+j*m] - xmin[j] ) / ( xmax[j] - xmin[j] );
      }
    }
    else
    {
      for ( i = 0; i < m; i++ )
      {
        xs[i+j*m] = 0.5;
      }
    }
  }

  delete [] xmax;
  delete [] xmin;

  return xs;
}
//****************************************************************************80

double *r8mat_scale_ab ( int m, int n, double x[], double a, double b )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_scale_ab() shifts and scales an R8MAT so columns have min A and max B.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    08 October 2018
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, N, the number of rows and columns.
//
//    double X[M*N], the array to be rescaled.
//
//    double A, B, the new scale limits.
//
//  Output:
//
//    double R8MAT_SCALE_AB[M*N], the rescaled array.
//
{
  int i;
  int j;
  double *xmax;
  double *xmin;
  double *xs;

  xmax = r8mat_max_columns ( m, n, x );
  xmin = r8mat_min_columns ( m, n, x );
  
  xs = new double [ m * n ];

  for ( j = 0; j < n; j++ )
  {
    if ( 0 < xmax[j] - xmin[j] )
    {
      for ( i = 0; i < m; i++ )
      {
        xs[i+j*m] = a + ( b - a ) * ( x[i+j*m] - xmin[j] ) / ( xmax[j] - xmin[j] );
      }
    }
    else
    {
      for ( i = 0; i < m; i++ )
      {
        xs[i+j*m] = ( a + b ) / 2.0;
      }
    }
  }

  delete [] xmax;
  delete [] xmin;

  return xs;
}
//****************************************************************************80

int r8mat_solve ( int n, int rhs_num, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_solve() uses Gauss-Jordan elimination to solve an N by N linear system.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    Entry A(I,J) is stored as A[I+J*N]
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    29 August 2003
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the order of the matrix.
//
//    int RHS_NUM, the number of right hand sides.  RHS_NUM
//    must be at least 0.
//
//    double A[N*(N+RHS_NUM)], contains in rows and columns 1
//    to N the coefficient matrix, and in columns N+1 through
//    N+RHS_NUM, the right hand sides. 
//
//  Output:
//
//    double A[N*(N+RHS_NUM)]: the coefficient matrix
//    area has been destroyed, while the right hand sides have
//    been overwritten with the corresponding solutions.
//
//    int R8MAT_SOLVE, singularity flag.
//    0, the matrix was not singular, the solutions were computed;
//    J, factorization failed on step J, and the solutions could not
//    be computed.
//
{
  double apivot;
  double factor;
  int i;
  int ipivot;
  int j;
  int k;
  double temp;

  for ( j = 0; j < n; j++ )
  {
//
//  Choose a pivot row.
//
    ipivot = j;
    apivot = a[j+j*n];

    for ( i = j; i < n; i++ )
    {
      if ( fabs ( apivot ) < fabs ( a[i+j*n] ) )
      {
        apivot = a[i+j*n];
        ipivot = i;
      }
    }

    if ( apivot == 0.0 )
    {
      return j;
    }
//
//  Interchange.
//
    for ( i = 0; i < n + rhs_num; i++ )
    {
      temp          = a[ipivot+i*n];
      a[ipivot+i*n] = a[j+i*n];
      a[j+i*n]      = temp;
    }
//
//  A(J,J) becomes 1.
//
    a[j+j*n] = 1.0;
    for ( k = j; k < n + rhs_num; k++ )
    {
      a[j+k*n] = a[j+k*n] / apivot;
    }
//
//  A(I,J) becomes 0.
//
    for ( i = 0; i < n; i++ )
    {
      if ( i != j )
      {
        factor = a[i+j*n];
        a[i+j*n] = 0.0;
        for ( k = j; k < n + rhs_num; k++ )
        {
          a[i+k*n] = a[i+k*n] - factor * a[j+k*n];
        }
      }
    }
  }

  return 0;
}
//****************************************************************************80

double *r8mat_solve_2d ( double a[], double b[], double *det )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_solve_2d() solves a 2 by 2 linear system using Cramer's rule.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    If the determinant DET is returned as zero, then the matrix A is
//    singular, and does not have an inverse.  In that case, X is
//    returned as the NULL vector.
//
//    If DET is nonzero, then its value is roughly an estimate
//    of how nonsingular the matrix A is.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    16 November 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double A[2*2], the matrix.
//
//    double B[2], the right hand side.
//
//  Output:
//
//    double *DET, the determinant of the system.
//
//    double R8MAT_SOLVE_2D[2], the solution of the system,
//    if DET is nonzero.  Otherwise, the NULL vector.
//
{
  double *x;
//
//  Compute the determinant.
//
  *det = a[0+0*2] * a[1+1*2] - a[0+1*2] * a[1+0*2];
//
//  If the determinant is zero, bail out.
//
  if ( *det == 0.0 )
  {
    return NULL;
  }
//
//  Compute the solution.
//
  x = new double[2];

  x[0] = (  a[1+1*2] * b[0] - a[0+1*2] * b[1] ) / ( *det );
  x[1] = ( -a[1+0*2] * b[0] + a[0+0*2] * b[1] ) / ( *det );

  return x;
}
//****************************************************************************80

double *r8mat_solve_3d ( double a[], double b[], double *det )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_solve_3d() solves a 3 by 3 linear system using Cramer's rule.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    If the determinant DET is returned as zero, then the matrix A is
//    singular, and does not have an inverse.  In that case, X is
//    returned as the NULL vector.
//
//    If DET is nonzero, then its value is roughly an estimate
//    of how nonsingular the matrix A is.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    05 December 2006
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double A[3*3], the matrix.
//
//    double B[3], the right hand side.
//
//  Output:
//
//    double *DET, the determinant of the system.
//
//    double R8MAT_SOLVE_3D[3], the solution of the system,
//    if DET is nonzero.  Otherwise, the NULL vector.
//
{
  double *x;
//
//  Compute the determinant.
//
  *det =  a[0+0*3] * ( a[1+1*3] * a[2+2*3] - a[1+2*3] * a[2+1*3] )
        + a[0+1*3] * ( a[1+2*3] * a[2+0*3] - a[1+0*3] * a[2+2*3] )
        + a[0+2*3] * ( a[1+0*3] * a[2+1*3] - a[1+1*3] * a[2+0*3] );
//
//  If the determinant is zero, bail out.
//
  if ( *det == 0.0 )
  {
    return NULL;
  }
//
//  Compute the solution.
//
  x = new double[3];

  x[0] = (   ( a[1+1*3] * a[2+2*3] - a[1+2*3] * a[2+1*3] ) * b[0]
           - ( a[0+1*3] * a[2+2*3] - a[0+2*3] * a[2+1*3] ) * b[1]
           + ( a[0+1*3] * a[1+2*3] - a[0+2*3] * a[1+1*3] ) * b[2] ) / ( *det );

  x[1] = ( - ( a[1+0*3] * a[2+2*3] - a[1+2*3] * a[2+0*3] ) * b[0]
           + ( a[0+0*3] * a[2+2*3] - a[0+2*3] * a[2+0*3] ) * b[1]
           - ( a[0+0*3] * a[1+2*3] - a[0+2*3] * a[1+0*3] ) * b[2] ) / ( *det );

  x[2] = (   ( a[1+0*3] * a[2+1*3] - a[1+1*3] * a[2+0*3] ) * b[0]
           - ( a[0+0*3] * a[2+1*3] - a[0+1*3] * a[2+0*3] ) * b[1]
           + ( a[0+0*3] * a[1+1*3] - a[0+1*3] * a[1+0*3] ) * b[2] ) / ( *det );

  return x;
}
//****************************************************************************80

double *r8mat_solve2 ( int n, double a[], double b[], int &ierror )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_solve2() computes the solution of an N by N linear system.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    The linear system may be represented as
//
//      A*X = B
//
//    If the linear system is singular, but consistent, then the routine will
//    still produce a solution.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    21 February 2014
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of equations.
//
//    double A[N*N]: the coefficient matrix to be inverted.
//
//    double B[N]: the right hand side of the system.
//
//  Output:
//
//    double A[N*N]: A has been overwritten.
//
//    double B[N]: B has been overwritten.
//
//    double R8MAT_SOLVE2[N], the solution of the linear system.
//
//    int &IERROR.
//    0, no error detected.
//    1, consistent singularity.
//    2, inconsistent singularity.
//
{
  double amax;
  int i;
  int imax;
  int j;
  int k;
  int *piv;
  double *x;

  ierror = 0;

  piv = i4vec_zeros_new ( n );
  x = r8vec_zeros_new ( n );
//
//  Process the matrix.
//
  for ( k = 1; k <= n; k++ )
  {
//
//  In column K:
//    Seek the row IMAX with the properties that:
//      IMAX has not already been used as a pivot;
//      A(IMAX,K) is larger in magnitude than any other candidate.
//
    amax = 0.0;
    imax = 0;
    for ( i = 1; i <= n; i++ )
    {
      if ( piv[i-1] == 0 )
      {
        if ( amax < fabs ( a[i-1+(k-1)*n] ) )
        {
          imax = i;
          amax = fabs ( a[i-1+(k-1)*n] );
        }
      }
    }
//
//  If you found a pivot row IMAX, then,
//    eliminate the K-th entry in all rows that have not been used for pivoting.
//
    if ( imax != 0 )
    {
      piv[imax-1] = k;
      for ( j = k+1; j <= n; j++ )
      {
        a[imax-1+(j-1)*n] = a[imax-1+(j-1)*n] / a[imax-1+(k-1)*n];
      }
      b[imax-1] = b[imax-1] / a[imax-1+(k-1)*n];
      a[imax-1+(k-1)*n] = 1.0;

      for ( i = 1; i <= n; i++ )
      {
        if ( piv[i-1] == 0 )
        {
          for ( j = k+1; j <= n; j++ )
          {
            a[i-1+(j-1)*n] = a[i-1+(j-1)*n] - a[i-1+(k-1)*n] * a[imax-1+(j-1)*n];
          }
          b[i-1] = b[i-1] - a[i-1+(k-1)*n] * b[imax-1];
          a[i-1+(k-1)*n] = 0.0;
        }
      }
    }
  }
//
//  Now, every row with nonzero PIV begins with a 1, and
//  all other rows are all zero.  Begin solution.
//
  for ( j = n; 1 <= j; j-- )
  {
    imax = 0;
    for ( k = 1; k <= n; k++ )
    {
      if ( piv[k-1] == j )
      {
        imax = k;
      }
    }

    if ( imax == 0 )
    {
      x[j-1] = 0.0;

      if ( b[j-1] == 0.0 )
      {
        ierror = 1;
        cout << "\n";
        cout << "R8MAT_SOLVE2 - Warning:\n";
        cout << "  Consistent singularity, equation = " << j << "\n";
      }
      else
      {
        ierror = 2;
        cout << "\n";
        cout << "R8MAT_SOLVE2 - Warning:\n";
        cout << "  Inconsistent singularity, equation = " << j << "\n";
      }
    }
    else
    {
      x[j-1] = b[imax-1];

      for ( i = 1; i <= n; i++ )
      {
        if ( i != imax )
        {
          b[i-1] = b[i-1] - a[i-1+(j-1)*n] * x[j-1];
        }
      }
    }
  }

  delete [] piv;

  return x;
}
//****************************************************************************80

double *r8mat_standardize ( int m, int n, double x[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_standardize() standardizes an R8MAT.
//
//  Discussion:
//
//    The output array will have columns of 0 mean and unit standard deviation.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    10 October 2018
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, N, the number of rows and columns.
//
//    double X[M*N], the array to be standardized.
//
//  Output:
//
//    double R8MAT_STANDARDIZE[M*N], the standardized array.
//
{
  int i;
  int j;
  double *mu;
  double *sigma;
  double *xs;

  mu = r8mat_mean_columns ( m, n, x );
  sigma = r8mat_std_columns ( m, n, x );
  
  xs = new double [ m * n ];

  for ( j = 0; j < n; j++ )
  {
    if ( sigma[j] != 0.0 )
    {
      for ( i = 0; i < m; i++ )
      {
        xs[i+j*m] = ( x[i+j*m] - mu[j] ) / sigma[j];
      }
    }
    else
    {
      for ( i = 0; i < m; i++ )
      {
        xs[i+j*m] = 0.0;
      }
    }
  }

  delete [] mu;
  delete [] sigma;

  return xs;
}
//****************************************************************************80

double *r8mat_std_columns ( int m, int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_std_columns() returns the column standard deviation of an R8MAT.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    04 October 2018
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, the number of rows in A.
//
//    int N, the number of columns in A.
//
//    double A[M*N], the M by N matrix.
//
//  Output:
//
//    double R8MAT_STD_COLUMNS[N], the column stds.
//
{
  int i;
  int j;
  double mean;
  double std;
  double *std_columns;

  std_columns = new double[n];

  for ( j = 0; j < n; j++ )
  {
    mean = 0.0;
    for ( i = 0; i < m; i++ )
    {
      mean = mean + a[i+j*m];
    }
    mean = mean / ( double ) m;
    std = 0.0;
    for ( i = 0; i < m; i++ )
    {
      std = std + pow ( a[i+j*m] - mean, 2 );
    }
    std = sqrt ( std / ( double ) ( m - 1 ) );
    std_columns[j] = std;
  }

  return std_columns;
}
//****************************************************************************80

double *r8mat_std_rows ( int m, int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_std_rows() returns the row standard deviations of an R8MAT.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    04 October 2018
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, the number of rows in A.
//
//    int N, the number of columns in A.
//
//    double A[M*N], the M by N matrix.
//
//  Output:
//
//    double R8MAT_STD_ROWS[M], the row stds.
//
{
  int i;
  int j;
  double mean;
  double *std_rows;
  double std;

  std_rows = new double[m];

  for ( i = 0; i < m; i++ )
  {
    mean = 0.0;
    for ( j = 0; j < n; j++ )
    {
      mean = mean + a[i+j*m];
    }
    mean = mean / ( double ) n;
    std = 0.0;
    for ( j = 0; j < n; j++ )
    {
      std = std + pow ( a[i+j*m] - mean, 2 );
    }
    std = sqrt ( std / ( double ) ( n - 1 ) );
    std_rows[i] = std;
  }

  return std_rows;
}
//****************************************************************************80

double r8mat_sum ( int m, int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_sum() returns the sum of an R8MAT.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    03 January 2012
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, N, the number of rows and columns.
//
//    double A[M*N], the array.
//
//  Output:
//
//    double R8MAT_SUM, the sum of the entries.
//
{
  int i;
  int j;
  double value;

  value = 0.0;
  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      value = value + a[i+j*m];
    }
  }
  return value;
}
//****************************************************************************80

double *r8mat_sum_columns ( int m, int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_sum_columns() returns the column sums of an R8MAT.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    04 October 2018
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, the number of rows in A.
//
//    int N, the number of columns in A.
//
//    double A[M*N], the M by N matrix.
//
//  Output:
//
//    double R8MAT_SUM_COLUMNS[N], the column sums.
//
{
  int i;
  int j;
  double *sum_columns;
  double value;

  sum_columns = new double[n];

  for ( j = 0; j < n; j++ )
  {
    value = 0.0;
    for ( i = 0; i < m; i++ )
    {
      value = value + a[i+j*m];
    }
    sum_columns[j] = value;
  }

  return sum_columns;
}
//****************************************************************************80

double *r8mat_sum_rows ( int m, int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_sum_rows() returns the row sums of an R8MAT.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    04 October 2018
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, the number of rows in A.
//
//    int N, the number of columns in A.
//
//    double A[M*N], the M by N matrix.
//
//  Output:
//
//    double R8MAT_SUM_ROWS[M], the row sums.
//
{
  int i;
  int j;
  double *sum_rows;
  double value;

  sum_rows = new double[m];

  for ( i = 0; i < m; i++ )
  {
    value = 0.0;
    for ( j = 0; j < n; j++ )
    {
      value = value + a[i+j*m];
    }
    sum_rows[i] = value;
  }

  return sum_rows;
}
//****************************************************************************80

double *r8mat_symm_eigen ( int n, double x[], double q[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_symm_eigen() returns a symmetric matrix with given eigensystem.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    The user must supply the desired eigenvalue vector, and the desired
//    eigenvector matrix.  The eigenvector matrix must be orthogonal.  A
//    suitable random orthogonal matrix can be generated by
//    R8MAT_ORTH_UNIFORM_NEW.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    17 October 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the order of A.
//
//    double X[N], the desired eigenvalues for the matrix.
//
//    double Q[N*N], the eigenvector matrix of A.
//
//  Output:
//
//    double R8MAT_SYMM_EIGEN[N*N], a symmetric N by N matrix with
//    eigenvalues X and eigenvectors the columns of Q.
//
{
  double *a;
  int i;
  int j;
  int k;
//
//  Set A = Q * Lambda * Q'.
//
  a = new double[n*n];

  for ( i = 0; i < n; i++ )
  {
    for ( j = 0; j < n; j++ )
    {
      a[i+j*n] = 0.0;
      for ( k = 0; k < n; k++ )
      {
        a[i+j*n] = a[i+j*n] + q[i+k*n] * x[k] * q[j+k*n];
      }
    }
  }

  return a;
}
//****************************************************************************80

void r8mat_symm_jacobi ( int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_symm_jacobi() applies Jacobi eigenvalue iteration to a symmetric matrix.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    This code was modified so that it treats as zero the off-diagonal
//    elements that are sufficiently close to, but not exactly, zero.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    09 October 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the order of A.
//
//    double A[N*N], a symmetric N by N matrix.
//
//  Output:
//
//    double A[N*N]: the matrix has been overwritten by an approximately
//    diagonal matrix, with the eigenvalues on the diagonal.
//
{
  double c;
  double eps = 0.00001;
  int i;
  int it;
  int it_max = 100;
  int j;
  int k;
  double norm_fro;
  double s;
  double sum2;
  double t;
  double t1;
  double t2;
  double u;

  norm_fro = r8mat_norm_fro ( n, n, a );

  it = 0;

  for ( ; ; )
  {
    it = it + 1;

    for ( i = 0; i < n; i++ )
    {
      for ( j = 0; j < i; j++ )
      {
        if ( eps * norm_fro < fabs ( a[i+j*n] ) + fabs ( a[j+i*n] ) )
        {
          u = ( a[j+j*n] - a[i+i*n] ) / ( a[i+j*n] + a[j+i*n] );

          t = r8_sign ( u ) / ( fabs ( u ) + sqrt ( u * u + 1.0 ) );
          c = 1.0 / sqrt ( t * t + 1.0 );
          s = t * c;
//
//  A -> A * Q.
//
          for ( k = 0; k < n; k++ )
          {
            t1 = a[i+k*n];
            t2 = a[j+k*n];
            a[i+k*n] = t1 * c - t2 * s;
            a[j+k*n] = t1 * s + t2 * c;
          }
//
//  A -> QT * A
//
          for ( k = 0; k < n; k++ )
          {
            t1 = a[k+i*n];
            t2 = a[k+j*n];
            a[k+i*n] = c * t1 - s * t2;
            a[k+j*n] = s * t1 + c * t2;
          }
        }
      }
    }
//
//  Test the size of the off-diagonal elements.
//
    sum2 = 0.0;
    for ( i = 0; i < n; i++ )
    {
      for ( j = 0; j < i; j++ )
      {
        sum2 = sum2 + fabs ( a[i+j*n] );
      }
    }

    if ( sum2 <= eps * ( norm_fro + 1.0 ) )
    {
      break;
    }

    if ( it_max <= it )
    {
      break;
    }

  }

  return;
}
//****************************************************************************80

double **r8mat_to_r8cmat_new (  int m, int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_to_r8cmat_new() copies data from an R8MAT to an R8CMAT.
//
//  Discussion:
//
//    An R8MAT is a column-major array stored as a vector, so
//    that element (I,J) of the M by N array is stored in location
//    I+J*M.
//
//    An R8CMAT is a column-major array, storing element (I,J)
//    as A[J][I], and can be created by a command like:
//      double **a;
//      a = r8cmat_new ( m, n );
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    07 January 2014
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, N, the number of rows and columns.
//
//    double A[M*N], the data, stored as an R8MAT.
//
//  Output:
//
//    double R8MAT_TO_R8CMAT_NEW[M][N], the data, stored as an R8CMAT.
//
{
  double **b;
  int i;
  int j;

  b = r8cmat_new ( m, n );

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      b[j][i] = a[i+j*m];
    }
  }

  return b;
}
//****************************************************************************80

int r8mat_to_r8plu ( int n, double a[], int pivot[], double lu[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_to_r8plu() factors a general matrix.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    This routine is a simplified version of the LINPACK routine DGEFA.
//    Fortran conventions are used to index doubly-dimensioned arrays.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    22 August 2003
//
//  Author:
//
//    John Burkardt
//
//  Reference:
//
//    Jack Dongarra, Jim Bunch, Cleve Moler, Pete Stewart,
//    LINPACK User's Guide,
//    SIAM, 1979
//
//  Input:
//
//    int N, the order of the matrix.
//    N must be positive.
//
//    double A[N*N], the matrix to be factored.
//
//  Output:
//
//    int PIVOT[N], a vector of pivot indices.
//
//    double LU[N*N], an upper triangular matrix U and the multipliers
//    L which were used to obtain it.  The factorization can be written
//    A = L * U, where L is a product of permutation and unit lower
//    triangular matrices and U is upper triangular.
//
//    int R8MAT_TO_R8PLU, singularity flag.
//    0, no singularity detected.
//    nonzero, the factorization failed on the R8MAT_TO_R8PLU-th step.
//
{
  int i;
  int info;
  int j;
  int k;
  int l;
  double temp;

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < n; i++ )
    {
      lu[i+j*n] = a[i+j*n];
    }
  }
  info = 0;

  for ( k = 1; k <= n-1; k++ )
  {
//
//  Find L, the index of the pivot row.
//
    l = k;
    for ( i = k+1; i <= n; i++ )
    {
      if ( fabs ( lu[l-1+(k-1)*n] ) < fabs ( lu[i-1+(k-1)*n] ) )
      {
        l = i;
      }
    }

    pivot[k-1] = l;
//
//  If the pivot index is zero, the algorithm has failed.
//
    if ( lu[l-1+(k-1)*n] == 0.0 )
    {
      info = k;
      return info;
    }
//
//  Interchange rows L and K if necessary.
//
    if ( l != k )
    {
      temp            = lu[l-1+(k-1)*n];
      lu[l-1+(k-1)*n] = lu[k-1+(k-1)*n];
      lu[k-1+(k-1)*n] = temp;
    }
//
//  Normalize the values that lie below the pivot entry A(K,K).
//
    for ( i = k+1; i <= n; i++ )
    {
      lu[i-1+(k-1)*n] = -lu[i-1+(k-1)*n] / lu[k-1+(k-1)*n];
    }
//
//  Row elimination with column indexing.
//
    for ( j = k+1; j <= n; j++ )
    {
      if ( l != k )
      {
        temp            = lu[l-1+(j-1)*n];
        lu[l-1+(j-1)*n] = lu[k-1+(j-1)*n];
        lu[k-1+(j-1)*n] = temp;
      }

      for ( i = k+1; i <= n; i++ )
      {
        lu[i-1+(j-1)*n] = lu[i-1+(j-1)*n] + lu[i-1+(k-1)*n] * lu[k-1+(j-1)*n];
      }
    }
  }

  pivot[n-1] = n;

  if ( lu[n-1+(n-1)*n] == 0.0 )
  {
    info = n;
  }

  return info;
}
//****************************************************************************80

double **r8mat_to_r8rmat ( int m, int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_to_r8rmat() copies data from an R8MAT to an R8RMAT.
//
//  Discussion:
//
//    An R8MAT is a column-major array stored as a vector, so
//    that element (I,J) of the M by N array is stored in location
//    I+J*M.
//
//    An R8RMAT is a row-major array that was created by a 
//    command like:
//
//      double **a;
//      a = r8rmat_new ( m, n );
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    07 January 2014
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, N, the number of rows and columns.
//
//    double A[M*N], the data, stored as an R8MAT.
//
//  Output:
//
//    double R8RMAT_TO_R8MAT[M][N], the data, stored as an R8RMAT.
//
{
  double **b;
  int i;
  int j;

  b = r8rmat_new ( m, n );

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      b[i][j] = a[i+j*m];
    }
  }

  return b;
}
//****************************************************************************80

double r8mat_trace ( int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_trace() computes the trace of an R8MAT.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    The trace of a square matrix is the sum of the diagonal elements.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    12 September 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the order of the matrix A.
//
//    double A[N*N], the matrix whose trace is desired.
//
//  Output:
//
//    double R8MAT_TRACE, the trace of the matrix.
//
{
  int i;
  double value;

  value = 0.0;
  for ( i = 0; i < n; i++ )
  {
    value = value + a[i+i*n];
  }

  return value;
}
//****************************************************************************80

void r8mat_transpose_in_place ( int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_transpose_in_place() transposes a square R8MAT in place.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    26 June 2008
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of rows and columns of the matrix A.
//
//    double A[N*N], the matrix to be transposed.
//
//  Output:
//
//    double A[N*N]: the transposed matrix.
{
  int i;
  int j;
  double t;

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < j; i++ )
    {
      t        = a[i+j*n];
      a[i+j*n] = a[j+i*n];
      a[j+i*n] = t;
    }
  }
  return;
}
//****************************************************************************80

double *r8mat_transpose_new ( int m, int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_transpose_new() returns the transpose of an R8MAT.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    12 October 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, N, the number of rows and columns of the matrix A.
//
//    double A[M*N], the matrix whose transpose is desired.
//
//  Output:
//
//    double R8MAT_TRANSPOSE_NEW[N*M], the transposed matrix.
//
{
  double *b;
  int i;
  int j;

  b = new double[n*m];

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      b[j+i*n] = a[i+j*m];
    }
  }
  return b;
}
//****************************************************************************80

void r8mat_transpose_print ( int m, int n, double a[], string title )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_transpose_print() prints an R8MAT, transposed.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    10 September 2009
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, N, the number of rows and columns.
//
//    double A[M*N], an M by N matrix to be printed.
//
//    string TITLE, a title.
//
{
  r8mat_transpose_print_some ( m, n, a, 1, 1, m, n, title );

  return;
}
//****************************************************************************80

void r8mat_transpose_print_some ( int m, int n, double a[], int ilo, int jlo,
  int ihi, int jhi, string title )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_transpose_print_some() prints some of an R8MAT, transposed.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    07 April 2014
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, N, the number of rows and columns.
//
//    double A[M*N], an M by N matrix to be printed.
//
//    int ILO, JLO, the first row and column to print.
//
//    int IHI, JHI, the last row and column to print.
//
//    string TITLE, a title.
//
{
# define INCX 5

  int i;
  int i2;
  int i2hi;
  int i2lo;
  int i2lo_hi;
  int i2lo_lo;
  int inc;
  int j;
  int j2hi;
  int j2lo;

  cout << "\n";
  cout << title << "\n";

  if ( m <= 0 || n <= 0 )
  {
    cout << "\n";
    cout << "  (None)\n";
    return;
  }

  if ( ilo < 1 )
  {
    i2lo_lo = 1;
  }
  else
  {
    i2lo_lo = ilo;
  }

  if ( ihi < m )
  {
    i2lo_hi = m;
  }
  else
  {
    i2lo_hi = ihi;
  }

  for ( i2lo = i2lo_lo; i2lo <= i2lo_hi; i2lo = i2lo + INCX )
  {
    i2hi = i2lo + INCX - 1;

    if ( m < i2hi )
    {
      i2hi = m;
    }
    if ( ihi < i2hi )
    {
      i2hi = ihi;
    }

    inc = i2hi + 1 - i2lo;

    cout << "\n";
    cout << "  Row: ";
    for ( i = i2lo; i <= i2hi; i++ )
    {
      cout << setw(7) << i - 1 << "       ";
    }
    cout << "\n";
    cout << "  Col\n";
    cout << "\n";

    if ( jlo < 1 )
    {
      j2lo = 1;
    }
    else
    {
      j2lo = jlo;
    }
    if ( n < jhi )
    {
      j2hi = n;
    }
    else
    {
      j2hi = jhi;
    }

    for ( j = j2lo; j <= j2hi; j++ )
    {
      cout << setw(5) << j - 1 << ":";
      for ( i2 = 1; i2 <= inc; i2++ )
      {
        i = i2lo - 1 + i2;
        cout << setw(14) << a[(i-1)+(j-1)*m];
      }
      cout << "\n";
    }
  }

  return;
# undef INCX
}
//****************************************************************************80

double *r8mat_u_inverse ( int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_u_inverse() inverts an upper triangular R8MAT.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    An upper triangular matrix is a matrix whose only nonzero entries
//    occur on or above the diagonal.
//
//    The inverse of an upper triangular matrix is an upper triangular matrix.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    12 September 2005
//
//  Author:
//
//    FORTRAN77 original version by Albert Nijenhuis, Herbert Wilf.
//    This version by John Burkardt
//
//  Reference:
//
//    Albert Nijenhuis, Herbert Wilf,
//    Combinatorial Algorithms,
//    Academic Press, 1978, second edition,
//    ISBN 0-12-519260-6.
//
//  Input:
//
//    int N, number of rows and columns in the matrix.
//
//    double A[N*N], the upper triangular matrix.
//
//  Output:
//
//    double R8MAT_U_INVERSE[N*N], the inverse matrix.
//
{
  double *b;
  int i;
  int j;
  int k;

  b = new double[n*n];

  for ( j = n-1; 0 <= j; j-- )
  {
    for ( i = n-1; 0 <= i; i-- )
    {
      if ( j < i )
      {
        b[i+j*n] = 0.0;
      }
      else if ( i == j )
      {
        b[i+j*n] = 1.0 / a[i+j*n];
      }
      else
      {
        b[i+j*n] = 0.0;
        for ( k = i+1; k <= j; k++ )
        {
          b[i+j*n] = b[i+j*n] - a[i+k*n] * b[k+j*n];
        }
       b[i+j*n] = b[i+j*n] / a[i+i*n];
      }
    }
  }

  return b;
}
//****************************************************************************80

double *r8mat_u_solve ( int n, double a[], double b[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_u_solve() solves an upper triangular linear system.
//
//  Discussion:
//
//    An R8MAT is an MxN array of R8's, stored by (I,J) -> [I+J*M].
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    21 October 2013
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of rows and columns of
//    the matrix A.
//
//    double A[N*N], the N by N upper triangular matrix.
//
//    double B[N], the right hand side of the linear system.
//
//  Output:
//
//    double R8MAT_U_SOLVE[N], the solution of the linear system.
//
{
  int i;
  int j;
  double *x;
//
//  Solve U * x = b.
//
  x = new double[n];

  for ( i = n - 1; 0 <= i; i-- )
  {
    x[i] = b[i];
    for ( j = i + 1; j < n; j++ )
    {
      x[i] = x[i] - a[i+j*n] * x[j];
    }
    x[i] = x[i] / a[i+i*n];
  }

  return x;
}
//****************************************************************************80

double *r8mat_u1_inverse ( int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_u1_inverse() inverts a unit upper triangular R8MAT.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    A unit upper triangular matrix is a matrix with only 1's on the main
//    diagonal, and only 0's below the main diagonal.
//
//    The inverse of a unit upper triangular matrix is also
//    a unit upper triangular matrix.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    12 September 2005
//
//  Author:
//
//    C++ translation by John Burkardt
//
//  Reference:
//
//    Albert Nijenhuis, Herbert Wilf,
//    Combinatorial Algorithms,
//    Academic Press, 1978, second edition,
//    ISBN 0-12-519260-6.
//
//  Input:
//
//    int N, number of rows and columns in the matrix.
//
//    double A[N*N], the unit upper triangular matrix.
//
//  Output:
//
//    double R8MAT_U1_INVERSE[N*N), the inverse matrix.
//
{
  double *b;
  int i;
  int j;
  int k;

  b = new double[n*n];

  for ( j = n-1; 0 <= j; j-- )
  {
    for ( i = n-1; 0 <= i; i-- )
    {
      if ( j < i )
      {
        b[i+j*n] = 0.0;
      }
      else if ( i == j )
      {
        b[i+j*n] = 1.0;
      }
      else
      {
        b[i+j*n] = 0.0;
        for ( k = i+1; k <= j; k++ )
        {
          b[i+j*n] = b[i+j*n] - a[i+k*n] * b[k+j*n];
        }
       b[i+j*n] = b[i+j*n] / a[i+i*n];
      }
    }
  }

  return b;
}
//****************************************************************************80

void r8mat_uniform_01 ( int m, int n, int &seed, double r[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_uniform_01() returns a unit pseudorandom R8MAT.
//
//  Discussion:
//
//    An R8MAT is an array of R8's.
//
//    This routine implements the recursion
//
//      seed = ( 16807 * seed ) mod ( 2^31 - 1 )
//      u = seed / ( 2^31 - 1 )
//
//    The integer arithmetic never requires more than 32 bits,
//    including a sign bit.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    03 October 2005
//
//  Author:
//
//    John Burkardt
//
//  Reference:
//
//    Paul Bratley, Bennett Fox, Linus Schrage,
//    A Guide to Simulation,
//    Second Edition,
//    Springer, 1987,
//    ISBN: 0387964673,
//    LC: QA76.9.C65.B73.
//
//    Bennett Fox,
//    Algorithm 647:
//    Implementation and Relative Efficiency of Quasirandom
//    Sequence Generators,
//    ACM Transactions on Mathematical Software,
//    Volume 12, Number 4, December 1986, pages 362-376.
//
//    Pierre L'Ecuyer,
//    Random Number Generation,
//    in Handbook of Simulation,
//    edited by Jerry Banks,
//    Wiley, 1998,
//    ISBN: 0471134031,
//    LC: T57.62.H37.
//
//    Peter Lewis, Allen Goodman, James Miller,
//    A Pseudo-Random Number Generator for the System/360,
//    IBM Systems Journal,
//    Volume 8, Number 2, 1969, pages 136-143.
//
//  Input:
//
//    int M, N, the number of rows and columns.
//
//    int &SEED, the "seed" value.
//
//  Output:
//
//    double R[M*N], a matrix of pseudorandom values.
//
//    int &SEED: an updated seed.
//
{
  int i;
  const int i4_huge = 2147483647;
  int j;
  int k;

  if ( seed == 0 )
  {
    cerr << "\n";
    cerr << "R8MAT_UNIFORM_01 - Fatal error!\n";
    cerr << "  Input value of SEED = 0.\n";
    exit ( 1 );
  }

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      k = seed / 127773;

      seed = 16807 * ( seed - k * 127773 ) - k * 2836;

      if ( seed < 0 )
      {
        seed = seed + i4_huge;
      }

      r[i+j*m] = ( double ) ( seed ) * 4.656612875E-10;
    }
  }
  return;
}
//****************************************************************************80

double *r8mat_uniform_01_new ( int m, int n, int &seed )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_uniform_01_new() returns a unit pseudorandom R8MAT.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8's,  stored as a vector
//    in column-major order.
//
//    This routine implements the recursion
//
//      seed = 16807 * seed mod ( 2^31 - 1 )
//      unif = seed / ( 2^31 - 1 )
//
//    The integer arithmetic never requires more than 32 bits,
//    including a sign bit.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    03 October 2005
//
//  Author:
//
//    John Burkardt
//
//  Reference:
//
//    Paul Bratley, Bennett Fox, Linus Schrage,
//    A Guide to Simulation,
//    Springer Verlag, pages 201-202, 1983.
//
//    Bennett Fox,
//    Algorithm 647:
//    Implementation and Relative Efficiency of Quasirandom
//    Sequence Generators,
//    ACM Transactions on Mathematical Software,
//    Volume 12, Number 4, pages 362-376, 1986.
//
//    Philip Lewis, Allen Goodman, James Miller,
//    A Pseudo-Random Number Generator for the System/360,
//    IBM Systems Journal,
//    Volume 8, pages 136-143, 1969.
//
//  Input:
//
//    int M, N, the number of rows and columns.
//
//    int &SEED, the "seed" value.
//
//  Output:
//
//    double R8MAT_UNIFORM_01_NEW[M*N], a matrix of pseudorandom values.
//
//    int &SEED: an updated seed.
//
{
  int i;
  const int i4_huge = 2147483647;
  int j;
  int k;
  double *r;

  r = new double[m*n];

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      k = seed / 127773;

      seed = 16807 * ( seed - k * 127773 ) - k * 2836;

      if ( seed < 0 )
      {
        seed = seed + i4_huge;
      }
      r[i+j*m] = ( double ) ( seed ) * 4.656612875E-10;
    }
  }

  return r;
}
//****************************************************************************80

void r8mat_uniform_ab ( int m, int n, double a, double b, int &seed, double r[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_uniform_ab() returns a scaled pseudorandom R8MAT.
//
//  Discussion:
//
//    An R8MAT is an array of R8's.
//
//    This routine implements the recursion
//
//      seed = ( 16807 * seed ) mod ( 2^31 - 1 )
//      u = seed / ( 2^31 - 1 )
//
//    The integer arithmetic never requires more than 32 bits,
//    including a sign bit.
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    09 April 2012
//
//  Author:
//
//    John Burkardt
//
//  Reference:
//
//    Paul Bratley, Bennett Fox, Linus Schrage,
//    A Guide to Simulation,
//    Second Edition,
//    Springer, 1987,
//    ISBN: 0387964673,
//    LC: QA76.9.C65.B73.
//
//    Bennett Fox,
//    Algorithm 647:
//    Implementation and Relative Efficiency of Quasirandom
//    Sequence Generators,
//    ACM Transactions on Mathematical Software,
//    Volume 12, Number 4, December 1986, pages 362-376.
//
//    Pierre L'Ecuyer,
//    Random Number Generation,
//    in Handbook of Simulation,
//    edited by Jerry Banks,
//    Wiley, 1998,
//    ISBN: 0471134031,
//    LC: T57.62.H37.
//
//    Peter Lewis, Allen Goodman, James Miller,
//    A Pseudo-Random Number Generator for the System/360,
//    IBM Systems Journal,
//    Volume 8, Number 2, 1969, pages 136-143.
//
//  Input:
//
//    int M, N, the number of rows and columns.
//
//    double A, B, the limits of the pseudorandom values.
//
//    int &SEED, the "seed" value.
//
//  Output:
//
//    double R[M*N], a matrix of pseudorandom values.
//
//    int &SEED: an updated seed.
//
{
  int i;
  const int i4_huge = 2147483647;
  int j;
  int k;

  if ( seed == 0 )
  {
    cerr << "\n";
    cerr << "R8MAT_UNIFORM_AB - Fatal error!\n";
    cerr << "  Input value of SEED = 0.\n";
    exit ( 1 );
  }

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      k = seed / 127773;

      seed = 16807 * ( seed - k * 127773 ) - k * 2836;

      if ( seed < 0 )
      {
        seed = seed + i4_huge;
      }

      r[i+j*m] = a + ( b - a ) * ( double ) ( seed ) * 4.656612875E-10;
    }
  }

  return;
}
//****************************************************************************80

double *r8mat_uniform_ab_new ( int m, int n, double a, double b, int &seed )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_uniform_ab_new() returns a new scaled pseudorandom R8MAT.
//
//  Discussion:
//
//    An R8MAT is an array of R8's.
//
//    This routine implements the recursion
//
//      seed = ( 16807 * seed ) mod ( 2^31 - 1 )
//      u = seed / ( 2^31 - 1 )
//
//    The integer arithmetic never requires more than 32 bits,
//    including a sign bit.
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    09 April 2012
//
//  Author:
//
//    John Burkardt
//
//  Reference:
//
//    Paul Bratley, Bennett Fox, Linus Schrage,
//    A Guide to Simulation,
//    Second Edition,
//    Springer, 1987,
//    ISBN: 0387964673,
//    LC: QA76.9.C65.B73.
//
//    Bennett Fox,
//    Algorithm 647:
//    Implementation and Relative Efficiency of Quasirandom
//    Sequence Generators,
//    ACM Transactions on Mathematical Software,
//    Volume 12, Number 4, December 1986, pages 362-376.
//
//    Pierre L'Ecuyer,
//    Random Number Generation,
//    in Handbook of Simulation,
//    edited by Jerry Banks,
//    Wiley, 1998,
//    ISBN: 0471134031,
//    LC: T57.62.H37.
//
//    Peter Lewis, Allen Goodman, James Miller,
//    A Pseudo-Random Number Generator for the System/360,
//    IBM Systems Journal,
//    Volume 8, Number 2, 1969, pages 136-143.
//
//  Input:
//
//    int M, N, the number of rows and columns.
//
//    double A, B, the limits of the pseudorandom values.
//
//    int &SEED, the "seed" value.
//
//  Output:
//
//    double R8MAT_UNIFORM_AB_NEW[M*N], a matrix of pseudorandom values.
//
//    int &SEED: an updated seed.
//
{
  int i;
  const int i4_huge = 2147483647;
  int j;
  int k;
  double *r;

  if ( seed == 0 )
  {
    cerr << "\n";
    cerr << "R8MAT_UNIFORM_AB_NEW - Fatal error!\n";
    cerr << "  Input value of SEED = 0.\n";
    exit ( 1 );
  }

  r = new double[m*n];

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      k = seed / 127773;

      seed = 16807 * ( seed - k * 127773 ) - k * 2836;

      if ( seed < 0 )
      {
        seed = seed + i4_huge;
      }

      r[i+j*m] = a + ( b - a ) * ( double ) ( seed ) * 4.656612875E-10;
    }
  }

  return r;
}
//****************************************************************************80

void r8mat_uniform_abvec ( int m, int n, double a[], double b[], int &seed, 
  double r[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_uniform_abvec() returns a scaled pseudorandom R8MAT.
//
//  Discussion:
//
//    An R8MAT is an array of R8's.
//
//    This routine implements the recursion
//
//      seed = ( 16807 * seed ) mod ( 2^31 - 1 )
//      u = seed / ( 2^31 - 1 )
//
//    The integer arithmetic never requires more than 32 bits,
//    including a sign bit.
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    09 April 2012
//
//  Author:
//
//    John Burkardt
//
//  Reference:
//
//    Paul Bratley, Bennett Fox, Linus Schrage,
//    A Guide to Simulation,
//    Second Edition,
//    Springer, 1987,
//    ISBN: 0387964673,
//    LC: QA76.9.C65.B73.
//
//    Bennett Fox,
//    Algorithm 647:
//    Implementation and Relative Efficiency of Quasirandom
//    Sequence Generators,
//    ACM Transactions on Mathematical Software,
//    Volume 12, Number 4, December 1986, pages 362-376.
//
//    Pierre L'Ecuyer,
//    Random Number Generation,
//    in Handbook of Simulation,
//    edited by Jerry Banks,
//    Wiley, 1998,
//    ISBN: 0471134031,
//    LC: T57.62.H37.
//
//    Peter Lewis, Allen Goodman, James Miller,
//    A Pseudo-Random Number Generator for the System/360,
//    IBM Systems Journal,
//    Volume 8, Number 2, 1969, pages 136-143.
//
//  Input:
//
//    int M, N, the number of rows and columns.
//
//    double A[M], B[M], the limits of the pseudorandom values.
//
//    int &SEED, the "seed" value.
//
//  Output:
//
//    double R[M*N], a matrix of pseudorandom values.
//
//    int &SEED: an updated seed.
//
{
  int i;
  const int i4_huge = 2147483647;
  int j;
  int k;

  if ( seed == 0 )
  {
    cerr << "\n";
    cerr << "R8MAT_UNIFORM_ABVEC - Fatal error!\n";
    cerr << "  Input value of SEED = 0.\n";
    exit ( 1 );
  }

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      k = seed / 127773;

      seed = 16807 * ( seed - k * 127773 ) - k * 2836;

      if ( seed < 0 )
      {
        seed = seed + i4_huge;
      }

      r[i+j*m] = a[i] + ( b[i] - a[i] ) * ( double ) ( seed ) * 4.656612875E-10;
    }
  }

  return;
}
//****************************************************************************80

double *r8mat_uniform_abvec_new ( int m, int n, double a[], double b[], 
  int &seed )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_uniform_abvec_new() returns a new scaled pseudorandom R8MAT.
//
//  Discussion:
//
//    An R8MAT is an array of R8's.
//
//    This routine implements the recursion
//
//      seed = ( 16807 * seed ) mod ( 2^31 - 1 )
//      u = seed / ( 2^31 - 1 )
//
//    The integer arithmetic never requires more than 32 bits,
//    including a sign bit.
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    09 April 2012
//
//  Author:
//
//    John Burkardt
//
//  Reference:
//
//    Paul Bratley, Bennett Fox, Linus Schrage,
//    A Guide to Simulation,
//    Second Edition,
//    Springer, 1987,
//    ISBN: 0387964673,
//    LC: QA76.9.C65.B73.
//
//    Bennett Fox,
//    Algorithm 647:
//    Implementation and Relative Efficiency of Quasirandom
//    Sequence Generators,
//    ACM Transactions on Mathematical Software,
//    Volume 12, Number 4, December 1986, pages 362-376.
//
//    Pierre L'Ecuyer,
//    Random Number Generation,
//    in Handbook of Simulation,
//    edited by Jerry Banks,
//    Wiley, 1998,
//    ISBN: 0471134031,
//    LC: T57.62.H37.
//
//    Peter Lewis, Allen Goodman, James Miller,
//    A Pseudo-Random Number Generator for the System/360,
//    IBM Systems Journal,
//    Volume 8, Number 2, 1969, pages 136-143.
//
//  Input:
//
//    int M, N, the number of rows and columns.
//
//    double A[M], B[M], the limits of the pseudorandom values.
//
//    int &SEED, the "seed" value.
//
//  Output:
//
//    double R8MAT_UNIFORM_ABVEC_NEW[M*N], a matrix of
//    pseudorandom values.
//
//    int &SEED: an updated seed.
//
{
  int i;
  const int i4_huge = 2147483647;
  int j;
  int k;
  double *r;

  if ( seed == 0 )
  {
    cerr << "\n";
    cerr << "R8MAT_UNIFORM_ABVEC_NEW - Fatal error!\n";
    cerr << "  Input value of SEED = 0.\n";
    exit ( 1 );
  }

  r = new double[m*n];

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      k = seed / 127773;

      seed = 16807 * ( seed - k * 127773 ) - k * 2836;

      if ( seed < 0 )
      {
        seed = seed + i4_huge;
      }

      r[i+j*m] = a[i] + ( b[i] - a[i] ) * ( double ) ( seed ) * 4.656612875E-10;
    }
  }

  return r;
}
//****************************************************************************80

double *r8mat_ut_solve ( int n, double a[], double b[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_ut_solve() solves a transposed upper triangular linear system.
//
//  Discussion:
//
//    An R8MAT is an MxN array of R8's, stored by (I,J) -> [I+J*M].
//
//    Given the upper triangular matrix A, the linear system to be solved is:
//
//      A' * x = b
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    22 October 2013
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of rows and columns of
//    the matrix A.
//
//    double A[N*N], the N by N upper triangular matrix.
//
//    double B[N], the right hand side of the linear system.
//
//  Output:
//
//    double R8MAT_UT_SOLVE[N], the solution of the linear system.
//
{
  int i;
  int j;
  double *x;
//
//  Solve U' * x = b.
//
  x = new double[n];

  for ( i = 0; i < n; i++ )
  {
    x[i] = b[i];
    for ( j = 0; j < i; j++ )
    {
      x[i] = x[i] - a[j+i*n] * x[j];
    }
    x[i] = x[i] / a[i+i*n];
  }

  return x;
}
//****************************************************************************80

double *r8mat_vand2 ( int n, double x[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_vand2() returns the N by N row Vandermonde matrix A.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//    The row Vandermonde matrix returned by this routine reads "across"
//    rather than down.  In particular, each row begins with a 1, followed by
//    some value X, followed by successive powers of X.
//
//    The formula for the matrix entries is:
//
//      A(I,J) = X(I)^(J-1)
//
//  Properties:
//
//    A is nonsingular if, and only if, the X values are distinct.
//
//    The determinant of A is
//
//      det(A) = product ( 2 <= I <= N ) (
//        product ( 1 <= J <= I-1 ) ( ( X(I) - X(J) ) ) ).
//
//    The matrix A is generally ill-conditioned.
//
//  Example:
//
//    N = 5, X = (2, 3, 4, 5, 6)
//
//    1 2  4   8   16
//    1 3  9  27   81
//    1 4 16  64  256
//    1 5 25 125  625
//    1 6 36 216 1296
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    12 September 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the order of the matrix desired.
//
//    double X[N], the values that define A.
//
//  Output:
//
//    double R8MAT_VAND2[N*N], the N by N row Vandermonde matrix.
//
{
  double *a;
  int i;
  int j;

  a = new double[n*n];

  for ( i = 0; i < n; i++ )
  {
    for ( j = 0; j < n; j++ )
    {
      if ( j == 0 && x[i] == 0.0 )
      {
        a[i+j*n] = 1.0;
      }
      else
      {
        a[i+j*n] = pow ( x[i], j );
      }
    }
  }

  return a;
}
//****************************************************************************80

double *r8mat_variance_columns ( int m, int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_variance_columns() returns the column variances of an R8MAT.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    04 October 2018
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, the number of rows in A.
//
//    int N, the number of columns in A.
//
//    double A[M*N], the M by N matrix.
//
//  Output:
//
//    double R8MAT_VARIANCE_COLUMNS[N], the column variances.
//
{
  int i;
  int j;
  double mean;
  double variance;
  double *variance_columns;

  variance_columns = new double[n];

  for ( j = 0; j < n; j++ )
  {
    mean = 0.0;
    for ( i = 0; i < m; i++ )
    {
      mean = mean + a[i+j*m];
    }
    mean = mean / ( double ) m;
    variance = 0.0;
    for ( i = 0; i < m; i++ )
    {
      variance = variance + pow ( a[i+j*m] - mean, 2 );
    }
    variance = variance / ( double ) ( m - 1 );
    variance_columns[j] = variance;
  }

  return variance_columns;
}
//****************************************************************************80

double *r8mat_variance_rows ( int m, int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_variance_rows() returns the row variances of an R8MAT.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    04 October 2018
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, the number of rows in A.
//
//    int N, the number of columns in A.
//
//    double A[M*N], the M by N matrix.
//
//  Output:
//
//    double R8MAT_VARIANCE_ROWS[M], the row variances.
//
{
  int i;
  int j;
  double mean;
  double *variance_rows;
  double variance;

  variance_rows = new double[m];

  for ( i = 0; i < m; i++ )
  {
    mean = 0.0;
    for ( j = 0; j < n; j++ )
    {
      mean = mean + a[i+j*m];
    }
    mean = mean / ( double ) n;
    variance = 0.0;
    for ( j = 0; j < n; j++ )
    {
      variance = variance + pow ( a[i+j*m] - mean, 2 );
    }
    variance = variance / ( double ) ( n - 1 );
    variance_rows[i] = variance;
  }

  return variance_rows;
}

//****************************************************************************80

double r8mat_vtmv ( int m, int n, double x[], double a[], double y[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_vtmv() multiplies computes the scalar x' * A * y.
//
//  Discussion:
//
//    An R8MAT is an MxN array of R8's, stored by (I,J) -> [I+J*M].
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    10 June 2013
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, N, the number of rows and columns of
//    the matrix.
//
//    double X[N], the first vector factor.
//
//    double A[M*N], the M by N matrix.
//
//    double Y[M], the second vector factor.
//
//  Output:
//
//    double R8MAT_VTMV, the value of X' * A * Y.
//
{
  int i;
  int j;
  double vtmv;

  vtmv = 0.0;
  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      vtmv = vtmv + x[i] * a[i+j*m] * y[j];
    }
  }
  return vtmv;
}
//****************************************************************************80

void r8mat_zeros ( int m, int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_zeros() zeroes an R8MAT.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    16 October 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, N, the number of rows and columns.
//
//  Output:
//
//    double A[M*N], a matrix of zeroes.
//
{
  int i;
  int j;

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      a[i+j*m] = 0.0;
    }
  }
  return;
}
//****************************************************************************80

double *r8mat_zeros_new ( int m, int n )

//****************************************************************************80
//
//  Purpose:
//
//    r8mat_zeros_new() returns a new zeroed R8MAT.
//
//  Discussion:
//
//    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
//    in column-major order.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    03 October 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, N, the number of rows and columns.
//
//  Output:
//
//    double R8MAT_ZEROS_NEW[M*N], the new zeroed matrix.
//
{
  double *a;
  int i;
  int j;

  a = new double[m*n];

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      a[i+j*m] = 0.0;
    }
  }
  return a;
}
//****************************************************************************80

double r8plu_det ( int n, int pivot[], double lu[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8plu_det() computes the determinant of a real PLU matrix.
//
//  Discussion:
//
//    The matrix should have been factored by R8MAT_TO_R8PLU.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    22 August 2003
//
//  Author:
//
//    John Burkardt
//
//  Reference:
//
//    Jack Dongarra, Jim Bunch, Cleve Moler, Pete Stewart,
//    LINPACK User's Guide,
//    SIAM, 1979
//
//  Input:
//
//    int N, the order of the matrix.
//    N must be positive.
//
//    int PIVOT[N], the pivot vector computed by R8MAT_TO_R8PLU.
//
//    double LU[N*N], the LU factors computed by R8MAT_TO_R8PLU.
//
//  Output:
//
//    double R8PLU_DET, the determinant of the matrix.
//
{
  double det;
  int i;

  det = 1.0;

  for ( i = 0; i < n; i++ )
  {
    det = det * lu[i+i*n];
    if ( pivot[i] != i+1 )
    {
      det = -det;
    }
  }

  return det;
}
//****************************************************************************80

void r8plu_inverse ( int n, int pivot[], double lu[], double a_inverse[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8plu_inverse() computes the inverse of a real PLU matrix.
//
//  Discussion:
//
//    The matrix should have been factored by R8MAT_TO_R8PLU.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    22 August 2003
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the order of the matrix A.
//
//    int PIVOT[N], the pivot vector from R8MAT_TO_R8PLU.
//
//    double LU[N*N], the LU factors computed by R8MAT_TO_R8PLU.
//
//  Output:
//
//    double A_INVERSE[N*N], the inverse of the original matrix
//    A that was factored by R8MAT_TO_R8PLU.
//
{
  int i;
  int j;
  int k;
  double temp;
  double *work;
//
  work = new double[n];

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < n; i++ )
    {
      a_inverse[i+j*n] = lu[i+j*n];
    }
  }
//
//  Compute Inverse(U).
//
  for ( k = 1; k <= n; k++ )
  {
    a_inverse[k-1+(k-1)*n]     = 1.0 / a_inverse[k-1+(k-1)*n];
    for ( i = 1; i <= k-1; i++ )
    {
      a_inverse[i-1+(k-1)*n] = -a_inverse[i-1+(k-1)*n] * a_inverse[k-1+(k-1)*n];
    }

    for ( j = k+1; j <= n; j++ )
    {
      temp                     = a_inverse[k-1+(j-1)*n];
      a_inverse[k-1+(j-1)*n]   = 0.0;
      for ( i = 1; i <= k; i++ )
      {
        a_inverse[i-1+(j-1)*n] = a_inverse[i-1+(j-1)*n]
          + temp * a_inverse[i-1+(k-1)*n];
      }
    }
  }
//
//  Form Inverse(U) * Inverse(L).
//
  for ( k = n-1; 1 <= k; k-- )
  {
    for ( i = k+1; i <= n; i++ )
    {
      work[i-1] = a_inverse[i-1+(k-1)*n];
      a_inverse[i-1+(k-1)*n] = 0.0;
    }

    for ( j = k+1; j <= n; j++ )
    {
      for ( i = 1; i <= n; i++ )
      {
        a_inverse[i-1+(k-1)*n] = a_inverse[i-1+(k-1)*n]
          + a_inverse[i-1+(j-1)*n] * work[j-1];
      }
    }

    if ( pivot[k-1] != k )
    {
      for ( i = 1; i <= n; i++ )
      {
        temp                            = a_inverse[i-1+(k-1)*n];
        a_inverse[i-1+(k-1)*n]          = a_inverse[i-1+(pivot[k-1]-1)*n];
        a_inverse[i-1+(pivot[k-1]-1)*n] = temp;
      }
    }
  }

  delete [] work;

  return;
}
//****************************************************************************80

void r8plu_mul ( int n, int pivot[], double lu[], double x[], double b[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8plu_mul() computes A * x using the PLU factors of A.
//
//  Discussion:
//
//    It is assumed that R8MAT_TO_R8PLU has computed the PLU factors of
//    the matrix A.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    22 August 2003
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the order of the matrix.
//    N must be positive.
//
//    int PIVOT[N], the pivot vector computed by R8MAT_TO_R8PLU.
//
//    double LU[N*N], the matrix factors computed by R8MAT_TO_R8PLU.
//
//    double X[N], the vector to be multiplied.
//
//  Output:
//
//    double B[N], the result of the multiplication.
//
{
  int i;
  int j;
  int k;
  double temp;
//
  for ( i = 0; i < n; i++ )
  {
    b[i] = x[i];
  }
//
//  Y = U * X.
//
  for ( j = 1; j <= n; j++ )
  {
    for ( i = 0; i < j-1; i++ )
    {
      b[i] = b[i] + lu[i+(j-1)*n] * b[j-1];
    }
    b[j-1] = lu[j-1+(j-1)*n] * b[j-1];
  }
//
//  B = PL * Y = PL * U * X = A * x.
//
  for ( j = n-1; 1 <= j; j-- )
  {
    for ( i = j; i < n; i++ )
    {
      b[i] = b[i] - lu[i+(j-1)*n] * b[j-1];
    }

    k = pivot[j-1];

    if ( k != j )
    {
      temp = b[k-1];
      b[k-1] = b[j-1];
      b[j-1] = temp;
    }
  }

  return;
}
//****************************************************************************80

void r8plu_sol ( int n, int pivot[], double lu[], double b[], double x[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8plu_sol() solves a linear system A*x=b from the PLU factors.
//
//  Discussion:
//
//    The PLU factors should have been computed by R8MAT_TO_R8PLU.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    22 August 2003
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the order of the matrix.
//
//    int PIVOT[N], the pivot vector from R8MAT_TO_R8PLU.
//
//    double LU[N*N], the LU factors from R8MAT_TO_R8PLU.
//
//    double B[N], the right hand side vector.
//
//  Output:
//
//    double X[N], the solution vector.
//
{
  int i;
  int j;
  int k;
  double temp;
//
//  Solve PL * Y = B.
//
  for ( i = 0; i < n; i++ )
  {
    x[i] = b[i];
  }

  for ( k = 1; k <= n-1; k++ )
  {
    j = pivot[k-1];

    if ( j != k )
    {
      temp   = x[j-1];
      x[j-1] = x[k-1];
      x[k-1] = temp;
    }

    for ( i = k+1; i <= n; i++ )
    {
      x[i-1] = x[i-1] + lu[i-1+(k-1)*n] * x[k-1];
    }
  }
//
//  Solve U * X = Y.
//
  for ( k = n; 1 <= k; k-- )
  {
    x[k-1] = x[k-1] / lu[k-1+(k-1)*n];
    for ( i = 1; i <= k-1; i++ )
    {
      x[i-1] = x[i-1] - lu[i-1+(k-1)*n] * x[k-1];
    }
  }

  return;
}
//****************************************************************************80

void r8plu_to_r8mat ( int n, int pivot[], double lu[], double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8plu_to_r8mat() recovers the matrix A that was factored by R8MAT_TO_R8PLU.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    22 August 2003
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the order of the matrix.
//    N must be positive.
//
//    int PIVOT[N], the pivot vector computed by R8MAT_TO_R8PLU.
//
//    double LU[N*N], the matrix factors computed by R8MAT_TO_R8PLU.
//
//  Output:
//
//    double A[N*N], the matrix whose factors are represented by
//    LU and PIVOT.
//
{
  int i;
  int j;
  int k;
  double temp;

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < n; i++ )
    {
      if ( i == j )
      {
        a[i+j*n] = 1.0;
      }
      else
      {
        a[i+j*n] = 0.0;
      }
    }
  }

  for ( j = 1; j <= n; j++ )
  {
    for ( i = 1; i <= n; i++ )
    {
      for ( k = 1; k <= i-1; k++ )
      {
        a[k-1+(j-1)*n] = a[k-1+(j-1)*n] + lu[k-1+(i-1)*n] * a[i-1+(j-1)*n];
      }
      a[i-1+(j-1)*n] = lu[i-1+(i-1)*n] * a[i-1+(j-1)*n];
    }
//
//  B = PL * Y = PL * U * X = A * x.
//
    for ( i = n-1; 1 <= i; i-- )
    {
      for ( k = i+1; k <= n; k++ )
      {
        a[k-1+(j-1)*n] = a[k-1+(j-1)*n] - lu[k-1+(i-1)*n] * a[i-1+(j-1)*n];
      }

      k = pivot[i-1];

      if ( k != i )
      {
        temp           = a[k-1+(j-1)*n];
        a[k-1+(j-1)*n] = a[i-1+(j-1)*n];
        a[i-1+(j-1)*n] = temp;
      }
    }
  }

  return;
}
//****************************************************************************80

void r8pp_delete ( int m, int n, double **a )

//****************************************************************************80
//
//  Purpose:
//
//    r8pp_delete() frees the memory set aside by R8PP_NEW.
//
//  Discussion:
//
//    An R8PP is a pointer to pointers to R8's, and is a sort of
//    variably-dimensioned matrix.
//
//    This function releases the memory associated with an array that was 
//    created by a command like:
//
//      double **a;
//      a = r8pp_new ( m, n );
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    07 November 2011
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, N, the number of rows and columns in the array.
//
//    double **A, the pointer to the pointers.
//
{
  int i;

  for ( i = 0; i < m; i++ )
  {
    delete [] a[i];
  }

  delete [] a;

  return;
}
//****************************************************************************80

double **r8pp_new ( int m, int n )

//****************************************************************************80
//
//  Purpose:
//
//    r8pp_new() allocates a new R8PP.
//
//  Discussion:
//
//    An R8PP is a pointer to pointers to R8's, and is a sort of
//    variably-dimensioned matrix.
//
//    A declaration of the form
//      double **a;
//    is necesary.  Then an assignment of the form:
//      a = r8pp_new ( m, n );
//    allows the user to assign entries to the matrix using typical
//    2D array notation:
//      a[2][3] = 17;
//      y = a[1][0];
//    and so on.
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    07 November 2011
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, N, the number of rows and columns in the matrix.
//
//  Output:
//
//    double **R8PP_NEW, a pointer to the pointers to the M by N array.
//
{
  double **a;
  int i;

  a = new double *[m];

  if ( a == NULL )
  {
    cerr << "\n";
    cerr << "R8PP_NEW - Fatal error!\n";
    cerr << "  Unable to allocate row pointer array.\n";
    exit ( 1 );
  }

  for ( i = 0; i < m; i++ )
  {
    a[i] = new double[n];
    if ( a[i] == NULL )
    {
      cerr << "\n";
      cerr << "R8PP_NEW - Fatal error!\n";
      cerr << "  Unable to allocate row array.\n";
      exit ( 1 );
    }
  }

  return a;
}
//****************************************************************************80

int r8r8_compare ( double x1, double y1, double x2, double y2 )

//****************************************************************************80
//
//  Purpose:
//
//    r8r8_compare() compares two R8R8's.
//
//  Discussion:
//
//    An R8R8 is simply a pair of R8 values, stored separately.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    07 October 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double X1, Y1, the first vector.
//
//    double X2, Y2, the second vector.
//
//  Output:
//
//    int R8R8_COMPARE:
//    -1, (X1,Y1) < (X2,Y2);
//     0, (X1,Y1) = (X2,Y2);
//    +1, (X1,Y1) > (X2,Y2).
//
{
  int value;

  if ( x1 < x2 )
  {
    value = -1;
  }
  else if ( x2 < x1 )
  {
    value = +1;
  }
  else if ( y1 < y2 )
  {
    value = -1;
  }
  else if ( y2 < y1 )
  {
    value = +1;
  }
  else
  {
    value = 0;
  }

  return value;
}
//****************************************************************************80

void r8r8_print ( double a1, double a2, string title )

//****************************************************************************80
//
//  Purpose:
//
//    r8r8_print() prints an R8R8.
//
//  Discussion:
//
//    An R8R8 is a pair of R8 values, regarded as a single item.
//
//    A format is used which suggests a coordinate pair:
//
//  Example:
//
//    Center : ( 1.23, 7.45 )
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    30 July 2009
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double A1, A2, the coordinates of the vector.
//
//    string TITLE, a title.
//
{
  cout << "  " << title << " : ";
  cout << "  ( " << setw(12) << a1
       << ", "   << setw(12) << a2 << " )\n";

  return;
}
//****************************************************************************80

int r8r8r8_compare ( double x1, double y1, double z1, double x2, double y2,
  double z2 )

//****************************************************************************80
//
//  Purpose:
//
//    r8r8r8_compare() compares two R8R8R8's.
//
//  Discussion:
//
//    An R8R8R8 is simply 3 R8 values, stored as scalars.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    07 October 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double X1, Y1, Z1, the first vector.
//
//    double X2, Y2, Z2, the second vector.
//
//  Output:
//
//    int R8R8R8_COMPARE:
//    -1, (X1,Y1,Z1) < (X2,Y2,Z2);
//     0, (X1,Y1,Z1) = (X2,Y2,Z2);
//    +1, (X1,Y1,Z1) > (X2,Y2,Z2).
//
{
  int value;

  if ( x1 < x2 )
  {
    value = -1;
  }
  else if ( x2 < x1 )
  {
    value = +1;
  }
  else if ( y1 < y2 )
  {
    value = -1;
  }
  else if ( y2 < y1 )
  {
    value = +1;
  }
  else if ( z1 < z2 )
  {
    value = -1;
  }
  else if ( z2 < z1 )
  {
    value = +1;
  }
  else
  {
    value = 0;
  }

  return value;
}
//****************************************************************************80

void r8r8r8vec_index_insert_unique ( int maxn, int &n, double x[], double y[],
  double z[], int indx[], double xval, double yval, double zval, int &ival,
  int &ierror )

//****************************************************************************80
//
//  Purpose:
//
//    r8r8r8vec_index_insert_unique() inserts a unique R8R8R8 value in an indexed sorted list.
//
//  Discussion:
//
//    If the input value does not occur in the current list, it is added,
//    and N, X, Y, Z and INDX are updated.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    10 October 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int MAXN, the maximum size of the list.
//
//    int &N, the size of the list.
//
//    double X[N], Y[N], Z[N], the R8R8R8 vector.
//
//    int INDX[N], the sort index of the list.
//
//    double XVAL, YVAL, ZVAL, the value to be inserted
//    if it is not already in the list.
//
//  Output:
//
//    int &N, the updated size of the list.
//
//    double X[N], Y[N], Z[N], the updated R8R8R8 vector.
//
//    int INDX[N], the updated sort index of the list.
//
//    int &IVAL, the index in X, Y, Z corresponding to the
//    value XVAL, YVAL, ZVAL.
//
//    int &IERROR, 0 for no error, 1 if an error occurred.
//
{
  int equal;
  int i;
  int less;
  int more;

  ierror = 0;

  if ( n <= 0 )
  {
    if ( maxn <= 0 )
    {
      ierror = 1;
      cerr << "\n";
      cerr << "R8R8R8VEC_INDEX_INSERT_UNIQUE - Fatal error!\n";
      cerr << "  Not enough space to store new data.\n";
      return;
    }
    n = 1;
    x[0] = xval;
    y[0] = yval;
    z[0] = zval;
    indx[0] = 1;
    ival = 1;
    return;
  }
//
//  Does ( XVAL, YVAL, ZVAL ) already occur in ( X, Y, Z)?
//
  r8r8r8vec_index_search ( n, x, y, z, indx, xval, yval, zval,
    less, equal, more );

  if ( equal == 0 )
  {
    if ( maxn <= n )
    {
      ierror = 1;
      cerr << "\n";
      cerr << "R8R8R8VEC_INDEX_INSERT_UNIQUE - Fatal error!\n";
      cerr << "  Not enough space to store new data.\n";
      return;
    }

    x[n] = xval;
    y[n] = yval;
    z[n] = zval;
    ival = n + 1;
    for ( i = n - 1; more - 1 <= i; i-- )
    {
      indx[i+1] = indx[i];
    }
    
    indx[more-1] = n + 1;
    n = n + 1;
  }
  else
  {
    ival = indx[equal-1];
  }

  return;
}
//****************************************************************************80

void r8r8r8vec_index_search ( int n, double x[], double y[], double z[],
  int indx[], double xval, double yval, double zval, int &less, int &equal,
  int &more )

//****************************************************************************80
//
//  Purpose:
//
//    r8r8r8vec_index_search() searches for an R8R8R8 value in an indexed sorted list.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    10 October 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the size of the list.
//
//    double X[N], Y[N], Z[N], the list.
//
//    int INDX[N], the sort index of the list.
//
//    double XVAL, YVAL, ZVAL, the value to be sought.
//
//  Output:
//
//    int &LESS, &EQUAL, &MORE, the indexes in INDX of the
//    entries of X that are just less than, equal to, and just greater
//    than XVAL.  If XVAL does not occur in X, then EQUAL is zero.
//    If XVAL is the minimum entry of X, then LESS is 0.  If XVAL
//    is the greatest entry of X, then MORE is N+1.
//
{
  int compare;
  int hi;
  int lo;
  int mid;
  double xhi;
  double xlo;
  double xmid;
  double yhi;
  double ylo;
  double ymid;
  double zhi;
  double zlo;
  double zmid;

  if ( n <= 0 )
  {
    less = 0;
    equal = 0;
    more = 0;
    return;
  }

  lo = 1;
  hi = n;

  xlo = x[indx[lo-1]-1];
  ylo = y[indx[lo-1]-1];
  zlo = z[indx[lo-1]-1];

  xhi = x[indx[hi-1]-1];
  yhi = y[indx[hi-1]-1];
  zhi = z[indx[hi-1]-1];

  compare = r8r8r8_compare ( xval, yval, zval, xlo, ylo, zlo );

  if ( compare == -1 )
  {
    less = 0;
    equal = 0;
    more = 1;
    return;
  }
  else if ( compare == 0 )
  {
    less = 0;
    equal = 1;
    more = 2;
    return;
  }

  compare = r8r8r8_compare ( xval, yval, zval, xhi, yhi, zhi );

  if ( compare == 1 )
  {
    less = n;
    equal = 0;
    more = n + 1;
    return;
  }
  else if ( compare == 0 )
  {
    less = n - 1;
    equal = n;
    more = n + 1;
    return;
  }

  for ( ; ; )
  {
    if ( lo + 1 == hi )
    {
      less = lo;
      equal = 0;
      more = hi;
      return;
    }

    mid = ( lo + hi ) / 2;
    xmid = x[indx[mid-1]-1];
    ymid = y[indx[mid-1]-1];
    zmid = z[indx[mid-1]-1];

    compare = r8r8r8_compare ( xval, yval, zval, xmid, ymid, zmid );

    if ( compare == 0 )
    {
      equal = mid;
      less = mid - 1;
      more = mid + 1;
      return;
    }
    else if ( compare == -1 )
    {
      hi = mid;
    }
    else if ( compare == +1 )
    {
      lo = mid;
    }
  }

  return;
}
//****************************************************************************80

void r8r8vec_index_insert_unique ( int maxn, int &n, double x[], double y[],
  int indx[], double xval, double yval, int &ival, int &ierror )

//****************************************************************************80
//
//  Purpose:
//
//    r8r8vec_index_insert_unique() inserts a unique R8R8 value in an indexed sorted list.
//
//  Discussion:
//
//    If the input value does not occur in the current list, it is added,
//    and N, X, Y and INDX are updated.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    10 October 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int MAXN, the maximum size of the list.
//
//    int &N, the size of the list.
//
//    double X[N], Y[N], the list of R8R8 vectors.
//
//    int INDX[N], the sort index of the list.
//
//    double XVAL, YVAL, the value to be inserted if it is
//    not already in the list.
//
//  Output:
//
//    int &N, the updated size of the list.
//
//    double X[N], Y[N], the updated list of R8R8 vectors.
//
//    int INDX[N], the updated sort index of the list.
//
//    int &IVAL, the index in X, Y corresponding to the
//    value XVAL, YVAL.
//
//    int &IERROR, 0 for no error, 1 if an error occurred.
//
{
  int equal;
  int i;
  int less;
  int more;

  ierror = 0;

  if ( n <= 0 )
  {
    if ( maxn <= 0 )
    {
      cerr << "\n";
      cerr << "R8R8VEC_INDEX_INSERT_UNIQUE - Fatal error!\n";
      cerr << "  Not enough space to store new data.\n";
      exit ( 1 );
    }

    n = 1;
    x[0] = xval;
    y[0] = yval;
    indx[0] = 1;
    ival = 1;
    return;
  }
//
//  Does ( XVAL, YVAL ) already occur in ( X, Y )?
//
  r8r8vec_index_search ( n, x, y, indx, xval, yval, less, equal, more );

  if ( equal == 0 )
  {
    if ( maxn <= n )
    {
      cerr << "\n";
      cerr << "R8R8VEC_INDEX_INSERT_UNIQUE - Fatal error!\n";
      cerr << "  Not enough space to store new data.\n";
      exit ( 1 );
    }

    x[n] = xval;
    y[n] = yval;
    ival = n + 1;
    for ( i = n - 1; more - 1 <= i; i-- )
    {
      indx[i+1] = indx[i];
    }
    indx[more-1] = n + 1;
    n = n + 1;
  }
  else
  {
    ival = indx[equal-1];
  }

  return;
}
//****************************************************************************80

void r8r8vec_index_search ( int n, double x[], double y[], int indx[],
  double xval, double yval, int &less, int &equal, int &more )

//****************************************************************************80
//
//  Purpose:
//
//    r8r8vec_index_search() searches for an R8R8 value in an indexed sorted list.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    10 October 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the size of the current list.
//
//    double X[N], Y[N], the list.
//
//    int INDX[N], the sort index of the list.
//
//    double XVAL, YVAL, the value to be sought.
//
//  Output:
//
//    int &LESS, &EQUAL, &MORE, the indexes in INDX of the
//    entries of X that are just less than, equal to, and just greater
//    than XVAL.  If XVAL does not occur in X, then EQUAL is zero.
//    If XVAL is the minimum entry of X, then LESS is 0.  If XVAL
//    is the greatest entry of X, then MORE is N+1.
//
{
  int compare;
  int hi;
  int lo;
  int mid;
  double xhi;
  double xlo;
  double xmid;
  double yhi;
  double ylo;
  double ymid;

  if ( n <= 0 )
  {
    less = 0;
    equal = 0;
    more = 0;
    return;
  }

  lo = 1;
  hi = n;

  xlo = x[indx[lo-1]-1];
  ylo = y[indx[lo-1]-1];

  xhi = x[indx[hi-1]-1];
  yhi = y[indx[hi-1]-1];

  compare = r8r8_compare ( xval, yval, xlo, ylo );

  if ( compare == -1 )
  {
    less = 0;
    equal = 0;
    more = 1;
    return;
  }
  else if ( compare == 0 )
  {
    less = 0;
    equal = 1;
    more = 2;
    return;
  }

  compare = r8r8_compare ( xval, yval, xhi, yhi );

  if ( compare == 1 )
  {
    less = n;
    equal = 0;
    more = n + 1;
    return;
  }
  else if ( compare == 0 )
  {
    less = n - 1;
    equal = n;
    more = n + 1;
    return;
  }

  for ( ; ; )
  {
    if ( lo + 1 == hi )
    {
      less = lo;
      equal = 0;
      more = hi;
      return;
    }

    mid = ( lo + hi ) / 2;
    xmid = x[indx[mid-1]-1];
    ymid = y[indx[mid-1]-1];

    compare = r8r8_compare ( xval, yval, xmid, ymid );

    if ( compare == 0 )
    {
      equal = mid;
      less = mid - 1;
      more = mid + 1;
      return;
    }
    else if ( compare == -1 )
    {
      hi = mid;
    }
    else if ( compare == +1 )
    {
      lo = mid;
    }
  }

  return;
}
//****************************************************************************80

double **r8rmat_copy_new ( int m, int n, double **a )

//****************************************************************************80
//
//  Purpose:
//
//    r8rmat_copy_new() makes a new copy of an R8RMAT .
//
//  Discussion:
//
//    An R8RMAT is a matrix stored in row major form, using M pointers
//    to the beginnings of rows.
//
//    A declaration of the form
//      double **a;
//    is necesary.  Then an assignment of the form:
//      a = r8rmat_new ( m, n );
//    allows the user to assign entries to the matrix using typical
//    2D array notation:
//      a[2][3] = 17.0;
//      y = a[1][0];
//    and so on.
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    27 May 2014
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, N, the number of rows and columns.
//
//    double **A, the array to copy.
//
//  Output:
//
//    double **R8RMAT_COPY_NEW, the copied array.
//
{
  double **b;
  int i;
  int j;

  b = r8rmat_new ( m, n );

  for ( i = 0; i < m; i++ )
  {
    for ( j = 0; j < n; j++ )
    {
      b[i][j] = a[i][j];
    }
  }
  return b;
}
//****************************************************************************80

void r8rmat_delete ( int m, int n, double **a )

//****************************************************************************80
//
//  Purpose:
//
//    r8rmat_delete() frees memory associated with an R8RMAT.
//
//  Discussion:
//
//    This function releases the memory associated with an R8RMAT.
// 
//    An R8RMAT is a row-major array that was created by a 
//    command like:
//
//      double **a;
//      a = r8rmat_new ( m, n );
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    09 September 2013
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, N, the number of rows and columns in the array.
//
//    double **A, the pointer to the array.
//
{
  int i;

  for ( i = 0; i < m; i++ )
  {
    delete [] a[i];
  }

  delete [] a;

  return;
}
//****************************************************************************80

double *r8rmat_fs_new ( int n, double **a, double b[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8rmat_fs_new() factors and solves an R8RMAT system with one right hand side.
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    27 May 2014
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the order of the matrix.
//    N must be positive.
//
//    double **A, the coefficient matrix of the linear system.
//
//    double B[N], the right hand side of the linear system.
//
//  Output:
//
//    double R8RMAT_FS_NEW[N], the solution of the linear system.
//
{
  double **a2;
  int i;
  int j;
  int k;
  int p;
  double t;
  double *x;

  a2 = r8rmat_copy_new ( n, n, a );
  x = r8vec_copy_new ( n, b );

  for ( k = 0; k < n; k++ )
  {
//
//  Find the maximum element in column I.
//
    p = k;

    for ( i = k + 1; i < n; i++ )
    {
      if ( fabs ( a2[p][k] ) < fabs ( a2[i][k] ) )
      {
        p = i;
      }
    }

    if ( a2[p][k] == 0.0 )
    {
      cerr << "\n";
      cerr << "R8RMAT_FS_NEW - Fatal error!\n";
      cerr << "  Zero pivot on step " << k << "\n";
      exit ( 1 );
    }
//
//  Switch rows K and P.
//
    if ( k != p )
    {
      for ( j = 0; j < n; j++ )
      {
        t        = a2[k][j];
        a2[k][j] = a2[p][j];
        a2[p][j] = t;
      }
      t    = x[k];
      x[k] = x[p];
      x[p] = t;
    }
//
//  Scale the pivot row.
//
    t = a2[k][k];
    a2[k][k] = 1.0;
    for ( j = k + 1; j < n; j++ )
    {
      a2[k][j] = a2[k][j] / t;
    }
    x[k] = x[k] / t;
//
//  Use the pivot row to eliminate lower entries in that column.
//
    for ( i = k + 1; i < n; i++ )
    {
      if ( a2[i][k] != 0.0 )
      {
        t = - a2[i][k];
        a2[i][k] = 0.0;
        for ( j = k + 1; j < n; j++ )
        {
          a2[i][j] = a2[i][j] + t * a2[k][j];
        }
        x[i] = x[i] + t * x[k];
      }
    }
  }
//
//  Back solve.
//
  for ( j = n - 1; 1 <= j; j-- )
  {
    for ( i = 0; i < j; i++ )
    {
      x[i] = x[i] - a2[i][j] * x[j];
    }
  }

  r8rmat_delete ( n, n, a2 );

  return x;
}
//****************************************************************************80

double **r8rmat_new ( int m, int n )

//****************************************************************************80
//
//  Purpose:
//
//    r8rmat_new() allocates a new R8RMAT.
//
//  Discussion:
//
//    An R8RMAT is a row-major array that was created by a 
//    command like:
//
//      double **a;
//      a = r8rmat_new ( m, n );
//
//    The user assigns entries to the matrix using typical
//    2D array notation:
//      a[2][3] = 17.0;
//      y = a[1][0];
//    and so on.
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    09 September 2013
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, N, the number of rows and columns in the matrix.
//
//  Output:
//
//    double **R8RMAT_NEW, a new matrix.
//
{
  double **a;
  int i;

  a = new double *[m];

  if ( a == NULL )
  {
    cerr << "\n";
    cerr << "R8RMAT_NEW - Fatal error!\n";
    cerr << "  Unable to allocate row pointer array.\n";
    exit ( 1 );
  }

  for ( i = 0; i < m; i++ )
  {
    a[i] = new double[n];
    if ( a[i] == NULL )
    {
      cerr << "\n";
      cerr << "R8RMAT_NEW - Fatal error!\n";
      cerr << "  Unable to allocate row array.\n";
      exit ( 1 );
    }
  }

  return a;
}
//****************************************************************************80

void r8rmat_print ( int m, int n, double **a, string title )

//****************************************************************************80
//
//  Purpose:
//
//    r8rmat_print() prints an R8RMAT.
//
//  Discussion:
//
//    An R8RMAT is a row-major array that was created by a 
//    command like:
//
//      double **a;
//      a = r8rmat_new ( m, n );
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    10 September 2009
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, the number of rows in A.
//
//    int N, the number of columns in A.
//
//    double **A = A[M][N], the M by N matrix.
//
//    string TITLE, a title.
//
{
  r8rmat_print_some ( m, n, a, 1, 1, m, n, title );

  return;
}
//****************************************************************************80

void r8rmat_print_some ( int m, int n, double **a, int ilo, int jlo, int ihi,
  int jhi, string title )

//****************************************************************************80
//
//  Purpose:
//
//    r8rmat_print_some() prints some of an R8RMAT.
//
//  Discussion:
//
//    An R8RMAT is a row-major array that was created by a 
//    command like:
//
//      double **a;
//      a = r8rmat_new ( m, n );
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    26 June 2013
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, the number of rows of the matrix.
//    M must be positive.
//
//    int N, the number of columns of the matrix.
//    N must be positive.
//
//    double **A = A[M][N], the matrix.
//
//    int ILO, JLO, IHI, JHI, designate the first row and
//    column, and the last row and column to be printed.
//
//    string TITLE, a title.
//
{
# define INCX 5

  int i;
  int i2hi;
  int i2lo;
  int j;
  int j2hi;
  int j2lo;

  cout << "\n";
  cout << title << "\n";

  if ( m <= 0 || n <= 0 )
  {
    cout << "\n";
    cout << "  (None)\n";
    return;
  }
//
//  Print the columns of the matrix, in strips of 5.
//
  for ( j2lo = jlo; j2lo <= jhi; j2lo = j2lo + INCX )
  {
    j2hi = j2lo + INCX - 1;
    if ( n < j2hi )
    {
      j2hi = n;
    }
    if ( jhi < j2hi )
    {
      j2hi = jhi;
    }
    cout << "\n";
//
//  For each column J in the current range...
//
//  Write the header.
//
    cout << "  Col:    ";
    for ( j = j2lo; j <= j2hi; j++ )
    {
      cout << setw(7) << j - 1 << "       ";
    }
    cout << "\n";
    cout << "  Row\n";
    cout << "\n";
//
//  Determine the range of the rows in this strip.
//
    if ( 1 < ilo )
    {
      i2lo = ilo;
    }
    else
    {
      i2lo = 1;
    }
    if ( ihi < m )
    {
      i2hi = ihi;
    }
    else
    {
      i2hi = m;
    }

    for ( i = i2lo; i <= i2hi; i++ )
    {
//
//  Print out (up to) 5 entries in row I, that lie in the current strip.
//
      cout << setw(5) << i - 1 << ": ";
      for ( j = j2lo; j <= j2hi; j++ )
      {
        cout << setw(12) << a[i-1][j-1] << "  ";
      }
      cout << "\n";
    }
  }

  return;
# undef INCX
}
//****************************************************************************80

double *r8rmat_to_r8mat ( int m, int n, double **a )

//****************************************************************************80
//
//  Purpose:
//
//    r8rmat_to_r8mat() copies data from an R8RMAT to an R8MAT.
//
//  Discussion:
//
//    An R8RMAT is a row-major array that was created by a 
//    command like:
//
//    double **a;
//    a = r8rmat_new ( m, n );
//
//    An R8MAT is a column-major array stored as a vector, so
//    that element (I,J) of the M by N array is stored in location
//    I+J*M.
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    07 January 2014
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, N, the number of rows and columns.
//
//    double **A = double A[M][N], the data, stored as an R8RMAT.
//
//  Output:
//
//    double R8RMAT_TO_R8MAT[M*N], the data, stored as an R8MAT.
//
{
  double *b;
  int i;
  int j;

  b = new double[m*n];

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      b[i+j*m] = a[i][j];
    }
  }

  return b;
}
//****************************************************************************80

double **r8rmat_zeros ( int m, int n )

//****************************************************************************80
//
//  Purpose:
//
//    r8rmat_zeros() allocates and zeroes a new R8RMAT.
//
//  Discussion:
//
//    An R8RMAT is a row-major array that was created by a 
//    command like:
//
//      double **a;
//      a = r8rmat_new ( m, n );
//
//    The user assigns entries to the matrix using typical
//    2D array notation:
//      a[2][3] = 17.0;
//      y = a[1][0];
//    and so on.
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    26 May 2014
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, N, the number of rows and columns in the matrix.
//
//  Output:
//
//    double **R8RMAT_ZEROS, a new matrix.
//
{
  double **a;
  int i;
  int j;

  a = new double *[m];

  if ( a == NULL )
  {
    cerr << "\n";
    cerr << "R8RMAT_ZEROS - Fatal error!\n";
    cerr << "  Unable to allocate row pointer array.\n";
    exit ( 1 );
  }

  for ( i = 0; i < m; i++ )
  {
    a[i] = new double[n];
    if ( a[i] == NULL )
    {
      cerr << "\n";
      cerr << "R8RMAT_ZEROS - Fatal error!\n";
      cerr << "  Unable to allocate row array.\n";
      exit ( 1 );
    }
  }

  for ( i = 0; i < m; i++ )
  {
    for ( j = 0; j < n; j++ )
    {
      a[i][j] = 0.0;
    }
  }
  return a;
}
//****************************************************************************80

double *r8rows_to_r8mat ( int m, int n, double r8rows[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8rows_to_r8mat() converts a row-major vector to an R8MAT.
//
//  Discussion:
//
//    This function allows me to declare a vector of the right type and length,
//    fill it with data that I can display row-wise, and then have the
//    data copied into a column-wise doubly dimensioned array array.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    10 September 2018
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, N, the number of rows and columns in the array.
//
//    double R8ROWS[M*N], the rowwise data.
//
//  Output:
//
//    double R8ROWS_TO_R8MAT[M*N], the doubly-dimensioned columnwise data.
//
{
  int i;
  double *r8mat;
  int j;
  int k;

  r8mat = new double[m*n];

  k = 0;
  for ( i = 0; i < m; i++ )
  {
    for ( j = 0; j < n; j++ )
    {
      r8mat[i+j*m] = r8rows[k];
      k = k + 1;
    }
  }

  return r8mat;
}
//****************************************************************************80

void r8slmat_print ( int m, int n, double a[], string title )

//****************************************************************************80
//
//  Purpose:
//
//    r8slmat_print() prints a strict lower triangular R8MAT.
//
//  Example:
//
//    M = 5, N = 5
//    A = (/ 21, 31, 41, 51, 32, 42, 52, 43, 53, 54 /)
//
//    21
//    31 32
//    41 42 43
//    51 52 53 54
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    21 October 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, the number of rows in A.
//
//    int N, the number of columns in A.
//
//    double A[*], the M by N matrix.  Only the strict
//    lower triangular elements are stored, in column major order.
//
//    string TITLE, a title.
//
{
  int i;
  int indx;
  int j;
  int jhi;
  int jlo;
  int jmax;
  int nn;

  cout << "\n";
  cout << title << "\n";

  jmax = min ( n, m - 1 );

  nn = 5;

  for ( jlo = 1; jlo <= jmax; jlo = jlo + nn )
  {
    jhi = min ( jlo + nn - 1, min ( m - 1, jmax ) );
    cout << "\n";
    cout << "  Col   ";
    for ( j = jlo; j <= jhi; j++ )
    {
      cout << setw(7) << j << "       ";
    }
    cout << "\n";
    cout << "  Row\n";
    for ( i = jlo + 1; i <= m; i++ )
    {
      cout << setw(5) << i << ":";
      jhi = min ( jlo + nn - 1, min ( i - 1, jmax ) );
      for ( j = jlo; j <= jhi; j++ )
      {
        indx = ( j - 1 ) * m + i - ( j * ( j + 1 ) ) / 2;
        cout << " " << setw(12) << a[indx-1];
      }
      cout << "\n";
    }
  }

  return;
}
//****************************************************************************80

void r8vec_01_to_ab ( int n, double a[], double amax, double amin )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_01_to_ab() shifts and rescales data to lie within given bounds.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//    On input, A contains the original data, which is presumed to lie
//    between 0 and 1.  However, it is not necessary that this be so.
//
//    On output, A has been shifted and rescaled so that all entries which
//    on input lay in [0,1] now lie between AMIN and AMAX.  Other entries will
//    be mapped in a corresponding way.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    18 September 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of data values.
//
//    double A[N], the vector to be rescaled.
//
//    double AMAX, AMIN, the maximum and minimum values
//    allowed for A.
//
//  Output:
//
//    double A[N}: the rescaled vector.
//
{
  double amax2;
  double amax3;
  double amin2;
  double amin3;
  int i;

  if ( amax == amin )
  {
    for ( i = 0; i < n; i++ )
    {
      a[i] = amin;
    }
    return;
  }

  amax2 = fmax ( amax, amin );
  amin2 = fmin ( amax, amin );

  amin3 = r8vec_min ( n, a );
  amax3 = r8vec_max ( n, a );

  if ( amax3 != amin3 )
  {
    for ( i = 0; i < n; i++ )
    {
      a[i] = ( ( amax3 - a[i]         ) * amin2
             + (         a[i] - amin3 ) * amax2 )
             / ( amax3          - amin3 );
    }
  }
  else
  {
    for ( i = 0; i < n; i++ )
    {
      a[i] = 0.5 * ( amax2 + amin2 );
    }
  }

  return;
}
//****************************************************************************80

void r8vec_add ( int n, double a1[], double a2[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_add() adds one R8VEC to another.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    22 September 2011
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of entries in the vectors.
//
//    double A1[N], the vector to be added.
//
//    double A2[N], the vector to be increased.
//
//  Output:
//
//    double A2[N]: the sum of A1 and A2.
//
{
  int i;

  for ( i = 0; i < n; i++ )
  {
    a2[i] = a2[i] + a1[i];
  }
  return;
}
//****************************************************************************80

double r8vec_amax ( int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_amax() returns the maximum absolute value in an R8VEC.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    18 September 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of entries in the array.
//
//    double A[N], the array.
//
//  Output:
//
//    double AMAX, the value of the entry
//    of largest magnitude.
//
{
  double amax;
  int i;

  amax = 0.0;
  for ( i = 0; i < n; i++ )
  {
    if ( amax < fabs ( a[i] ) )
    {
      amax = fabs ( a[i] );
    }
  }

  return amax;
}
//****************************************************************************80

int r8vec_amax_index ( int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_amax_index() returns the index of the maximum absolute value in an R8VEC.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    20 September 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of entries in the array.
//
//    double A[N], the array.
//
//  Output:
//
//    int R8VEC_AMAX_INDEX, the index of the entry of largest magnitude.
//
{
  double amax;
  int amax_index;
  int i;

  if ( n <= 0 )
  {
    amax_index = -1;
  }
  else
  {
    amax_index = 1;
    amax = fabs ( a[0] );

    for ( i = 2; i <= n; i++ )
    {
      if ( amax < fabs ( a[i-1] ) )
      {
        amax_index = i;
        amax = fabs ( a[i-1] );
      }
    }
  }

  return amax_index;
}
//****************************************************************************80

double r8vec_amin ( int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_amin() returns the minimum absolute value in an R8VEC.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    18 September 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of entries in the array.
//
//    double A[N], the array.
//
//  Output:
//
//    double R8VEC_AMIN, the value of the entry
//    of smallest magnitude.
//
{
  int i;
  double value;

  value = HUGE_VAL;
  for ( i = 0; i < n; i++ )
  {
    if ( fabs ( a[i] ) < value )
    {
      value = fabs ( a[i] );
    }
  }

  return value;
}
//****************************************************************************80

int r8vec_amin_index ( int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_amin_index() returns the index of the minimum absolute value in an R8VEC.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    20 September 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of entries in the array.
//
//    double A[N], the array.
//
//  Output:
//
//    int R8VEC_AMIN_INDEX, the index of the entry of smallest magnitude.
//
{
  double amin;
  int amin_index;
  int i;

  if ( n <= 0 )
  {
    amin_index = -1;
  }
  else
  {
    amin_index = 1;
    amin = fabs ( a[0] );

    for ( i = 2; i <= n; i++ )
    {
      if ( fabs ( a[i-1] ) < amin )
      {
        amin_index = i;
        amin = fabs ( a[i-1] );
      }
    }
  }

  return amin_index;
}
//****************************************************************************80

double *r8vec_any_normal ( int dim_num, double v1[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_any_normal() returns some normal vector to V1.
//
//  Discussion:
//
//    If DIM_NUM < 2, then no normal vector can be returned.
//
//    If V1 is the zero vector, then any unit vector will do.
//
//    No doubt, there are better, more robust algorithms.  But I will take
//    just about ANY reasonable unit vector that is normal to V1.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    23 August 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int DIM_NUM, the spatial dimension.
//
//    double V1[DIM_NUM], the vector.
//
//  Output:
//
//    double R8VEC_ANY_NORMAL[DIM_NUM], a vector that is
//    normal to V2, and has unit Euclidean length.
//
{
  int i;
  int j;
  int k;
  double *v2;
  double vj;
  double vk;

  if ( dim_num < 2 )
  {
    cerr << "\n";
    cerr << "R8VEC_ANY_NORMAL - Fatal error!\n";
    cerr << "  Called with DIM_NUM < 2.\n";
    exit ( 1 );
  }

  v2 = new double[dim_num];

  if ( r8vec_norm ( dim_num, v1 ) == 0.0 )
  {
    r8vec_zeros ( dim_num, v2 );
    v2[0] = 1.0;
    return v2;
  }
//
//  Seek the largest entry in V1, VJ = V1(J), and the
//  second largest, VK = V1(K).
//
//  Since V1 does not have zero norm, we are guaranteed that
//  VJ, at least, is not zero.
//
  j = -1;
  vj = 0.0;

  k = -1;
  vk = 0.0;

  for ( i = 0; i < dim_num; i++ )
  {
    if ( fabs ( vk ) < fabs ( v1[i] ) || k == -1 )
    {
      if ( fabs ( vj ) < fabs ( v1[i] ) || j == -1 )
      {
        k = j;
        vk = vj;
        j = i;
        vj = v1[i];
      }
      else
      {
        k = i;
        vk = v1[i];
      }
    }
  }
//
//  Setting V2 to zero, except that V2(J) = -VK, and V2(K) = VJ,
//  will just about do the trick.
//
  r8vec_zeros ( dim_num, v2 );

  v2[j] = -vk / sqrt ( vk * vk + vj * vj );
  v2[k] =  vj / sqrt ( vk * vk + vj * vj );

  return v2;
}
//****************************************************************************80

void r8vec_append ( int *n, double **a, double value )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_append() appends an entry to an R8VEC.
//
//  Licensing:
//
//    This code is distributed under the MIT license. 
//
//  Modified:
//
//    14 May 2018
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int *N, the current size of the array.
//
//    double **A, the array.
//
//    double VALUE, a value to be appended to A.
//
//  Output:
//
//    int *N, the updated size of the array.
//
//    double **A, the updated array, with VALUE appended.
//
{
  double *a_old;
  int i;
//
//  Save a pointer to the old array.
//
  a_old = *a;
//
//  Create a new array.
//
  *a = new double[*n+1];
//
//  Copy the old data and append the new item.
//
  for ( i = 0; i < *n; i++ )
  {
    (*a)[i] = a_old[i];
  }
  (*a)[*n] = value;
//
//  Increase N.
//
  *n = *n + 1;
//
//  Release memory.
//
  delete [] a_old;

  return;
}
//****************************************************************************80

double *r8vec_append_new ( int n, double a[], double value )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_append_new() appends a value to an R8VEC.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    14 May 2018
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of entries in the input vector.
//
//    double A[N], the vector to be modified.  On output, the vector
//    has been reallocated, has one more entry than on input, and that last
//    entry is VALUE.
//
//    double VALUE, the value to be appended to the vector.
//
//  Output:
//
//    double R8VEC_APPEND[N+1], a copy of the vector
//    with one more entry than on input, and that last
//    entry is VALUE.
//
{
  double *b;
  int i;

  b = new double[n+1];

  for ( i = 0; i < n; i++ )
  {
    b[i] = a[i];
  }
  b[n] = value;

  return b;
}
//****************************************************************************80

double r8vec_asum ( int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_asum() sums the absolute values of the entries of an R8VEC.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    24 January 2015
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of entries in the vector.
//
//    double A[N], the vector.
//
//  Output:
//
//    double R8VEC_ASUM, the sum of absolute values of the entries.
//
{
  int i;
  double value;

  value = 0.0;
  for ( i = 0; i < n; i++ )
  {
    value = value + fabs ( a[i] );
  }
  return value;
}
//****************************************************************************80

void r8vec_bin ( int n, double x[], int bin_num, double bin_min, double bin_max,
  int bin[], double bin_limit[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_bin() computes bins based on a given R8VEC.
//
//  Discussion:
//
//    The user specifies minimum and maximum bin values, BIN_MIN and
//    BIN_MAX, and the number of bins, BIN_NUM.  This determines a
//    "bin width":
//
//      H = ( BIN_MAX - BIN_MIN ) / BIN_NUM
//
//    so that bin I will count all entries X(J) such that
//
//      BIN_LIMIT(I-1) <= X(J) < BIN_LIMIT(I).
//
//    The array X does NOT have to be sorted.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    22 February 2012
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of entries of X.
//
//    double X[N], an (unsorted) array to be binned.
//
//    int BIN_NUM, the number of bins.  Two extra bins,
//    #0 and #BIN_NUM+1, count extreme values.
//
//    double BIN_MIN, BIN_MAX, define the range and size
//    of the bins.  BIN_MIN and BIN_MAX must be distinct.
//    Normally, BIN_MIN < BIN_MAX, and the documentation will assume
//    this, but proper results will be computed if BIN_MIN > BIN_MAX.
//
//  Output:
//
//    int BIN[BIN_NUM+2].
//    BIN(0) counts entries of X less than BIN_MIN.
//    BIN(BIN_NUM+1) counts entries greater than or equal to BIN_MAX.
//    For 1 <= I <= BIN_NUM, BIN(I) counts the entries X(J) such that
//      BIN_LIMIT(I-1) <= X(J) < BIN_LIMIT(I).
//    where H is the bin spacing.
//
//    double BIN_LIMIT[BIN_NUM+1], the "limits" of the bins.
//    BIN(I) counts the number of entries X(J) such that
//      BIN_LIMIT(I-1) <= X(J) < BIN_LIMIT(I).
//
{
  int i;
  int j;
  double t;

  if ( bin_max == bin_min )
  {
    cerr << "\n";
    cerr << "R8VEC_BIN - Fatal error!\n";
    cerr << "  BIN_MIN = BIN_MAX = " << bin_max << ".\n";
    exit ( 1 );
  }

  for ( i = 0; i <= bin_num + 1; i++ )
  {
    bin[i] = 0;
  }

  for ( i = 0; i < n; i++ )
  {
    t = ( x[i] - bin_min ) / ( bin_max - bin_min );

    if ( t < 0.0 )
    {
      j = 0;
    }
    else if ( 1.0 <= t )
    {
      j = bin_num + 1;
    }
    else
    {
      j = 1 + ( int ) ( ( double ) ( bin_num ) * t );
    }
    bin[j] = bin[j] + 1;
  }
//
//  Compute the bin limits.
//
  for ( i = 0; i <= bin_num; i++ )
  {
    bin_limit[i] = (   ( double ) ( bin_num - i ) * bin_min   
                     + ( double ) (           i ) * bin_max ) 
                     / ( double ) ( bin_num     );
  }

  return;
}
//****************************************************************************80

void r8vec_binary_next ( int n, double bvec[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_binary_next() generates the next binary vector.
//
//  Discussion:
//
//    The vectors have the order
//
//      (0,0,...,0),
//      (0,0,...,1),
//      ...
//      (1,1,...,1)
//
//    and the "next" vector after (1,1,...,1) is (0,0,...,0).  That is,
//    we allow wrap around.
//
//  Example:
//
//    N = 3
//
//    Input      Output
//    -----      ------
//    0 0 0  =>  0 0 1
//    0 0 1  =>  0 1 0
//    0 1 0  =>  0 1 1
//    0 1 1  =>  1 0 0
//    1 0 0  =>  1 0 1
//    1 0 1  =>  1 1 0
//    1 1 0  =>  1 1 1
//    1 1 1  =>  0 0 0
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    31 March 2018
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the dimension of the vectors.
//
//    double BVEC[N], the vector whose successor is desired.
//
//  Output:
//
//    double BVEC[N], the successor to the input vector.
//
{
  int i;

  for ( i = n - 1; 0 <= i; i-- )
  {
    if ( bvec[i] == 0.0 )
    {
      bvec[i] = 1.0;
      return;
    }
    bvec[i] = 0.0;
  }

  return;
}
//****************************************************************************80

void r8vec_bracket ( int n, double x[], double xval, int &left, int &right )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_bracket() searches a sorted array for successive brackets of a value.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//    If the values in the vector are thought of as defining intervals
//    on the real line, then this routine searches for the interval
//    nearest to or containing the given value.
//
//    It is always true that RIGHT = LEFT+1.
//
//    If XVAL < X[0], then LEFT = 1, RIGHT = 2, and
//      XVAL   < X[0] < X[1];
//    If X(1) <= XVAL < X[N-1], then
//      X[LEFT-1] <= XVAL < X[RIGHT-1];
//    If X[N-1] <= XVAL, then LEFT = N-1, RIGHT = N, and
//      X[LEFT-1] <= X[RIGHT-1] <= XVAL.
//
//    For consistency, this routine computes indices RIGHT and LEFT
//    that are 1-based, although it would be more natural in C and
//    C++ to use 0-based values.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    24 February 2004
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, length of input array.
//
//    double X[N], an array that has been sorted into ascending order.
//
//    double XVAL, a value to be bracketed.
//
//  Output:
//
//    int &LEFT, &RIGHT, the results of the search.
//
{
  int i;

  for ( i = 2; i <= n - 1; i++ )
  {
    if ( xval < x[i-1] )
    {
      left = i - 1;
      right = i;
      return;
    }

   }

  left = n - 1;
  right = n;

  return;
}
//****************************************************************************80

void r8vec_bracket2 ( int n, double x[], double xval, int start, int &left,
  int &right )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_bracket2() searches a sorted array for successive brackets of a value.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//    If the values in the vector are thought of as defining intervals
//    on the real line, then this routine searches for the interval
//    containing the given value.
//
//    R8VEC_BRACKET2 is a variation on R8VEC_BRACKET.  It seeks to reduce
//    the search time by allowing the user to suggest an interval that
//    probably contains the value.  The routine will look in that interval
//    and the intervals to the immediate left and right.  If this does
//    not locate the point, a binary search will be carried out on
//    appropriate subportion of the sorted array.
//
//    In the most common case, 1 <= LEFT < LEFT + 1 = RIGHT <= N,
//    and X(LEFT) <= XVAL <= X(RIGHT).
//
//    Special cases:
//      Value is less than all data values:
//    LEFT = -1, RIGHT = 1, and XVAL < X(RIGHT).
//      Value is greater than all data values:
//    LEFT = N, RIGHT = -1, and X(LEFT) < XVAL.
//      Value is equal to a data value:
//    LEFT = RIGHT, and X(LEFT) = X(RIGHT) = XVAL.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    18 September 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, length of the input array.
//
//    double X[N], an array that has been sorted into
//    ascending order.
//
//    double XVAL, a value to be bracketed by entries of X.
//
//    int START, between 1 and N, specifies that XVAL
//    is likely to be in the interval:
//      [ X(START), X(START+1) ]
//    or, if not in that interval, then either
//      [ X(START+1), X(START+2) ]
//    or
//      [ X(START-1), X(START) ].
//
//  Output:
//
//    int &LEFT, &RIGHT, the results of the search.
//
{
  int high;
  int low;
//
//  Check.
//
  if ( n < 1 )
  {
    cerr << "\n";
    cerr << "R8VEC_BRACKET2 - Fatal error!\n";
    cerr << "  N < 1.\n";
    exit ( 1 );
  }

  if ( start < 1 || n < start )
  {
    start = ( n + 1 ) / 2;
  }
//
//  XVAL = X(START)?
//
  if ( x[start-1] == xval )
  {
    left = start;
    right = start;
    return;
  }
//
//  X(START) < XVAL?
//
  else if ( x[start-1] < xval )
  {
//
//  X(START) = X(N) < XVAL < oo?
//
    if ( n < start + 1 )
    {
      left = start;
      right = -1;
      return;
    }
//
//  XVAL = X(START+1)?
//
    else if ( xval == x[start] )
    {
      left = start + 1;
      right = start + 1;
      return;
    }
//
//  X(START) < XVAL < X(START+1)?
//
    else if ( xval < x[start] )
    {
      left = start;
      right = start + 1;
      return;
    }
//
//  X(START+1) = X(N) < XVAL < oo?
//
    else if ( n < start + 2 )
    {
      left = start + 1;
      right = -1;
      return;
    }
//
//  XVAL = X(START+2)?
//
    else if ( xval == x[start+1] )
    {
      left = start + 2;
      right = start + 2;
      return;
    }
//
//  X(START+1) < XVAL < X(START+2)?
//
    else if ( xval < x[start+1] )
    {
      left = start + 1;
      right = start + 2;
      return;
    }
//
//  Binary search for XVAL in [ X(START+2), X(N) ],
//  where XVAL is guaranteed to be greater than X(START+2).
//
    else
    {
      low = start + 2;
      high = n;

      r8vec_bracket ( high + 1 - low, x+low-1, xval, left, right );

      left = left + low - 1;
      right = right + low - 1;
    }
  }
//
//  -oo < XVAL < X(START) = X(1).
//
  else if ( start == 1 )
  {
    left = -1;
    right = start;
    return;
  }
//
//  XVAL = X(START-1)?
//
  else if ( xval == x[start-2] )
  {
    left = start - 1;
    right = start - 1;
    return;
  }
//
//  X(START-1) < XVAL < X(START)?
//
  else if ( x[start-2] <= xval )
  {
    left = start - 1;
    right = start;
    return;
  }
//
//  Binary search for XVAL in [ X(1), X(START-1) ],
//  where XVAL is guaranteed to be less than X(START-1).
//
  else
  {
    low = 1;
    high = start - 1;
    r8vec_bracket ( high + 1 - low, x, xval, left, right );
  }

  return;
}
//****************************************************************************80

void r8vec_bracket3 ( int n, double t[], double tval, int &left )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_bracket3() finds the interval containing or nearest a given value.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//    The routine always returns the index LEFT of the sorted array
//    T with the property that either
//    *  T is contained in the interval [ T[LEFT], T[LEFT+1] ], or
//    *  T < T[LEFT] = T[0], or
//    *  T > T[LEFT+1] = T[N-1].
//
//    The routine is useful for interpolation problems, where
//    the abscissa must be located within an interval of data
//    abscissas for interpolation, or the "nearest" interval
//    to the (extreme) abscissa must be found so that extrapolation
//    can be carried out.
//
//    This version of the function has been revised so that the value of
//    LEFT that is returned uses the 0-based indexing natural to C++.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    30 April 2009
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, length of the input array.
//
//    double T[N], an array that has been sorted into ascending order.
//
//    double TVAL, a value to be bracketed by entries of T.
//
//    int &LEFT.: if 0 <= LEFT <= N-2, LEFT is taken as a suggestion for the
//    interval [ T[LEFT-1] T[LEFT] ] in which TVAL lies.  This interval
//    is searched first, followed by the appropriate interval to the left
//    or right.  After that, a binary search is used.
//
//  Output:
//
//    int &LEFT: LEFT is set so that the interval [ T[LEFT], T[LEFT+1] ]
//    is the closest to TVAL; it either contains TVAL, or else TVAL
//    lies outside the interval [ T[0], T[N-1] ].
//
{
  int high;
  int low;
  int mid;
//
//  Check the input data.
//
  if ( n < 2 )
  {
    cerr << "\n";
    cerr << "R8VEC_BRACKET3 - Fatal error!\n";
    cerr << "  N must be at least 2.\n";
    exit ( 1 );
  }
//
//  If LEFT is not between 0 and N-2, set it to the middle value.
//
  if ( left < 0 || n - 2 < left )
  {
    left = ( n - 1 ) / 2;
  }
//
//  CASE 1: TVAL < T[LEFT]:
//  Search for TVAL in (T[I],T[I+1]), for I = 0 to LEFT-1.
//
  if ( tval < t[left] )
  {
    if ( left == 0 )
    {
      return;
    }
    else if ( left == 1 )
    {
      left = 0;
      return;
    }
    else if ( t[left-1] <= tval )
    {
      left = left - 1;
      return;
    }
    else if ( tval <= t[1] )
    {
      left = 0;
      return;
    }
//
//  ...Binary search for TVAL in (T[I],T[I+1]), for I = 1 to LEFT-2.
//
    low = 1;
    high = left - 2;

    for ( ; ; )
    {
      if ( low == high )
      {
        left = low;
        return;
      }

      mid = ( low + high + 1 ) / 2;

      if ( t[mid] <= tval )
      {
        low = mid;
      }
      else
      {
        high = mid - 1;
      }
    }
  }
//
//  CASE 2: T[LEFT+1] < TVAL:
//  Search for TVAL in (T[I],T[I+1]) for intervals I = LEFT+1 to N-2.
//
  else if ( t[left+1] < tval )
  {
    if ( left == n - 2 )
    {
      return;
    }
    else if ( left == n - 3 )
    {
      left = left + 1;
      return;
    }
    else if ( tval <= t[left+2] )
    {
      left = left + 1;
      return;
    }
    else if ( t[n-2] <= tval )
    {
      left = n - 2;
      return;
    }
//
//  ...Binary search for TVAL in (T[I],T[I+1]) for intervals I = LEFT+2 to N-3.
//
    low = left + 2;
    high = n - 3;

    for ( ; ; )
    {

      if ( low == high )
      {
        left = low;
        return;
      }

      mid = ( low + high + 1 ) / 2;

      if ( t[mid] <= tval )
      {
        low = mid;
      }
      else
      {
        high = mid - 1;
      }
    }
  }
//
//  CASE 3: T[LEFT] <= TVAL <= T[LEFT+1]:
//  T is just where the user said it might be.
//
  else
  {
  }

  return;
}
//****************************************************************************80

void r8vec_bracket4 ( int nt, double t[], int ns, double s[], int left[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_bracket4() finds the interval containing or nearest a given value.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//    The routine always returns the index LEFT of the sorted array
//    T with the property that either
//    *  T is contained in the interval [ T[LEFT], T[LEFT+1] ], or
//    *  T < T[LEFT] = T[0], or
//    *  T > T[LEFT+1] = T[NT-1].
//
//    The routine is useful for interpolation problems, where
//    the abscissa must be located within an interval of data
//    abscissas for interpolation, or the "nearest" interval
//    to the (extreme) abscissa must be found so that extrapolation
//    can be carried out.
//
//    This version of the function has been revised so that the value of
//    LEFT that is returned uses the 0-based indexing natural to C++.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    30 April 2009
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int NT, length of the input array.
//
//    double T[NT], an array that has been sorted
//    into ascending order.
//
//    int NS, the number of points to be bracketed.
//
//    double S[NS], values to be bracketed by entries of T.
//
//  Output:
//
//    int LEFT[NS].
//    LEFT[I] is set so that the interval [ T[LEFT[I]], T[LEFT[I]+1] ]
//    is the closest to S[I]; it either contains S[I], or else S[I]
//    lies outside the interval [ T[0], T[NT-1] ].
//
{
  int high;
  int i;
  int low;
  int mid;
//
//  Check the input data.
//
  if ( nt < 2 )
  {
    cerr << "\n";
    cerr << "R8VEC_BRACKET4 - Fatal error!\n";
    cerr << "  NT must be at least 2.\n";
    exit ( 1 );
  }

  for ( i = 0; i < ns; i++ )
  {
    left[i] = ( nt - 1 ) / 2;
//
//  CASE 1: S[I] < T[LEFT]:
//  Search for S[I] in (T[I],T[I+1]), for I = 0 to LEFT-1.
//
    if ( s[i] < t[left[i]] )
    {
      if ( left[i] == 0 )
      {
        continue;
      }
      else if ( left[i] == 1 )
      {
        left[i] = 0;
        continue;
      }
      else if ( t[left[i]-1] <= s[i] )
      {
        left[i] = left[i] - 1;
        continue;
      }
      else if ( s[i] <= t[1] )
      {
        left[i] = 0;
        continue;
      }
//
//  ...Binary search for S[I] in (T[I],T[I+1]), for I = 1 to *LEFT-2.
//
      low = 1;
      high = left[i] - 2;

      for ( ; ; )
      {
        if ( low == high )
        {
          left[i] = low;
          break;
        }

        mid = ( low + high + 1 ) / 2;

        if ( t[mid] <= s[i] )
        {
          low = mid;
        }
        else
        {
          high = mid - 1;
        }
      }
    }
//
//  CASE 2: T[LEFT+1] < S[I]:
//  Search for S[I] in (T[I],T[I+1]) for intervals I = LEFT+1 to NT-2.
//
    else if ( t[left[i]+1] < s[i] )
    {
      if ( left[i] == nt - 2 )
      {
        continue;
      }
      else if ( left[i] == nt - 3 )
      {
        left[i] = left[i] + 1;
        continue;
      }
      else if ( s[i] <= t[left[i]+2] )
      {
        left[i] = left[i] + 1;
        continue;
      }
      else if ( t[nt-2] <= s[i] )
      {
        left[i] = nt - 2;
        continue;
      }
//
//  ...Binary search for S[I] in (T[I],T[I+1]) for intervals I = LEFT+2 to NT-3.
//
      low = left[i] + 2;
      high = nt - 3;

      for ( ; ; )
      {

        if ( low == high )
        {
          left[i] = low;
          break;
        }

        mid = ( low + high + 1 ) / 2;

        if ( t[mid] <= s[i] )
        {
          low = mid;
        }
        else
        {
          high = mid - 1;
        }
      }
    }
//
//  CASE 3: T[LEFT] <= S[I] <= T[LEFT+1]:
//
    else
    {
    }
  }
  return;
}
//****************************************************************************80

int r8vec_bracket5 ( int nd, double xd[], double xi )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_bracket5() brackets data between successive entries of a sorted R8VEC.
//
//  Discussion:
//
//    We assume XD is sorted.
//
//    If XI is contained in the interval [XD(1),XD(N)], then the returned 
//    value B indicates that XI is contained in [ XD(B), XD(B+1) ].
//
//    If XI is not contained in the interval [XD(1),XD(N)], then B = -1.
//
//    This code implements a version of binary search which is perhaps more
//    understandable than the usual ones.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    14 October 2012
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int ND, the number of data values.
//
//    double XD[N], the sorted data.
//
//    double XD, the query value.
//
//  Output:
//
//    int R8VEC_BRACKET5, the bracket information.
//
{
  int b;
  int l;
  int m;
  int r;

  if ( xi < xd[0] || xd[nd-1] < xi )
  {
    b = -1;
  }
  else
  {
    l = 0;
    r = nd - 1;

    while ( l + 1 < r )
    {
      m = ( l + r ) / 2;
      if ( xi < xd[m] )
      {
        r = m;
      }
      else
      {
        l = m;
      }
    }
    b = l;
  }

  return b;
}
//****************************************************************************80

int *r8vec_bracket6 ( int nd, double xd[], int ni, double xi[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_bracket6() brackets data between successive entries of a sorted R8VEC.
//
//  Discussion:
//
//    We assume XD is sorted.
//
//    If XI(I) is contained in the interval [XD(1),XD(N)], then the value of
//    B(I) indicates that XI(I) is contained in [ XD(B(I)), XD(B(I)+1) ].
//
//    If XI(I) is not contained in the interval [XD(1),XD(N)], then B(I) = -1.
//
//    This code implements a version of binary search which is perhaps more
//    understandable than the usual ones.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    14 October 2012
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int ND, the number of data values.
//
//    double XD[N], the sorted data.
//
//    int NI, the number of inquiry values.
//
//    double XD[NI], the query values.
//
//  Output:
//
//    int R8VEC_BRACKET6[NI], the bracket information.
//
{
  int *b;
  int i;
  int l;
  int m;
  int r;

  b = new int[ni];

  for ( i = 0; i < ni; i++ )
  {
    if ( xi[i] < xd[0] || xd[nd-1] < xi[i] )
    {
      b[i] = -1;
    }
    else
    {
      l = 0;
      r = nd - 1;

      while ( l + 1 < r )
      {
        m = ( l + r ) / 2;
        if ( xi[i] < xd[m] )
        {
          r = m;
        }
        else
        {
          l = m;
        }
      }

      b[i] = l;
    }
  }

  return b;
}
//****************************************************************************80

double *r8vec_cheby_extreme_new ( int n, double a, double b )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_cheby_exterme_new() creates Chebyshev Extreme values in [A,B].
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    17 September 2012
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of entries in the vector.
//
//    double A, B, the interval.
//
//  Output:
//
//    double R8VEC_CHEBY_EXTREME_NEW[N], a vector of Chebyshev spaced data.
//
{
  double c;
  int i;
  const double r8_pi = 3.141592653589793;
  double theta;
  double *x;

  x = new double[n];

  if ( n == 1 )
  {
    x[0] = ( a + b ) / 2.0;
  }
  else
  {
    for ( i = 0; i < n; i++ )
    {
      theta = ( double ) ( n - i - 1 ) * r8_pi / ( double ) ( n - 1 );

      c = cos ( theta );

      if ( ( n % 2 ) == 1 )
      {
        if ( 2 * i + 1 == n )
        {
          c = 0.0;
        }
      }

      x[i] = ( ( 1.0 - c ) * a  
             + ( 1.0 + c ) * b ) 
             /   2.0;
    }
  }

  return x;
}
//****************************************************************************80

double *r8vec_cheby_zero_new ( int n, double a, double b )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_cheby_zero_new() creates Chebyshev Zero values in [A,B].
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    17 September 2012
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of entries in the vector.
//
//    double A, B, the interval.
//
//  Output:
//
//    double R8VEC_CHEBY_ZERO_NEW[N], a vector of Chebyshev spaced data.
//
{
  double c;
  int i;
  const double r8_pi = 3.141592653589793;
  double theta;
  double *x;

  x = new double[n];

  if ( n == 1 )
  {
    x[0] = ( a + b ) / 2.0;
  }
  else
  {
    for ( i = 0; i < n; i++ )
    {
      theta = ( double ) ( 2 * ( n - i ) - 1 ) * r8_pi / ( double ) ( 2 * n );

      c = cos ( theta );

      if ( ( n % 2 ) == 1 )
      {
        if ( 2 * i + 1 == n )
        {
          c = 0.0;
        }
      }

      x[i] = ( ( 1.0 - c ) * a  
             + ( 1.0 + c ) * b ) 
             /   2.0;
    }
  }

  return x;
}
//****************************************************************************80

double *r8vec_cheby1space_new ( int n, double a, double b )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_cheby1space_new() creates Type 1 Chebyshev values in [A,B].
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    17 September 2012
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of entries in the vector.
//
//    double A, B, the interval.
//
//  Output:
//
//    double R8VEC_CHEBY1SPACE_NEW[N], a vector of Type 1
//    Chebyshev spaced data.
//
{
  double c;
  int i;
  const double r8_pi = 3.141592653589793;
  double theta;
  double *x;

  x = new double[n];

  if ( n == 1 )
  {
    x[0] = ( a + b ) / 2.0;
  }
  else
  {
    for ( i = 0; i < n; i++ )
    {
      theta = ( double ) ( n - i - 1 ) * r8_pi / ( double ) ( n - 1 );

      c = cos ( theta );

      if ( ( n % 2 ) == 1 )
      {
        if ( 2 * i + 1 == n )
        {
          c = 0.0;
        }
      }

      x[i] = ( ( 1.0 - c ) * a  
             + ( 1.0 + c ) * b ) 
             /   2.0;
    }
  }

  return x;
}
//****************************************************************************80

double *r8vec_cheby2space_new ( int n, double a, double b )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_cheby2space_new() creates Type 2 Chebyshev values in [A,B].
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    05 July 2017
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of entries in the vector.
//
//    double A, B, the interval.
//
//  Output:
//
//    double R8VEC_CHEBY2SPACE_NEW[N], a vector of Type 2
//    Chebyshev spaced data.
//
{
  double c;
  int i;
  const double r8_pi = 3.141592653589793;
  double theta;
  double *x;

  x = new double[n];

  for ( i = 0; i < n; i++ )
  {
    theta = ( double ) ( n - i ) * r8_pi / ( double ) ( n + 1 );

    c = cos ( theta );

    x[i] = ( ( 1.0 - c ) * a  
           + ( 1.0 + c ) * b ) 
           /   2.0;
  }

  return x;
}
//****************************************************************************80

int r8vec_compare ( int n, double a[], double b[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_compare() compares two R8VEC's.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//    The lexicographic ordering is used.
//
//  Example:
//
//    Input:
//
//      A1 = ( 2.0, 6.0, 2.0 )
//      A2 = ( 2.0, 8.0, 12.0 )
//
//    Output:
//
//      ISGN = -1
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    23 September 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of entries in the vectors.
//
//    double A[N], B[N], the vectors to be compared.
//
//  Output:
//
//    int R8VEC_COMPARE, the results of the comparison:
//    -1, A is lexicographically less than B,
//     0, A is equal to B,
//    +1, A is lexicographically greater than B.
//
{
  int isgn;
  int k;

  isgn = 0;

  for ( k = 0; k < n; k++ )
  {
    if ( a[k] < b[k] )
    {
      isgn = -1;
      return isgn;
    }
    else if ( b[k] < a[k] )
    {
      isgn = +1;
      return isgn;
    }
  }
  return isgn;
}
//****************************************************************************80

void r8vec_concatenate ( int n1, double a[], int n2, double b[], double c[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_concatenate() concatenates two R8VEC's.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    22 November 2013
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N1, the number of entries in the first vector.
//
//    double A[N1], the first vector.
//
//    int N2, the number of entries in the second vector.
//
//    double B[N2], the second vector.
//
//  Output:
//
//    double C[N1+N2], the concatenated vector.
//
{
  int i;

  for ( i = 0; i < n1; i++ )
  {
    c[i] = a[i];
  }
  for ( i = 0; i < n2; i++ )
  {
    c[n1+i] = b[i];
  }

  return;
}
//****************************************************************************80

double *r8vec_concatenate_new ( int n1, double a[], int n2, double b[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_concatenate_new() concatenates two R8VEC's.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    22 November 2013
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N1, the number of entries in the first vector.
//
//    double A[N1], the first vector.
//
//    int N2, the number of entries in the second vector.
//
//    double B[N2], the second vector.
//
//  Output:
//
//    double R8VEC_CONCATENATE_NEW[N1+N2], the concatenated vector.
//
{
  int i;
  double *c;

  c = new double[n1+n2];

  for ( i = 0; i < n1; i++ )
  {
    c[i] = a[i];
  }
  for ( i = 0; i < n2; i++ )
  {
    c[n1+i] = b[i];
  }

  return c;
}
//****************************************************************************80

double *r8vec_convolution ( int m, double x[], int n, double y[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_convolution() returns the convolution of two R8VEC's.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//    The I-th entry of the convolution can be formed by summing the products 
//    that lie along the I-th diagonal of the following table:
//
//    Y3 | 3   4   5   6   7
//    Y2 | 2   3   4   5   6
//    Y1 | 1   2   3   4   5
//       +------------------
//        X1  X2  X3  X4  X5
//
//    which will result in:
//
//    Z = ( X1 * Y1,
//          X1 * Y2 + X2 * Y1,
//          X1 * Y3 + X2 * Y2 + X3 * Y1,
//                    X2 * Y3 + X3 * Y2 + X4 * Y1,
//                              X3 * Y3 + X4 * Y2 + X5 * Y1,
//                                        X4 * Y3 + X5 * Y2,
//                                                  X5 * Y3 )
//            
//  Example:
//
//    Input:
//
//      X = (/ 1, 2, 3, 4 /)
//      Y = (/ -1, 5, 3 /)
//
//    Output:
//
//      Z = (/ -1, 3, 10, 17, 29, 12 /)
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    05 May 2012
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int M, the dimension of X.
//
//    double X[M], the first vector to be convolved.
//
//    int N, the dimension of Y.
//
//    double Y[N], the second vector to be convolved.
//
//  Output:
//
//    double R8VEC_CONVOLUTION[M+N-1], the convolution of X and Y.
//
{
  int i;
  int j;
  double *z;

  z = new double[m+n-1];

  for ( i = 0; i < m + n - 1; i++ )
  {
    z[i] = 0.0;
  }

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      z[j+i] = z[j+i] + x[i] * y[j];
    }
  }
  return z;
}
//****************************************************************************80

double *r8vec_convolution_circ ( int n, double x[], double y[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_convolution_circ() returns the discrete circular convolution of two R8VEC's.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//    z(1+m) = xCCy(m) = sum ( 0 <= k <= n-1 ) x(1+k) * y(1+m-k)
//
//    Here, if the index of Y becomes nonpositive, it is "wrapped around"
//    by having N added to it.
//
//    The circular convolution is equivalent to multiplication of Y by a
//    circulant matrix formed from the vector X.
//
//  Example:
//
//    Input:
//
//      X = (/ 1, 2, 3, 4 /)
//      Y = (/ 1, 2, 4, 8 /)
//
//    Output:
//
//      Circulant form:
//
//      Z = ( 1 4 3 2 )   ( 1 )
//          ( 2 1 4 3 )   ( 2 )
//          ( 3 2 1 4 ) * ( 4 )
//          ( 4 3 2 1 )   ( 8 )
//
//      The formula:
//
//      Z = (/ 1*1 + 2*8 + 3*4 + 4*2,
//             1*2 + 2*1 + 3*8 + 4*4,
//             1*4 + 2*2 + 3*1 + 4*8,
//             1*8 + 2*4 + 3*2 + 4*1 /)
//
//      Result:
//
//      Z = (/ 37, 44, 43, 26 /)
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    17 September 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the dimension of the vectors.
//
//    double X[N], Y[N], the vectors to be convolved.
//
//  Output:
//
//    double R8VEC_CONVOLVE_CIRC[N], the circular convolution of X and Y.
//
{
  int i;
  int m;
  double *z;

  z = new double[n];

  for ( m = 1; m <= n; m++ )
  {
    z[m-1] = 0.0;
    for ( i = 1; i <= m; i++ )
    {
      z[m-1] = z[m-1] + x[i-1] * y[m-i];
    }
    for ( i = m+1; i <= n; i++ )
    {
      z[m-1] = z[m-1] + x[i-1] * y[n+m-i];
    }
  }

  return z;
}
//****************************************************************************80

void r8vec_copy ( int n, double a1[], double a2[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_copy() copies an R8VEC.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    27 March 2023
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of entries in the vectors.
//
//    double A1[N], the vector to be copied.
//
//  Output:
//
//    double A2[N], the copy of A1.
//
{
  int i;

  for ( i = 0; i < n; i++ )
  {
    a2[i] = a1[i];
  }
  return;
}
//****************************************************************************80

double *r8vec_copy_new ( int n, double a1[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_copy_new() copies an R8VEC to a new R8VEC.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    27 March 2023
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of entries in the vectors.
//
//    double A1[N], the vector to be copied.
//
//  Output:
//
//    double R8VEC_COPY_NEW[N], the copy of A1.
//
{
  double *a2;
  int i;

  a2 = new double[n];

  for ( i = 0; i < n; i++ )
  {
    a2[i] = a1[i];
  }
  return a2;
}
//****************************************************************************80

double r8vec_correlation ( int n, double x[], double y[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_correlation() returns the correlation of two R8VEC's.
//
//  Discussion:
//
//    The correlation coefficient is also known as Pearson's r coefficient.
//
//    It must be the case that -1 <= r <= +1.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    12 August 2019
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int n: the dimension of the vectors.
//
//    double x[n], y[n]: the vectors.
//
//  Output:
//
//    double r8vec_correlation, the correlation of X and Y.
//
{
  double dot;
  int i;
  double r;
  double x_mean;
  double x_std;
  double y_mean;
  double y_std;

  if ( n <= 1 )
  {
    r = 0.0;
  }
  else
  {
    x_mean = r8vec_mean ( n, x );
    x_std = r8vec_std_sample ( n, x );

    y_mean = r8vec_mean ( n, y );
    y_std = r8vec_std_sample ( n, y );
 
    dot = 0.0;
    for ( i = 0; i < n; i++ )
    {
      dot = dot + ( x[i] - x_mean ) * ( y[i] - y_mean );
    }
    r = dot / x_std / y_std / ( double ) ( n - 1 );
  }

  return r;
}
//****************************************************************************80

double r8vec_covariance ( int n, double x[], double y[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_covariance() computes the covariance of two vectors.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    20 April 2013
//
//  Author:
//
//    John Burkardt.
//
//  Input:
//
//    int N, the dimension of the two vectors.
//
//    double X[N], Y[N], the two vectors.
//
//  Output:
//
//    double R8VEC_COVARIANCE, the covariance of the two vectors.
//
{
  int i;
  double value;
  double x_average;
  double y_average;

  x_average = 0.0;
  for ( i = 0; i < n; i++ )
  {
    x_average = x_average + x[i];
  }
  x_average = x_average / ( double ) ( n );

  y_average = 0.0;
  for ( i = 0; i < n; i++ )
  {
    y_average = y_average + x[i];
  }
  y_average = y_average / ( double ) ( n );

  value = 0.0;
  for ( i = 0; i < n; i++ )
  {
    value = value + ( x[i] - x_average ) * ( y[i] - y_average );
  }

  value = value / ( double ) ( n - 1 );

  return value;
}
//****************************************************************************80

double r8vec_cross_product_2d ( double v1[2], double v2[2] )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_cross_product_2d() finds the cross product of a pair of R8VEC's in 2D.
//
//  Discussion:
//
//    Strictly speaking, the vectors lie in the (X,Y) plane, and
//    the cross product here is a vector in the Z direction.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    07 August 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double V1[2], V2[2], the vectors.
//
//  Output:
//
//    double R8VEC_CROSS_PRODUCT_2D, the Z component of the cross product
//    of V1 and V2.
//
{
  double value;

  value = v1[0] * v2[1] - v1[1] * v2[0];

  return value;
}
//****************************************************************************80

double r8vec_cross_product_affine_2d ( double v0[2], double v1[2],
  double v2[2] )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_cross_product_affine_2d() finds the affine cross product in 2D.
//
//  Discussion:
//
//    Strictly speaking, the vectors lie in the (X,Y) plane, and
//    the cross product here is a vector in the Z direction.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    27 October 2010
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double V0[2], the base vector.
//
//    double V1[2], V2[2], the vectors.
//
//  Output:
//
//    double R8VEC_CROSS_PRODUCT_AFFINE_2D, the Z component of the
//    cross product of V1 and V2.
//
{
  double value;

  value =
      ( v1[0] - v0[0] ) * ( v2[1] - v0[1] )
    - ( v2[0] - v0[0] ) * ( v1[1] - v0[1] );

  return value;
}
//****************************************************************************80

double *r8vec_cross_product_3d ( double v1[3], double v2[3] )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_cross_product_3d() computes the cross product of two R8VEC's in 3D.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    07 August 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double V1[3], V2[3], the coordinates of the vectors.
//
//  Output:
//
//    double R8VEC_CROSS_PRODUCT_3D[3], the cross product vector.
//
{
  double *v3;

  v3 = new double[3];

  v3[0] = v1[1] * v2[2] - v1[2] * v2[1];
  v3[1] = v1[2] * v2[0] - v1[0] * v2[2];
  v3[2] = v1[0] * v2[1] - v1[1] * v2[0];

  return v3;
}
//****************************************************************************80

double *r8vec_cross_product_affine_3d ( double v0[3], double v1[3],
  double v2[3] )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_cross_product_affine_3d() computes the affine cross product in 3D.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    27 October 2010
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    double V0[3], the base vector.
//
//    double V1[3], V2[3], the coordinates of the vectors.
//
//  Output:
//
//    double R8VEC_CROSS_PRODUCT_AFFINE_3D[3], the cross product vector.
//
{
  double *v3;

  v3 = new double[ 3 ];

  v3[0] =
      ( v1[1] - v0[1] ) * ( v2[2] - v0[2] )
    - ( v2[1] - v0[1] ) * ( v1[2] - v0[2] );

  v3[1] =
      ( v1[2] - v0[2] ) * ( v2[0] - v0[0] )
    - ( v2[2] - v0[2] ) * ( v1[0] - v0[0] );

  v3[2] =
      ( v1[0] - v0[0] ) * ( v2[1] - v0[1] )
    - ( v2[0] - v0[0] ) * ( v1[1] - v0[1] );

  return v3;
}
//****************************************************************************80

double *r8vec_cum_new ( int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_cum_new() computes the cumulutive sums of an R8VEC.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//    Input:
//
//      A = { 1.0, 2.0, 3.0, 4.0 }
//
//    Output:
//
//      A_CUM = { 1.0, 3.0, 6.0, 10.0 }
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    07 May 2012
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of entries in the vector.
//
//    double A[N], the vector to be summed.
//
//  Output:
//
//    double R8VEC_CUM_NEW[N], the cumulative sums.
//
{
  double *a_cum;
  int i;

  a_cum = new double[n];

  a_cum[0] = a[0];

  for ( i = 1; i < n; i++ )
  {
    a_cum[i] = a_cum[i-1] + a[i];
  }

  return a_cum;
}
//****************************************************************************80

double *r8vec_cum0_new ( int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_cum0_new() computes the cumulutive sums of an R8VEC.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//    Input:
//
//      A = { 1.0, 2.0, 3.0, 4.0 }
//
//    Output:
//
//      A_CUM = { 0.0, 1.0, 3.0, 6.0, 10.0 }
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    07 May 2012
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of entries in the vector.
//
//    double A[N], the vector to be summed.
//
//  Output:
//
//    double R8VEC_CUM0_NEW[N+1], the cumulative sums.
//
{
  double *a_cum;
  int i;

  a_cum = new double[n+1];

  a_cum[0] = 0.0;

  for ( i = 1; i <= n; i++ )
  {
    a_cum[i] = a_cum[i-1] + a[i-1];
  }

  return a_cum;
}
//****************************************************************************80

double *r8vec_dif ( int n, double h )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_dif() computes coefficients for estimating the N-th derivative.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//    The routine computes the N+1 coefficients for a centered finite difference
//    estimate of the N-th derivative of a function.
//
//    The estimate has the form
//
//      FDIF(N,X) = Sum (I = 0 to N) COF(I) * F ( X(I) )
//
//    To understand the computation of the coefficients, it is enough
//    to realize that the first difference approximation is
//
//      FDIF(1,X) = F(X+DX) - F(X-DX) ) / (2*DX)
//
//    and that the second difference approximation can be regarded as
//    the first difference approximation repeated:
//
//      FDIF(2,X) = FDIF(1,X+DX) - FDIF(1,X-DX) / (2*DX)
//         = F(X+2*DX) - 2 F(X) + F(X-2*DX) / (4*DX)
//
//    and so on for higher order differences.
//
//    Thus, the next thing to consider is the integer coefficients of
//    the sampled values of F, which are clearly the Pascal coefficients,
//    but with an alternating negative sign.  In particular, if we
//    consider row I of Pascal's triangle to have entries j = 0 through I,
//    then P(I,J) = P(I-1,J-1) - P(I-1,J), where P(*,-1) is taken to be 0,
//    and P(0,0) = 1.
//
//       1
//      -1  1
//       1 -2   1
//      -1  3  -3   1
//       1 -4   6  -4   1
//      -1  5 -10  10  -5  1
//       1 -6  15 -20  15 -6 1
//
//    Next, note that the denominator of the approximation for the
//    N-th derivative will be (2*DX)^N.
//
//    And finally, consider the location of the N+1 sampling
//    points for F:
//
//      X-N*DX, X-(N-2)*DX, X-(N-4)*DX, ..., X+(N-4)*DX, X+(N-2*DX), X+N*DX.
//
//    Thus, a formula for evaluating FDIF(N,X) is
//
//      fdif = 0.0
//      do i = 0, n
//        xi = x + (2*i-n) * h
//        fdif = fdif + cof(i) * f(xi)
//      end do
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    23 October 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the order of the derivative to be approximated.
//    N must be 0 or greater.
//
//    double H, the half spacing between points.
//    H must be positive.
//
//  Output:
//
//    double R8VEC_DIF[N+1], the coefficients needed to approximate
//    the N-th derivative of a function F.
//
{
  double *cof;
  int i;
  int j;

  if ( n < 0 )
  {
    cerr << "\n";
    cerr << "R8VEC_DIF - Fatal error!\n";
    cerr << "  Derivative order N = " << n << "\n";
    cerr << "  but N must be at least 0.\n";
    exit ( 1 );
  }

  if ( h <= 0.0 )
  {
    cerr << "\n";
    cerr << "R8VEC_DIF - Fatal error!\n";
    cerr << "  The half sampling spacing is H = " << h << "\n";
    cerr << "  but H must be positive.\n";
    exit ( 1 );
  }

  cof = new double[n+1];

  for ( i = 0; i <= n; i++ )
  {
    cof[i] = 1.0;

    for ( j = i - 1; 1 <= j; j-- )
    {
      cof[j] = -cof[j] + cof[j-1];
    }

    if ( 0 < i )
    {
      cof[0] = - cof[0];
    }
  }

  for ( i = 0; i <= n; i++ )
  {
    cof[i] = cof[i] / pow ( 2.0 * h, n );
  }

  return cof;
}
//****************************************************************************80

double r8vec_diff_norm ( int n, double a[], double b[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_diff_norm() returns the L2 norm of the difference of R8VEC's.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//    The vector L2 norm is defined as:
//
//      R8VEC_NORM_L2 = sqrt ( sum ( 1 <= I <= N ) A(I)^2 ).
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    24 June 2011
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of entries in A.
//
//    double A[N], B[N], the vectors.
//
//  Output:
//
//    double R8VEC_DIFF_NORM, the L2 norm of A - B.
//
{
  int i;
  double value;

  value = 0.0;

  for ( i = 0; i < n; i++ )
  {
    value = value + ( a[i] - b[i] ) * ( a[i] - b[i] );
  }
  value = sqrt ( value );

  return value;
}
//****************************************************************************80

double r8vec_diff_norm_l1 ( int n, double a[], double b[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_diff_norm_l1() returns the L1 norm of the difference of R8VEC's.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//    The vector L1 norm is defined as:
//
//      R8VEC_NORM_L1 = sum ( 1 <= I <= N ) abs ( A(I) ).
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    02 April 2010
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of entries in A.
//
//    double A[N], B[N], the vectors.
//
//  Output:
//
//    double R8VEC_DIFF_NORM_L1, the L1 norm of A - B.
//
{
  int i;
  double value;

  value = 0.0;

  for ( i = 0; i < n; i++ )
  {
    value = value + fabs ( a[i] - b[i] );
  }
  return value;
}
//****************************************************************************80

double r8vec_diff_norm_l2 ( int n, double a[], double b[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_diff_norm_l2() returns the L2 norm of the difference of R8VEC's.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//    The vector L2 norm is defined as:
//
//      R8VEC_NORM_L2 = sqrt ( sum ( 1 <= I <= N ) A(I)^2 ).
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    24 June 2011
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of entries in A.
//
//    double A[N], B[N], the vectors.
//
//  Output:
//
//    double R8VEC_DIFF_NORM_L2, the L2 norm of A - B.
//
{
  int i;
  double value;

  value = 0.0;

  for ( i = 0; i < n; i++ )
  {
    value = value + ( a[i] - b[i] ) * ( a[i] - b[i] );
  }
  value = sqrt ( value );

  return value;
}
//****************************************************************************80

double r8vec_diff_norm_li ( int n, double a[], double b[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_diff_norm_li() returns the L-oo norm of the difference of R8VEC's.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//    The vector L-oo norm is defined as:
//
//      R8VEC_NORM_LI = max ( 1 <= I <= N ) abs ( A(I) ).
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    02 April 2010
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of entries in A.
//
//    double A[N], B[N], the vectors.
//
//  Output:
//
//    double R8VEC_DIFF_NORM_LI, the L-oo norm of A - B.
//
{
  int i;
  double value;

  value = 0.0;

  for ( i = 0; i < n; i++ )
  {
    value = fmax ( value, fabs ( a[i] - b[i] ) );
  }
  return value;
}
//****************************************************************************80

double r8vec_diff_norm_squared ( int n, double a[], double b[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_diff_norm_squared(): square of the L2 norm of the difference of R8VEC's.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//    The square of the L2 norm of the difference of A and B is:
//
//      R8VEC_DIFF_NORM_SQUARED = sum ( 1 <= I <= N ) ( A[I] - B[I] )^2.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    24 June 2011
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of entries in A.
//
//    double A[N], B[N], the vectors.
//
//  Output:
//
//    double R8VEC_DIFF_NORM_SQUARED, the square of the L2 norm of A - B.
//
{
  int i;
  double value;

  value = 0.0;

  for ( i = 0; i < n; i++ )
  {
    value = value + ( a[i] - b[i] ) * ( a[i] - b[i] );
  }

  return value;
}
//****************************************************************************80

void r8vec_direct_product ( int factor_index, int factor_order,
  double factor_value[], int factor_num, int point_num, double x[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_direct_product() creates a direct product of R8VEC's.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//    To explain what is going on here, suppose we had to construct
//    a multidimensional quadrature rule as the product of K rules
//    for 1D quadrature.
//
//    The product rule will be represented as a list of points and weights.
//
//    The J-th item in the product rule will be associated with
//      item J1 of 1D rule 1,
//      item J2 of 1D rule 2,
//      ...,
//      item JK of 1D rule K.
//
//    In particular,
//      X(J) = ( X(1,J1), X(2,J2), ..., X(K,JK))
//    and
//      W(J) = W(1,J1) * W(2,J2) * ... * W(K,JK)
//
//    So we can construct the quadrature rule if we can properly
//    distribute the information in the 1D quadrature rules.
//
//    This routine carries out that task.
//
//    Another way to do this would be to compute, one by one, the
//    set of all possible indices (J1,J2,...,JK), and then index
//    the appropriate information.  An advantage of the method shown
//    here is that you can process the K-th set of information and
//    then discard it.
//
//  Example:
//
//    Rule 1:
//      Order = 4
//      X(1:4) = ( 1, 2, 3, 4 )
//
//    Rule 2:
//      Order = 3
//      X(1:3) = ( 10, 20, 30 )
//
//    Rule 3:
//      Order = 2
//      X(1:2) = ( 100, 200 )
//
//    Product Rule:
//      Order = 24
//      X(1:24) =
//        ( 1, 10, 100 )
//        ( 2, 10, 100 )
//        ( 3, 10, 100 )
//        ( 4, 10, 100 )
//        ( 1, 20, 100 )
//        ( 2, 20, 100 )
//        ( 3, 20, 100 )
//        ( 4, 20, 100 )
//        ( 1, 30, 100 )
//        ( 2, 30, 100 )
//        ( 3, 30, 100 )
//        ( 4, 30, 100 )
//        ( 1, 10, 200 )
//        ( 2, 10, 200 )
//        ( 3, 10, 200 )
//        ( 4, 10, 200 )
//        ( 1, 20, 200 )
//        ( 2, 20, 200 )
//        ( 3, 20, 200 )
//        ( 4, 20, 200 )
//        ( 1, 30, 200 )
//        ( 2, 30, 200 )
//        ( 3, 30, 200 )
//        ( 4, 30, 200 )
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    18 April 2009
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int FACTOR_INDEX, the index of the factor being processed.
//    The first factor processed must be factor 0.
//
//    int FACTOR_ORDER, the order of the factor.
//
//    double FACTOR_VALUE[FACTOR_ORDER], the factor values
//    for factor FACTOR_INDEX.
//
//    int FACTOR_NUM, the number of factors.
//
//    int POINT_NUM, the number of elements in the direct product.
//
//    double X[FACTOR_NUM*POINT_NUM], the elements of the
//    direct product, which are built up gradually.
//
//  Output:
//
//    double X[FACTOR_NUM*POINT_NUM], the elements of the
//    direct product, to which another component has been included.
//
//  Local:
//
//    int START, the first location of a block of values to set.
//
//    int CONTIG, the number of consecutive values to set.
//
//    int SKIP, the distance from the current value of START
//    to the next location of a block of values to set.
//
//    int REP, the number of blocks of values to set.
//
{
  static int contig = 0;
  int i;
  int j;
  int k;
  static int rep = 0;
  static int skip = 0;
  int start;

  if ( factor_index == 0 )
  {
    contig = 1;
    skip = 1;
    rep = point_num;
    for ( j = 0; j < point_num; j++ )
    {
      for ( i = 0; i < factor_num; i++ )
      {
        x[i+j*factor_num] = 0.0;
      }
    }
  }

  rep = rep / factor_order;
  skip = skip * factor_order;

  for ( i = 0; i < factor_order; i++ )
  {
    start = 0 + i * contig;

    for ( k = 1; k <= rep; k++ )
    {
      for ( j = start; j < start + contig; j++ )
      {
        x[factor_index+j*factor_num] = factor_value[i];
      }
      start = start + skip;
    }
  }
  contig = contig * factor_order;

  return;
}
//****************************************************************************80

void r8vec_direct_product2 ( int factor_index, int factor_order,
  double factor_value[], int factor_num, int point_num, double w[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_direct_product2() creates a direct product of R8VEC's.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//    To explain what is going on here, suppose we had to construct
//    a multidimensional quadrature rule as the product of K rules
//    for 1D quadrature.
//
//    The product rule will be represented as a list of points and weights.
//
//    The J-th item in the product rule will be associated with
//      item J1 of 1D rule 1,
//      item J2 of 1D rule 2,
//      ...,
//      item JK of 1D rule K.
//
//    In particular,
//      X(J) = ( X(1,J1), X(2,J2), ..., X(K,JK))
//    and
//      W(J) = W(1,J1) * W(2,J2) * ... * W(K,JK)
//
//    So we can construct the quadrature rule if we can properly
//    distribute the information in the 1D quadrature rules.
//
//    This routine carries out that task for the weights W.
//
//    Another way to do this would be to compute, one by one, the
//    set of all possible indices (J1,J2,...,JK), and then index
//    the appropriate information.  An advantage of the method shown
//    here is that you can process the K-th set of information and
//    then discard it.
//
//  Example:
//
//    Rule 1:
//      Order = 4
//      W(1:4) = ( 2, 3, 5, 7 )
//
//    Rule 2:
//      Order = 3
//      W(1:3) = ( 11, 13, 17 )
//
//    Rule 3:
//      Order = 2
//      W(1:2) = ( 19, 23 )
//
//    Product Rule:
//      Order = 24
//      W(1:24) =
//        ( 2 * 11 * 19 )
//        ( 3 * 11 * 19 )
//        ( 4 * 11 * 19 )
//        ( 7 * 11 * 19 )
//        ( 2 * 13 * 19 )
//        ( 3 * 13 * 19 )
//        ( 5 * 13 * 19 )
//        ( 7 * 13 * 19 )
//        ( 2 * 17 * 19 )
//        ( 3 * 17 * 19 )
//        ( 5 * 17 * 19 )
//        ( 7 * 17 * 19 )
//        ( 2 * 11 * 23 )
//        ( 3 * 11 * 23 )
//        ( 5 * 11 * 23 )
//        ( 7 * 11 * 23 )
//        ( 2 * 13 * 23 )
//        ( 3 * 13 * 23 )
//        ( 5 * 13 * 23 )
//        ( 7 * 13 * 23 )
//        ( 2 * 17 * 23 )
//        ( 3 * 17 * 23 )
//        ( 5 * 17 * 23 )
//        ( 7 * 17 * 23 )
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    24 April 2009
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int FACTOR_INDEX, the index of the factor being processed.
//    The first factor processed must be factor 0.
//
//    int FACTOR_ORDER, the order of the factor.
//
//    double FACTOR_VALUE[FACTOR_ORDER], the factor values for
//    factor FACTOR_INDEX.
//
//    int FACTOR_NUM, the number of factors.
//
//    int POINT_NUM, the number of elements in the direct product.
//
//    double W[POINT_NUM], the elements of the direct product, which are 
//    built up gradually.
//
//  Output:
//
//    double W[POINT_NUM], the updated elements of the
//    direct product.
//
//  Local:
//
//    integer START, the first location of a block of values to set.
//
//    integer CONTIG, the number of consecutive values to set.
//
//    integer SKIP, the distance from the current value of START
//    to the next location of a block of values to set.
//
//    integer REP, the number of blocks of values to set.
//
{
  static int contig = 0;
  int i;
  int j;
  int k;
  static int rep = 0;
  static int skip = 0;
  int start;

  if ( factor_index == 0 )
  {
    contig = 1;
    skip = 1;
    rep = point_num;
    for ( i = 0; i < point_num; i++ )
    {
      w[i] = 1.0;
    }
  }

  rep = rep / factor_order;
  skip = skip * factor_order;

  for ( j = 0; j < factor_order; j++ )
  {
    start = 0 + j * contig;

    for ( k = 1; k <= rep; k++ )
    {
      for ( i = start; i < start + contig; i++ )
      {
        w[i] = w[i] * factor_value[j];
      }
      start = start + skip;
    }
  }

  contig = contig * factor_order;

  return;
}
//****************************************************************************80

double r8vec_distance ( int dim_num, double v1[], double v2[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_distance() returns the Euclidean distance between two R8VEC's.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    11 August 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int DIM_NUM, the spatial dimension.
//
//    double V1[DIM_NUM], V2[DIM_NUM], the vectors.
//
//  Output:
//
//    double R8VEC_DISTANCE, the Euclidean distance
//    between the vectors.
//
{
  int i;
  double value;

  value = 0.0;
  for ( i = 0; i < dim_num; i++ )
  {
    value = pow ( v1[i] - v2[i], 2 );
  }
  value = sqrt ( value );

  return value;
}
//****************************************************************************80

void r8vec_divide ( int n, double a[], double s )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_divide() divides an R8VEC by a nonzero scalar.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    30 August 2008
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of entries in the vector.
//
//    double A[N]: the vector to be scaled.
//
//    double S, the divisor.
//
//  Output:
//
//    double A[N]: the scaled vector.
//
{
  int i;

  for ( i = 0; i < n; i++ )
  {
    a[i] = a[i] / s;
  }
  return;
}
//****************************************************************************80

double r8vec_dot_product ( int n, double a1[], double a2[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_dot_product() computes the dot product of a pair of R8VEC's.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    03 July 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of entries in the vectors.
//
//    double A1[N], A2[N], the two vectors to be considered.
//
//  Output:
//
//    double R8VEC_DOT_PRODUCT, the dot product of the vectors.
//
{
  int i;
  double value;

  value = 0.0;
  for ( i = 0; i < n; i++ )
  {
    value = value + a1[i] * a2[i];
  }
  return value;
}
//****************************************************************************80

double r8vec_dot_product_affine ( int n, double v0[], double v1[], double v2[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_dot_product_affine() computes the affine dot product.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    27 October 2010
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of entries in the vectors.
//
//    double V0[N], the base vector.
//
//    double V1[N], V2[N], the two vectors to be considered.
//
//  Output:
//
//    double R8VEC_DOT_PRODUCT_AFFINE, the dot product of the vectors.
//
{
  int i;
  double value;

  value = 0.0;
  for ( i = 0; i < n; i++ )
  {
    value = value + ( v1[i] - v0[i] ) * ( v2[i] - v0[i] );
  }
  return value;
}
//****************************************************************************80

double r8vec_entropy ( int n, double x[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_entropy() computes the entropy of an R8VEC.
//
//  Discussion:
//
//    Typically, the entries represent probabilities, and must sum to 1.
//    For this function, the only requirement is that the entries be nonnegative.
//
//    An R8VEC is a vector of R8's.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    30 August 2013
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of entries.
//
//    double X[N], the vector.
//    Each entry must be nonnegative.
//
//  Output:
//
//    double R8VEC_ENTROPY, the entropy of the
//    normalized vector.
//
{
  int i;
  double value;
  double x_sum;
  double xi;

  for ( i = 0; i < n; i++ )
  {
    if ( x[i] < 0.0 )
    {
      cerr << "\n";
      cerr << "R8VEC_ENTROPY - Fatal error!\n";
      cerr << "  Some entries are negative.\n";
      exit ( 1 );
    }
  }

  x_sum = 0.0;
  for ( i = 0; i < n; i++ )
  {
    x_sum = x_sum + x[i];
  }

  if ( x_sum == 0.0 )
  {
    cerr << "\n";
    cerr << "R8VEC_ENTROPY - Fatal error!\n";
    cerr << "  Entries sum to 0.\n";
    exit ( 1 );
  }

  value = 0.0;
  for ( i = 0; i < n; i++ )
  {
    if ( 0.0 < x[i] )
    {
      xi = x[i] / x_sum;
      value = value - r8_log_2 ( xi ) * xi;
    }
  }

  return value;
}
//****************************************************************************80

bool r8vec_eq ( int n, double a1[], double a2[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_eq() is true if every pair of entries in two R8VEC's is equal.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    28 August 2003
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of entries in the vectors.
//
//    double A1[N], A2[N], two vectors to compare.
//
//  Output:
//
//    bool R8VEC_EQ, is TRUE if every pair of elements A1(I)
//    and A2(I) are equal, and FALSE otherwise.
//
{
  int i;

  for ( i = 0; i < n; i++ )
  {
    if ( a1[i] != a2[i] )
    {
      return false;
    }
  }
  return true;
}
//****************************************************************************80

void r8vec_even ( int n, double alo, double ahi, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_even() returns an R8VEC of values evenly spaced between ALO and AHI.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    17 February 2011
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of values.
//
//    double ALO, AHI, the low and high values.
//
//  Output:
//
//    double A[N], N evenly spaced values.
//    Normally, A[0] = ALO and A[N-1] = AHI.
//    However, if N = 1, then A[0] = 0.5*(ALO+AHI).
//
{
  int i;

  if ( n == 1 )
  {
    a[0] = 0.5 * ( alo + ahi );
  }
  else
  {
    for ( i = 0; i < n; i++ )
    {
      a[i] = ( ( double ) ( n - i - 1 ) * alo
             + ( double ) (     i     ) * ahi )
             / ( double ) ( n     - 1 );
    }
  }

  return;
}
//****************************************************************************80

double *r8vec_even_new ( int n, double alo, double ahi )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_even_new() returns an R8VEC of values evenly spaced between ALO and AHI.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    18 May 2009
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of values.
//
//    double ALO, AHI, the low and high values.
//
//  Output:
//
//    double R8VEC_EVEN_NEW[N], N evenly spaced values.
//    Normally, A[0] = ALO and A[N-1] = AHI.
//    However, if N = 1, then A[0] = 0.5*(ALO+AHI).
//
{
  double *a;
  int i;

  a = new double[n];

  if ( n == 1 )
  {
    a[0] = 0.5 * ( alo + ahi );
  }
  else
  {
    for ( i = 0; i < n; i++ )
    {
      a[i] = ( ( double ) ( n - i - 1 ) * alo
             + ( double ) (     i     ) * ahi )
             / ( double ) ( n     - 1 );
    }
  }

  return a;
}
//****************************************************************************80

double r8vec_even_select ( int n, double xlo, double xhi, int ival )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_even_select() returns the I-th of N evenly spaced values in [ XLO, XHI ].
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//    XVAL = ( (N-IVAL) * XLO + (IVAL-1) * XHI ) / ( N - 1 )
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    24 January 2004
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of values.
//
//    double XLO, XHI, the low and high values.
//
//    int IVAL, the index of the desired point.
//    IVAL is normally between 1 and N, but may be any integer value.
//
//  Output:
//
//    double R8VEC_EVEN_SELECT, the IVAL-th of N evenly spaced values
//    between XLO and XHI.
//    Unless N = 1, X(1) = XLO and X(N) = XHI.
//    If N = 1, then X(1) = 0.5*(XLO+XHI).
//
{
  double xval;

  if ( n == 1 )
  {
    xval = 0.5 * ( xlo + xhi );
  }
  else
  {
    xval = ( ( double ) ( n - ival     ) * xlo
           + ( double ) (     ival - 1 ) * xhi )
           / ( double ) ( n        - 1 );
  }

  return xval;
}
//****************************************************************************80

void r8vec_even2 ( int maxval, int nfill[], int nold, double xold[],
  int &nval, double xval[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_even2() linearly interpolates new numbers into an R8VECa.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//    The number of values created between two old values can vary from
//    one pair of values to the next.
//
//    The interpolated values are evenly spaced.
//
//    This routine is a generalization of R8VEC_EVEN.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    26 November 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int MAXVAL, the size of the XVAL array, as declared by the
//    user.  MAXVAL must be large enough to hold the NVAL values computed by
//    this routine.  In other words, MAXVAL must be at least equal to
//    NOLD + SUM (1 <= I <= NOLD-1) NFILL(I).
//
//    int NFILL[NOLD-1], the number of values
//    to be interpolated between XOLD(I) and XOLD(I+1).
//    NFILL(I) does not count the endpoints.  Thus, if
//    NFILL(I) is 1, there will be one new point generated
//    between XOLD(I) and XOLD(I+1).
//    NFILL(I) must be nonnegative.
//
//    int NOLD, the number of values XOLD,
//    between which extra values are to be interpolated.
//
//    double XOLD[NOLD], the original vector of numbers
//    between which new values are to be interpolated.
//
//  Output:
//
//    int &NVAL, the number of values computed
//    in the XVAL array.
//    NVAL = NOLD + SUM ( 1 <= I <= NOLD-1 ) NFILL(I)
//
//    double XVAL[MAXVAL].  On output, XVAL contains the
//    NOLD values of XOLD, as well as the interpolated
//    values, making a total of NVAL values.
//
{
  int i;
  int j;
  int nadd;

  nval = 1;

  for ( i = 1; i <= nold - 1; i++ )
  {

    if ( nfill[i-1] < 0 )
    {
      cerr << "\n";
      cerr << "R8VEC_EVEN2 - Fatal error!\n";
      cerr << "  NFILL[I-1] is negative for I = " << i << "\n";
      cerr << "  NFILL[I-1] = " << nfill[i-1] << "\n";
      exit ( 1 );
    }

    if ( maxval < nval + nfill[i-1] + 1 )
    {
      cerr << "\n";
      cerr << "R8VEC_EVEN2 - Fatal error!\n";
      cerr << "  MAXVAL = " << maxval << " is not large enough.\n";
      cerr << "  for the storage for interval I = " << i << "\n";
      exit ( 1 );
    }

    nadd = nfill[i-1] + 2;

    for ( j = 1; j <= nadd; j++ )
    {
      xval[nval+j-2] = ( ( double ) ( nadd - j     ) * xold[i-1]
                       + ( double ) (        j - 1 ) * xold[i] )
                       / ( double ) ( nadd     - 1 );
    }

    nval = nval + nfill[i-1] + 1;
  }

  return;
}
//****************************************************************************80

double r8vec_even2_select ( int n, double xlo, double xhi, int ival )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_even2_select() returns the I-th of N evenly spaced midpoint values.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//    This function returns the I-th of N evenly spaced midpoints of N
//    equal subintervals of [XLO,XHI].
//
//    XVAL = ( ( 2 * N - 2 * IVAL + 1 ) * XLO 
//           + (         2 * IVAL - 1 ) * XHI ) 
//           / ( 2 * N                )
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    25 July 2012
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of values.
//
//    double XLO, XHI, the low and high values.
//
//    int IVAL, the index of the desired point.
//    IVAL is normally between 1 and N, but may be any integer value.
//
//  Output:
//
//    double R8VEC_EVEN2_SELECT, the IVAL-th of N evenly spaced midpoints
//    between XLO and XHI.
//
{
  double xval;

  xval = ( ( double ) ( 2 * n - 2 * ival + 1 ) * xlo
         + ( double ) (         2 * ival - 1 ) * xhi )
         / ( double ) ( 2 * n                );

  return xval;
}
//****************************************************************************80

void r8vec_even3 ( int nold, int nval, double xold[], double xval[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_even3() evenly interpolates new data into an R8VEC.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//    This routine accepts a short vector of numbers, and returns a longer
//    vector of numbers, created by interpolating new values between
//    the given values.
//
//    Between any two original values, new values are evenly interpolated.
//
//    Over the whole vector, the new numbers are interpolated in
//    such a way as to try to minimize the largest distance interval size.
//
//    The algorithm employed is not "perfect".
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    30 October 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int NOLD, the number of values XOLD, between which extra
//    values are to be interpolated.
//
//    int NVAL, the number of values to be computed
//    in the XVAL array.  NVAL should be at least NOLD.
//
//    double XOLD[NOLD], the original vector of numbers
//    between which new values are to be interpolated.
//
//  Output:
//
//    double XVAL[NVAL].  On output, XVAL contains the
//    NOLD values of XOLD, as well as interpolated
//    values, making a total of NVAL values.
//
{
  double density;
  int i;
  int ival;
  int j;
  int nmaybe;
  int npts;
  int ntemp;
  int ntot;
  double xlen;
  double xleni;
  double xlentot;

  xlen = 0.0;
  for ( i = 1; i <= nold - 1; i++ )
  {
    xlen = xlen + fabs ( xold[i] - xold[i-1] );
  }

  ntemp = nval - nold;

  density = ( double ) ( ntemp ) / xlen;

  ival = 1;
  ntot = 0;
  xlentot = 0.0;

  for ( i = 1; i <= nold - 1; i++ )
  {
    xleni = fabs ( xold[i] - xold[i-1] );
    npts = ( int ) ( density * xleni );
    ntot = ntot + npts;
//
//  Determine if we have enough left-over density that it should
//  be changed into a point.  A better algorithm would agonize
//  more over where that point should go.
//
    xlentot = xlentot + xleni;
    nmaybe = round ( xlentot * density );

    if ( ntot < nmaybe )
    {
      npts = npts + nmaybe - ntot;
      ntot = nmaybe;
    }
    for ( j = 1; j <= npts + 2; j++ )
    {
      xval[ival+j-2] = ( ( double ) ( npts+2 - j     ) * xold[i-1]
                       + ( double ) (          j - 1 ) * xold[i] )
                       / ( double ) ( npts+2     - 1 );
    }
    ival = ival + npts + 1;
  }

  return;
}
//****************************************************************************80

double *r8vec_expand_linear ( int n, double x[], int fat )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_expand_linear() linearly interpolates new data into an R8VEC.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    26 September 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of input data values.
//
//    double X[N], the original data.
//
//    int FAT, the number of data values to interpolate
//    between each pair of original data values.
//
//  Output:
//
//    double R8VEC_EXPAND_LINEAR[(N-1)*(FAT+1)+1], the "fattened" data.
//
{
  int i;
  int j;
  int k;
  double *xfat;

  xfat = new double[(n-1)*(fat+1)+1];

  k = 0;

  for ( i = 0; i < n-1; i++ )
  {
    xfat[k] = x[i];
    k = k + 1;

    for ( j = 1; j <= fat; j++ )
    {
      xfat[k] = ( ( double ) ( fat - j + 1 ) * x[i]
                + ( double ) (       j     ) * x[i+1] )
                / ( double ) ( fat     + 1 );
      k = k + 1;
    }
  }

  xfat[k] = x[n-1];
  k = k + 1;

  return xfat;
}
//****************************************************************************80

double *r8vec_expand_linear2 ( int n, double x[], int before, int fat, 
  int after )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_expand_linear2() linearly interpolates new data into an R8VEC.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//    This routine starts with a vector of data.
//
//    The intent is to "fatten" the data, that is, to insert more points
//    between successive values of the original data.
//
//    There will also be extra points placed BEFORE the first original
//    value and AFTER that last original value.
//
//    The "fattened" data is equally spaced between the original points.
//
//    The BEFORE data uses the spacing of the first original interval,
//    and the AFTER data uses the spacing of the last original interval.
//
//  Example:
//
//    N = 3
//    BEFORE = 3
//    FAT = 2
//    AFTER = 1
//
//    X    = (/                   0.0,           6.0,             7.0       /)
//    XFAT = (/ -6.0, -4.0, -2.0, 0.0, 2.0, 4.0, 6.0, 6.33, 6.66, 7.0, 7.66 /)
//            3 "BEFORE's"        Old  2 "FATS"  Old    2 "FATS"  Old  1 "AFTER"
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    02 July 2012
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of input data values.
//    N must be at least 2.
//
//    double X[N], the original data.
//
//    int BEFORE, the number of "before" values.
//
//    int FAT, the number of data values to interpolate
//    between each pair of original data values.
//
//    int AFTER, the number of "after" values.
//
//  Output:
//
//    double R8VEC_EXPAND_LINEAR2[BEFORE+(N-1)*(FAT+1)+1+AFTER], the
//    "fattened" data.
//
{
  int i;
  int j;
  int k;
  double *xfat;

  xfat = new double[before+(n-1)*(fat+1)+1+after];

  k = 0;
//
//  Points BEFORE.
//
  for ( j = 1 - before + fat; j <= fat; j++ )
  {
    xfat[k] = ( ( double ) ( fat - j + 1 ) * ( x[0] - ( x[1] - x[0] ) ) 
              + ( double ) (       j     ) *   x[0]          ) 
              / ( double ) ( fat     + 1 );
    k = k + 1;
  }
//
//  Original points and FAT points.
//
  for ( i = 0; i < n - 1; i++ )
  {
    xfat[k] = x[0];
    k = k + 1;
    for ( j = 1; j <= fat; j++ )
    {
      xfat[k] = ( ( double ) ( fat - j + 1 ) * x[i]
                + ( double ) (       j     ) * x[i+1] ) 
                / ( double ) ( fat     + 1 );
      k = k + 1;
    }
  }

  xfat[k] = x[n-1];
  k = k + 1;
//
//  Points AFTER.
//
  for ( j = 1; j <= after; j++ )
  {
    xfat[k] = ( ( double ) ( fat - j + 1 ) * x[n-1]
              + ( double ) (       j     ) * ( x[n-1] + ( x[n-1] - x[n-2] ) ) ) 
              / ( double ) ( fat     + 1 );
    k = k + 1;
  }

  return xfat;
}
//****************************************************************************80

void r8vec_fill ( int n, double value, double x[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_fill() sets all entries of an R8VEC to a given value.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    03 December 2016
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of elements of A.
//
//    double VALUE, the value.
//
//  Output:
//
//    double X[N], the array.
//
{
  int i;

  for ( i = 0; i < n; i++ )
  {
    x[i] = value;
  }
  return;
}
//****************************************************************************80

double *r8vec_fill_new ( int n, double value )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_fill_new() creates an R8VEC, setting all entries to a given value.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    03 December 2016
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of elements of A.
//
//    double VALUE, the value.
//
//  Output:
//
//    double R8VEC_FILL_NEW[N], the array.
//
{
  int i;
  double *x;

  x = new double[n];

  for ( i = 0; i < n; i++ )
  {
    x[i] = value;
  }
  return x;
}
//****************************************************************************80

int *r8vec_first_index ( int n, double a[], double tol )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_first_index() indexes the first occurrence of values in an R8VEC.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//    For element A(I) of the vector, FIRST_INDEX(I) is the index in A of
//    the first occurrence of the value A(I).
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    24 August 2008
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of elements of A.
//
//    double A[N], the unsorted array to examine.
//
//    double TOL, a tolerance for equality.
//
//  Output:
//
//    int R8VEC_FIRST_INDEX[N], the first occurrence index.
//
{
  int *first_index;
  int i;
  int j;

  first_index = new int[n];

  for ( i = 0; i < n; i++ )
  {
    first_index[i] = -1;
  }
  for ( i = 0; i < n; i++ )
  {
    if ( first_index[i] == -1 )
    {
      first_index[i] = i;
      for ( j = i + 1; j < n; j++ )
      {
        if ( fabs ( a[i] - a[j] ) <= tol )
        {
          first_index[j] = i;
        }
      }
    }
  }
  return first_index;
}
//****************************************************************************80

double r8vec_frac ( int n, double a[], int k )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_frac() searches for the K-th smallest entry in an R8VEC.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//    Hoare's algorithm is used.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    29 August 2004
//
//  Input:
//
//    int N, the number of elements of A.
//
//    double A[N]: the array to search.
//
//    int K, the fractile to be sought.  If K = 1, the minimum
//    entry is sought.  If K = N, the maximum is sought.  Other values
//    of K search for the entry which is K-th in size.  K must be at
//    least 1, and no greater than N.
//
//  Output:
//
//    Idouble A[N]: the elements of A have been somewhat rearranged.
//
//    double R8VEC_FRAC, the value of the K-th fractile of A.
//
{
  double frac;
  int i;
  int iryt;
  int j;
  int left;
  double temp;
  double x;

  if ( n <= 0 )
  {
    cerr << "\n";
    cerr << "R8VEC_FRAC - Fatal error!\n";
    cerr << "  Illegal nonpositive value of N = " << n << "\n";
    exit ( 1 );
  }

  if ( k <= 0 )
  {
    cerr << "\n";
    cerr << "R8VEC_FRAC - Fatal error!\n";
    cerr << "  Illegal nonpositive value of K = " << k << "\n";
    exit ( 1 );
  }

  if ( n < k )
  {
    cerr << "\n";
    cerr << "R8VEC_FRAC - Fatal error!\n";
    cerr << "  Illegal N < K, K = " << k << "\n";
    exit ( 1 );
  }

  left = 1;
  iryt = n;

  for ( ; ; )
  {
    if ( iryt <= left )
    {
      frac = a[k-1];
      break;
    }

    x = a[k-1];
    i = left;
    j = iryt;

    for ( ; ; )
    {
      if ( j < i )
      {
        if ( j < k )
        {
          left = i;
        }
        if ( k < i )
        {
          iryt = j;
        }
        break;
      }
//
//  Find I so that X <= A(I).
//
      while ( a[i-1] < x )
      {
        i = i + 1;
      }
//
//  Find J so that A(J) <= X.
//
      while ( x < a[j-1] )
      {
        j = j - 1;
      }

      if ( i <= j )
      {
        temp   = a[i-1];
        a[i-1] = a[j-1];
        a[j-1] = temp;
        i = i + 1;
        j = j - 1;
      }
    }
  }

  return frac;
}
//****************************************************************************80

double *r8vec_fraction ( int n, double x[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_fraction() returns the fraction parts of an R8VEC.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//    If we regard a real number as
//
//      R8 = SIGN * ( WHOLE + FRACTION )
//
//    where
//
//      SIGN is +1 or -1,
//      WHOLE is a nonnegative integer
//      FRACTION is a nonnegative real number strictly less than 1,
//
//    then this routine returns the value of FRACTION.
//
//  Example:
//
//     R8    R8_FRACTION
//
//    0.00      0.00
//    1.01      0.01
//    2.02      0.02
//   19.73      0.73
//   -4.34      0.34
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    19 April 2007
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of arguments.
//
//    double X[N], the arguments.
//
//  Output:
//
//    double R8_FRACTION[N], the fraction parts.
//
{
  double *fraction;
  int i;

  fraction = new double[n];

  for ( i = 0; i < n; i++ )
  {
    fraction[i] = fabs ( x[i] ) - ( double ) ( ( int ) ( fabs ( x[i] ) ) );
  }

  return fraction;
}
//****************************************************************************80

bool r8vec_gt ( int n, double a1[], double a2[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_gt() == ( A1 > A2 ) for two R8VEC's.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//    The comparison is lexicographic.
//
//    A1 > A2  <=>                              A1(1) > A2(1) or
//                 ( A1(1)     == A2(1)     and A1(2) > A2(2) ) or
//                 ...
//                 ( A1(1:N-1) == A2(1:N-1) and A1(N) > A2(N)
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    28 August 2003
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the dimension of the vectors.
//
//    double A1[N], A2[N], the vectors to be compared.
//
//  Output:
//
//    bool R8VEC_GT, is TRUE if and only if A1 > A2.
//
{
  int i;

  for ( i = 0; i < n; i++ )
  {

    if ( a2[i] < a1[i] )
    {
       return true;
    }
    else if ( a1[i] < a2[i] )
    {
      return false;
    }

  }

  return false;
}
//****************************************************************************80

void r8vec_heap_a ( int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_heap_a() reorders an R8VEC into a ascending heap.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//    An ascending heap is an array A with the property that, for every index J,
//    A[J] <= A[2*J+1] and A[J] <= A[2*J+2], (as long as the indices
//    2*J+1 and 2*J+2 are legal).
//
//  Diagram:
//
//                  A(0)
//
//            A(1)         A(2)
//
//      A(3)       A(4)  A(5) A(6)
//
//    A(7) A(8)  A(9) A(10)
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    17 September 2005
//
//  Author:
//
//    John Burkardt
//
//  Reference:
//
//    Albert Nijenhuis, Herbert Wilf,
//    Combinatorial Algorithms,
//    Academic Press, 1978, second edition,
//    ISBN 0-12-519260-6.
//
//  Input:
//
//    int N, the size of the input array.
//
//    double A[N]: an unsorted array.
//
//  Output:
//
//    double A[N]: reordered into a heap.
{
  int i;
  int ifree;
  double key;
  int m;
//
//  Only nodes (N/2)-1 down to 0 can be "parent" nodes.
//
  for ( i = (n/2)-1; 0 <= i; i-- )
  {
//
//  Copy the value out of the parent node.
//  Position IFREE is now "open".
//
    key = a[i];
    ifree = i;

    for ( ; ; )
    {
//
//  Positions 2*IFREE + 1 and 2*IFREE + 2 are the descendants of position
//  IFREE.  (One or both may not exist because they equal or exceed N.)
//
      m = 2 * ifree + 1;
//
//  Does the first position exist?
//
      if ( n <= m )
      {
        break;
      }
      else
      {
//
//  Does the second position exist?
//
        if ( m + 1 < n )
        {
//
//  If both positions exist, take the larger of the two values,
//  and update M if necessary.
//
          if ( a[m+1] < a[m] )
          {
            m = m + 1;
          }
        }
//
//  If the large descendant is larger than KEY, move it up,
//  and update IFREE, the location of the free position, and
//  consider the descendants of THIS position.
//
        if ( a[m] <= key )
        {
          break;
        }
        a[ifree] = a[m];
        ifree = m;
      }
    }
//
//  When you have stopped shifting items up, return the item you
//  pulled out back to the heap.
//
    a[ifree] = key;
  }

  return;
}
//****************************************************************************80

void r8vec_heap_d ( int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_heap_d() reorders an R8VEC into a descending heap.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//    A heap is an array A with the property that, for every index J,
//    A[J] >= A[2*J+1] and A[J] >= A[2*J+2], (as long as the indices
//    2*J+1 and 2*J+2 are legal).
//
//  Diagram:
//
//                  A(0)
//
//            A(1)         A(2)
//
//      A(3)       A(4)  A(5) A(6)
//
//    A(7) A(8)  A(9) A(10)
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    30 April 1999
//
//  Author:
//
//    John Burkardt
//
//  Reference:
//
//    Albert Nijenhuis, Herbert Wilf,
//    Combinatorial Algorithms,
//    Academic Press, 1978, second edition,
//    ISBN 0-12-519260-6.
//
//  Input:
//
//    int N, the size of the input array.
//
//    double A[N]: an unsorted array.
//
//  Output:
//
//    double A[N]: has been reordered into a heap.
//
{
  int i;
  int ifree;
  double key;
  int m;
//
//  Only nodes (N/2)-1 down to 0 can be "parent" nodes.
//
  for ( i = (n/2)-1; 0 <= i; i-- )
  {
//
//  Copy the value out of the parent node.
//  Position IFREE is now "open".
//
    key = a[i];
    ifree = i;

    for ( ; ; )
    {
//
//  Positions 2*IFREE + 1 and 2*IFREE + 2 are the descendants of position
//  IFREE.  (One or both may not exist because they equal or exceed N.)
//
      m = 2 * ifree + 1;
//
//  Does the first position exist?
//
      if ( n <= m )
      {
        break;
      }
      else
      {
//
//  Does the second position exist?
//
        if ( m + 1 < n )
        {
//
//  If both positions exist, take the larger of the two values,
//  and update M if necessary.
//
          if ( a[m] < a[m+1] )
          {
            m = m + 1;
          }
        }
//
//  If the large descendant is larger than KEY, move it up,
//  and update IFREE, the location of the free position, and
//  consider the descendants of THIS position.
//
        if ( key < a[m] )
        {
          a[ifree] = a[m];
          ifree = m;
        }
        else
        {
          break;
        }
      }
    }
//
//  When you have stopped shifting items up, return the item you
//  pulled out back to the heap.
//
    a[ifree] = key;
  }

  return;
}
//****************************************************************************80

int *r8vec_histogram ( int n, double a[], double a_lo, double a_hi,
  int histo_num )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_histogram() histograms an R8VEC.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//    Values between A_LO and A_HI will be histogrammed into the bins
//    1 through HISTO_NUM.  Values below A_LO are counted in bin 0,
//    and values greater than A_HI are counted in bin HISTO_NUM+1.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    09 September 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of elements of A.
//
//    double A[N], the array to examine.
//
//    double A_LO, A_HI, the lowest and highest
//    values to be histogrammed.  These values will also define the bins.
//
//    int HISTO_NUM, the number of bins to use.
//
//  Output:
//
//    int HISTO_GRAM[HISTO_NUM+2], contains the number of
//    entries of A in each bin.
//
{
  double delta;
  int *histo_gram;
  int i;
  int j;

  histo_gram = new int[histo_num+2];

  i4vec_zeros ( histo_num+2, histo_gram );

  delta = ( a_hi - a_lo ) / ( double ) ( 2 * histo_num );

  for ( i = 0; i < n; i++ )
  {
    if ( a[i] < a_lo )
    {
      histo_gram[0] = histo_gram[0] + 1;
    }
    else if ( a[i] <= a_hi )
    {
      j = round (
        ( ( a_hi -       delta - a[i]        ) * ( double ) ( 1         )
        + (      -       delta + a[i] - a_lo ) * ( double ) ( histo_num ) )
        / ( a_hi - 2.0 * delta        - a_lo ) );

      histo_gram[j] = histo_gram[j] + 1;
    }
    else if ( a_hi < a[i] )
    {
      histo_gram[histo_num+1] = histo_gram[histo_num+1] + 1;
    }
  }

  return histo_gram;
}
//****************************************************************************80

double *r8vec_house_column ( int n, double a_vec[], int k )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_house_column() defines a Householder premultiplier that "packs" a column.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//    The routine returns a vector V that defines a Householder
//    premultiplier matrix H(V) that zeros out the subdiagonal entries of
//    column K of the matrix A.
//
//       H(V) = I - 2 * v * v'
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    08 October 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the order of the matrix A.
//
//    double A_VEC[N], a row or column of the matrix A.
//
//    int K, the index of the row or column.
//
//  Output:
//
//    double R8VEC_HOUSE_COLUMN[N], a vector of unit L2 norm which
//    defines an orthogonal Householder premultiplier matrix H with the property
//    that the K-th column of H*A is zero below the diagonal.
//
{
  int i;
  double s;
  double *v;

  v = r8vec_zeros_new ( n );

  if ( k < 1 || n <= k )
  {
    return v;
  }

  s = r8vec_norm_l2 ( n+1-k, a_vec+k-1 );

  if ( s == 0.0 )
  {
    return v;
  }

  v[k-1] = a_vec[k-1] + fabs ( s ) * r8_sign ( a_vec[k-1] );

  r8vec_copy ( n-k, a_vec+k, v+k );
//
//  Normalize.
//
  s = r8vec_norm_l2 ( n-k+1, v+k-1 );

  for ( i = k - 1; i < n; i++ )
  {
    v[i] = v[i] / s;
  }

  return v;
}
//****************************************************************************80

double r8vec_i4vec_dot_product ( int n, double r8vec[], int i4vec[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_i4vec_dot_product() computes the dot product of an R8VEC and an I4VEC.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//    An I4VEC is a vector of I4's.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    30 June 2009
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of entries in the vectors.
//
//    double R8VEC[N], the first vector.
//
//    int I4VEC[N], the second vector.
//
//  Output:
//
//    double R8VEC_I4VEC_DOT_PRODUCT, the dot product of the vectors.
//
{
  int i;
  double value;

  value = 0.0;
  for ( i = 0; i < n; i++ )
  {
    value = value + r8vec[i] * ( double ) ( i4vec[i] );
  }
  return value;
}
//****************************************************************************80

double *r8vec_identity_row_new ( int n, int i )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_identity_row_new() sets an R8VEC to the I-th row of the identity.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    28 March 2018
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the number of elements of A.
//
//    int I, indicates the row.  0 <= I < N.
//
//  Output:
//
//    double R8VEC_IDENTITY_ROW_NEW[N], the array.
//
{
  double *a;
  int j;

  a = new double[n];

  for ( j = 0; j < n; j++ )
  {
    a[j] = 0.0;
  }

  if ( 0 <= i && i < n )
  {
    a[i] = 1.0;
  }

  return a;
}
//****************************************************************************80

void r8vec_index_delete_all ( int n, double x[], int indx[], double xval,
  int &n2, double x2[], int indx2[] )

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_index_delete_all() deletes all occurrences of a value from an indexed sorted list.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//    Note that the value of N is adjusted because of the deletions.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    23 October 2005
//
//  Author:
//
//    John Burkardt
//
//  Input:
//
//    int N, the size of the current list.
//
//    double X[N], the list.
//
//    int INDX[N], the sort index of the list.
//
//    double XVAL, the value to be sought.
//
//  Output:
//
//    int &N2, the size of the current list.
//
//    double X2[N2], the list.
//
//    int INDX2[N2], the sort index of the list.
//
{
  int equal;
  int equal1;
  int equal2;
  int get;
  int i;
  int less;
  int more;
  int put;

  if ( n < 1 )
  {
    n2 = 0;
    return;
  }

  i4vec_copy ( n, indx, indx2 );
  r8vec_copy ( n, x, x2 );
  n2 = n;

  r8vec_index_search ( n2, x2, indx2, xval, less, equal, more );

  if ( equal == 0 )
  {
    return;
  }

  equal1 = equal;

  for ( ; ; )
  {
    if ( equal1 <= 1 )
    {
      break;
    }

    if ( x2[indx2[equal1-2]-1] != xval )
    {
      break;
    }
    equal1 = equal1 - 1;
  }

  equal2 = equal;

  for ( ; ; )
  {
    if ( n2 <= equal2 )
    {
      break;
    }

    if ( x2[indx2[equal2]-1] != xval )
    {
      break;
    }
    equal2 = equal2 + 1;
  }
//
//  Discard certain X values.
//
  put = 0;

  for ( get = 1; get <= n2; get++ )
  {
    if ( x2[get-1] != xval )
    {
      put = put + 1;
      x2[put-1] = x2[get-1];
    }
  }
//
//  Adjust the INDX values.
//
  for ( equal = equal1; equal <= equal2; equal++ )
  {
    for ( i = 1; i <= n2; i++ )
    {
      if ( indx2[equal-1] < indx2[i-1] )
      {
        indx2[i-1] = indx2[i-1] - 1;
      }
    }
  }
//
//  Discard certain INDX values.
//
  for ( i = 0; i <= n2 - equal2 - 1; i++ )
  {
    indx2[equal1+i-1] = indx2[equal2+i];
  }
  for ( i = n2 + equal1 - equal2; i <= n2; i++ )
  {
    indx2[i-1] = 0;
  }
//
//  Adjust N.
//
  n2 = put;

  return;
}
//****************************************************************************80

void r8vec_index_delete_dupes ( int n, double x[], int indx[],
  int &n2, double x2[], int indx2[] );

//****************************************************************************80
//
//  Purpose:
//
//    r8vec_index_delete_dupes() deletes duplicates from an indexed sorted list.
//
//  Discussion:
//
//    An R8VEC is a vector of R8's.
//
//    The output quantities N2, X2, and INDX2 are computed from the
//    input quantities by sorting, and eliminating duplicates.
//
//    The output arrays should be dimensioned of size N, unless the user
//    knows in advance what the value of N2 will be.
//
//    The output arrays may be identified with the input arrays.
//
//  Licensing:
//
//    This code is distributed under the MIT license.
//
//  Modified:
//
//    15 October 2005
//
//  Author:
//
//    John Burkardt
//
//
