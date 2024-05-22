#pragma once


#include <mpsv/core/MPSVCommon.hpp>


namespace mpsv {


namespace math {


namespace details {


inline double SVD3_xnrm2(int32_t n, const double x[9], int32_t ix0) noexcept {
  double y;
  double scale;
  int32_t kend;
  double absxk;
  double t;
  int32_t k;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = std::fabs(x[ix0 - 1]);
    } else {
      scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (k = ix0; k <= kend; k++) {
        absxk = std::fabs(x[k - 1]);
        if (absxk > scale) {
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * std::sqrt(y);
    }
  }

  return y;
}

inline double SVD3_xnrm2_e(int32_t n, const double x[3], int32_t ix0) noexcept {
  double y;
  double scale;
  int32_t kend;
  double absxk;
  double t;
  int32_t k;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = std::fabs(x[ix0 - 1]);
    } else {
      scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (k = ix0; k <= kend; k++) {
        absxk = std::fabs(x[k - 1]);
        if (absxk > scale) {
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * std::sqrt(y);
    }
  }

  return y;
}

inline void SVD3_xaxpy_hy(int32_t n, double a, const double x[3], int32_t ix0, double y[9], int32_t iy0) noexcept {
  int32_t ix;
  int32_t iy;
  int32_t k;
  if ((n >= 1) && (!(a == 0.0))) {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * x[ix];
      ix++;
      iy++;
    }
  }
}

inline void SVD3_xaxpy_h(int32_t n, double a, const double x[9], int32_t ix0, double y[3], int32_t iy0) noexcept {
  int32_t ix;
  int32_t iy;
  int32_t k;
  if ((n >= 1) && (!(a == 0.0))) {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * x[ix];
      ix++;
      iy++;
    }
  }
}

inline double SVD3_xdotc(int32_t n, const double x[9], int32_t ix0, const double y[9], int32_t iy0) noexcept {
  double d;
  int32_t ix;
  int32_t iy;
  int32_t k;
  d = 0.0;
  if (n >= 1) {
    ix = ix0;
    iy = iy0;
    for (k = 0; k < n; k++) {
      d += x[ix - 1] * y[iy - 1];
      ix++;
      iy++;
    }
  }

  return d;
}

inline void SVD3_xaxpy(int32_t n, double a, int32_t ix0, double y[9], int32_t iy0) noexcept {
  int32_t ix;
  int32_t iy;
  int32_t k;
  if ((n >= 1) && (!(a == 0.0))) {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * y[ix];
      ix++;
      iy++;
    }
  }
}

inline void SVD3_xscal(double a, double x[9], int32_t ix0) noexcept {
  int32_t k;
  for (k = ix0; k <= ix0 + 2; k++) {
    x[k - 1] *= a;
  }
}

inline void SVD3_xswap(double x[9], int32_t ix0, int32_t iy0) noexcept {
  int32_t ix;
  int32_t iy;
  double temp;
  ix = ix0 - 1;
  iy = iy0 - 1;
  temp = x[ix];
  x[ix] = x[iy];
  x[iy] = temp;
  ix++;
  iy++;
  temp = x[ix];
  x[ix] = x[iy];
  x[iy] = temp;
  ix++;
  iy++;
  temp = x[ix];
  x[ix] = x[iy];
  x[iy] = temp;
}

inline void SVD3_xrotg(double *a, double *b, double *c, double *s) noexcept {
  double roe;
  double absa;
  double absb;
  double scale;
  double ads;
  double bds;
  roe = *b;
  absa = std::fabs(*a);
  absb = std::fabs(*b);
  if (absa > absb) {
    roe = *a;
  }

  scale = absa + absb;
  if (scale == 0.0) {
    *s = 0.0;
    *c = 1.0;
    absa = 0.0;
  } else {
    ads = absa / scale;
    bds = absb / scale;
    scale *= std::sqrt(ads * ads + bds * bds);
    if (roe < 0.0) {
      scale = -scale;
    }

    *c = *a / scale;
    *s = *b / scale;
    if (absa > absb) {
      absa = *s;
    } else if (*c != 0.0) {
      absa = 1.0 / *c;
    } else {
      absa = 1.0;
    }
  }

  *a = scale;
  *b = absa;
}

inline void SVD3_xrot(double x[9], int32_t ix0, int32_t iy0, double c, double s) noexcept {
  int32_t ix;
  int32_t iy;
  double temp;
  ix = ix0 - 1;
  iy = iy0 - 1;
  temp = c * x[ix] + s * x[iy];
  x[iy] = c * x[iy] - s * x[ix];
  x[ix] = temp;
  iy++;
  ix++;
  temp = c * x[ix] + s * x[iy];
  x[iy] = c * x[iy] - s * x[ix];
  x[ix] = temp;
  iy++;
  ix++;
  temp = c * x[ix] + s * x[iy];
  x[iy] = c * x[iy] - s * x[ix];
  x[ix] = temp;
}


} /* namespace: details */


/**
 * @brief Singular value decomposition for 3-by-3 matrix.
 * @param[out] U The 3-by-3 matrix U stored in a column-major-order.
 * @param[out] V The 3-by-3 matrix V stored in a column-major-order.
 * @param[in] A The 3-by-3 matrix A from which to obtain the singular value decomposition, stored in a column-major-order.
 * @details The implementation of this function was generated using MATLABs svd() function.
 */
inline void SVD3(double U[9], double V[9], const double A[9]) noexcept {
  double b_A[9];
  double b_s[3];
  double e[3];
  double work[3];
  int32_t qq;
  bool apply_transform;
  double nrm;
  int32_t qjj;
  int32_t m;
  double rt;
  int32_t kase;
  double ztest;
  double smm1;
  double emm1;
  double sqds;
  double shift;
  int32_t k_ii;
  int32_t d;
  int32_t exitg1;
  bool exitg2;
  e[0] = 0.0;
  work[0] = 0.0;
  e[1] = 0.0;
  work[1] = 0.0;
  e[2] = 0.0;
  work[2] = 0.0;
  for (qq = 0; qq < 9; qq++) {
    b_A[qq] = A[qq];
    U[qq] = 0.0;
    V[qq] = 0.0;
  }

  qq = 0;
  apply_transform = false;
  nrm = details::SVD3_xnrm2(3, b_A, 1);
  if (nrm > 0.0) {
    apply_transform = true;
    if (b_A[0] < 0.0) {
      b_s[0] = -nrm;
    } else {
      b_s[0] = nrm;
    }

    if (std::fabs(b_s[0]) >= 1.0020841800044864E-292) {
      nrm = 1.0 / b_s[0];
      for (qjj = 0; qjj < 3; qjj++) {
        b_A[qjj] *= nrm;
      }
    } else {
      for (qjj = 0; qjj < 3; qjj++) {
        b_A[qjj] /= b_s[0];
      }
    }

    b_A[0]++;
    b_s[0] = -b_s[0];
  } else {
    b_s[0] = 0.0;
  }

  for (d = 1; d + 1 < 4; d++) {
    qjj = 3 * d;
    if (apply_transform) {
      details::SVD3_xaxpy(3, -(details::SVD3_xdotc(3, b_A, 1, b_A, qjj + 1) / b_A[0]), 1, b_A, qjj
                 + 1);
    }

    e[d] = b_A[qjj];
  }

  while (qq + 1 < 4) {
    U[qq] = b_A[qq];
    qq++;
  }

  nrm = details::SVD3_xnrm2_e(2, e, 2);
  if (nrm == 0.0) {
    e[0] = 0.0;
  } else {
    if (e[1] < 0.0) {
      nrm = -nrm;
    }

    e[0] = nrm;
    if (std::fabs(nrm) >= 1.0020841800044864E-292) {
      nrm = 1.0 / nrm;
      for (qq = 1; qq < 3; qq++) {
        e[qq] *= nrm;
      }
    } else {
      for (qq = 1; qq < 3; qq++) {
        e[qq] /= nrm;
      }
    }

    e[1]++;
    e[0] = -e[0];
    for (qq = 2; qq < 4; qq++) {
      work[qq - 1] = 0.0;
    }

    for (qq = 1; qq + 1 < 4; qq++) {
      details::SVD3_xaxpy_h(2, e[qq], b_A, 3 * qq + 2, work, 2);
    }

    for (qq = 1; qq + 1 < 4; qq++) {
      details::SVD3_xaxpy_hy(2, -e[qq] / e[1], work, 2, b_A, 3 * qq + 2);
    }
  }

  for (qq = 1; qq + 1 < 4; qq++) {
    V[qq] = e[qq];
  }

  apply_transform = false;
  nrm = details::SVD3_xnrm2(2, b_A, 5);
  if (nrm > 0.0) {
    apply_transform = true;
    if (b_A[4] < 0.0) {
      b_s[1] = -nrm;
    } else {
      b_s[1] = nrm;
    }

    if (std::fabs(b_s[1]) >= 1.0020841800044864E-292) {
      nrm = 1.0 / b_s[1];
      for (qjj = 4; qjj < 6; qjj++) {
        b_A[qjj] *= nrm;
      }
    } else {
      for (qjj = 4; qjj < 6; qjj++) {
        b_A[qjj] /= b_s[1];
      }
    }

    b_A[4]++;
    b_s[1] = -b_s[1];
  } else {
    b_s[1] = 0.0;
  }

  for (d = 2; d + 1 < 4; d++) {
    qjj = 3 * d + 1;
    if (apply_transform) {
      details::SVD3_xaxpy(2, -(details::SVD3_xdotc(2, b_A, 5, b_A, qjj + 1) / b_A[4]), 5, b_A, qjj
                 + 1);
    }

    e[d] = b_A[qjj];
  }

  for (qq = 1; qq + 1 < 4; qq++) {
    U[qq + 3] = b_A[qq + 3];
  }

  m = 1;
  b_s[2] = b_A[8];
  U[6] = 0.0;
  U[7] = 0.0;
  U[8] = 1.0;
  for (d = 1; d >= 0; d--) {
    qq = 3 * d + d;
    if (b_s[d] != 0.0) {
      for (kase = d + 1; kase + 1 < 4; kase++) {
        qjj = (3 * kase + d) + 1;
        details::SVD3_xaxpy(3 - d, -(details::SVD3_xdotc(3 - d, U, qq + 1, U, qjj) / U[qq]), qq +
                   1, U, qjj);
      }

      for (qjj = d; qjj + 1 < 4; qjj++) {
        U[qjj + 3 * d] = -U[3 * d + qjj];
      }

      U[qq]++;
      if (0 <= d - 1) {
        U[3 * d] = 0.0;
      }
    } else {
      U[3 * d] = 0.0;
      U[1 + 3 * d] = 0.0;
      U[2 + 3 * d] = 0.0;
      U[qq] = 1.0;
    }
  }

  for (qq = 2; qq >= 0; qq--) {
    if ((qq + 1 <= 1) && (e[0] != 0.0)) {
      for (d = 2; d < 4; d++) {
        qjj = (d - 1) * 3 + 2;
        details::SVD3_xaxpy(2, -(details::SVD3_xdotc(2, V, 2, V, qjj) / V[1]), 2, V, qjj);
      }
    }

    V[3 * qq] = 0.0;
    V[1 + 3 * qq] = 0.0;
    V[2 + 3 * qq] = 0.0;
    V[qq + 3 * qq] = 1.0;
  }

  ztest = e[0];
  if (b_s[0] != 0.0) {
    rt = std::fabs(b_s[0]);
    nrm = b_s[0] / rt;
    b_s[0] = rt;
    ztest = e[0] / nrm;
    details::SVD3_xscal(nrm, U, 1);
  }

  if (ztest != 0.0) {
    rt = std::fabs(ztest);
    nrm = rt / ztest;
    ztest = rt;
    b_s[1] *= nrm;
    details::SVD3_xscal(nrm, V, 4);
  }

  e[0] = ztest;
  ztest = b_A[7];
  if (b_s[1] != 0.0) {
    rt = std::fabs(b_s[1]);
    nrm = b_s[1] / rt;
    b_s[1] = rt;
    ztest = b_A[7] / nrm;
    details::SVD3_xscal(nrm, U, 4);
  }

  if (ztest != 0.0) {
    rt = std::fabs(ztest);
    nrm = rt / ztest;
    ztest = rt;
    b_s[2] = b_A[8] * nrm;
    details::SVD3_xscal(nrm, V, 7);
  }

  e[1] = ztest;
  if (b_s[2] != 0.0) {
    rt = std::fabs(b_s[2]);
    nrm = b_s[2] / rt;
    b_s[2] = rt;
    details::SVD3_xscal(nrm, U, 7);
  }

  e[2] = 0.0;
  qq = 0;
  if ((b_s[0] > e[0]) || std::isnan(e[0])) {
    nrm = b_s[0];
  } else {
    nrm = e[0];
  }

  if ((b_s[1] > ztest) || std::isnan(ztest)) {
    ztest = b_s[1];
  }

  if ((!(nrm > ztest)) && (!std::isnan(ztest))) {
    nrm = ztest;
  }

  if (b_s[2] > 0.0) {
    rt = b_s[2];
  } else {
    rt = 0.0;
  }

  if ((!(nrm > rt)) && (!std::isnan(rt))) {
    nrm = rt;
  }

  while ((m + 2 > 0) && (qq < 75)) {
    kase = m + 1;
    do {
      exitg1 = 0;
      d = kase;
      if (kase == 0) {
        exitg1 = 1;
      } else {
        rt = std::fabs(e[kase - 1]);
        if ((rt <= (std::fabs(b_s[kase - 1]) + std::fabs(b_s[kase])) *
             2.2204460492503131E-16) || ((rt <= 1.0020841800044864E-292) || ((qq
               > 20) && (rt <= 2.2204460492503131E-16 * nrm)))) {
          e[kase - 1] = 0.0;
          exitg1 = 1;
        } else {
          kase--;
        }
      }
    } while (exitg1 == 0);

    if (m + 1 == kase) {
      kase = 4;
    } else {
      qjj = m + 2;
      k_ii = m + 2;
      exitg2 = false;
      while ((!exitg2) && (k_ii >= kase)) {
        qjj = k_ii;
        if (k_ii == kase) {
          exitg2 = true;
        } else {
          rt = 0.0;
          if (k_ii < m + 2) {
            rt = std::fabs(e[k_ii - 1]);
          }

          if (k_ii > kase + 1) {
            rt += std::fabs(e[k_ii - 2]);
          }

          ztest = std::fabs(b_s[k_ii - 1]);
          if ((ztest <= 2.2204460492503131E-16 * rt) || (ztest <=
               1.0020841800044864E-292)) {
            b_s[k_ii - 1] = 0.0;
            exitg2 = true;
          } else {
            k_ii--;
          }
        }
      }

      if (qjj == kase) {
        kase = 3;
      } else if (m + 2 == qjj) {
        kase = 1;
      } else {
        kase = 2;
        d = qjj;
      }
    }

    switch (kase) {
     case 1:
      rt = e[m];
      e[m] = 0.0;
      for (qjj = m; qjj + 1 >= d + 1; qjj--) {
        details::SVD3_xrotg(&b_s[qjj], &rt, &ztest, &sqds);
        if (qjj + 1 > d + 1) {
          rt = -sqds * e[0];
          e[0] *= ztest;
        }

        details::SVD3_xrot(V, 1 + 3 * qjj, 1 + 3 * (m + 1), ztest, sqds);
      }
      break;

     case 2:
      rt = e[d - 1];
      e[d - 1] = 0.0;
      for (qjj = d; qjj < m + 2; qjj++) {
        details::SVD3_xrotg(&b_s[qjj], &rt, &ztest, &sqds);
        rt = -sqds * e[qjj];
        e[qjj] *= ztest;
        details::SVD3_xrot(U, 1 + 3 * qjj, 1 + 3 * (d - 1), ztest, sqds);
      }
      break;

     case 3:
      rt = b_s[m + 1];
      ztest = std::fabs(rt);
      sqds = std::fabs(b_s[m]);
      if ((ztest > sqds) || std::isnan(sqds)) {
        sqds = ztest;
      }

      ztest = std::fabs(e[m]);
      if ((sqds > ztest) || std::isnan(ztest)) {
        ztest = sqds;
      }

      sqds = std::fabs(b_s[d]);
      if ((ztest > sqds) || std::isnan(sqds)) {
        sqds = ztest;
      }

      ztest = std::fabs(e[d]);
      if ((sqds > ztest) || std::isnan(ztest)) {
        ztest = sqds;
      }

      rt /= ztest;
      smm1 = b_s[m] / ztest;
      emm1 = e[m] / ztest;
      sqds = b_s[d] / ztest;
      smm1 = ((smm1 + rt) * (smm1 - rt) + emm1 * emm1) / 2.0;
      emm1 *= rt;
      emm1 *= emm1;
      if ((smm1 != 0.0) || (emm1 != 0.0)) {
        shift = std::sqrt(smm1 * smm1 + emm1);
        if (smm1 < 0.0) {
          shift = -shift;
        }

        shift = emm1 / (smm1 + shift);
      } else {
        shift = 0.0;
      }

      rt = (sqds + rt) * (sqds - rt) + shift;
      ztest = e[d] / ztest * sqds;
      for (qjj = d + 1; qjj <= m + 1; qjj++) {
        details::SVD3_xrotg(&rt, &ztest, &sqds, &smm1);
        if (qjj > d + 1) {
          e[0] = rt;
        }

        ztest = e[qjj - 1];
        emm1 = b_s[qjj - 1];
        rt = emm1 * sqds + ztest * smm1;
        e[qjj - 1] = ztest * sqds - emm1 * smm1;
        ztest = smm1 * b_s[qjj];
        b_s[qjj] *= sqds;
        details::SVD3_xrot(V, 1 + 3 * (qjj - 1), 1 + 3 * qjj, sqds, smm1);
        details::SVD3_xrotg(&rt, &ztest, &sqds, &smm1);
        b_s[qjj - 1] = rt;
        ztest = e[qjj - 1];
        rt = ztest * sqds + smm1 * b_s[qjj];
        b_s[qjj] = ztest * -smm1 + sqds * b_s[qjj];
        ztest = smm1 * e[qjj];
        e[qjj] *= sqds;
        details::SVD3_xrot(U, 1 + 3 * (qjj - 1), 1 + 3 * qjj, sqds, smm1);
      }

      e[m] = rt;
      qq++;
      break;

     default:
      if (b_s[d] < 0.0) {
        b_s[d] = -b_s[d];
        details::SVD3_xscal(-1.0, V, 1 + 3 * d);
      }

      qq = d + 1;
      while ((d + 1 < 3) && (b_s[d] < b_s[qq])) {
        rt = b_s[d];
        b_s[d] = b_s[qq];
        b_s[qq] = rt;
        details::SVD3_xswap(V, 1 + 3 * d, 1 + 3 * (d + 1));
        details::SVD3_xswap(U, 1 + 3 * d, 1 + 3 * (d + 1));
        d = qq;
        qq++;
      }

      qq = 0;
      m--;
      break;
    }
  }
}


} /* namespace: math */


} /* namespace: mpsv */

