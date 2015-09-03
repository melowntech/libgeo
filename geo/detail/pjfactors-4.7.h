/******************************************************************************
 * Copied from src/projects.h. We need pj_factors function so we grabbed the
 * interface here.
 *
 * Original licence comes here:
 ******************************************************************************
 * Project:  PROJ.4
 * Purpose:  Primary (private) include file for PROJ.4 library.
 * Author:   Gerald Evenden
 *
 ******************************************************************************
 * Copyright (c) 2000, Frank Warmerdam
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *****************************************************************************/

#ifndef geo_detail_pjfactors_4_7_h_included_
#define geo_detail_pjfactors_4_7_h_included_

#include <proj_api.h>

#ifdef __cplusplus
extern "C" {
#endif

struct DERIVS {
    double x_l, x_p; /* derivatives of x for lambda-phi */
    double y_l, y_p; /* derivatives of y for lambda-phi */
};

struct FACTORS {
    struct DERIVS der;
    double h, k;    /* meridinal, parallel scales */
    double omega, thetap;   /* angular distortion, theta prime */
    double conv;    /* convergence */
    double s;       /* areal scale factor */
    double a, b; /* max-min scale error */
    int code;       /* info as to analytics, see following */
};
#define IS_ANAL_XL_YL 01    /* derivatives of lon analytic */
#define IS_ANAL_XP_YP 02    /* derivatives of lat analytic */
#define IS_ANAL_HK  04      /* h and k analytic */
#define IS_ANAL_CONV 010    /* convergence analytic */

int pj_factors(projLP, projPJ, double, struct FACTORS *);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // geo_detail_pjfactors_4_7_h_included_
