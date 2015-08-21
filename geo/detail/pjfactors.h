#ifndef geo_detail_pjfactors_h_included_
#define geo_detail_pjfactors_h_included_

#ifdef __cplusplus
extern "C" {
#endif

struct geo_detail_pj_factors {
    double meridionalScale;
    double parallelScale;
    double angularDistortion;
    double thetaPrime;
    double convergence;
    double arealScaleFactor;
    double minScaleError;
    double maxScaleError;

    double lambdaDx;
    double phiDx;
    double lambdaDy;
    double phiDy;
};

int geo_detail_pj_factors(double x, double y, void *pj
                          , struct geo_detail_pj_factors *factors);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // geo_detail_pjfactors_h_included_
