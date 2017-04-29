/* DO NOT EDIT THIS FILE - it is machine generated */
#include <jni.h>
/* Header for class edu_wlu_cs_levy_breezyslam_algorithms_RMHCSLAM */

#ifndef _Included_edu_wlu_cs_levy_breezyslam_algorithms_RMHCSLAM
#define _Included_edu_wlu_cs_levy_breezyslam_algorithms_RMHCSLAM
#ifdef __cplusplus
extern "C" {
#endif
#undef edu_wlu_cs_levy_breezyslam_algorithms_RMHCSLAM_DEFAULT_SIGMA_XY_MM
#define edu_wlu_cs_levy_breezyslam_algorithms_RMHCSLAM_DEFAULT_SIGMA_XY_MM 75.0
#undef edu_wlu_cs_levy_breezyslam_algorithms_RMHCSLAM_DEFAULT_SIGMA_THETA_DEGREES
#define edu_wlu_cs_levy_breezyslam_algorithms_RMHCSLAM_DEFAULT_SIGMA_THETA_DEGREES 15.0
#undef edu_wlu_cs_levy_breezyslam_algorithms_RMHCSLAM_DEFAULT_MAX_SEARCH_ITER
#define edu_wlu_cs_levy_breezyslam_algorithms_RMHCSLAM_DEFAULT_MAX_SEARCH_ITER 1000L
/*
 * Class:     edu_wlu_cs_levy_breezyslam_algorithms_RMHCSLAM
 * Method:    init
 * Signature: (I)V
 */
JNIEXPORT void JNICALL Java_edu_wlu_cs_levy_breezyslam_algorithms_RMHCSLAM_init
  (JNIEnv *, jobject, jint);

/*
 * Class:     edu_wlu_cs_levy_breezyslam_algorithms_RMHCSLAM
 * Method:    positionSearch
 * Signature: (Ledu/wlu/cs/levy/breezyslam/components/Position;Ledu/wlu/cs/levy/breezyslam/components/Map;Ledu/wlu/cs/levy/breezyslam/components/Scan;DDI)Ljava/lang/Object;
 */
JNIEXPORT jobject JNICALL Java_edu_wlu_cs_levy_breezyslam_algorithms_RMHCSLAM_positionSearch
  (JNIEnv *, jobject, jobject, jobject, jobject, jdouble, jdouble, jint);

#ifdef __cplusplus
}
#endif
#endif
