/* Copyright 2015 The TensorFlow Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

// This file binds the native image utility code to the Java class
// which exposes them.

#include <jni.h>
#include <stdio.h>
#include <stdlib.h>

#include "tensorflow/examples/android/jni/rgb2yuv.h"
#include "tensorflow/examples/android/jni/yuv2rgb.h"

#define IMAGEUTILS_METHOD(METHOD_NAME) \
  Java_org_tensorflow_demo_env_ImageUtils_##METHOD_NAME  // NOLINT

#ifdef __cplusplus
extern "C" {
#endif

JNIEXPORT void JNICALL
IMAGEUTILS_METHOD(convertYUV420SPToARGB8888)(
    JNIEnv* env, jclass clazz, jbyteArray input, jintArray output,
    jint width, jint height, jboolean halfSize);

JNIEXPORT void JNICALL IMAGEUTILS_METHOD(convertYUV420ToARGB8888)(
    JNIEnv* env, jclass clazz, jbyteArray y, jbyteArray u, jbyteArray v,
    jintArray output, jint width, jint height, jint y_row_stride,
    jint uv_row_stride, jint uv_pixel_stride, jboolean halfSize);

JNIEXPORT void JNICALL IMAGEUTILS_METHOD(convertYUV420SPToRGB565)(
    JNIEnv* env, jclass clazz, jbyteArray input, jbyteArray output, jint width,
    jint height);

JNIEXPORT void JNICALL
IMAGEUTILS_METHOD(convertARGB8888ToYUV420SP)(
    JNIEnv* env, jclass clazz, jintArray input, jbyteArray output,
    jint width, jint height);

JNIEXPORT void JNICALL
IMAGEUTILS_METHOD(convertRGB565ToYUV420SP)(
    JNIEnv* env, jclass clazz, jbyteArray input, jbyteArray output,
    jint width, jint height);

#ifdef __cplusplus
}
#endif

JNIEXPORT void JNICALL
IMAGEUTILS_METHOD(convertYUV420SPToARGB8888)(
    JNIEnv* env, jclass clazz, jbyteArray input, jintArray output,
    jint width, jint height, jboolean halfSize) {
  jboolean inputCopy = JNI_FALSE;
  jbyte* const i = env->GetByteArrayElements(input, &inputCopy);

  jboolean outputCopy = JNI_FALSE;
  jint* const o = env->GetIntArrayElements(output, &outputCopy);

  if (halfSize) {
    ConvertYUV420SPToARGB8888HalfSize(reinterpret_cast<uint8_t*>(i),
                                      reinterpret_cast<uint32_t*>(o), width,
                                      height);
  } else {
    ConvertYUV420SPToARGB8888(reinterpret_cast<uint8_t*>(i),
                              reinterpret_cast<uint8_t*>(i) + width * height,
                              reinterpret_cast<uint32_t*>(o), width, height);
  }

  env->ReleaseByteArrayElements(input, i, JNI_ABORT);
  env->ReleaseIntArrayElements(output, o, 0);
}

JNIEXPORT void JNICALL IMAGEUTILS_METHOD(convertYUV420ToARGB8888)(
    JNIEnv* env, jclass clazz, jbyteArray y, jbyteArray u, jbyteArray v,
    jintArray output, jint width, jint height, jint y_row_stride,
    jint uv_row_stride, jint uv_pixel_stride, jboolean halfSize) {
  jboolean inputCopy = JNI_FALSE;
  jbyte* const y_buff = env->GetByteArrayElements(y, &inputCopy);
  jboolean outputCopy = JNI_FALSE;
  jint* const o = env->GetIntArrayElements(output, &outputCopy);

  if (halfSize) {
    ConvertYUV420SPToARGB8888HalfSize(reinterpret_cast<uint8_t*>(y_buff),
                                      reinterpret_cast<uint32_t*>(o), width,
                                      height);
  } else {
    jbyte* const u_buff = env->GetByteArrayElements(u, &inputCopy);
    jbyte* const v_buff = env->GetByteArrayElements(v, &inputCopy);

    ConvertYUV420ToARGB8888(
        reinterpret_cast<uint8_t*>(y_buff), reinterpret_cast<uint8_t*>(u_buff),
        reinterpret_cast<uint8_t*>(v_buff), reinterpret_cast<uint32_t*>(o),
        width, height, y_row_stride, uv_row_stride, uv_pixel_stride);

    env->ReleaseByteArrayElements(u, u_buff, JNI_ABORT);
    env->ReleaseByteArrayElements(v, v_buff, JNI_ABORT);
  }

  env->ReleaseByteArrayElements(y, y_buff, JNI_ABORT);
  env->ReleaseIntArrayElements(output, o, 0);
}

JNIEXPORT void JNICALL IMAGEUTILS_METHOD(convertYUV420SPToRGB565)(
    JNIEnv* env, jclass clazz, jbyteArray input, jbyteArray output, jint width,
    jint height) {
  jboolean inputCopy = JNI_FALSE;
  jbyte* const i = env->GetByteArrayElements(input, &inputCopy);

  jboolean outputCopy = JNI_FALSE;
  jbyte* const o = env->GetByteArrayElements(output, &outputCopy);

  ConvertYUV420SPToRGB565(reinterpret_cast<uint8_t*>(i),
                          reinterpret_cast<uint16_t*>(o), width, height);

  env->ReleaseByteArrayElements(input, i, JNI_ABORT);
  env->ReleaseByteArrayElements(output, o, 0);
}

JNIEXPORT void JNICALL
IMAGEUTILS_METHOD(convertARGB8888ToYUV420SP)(
    JNIEnv* env, jclass clazz, jintArray input, jbyteArray output,
    jint width, jint height) {
  jboolean inputCopy = JNI_FALSE;
  jint* const i = env->GetIntArrayElements(input, &inputCopy);

  jboolean outputCopy = JNI_FALSE;
  jbyte* const o = env->GetByteArrayElements(output, &outputCopy);

  ConvertARGB8888ToYUV420SP(reinterpret_cast<uint32_t*>(i),
                            reinterpret_cast<uint8_t*>(o), width, height);

  env->ReleaseIntArrayElements(input, i, JNI_ABORT);
  env->ReleaseByteArrayElements(output, o, 0);
}

JNIEXPORT void JNICALL
IMAGEUTILS_METHOD(convertRGB565ToYUV420SP)(
    JNIEnv* env, jclass clazz, jbyteArray input, jbyteArray output,
    jint width, jint height) {
  jboolean inputCopy = JNI_FALSE;
  jbyte* const i = env->GetByteArrayElements(input, &inputCopy);

  jboolean outputCopy = JNI_FALSE;
  jbyte* const o = env->GetByteArrayElements(output, &outputCopy);

  ConvertRGB565ToYUV420SP(reinterpret_cast<uint16_t*>(i),
                          reinterpret_cast<uint8_t*>(o), width, height);

  env->ReleaseByteArrayElements(input, i, JNI_ABORT);
  env->ReleaseByteArrayElements(output, o, 0);
}

extern "C"
JNIEXPORT void JNICALL
Java_utility_ImageUtils_convertRGB565ToYUV420SP(JNIEnv *env, jclass type, jbyteArray input_,
                                                jbyteArray output_, jint width, jint height) {
    jbyte *input = env->GetByteArrayElements(input_, NULL);
    jbyte *output = env->GetByteArrayElements(output_, NULL);

    // TODO

    env->ReleaseByteArrayElements(input_, input, 0);
    env->ReleaseByteArrayElements(output_, output, 0);
}extern "C"
JNIEXPORT void JNICALL
Java_utility_ImageUtils_convertARGB8888ToYUV420SP(JNIEnv *env, jclass type, jintArray input_,
                                                  jbyteArray output_, jint width, jint height) {
    jint *input = env->GetIntArrayElements(input_, NULL);
    jbyte *output = env->GetByteArrayElements(output_, NULL);

    // TODO

    env->ReleaseIntArrayElements(input_, input, 0);
    env->ReleaseByteArrayElements(output_, output, 0);
}extern "C"
JNIEXPORT void JNICALL
Java_utility_ImageUtils_convertYUV420SPToRGB565(JNIEnv *env, jclass type, jbyteArray input_,
                                                jbyteArray output_, jint width, jint height) {
    jbyte *input = env->GetByteArrayElements(input_, NULL);
    jbyte *output = env->GetByteArrayElements(output_, NULL);

    // TODO

    env->ReleaseByteArrayElements(input_, input, 0);
    env->ReleaseByteArrayElements(output_, output, 0);
}extern "C"
JNIEXPORT void JNICALL
Java_utility_ImageUtils_convertYUV420ToARGB8888__Lbyte_3_093_2Lbyte_3_093_2Lbyte_3_093_2Lint_3_093_2IIIIIZ(
        JNIEnv *env, jclass type, jbyteArray y_, jbyteArray u_, jbyteArray v_, jintArray output_,
        jint width, jint height, jint yRowStride, jint uvRowStride, jint uvPixelStride,
        jboolean halfSize) {
    jbyte *y = env->GetByteArrayElements(y_, NULL);
    jbyte *u = env->GetByteArrayElements(u_, NULL);
    jbyte *v = env->GetByteArrayElements(v_, NULL);
    jint *output = env->GetIntArrayElements(output_, NULL);

    // TODO

    env->ReleaseByteArrayElements(y_, y, 0);
    env->ReleaseByteArrayElements(u_, u, 0);
    env->ReleaseByteArrayElements(v_, v, 0);
    env->ReleaseIntArrayElements(output_, output, 0);
}extern "C"
JNIEXPORT void JNICALL
Java_utility_ImageUtils_convertYUV420SPToARGB8888__Lbyte_3_093_2Lint_3_093_2IIZ(JNIEnv *env,
                                                                                jclass type,
                                                                                jbyteArray input_,
                                                                                jintArray output_,
                                                                                jint width,
                                                                                jint height,
                                                                                jboolean halfSize) {
    jbyte *input = env->GetByteArrayElements(input_, NULL);
    jint *output = env->GetIntArrayElements(output_, NULL);

    // TODO

    env->ReleaseByteArrayElements(input_, input, 0);
    env->ReleaseIntArrayElements(output_, output, 0);
}extern "C"
JNIEXPORT void JNICALL
Java_tracking_ObjectTracker_initNative(JNIEnv *env, jobject instance, jint imageWidth,
                                       jint imageHeight, jboolean alwaysTrack) {

    // TODO

}extern "C"
JNIEXPORT void JNICALL
Java_tracking_ObjectTracker_registerNewObjectWithAppearanceNative(JNIEnv *env, jobject instance,
                                                                  jstring objectId_, jfloat x1,
                                                                  jfloat y1, jfloat x2, jfloat y2,
                                                                  jbyteArray data_) {
    const char *objectId = env->GetStringUTFChars(objectId_, 0);
    jbyte *data = env->GetByteArrayElements(data_, NULL);

    // TODO

    env->ReleaseStringUTFChars(objectId_, objectId);
    env->ReleaseByteArrayElements(data_, data, 0);
}extern "C"
JNIEXPORT void JNICALL
Java_tracking_ObjectTracker_setPreviousPositionNative(JNIEnv *env, jobject instance,
                                                      jstring objectId_, jfloat x1, jfloat y1,
                                                      jfloat x2, jfloat y2, jlong timestamp) {
    const char *objectId = env->GetStringUTFChars(objectId_, 0);

    // TODO

    env->ReleaseStringUTFChars(objectId_, objectId);
}extern "C"
JNIEXPORT void JNICALL
Java_tracking_ObjectTracker_setCurrentPositionNative(JNIEnv *env, jobject instance,
                                                     jstring objectId_, jfloat x1, jfloat y1,
                                                     jfloat x2, jfloat y2) {
    const char *objectId = env->GetStringUTFChars(objectId_, 0);

    // TODO

    env->ReleaseStringUTFChars(objectId_, objectId);
}extern "C"
JNIEXPORT void JNICALL
Java_tracking_ObjectTracker_forgetNative(JNIEnv *env, jobject instance, jstring key_) {
    const char *key = env->GetStringUTFChars(key_, 0);

    // TODO

    env->ReleaseStringUTFChars(key_, key);
}extern "C"
JNIEXPORT jstring JNICALL
Java_tracking_ObjectTracker_getModelIdNative(JNIEnv *env, jobject instance, jstring key_) {
    const char *key = env->GetStringUTFChars(key_, 0);

    // TODO

    env->ReleaseStringUTFChars(key_, key);

    return env->NewStringUTF(key);
}extern "C"
JNIEXPORT jboolean JNICALL
Java_tracking_ObjectTracker_haveObject(JNIEnv *env, jobject instance, jstring key_) {
    const char *key = env->GetStringUTFChars(key_, 0);

    // TODO

    env->ReleaseStringUTFChars(key_, key);
}extern "C"
JNIEXPORT jfloat JNICALL
Java_tracking_ObjectTracker_getCurrentCorrelation(JNIEnv *env, jobject instance, jstring key_) {
    const char *key = env->GetStringUTFChars(key_, 0);

    // TODO

    env->ReleaseStringUTFChars(key_, key);
}extern "C"
JNIEXPORT jboolean JNICALL
Java_tracking_ObjectTracker_isObjectVisible(JNIEnv *env, jobject instance, jstring key_) {
    const char *key = env->GetStringUTFChars(key_, 0);

    // TODO

    env->ReleaseStringUTFChars(key_, key);
}extern "C"
JNIEXPORT jfloat JNICALL
Java_tracking_ObjectTracker_getMatchScore(JNIEnv *env, jobject instance, jstring key_) {
    const char *key = env->GetStringUTFChars(key_, 0);

    // TODO

    env->ReleaseStringUTFChars(key_, key);
}extern "C"
JNIEXPORT void JNICALL
Java_tracking_ObjectTracker_getTrackedPositionNative(JNIEnv *env, jobject instance, jstring key_,
                                                     jfloatArray points_) {
    const char *key = env->GetStringUTFChars(key_, 0);
    jfloat *points = env->GetFloatArrayElements(points_, NULL);

    // TODO

    env->ReleaseStringUTFChars(key_, key);
    env->ReleaseFloatArrayElements(points_, points, 0);
}extern "C"
JNIEXPORT void JNICALL
Java_tracking_ObjectTracker_nextFrameNative(JNIEnv *env, jobject instance, jbyteArray frameData_,
                                            jbyteArray uvData_, jlong timestamp,
                                            jfloatArray frameAlignMatrix_) {
    jbyte *frameData = env->GetByteArrayElements(frameData_, NULL);
    jbyte *uvData = env->GetByteArrayElements(uvData_, NULL);
    jfloat *frameAlignMatrix = env->GetFloatArrayElements(frameAlignMatrix_, NULL);

    // TODO

    env->ReleaseByteArrayElements(frameData_, frameData, 0);
    env->ReleaseByteArrayElements(uvData_, uvData, 0);
    env->ReleaseFloatArrayElements(frameAlignMatrix_, frameAlignMatrix, 0);
}extern "C"
JNIEXPORT void JNICALL
Java_tracking_ObjectTracker_getCurrentPositionNative(JNIEnv *env, jobject instance, jlong timestamp,
                                                     jfloat positionX1, jfloat positionY1,
                                                     jfloat positionX2, jfloat positionY2,
                                                     jfloatArray delta_) {
    jfloat *delta = env->GetFloatArrayElements(delta_, NULL);

    // TODO

    env->ReleaseFloatArrayElements(delta_, delta, 0);
}extern "C"
JNIEXPORT void JNICALL
Java_tracking_ObjectTracker_releaseMemoryNative(JNIEnv *env, jobject instance) {

    // TODO

}extern "C"
JNIEXPORT jbyteArray JNICALL
Java_tracking_ObjectTracker_getKeypointsPacked(JNIEnv *env, jobject instance, jfloat scaleFactor) {

    // TODO

}extern "C"
JNIEXPORT jfloatArray JNICALL
Java_tracking_ObjectTracker_getKeypointsNative(JNIEnv *env, jobject instance,
                                               jboolean onlyReturnCorrespondingKeypoints) {

    // TODO

}extern "C"
JNIEXPORT void JNICALL
Java_tracking_ObjectTracker_downsampleImageNative(JNIEnv *env, jclass type, jint width, jint height,
                                                  jint rowStride, jbyteArray input_, jint factor,
                                                  jbyteArray output_) {
    jbyte *input = env->GetByteArrayElements(input_, NULL);
    jbyte *output = env->GetByteArrayElements(output_, NULL);

    // TODO

    env->ReleaseByteArrayElements(input_, input, 0);
    env->ReleaseByteArrayElements(output_, output, 0);
}extern "C"
JNIEXPORT void JNICALL
Java_tracking_ObjectTracker_drawNative(JNIEnv *env, jobject instance, jint viewWidth,
                                       jint viewHeight, jfloatArray frameToCanvas_) {
    jfloat *frameToCanvas = env->GetFloatArrayElements(frameToCanvas_, NULL);

    // TODO

    env->ReleaseFloatArrayElements(frameToCanvas_, frameToCanvas, 0);
}