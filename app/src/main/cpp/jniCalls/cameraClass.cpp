#include <jni.h>
#include "simpleARClass.h"

#ifdef __cplusplus
extern "C" {
#endif

extern SimpleARClass *gSimpleARObject;

/**
 * Create a RGB image from camera's preview data and send it to native class
 */
JNIEXPORT void JNICALL
Java_com_example_arwall_CameraClass_SendCamImageToNative(JNIEnv *env,
                                                                              jobject instance,
                                                                              jbyteArray data_,
                                                                              jint previewHeight,
                                                                              jint previewWidth) {
    if(gSimpleARObject == NULL) {
        return;
    }

    jbyte *data = env->GetByteArrayElements(data_, NULL);

    // Android returns data in NV21 format, convert it to RGB
    cv::Mat cameraNV21Image(previewHeight * 1.5, previewWidth, CV_8UC1, data);
    cv::Mat cameraRGBImage;
    cv::cvtColor(cameraNV21Image, cameraRGBImage, CV_YUV2RGB_NV21, 3);

    gSimpleARObject->ProcessCameraImage(cameraRGBImage);

    env->ReleaseByteArrayElements(data_, data, 0);
}

#ifdef __cplusplus
}
#endif

