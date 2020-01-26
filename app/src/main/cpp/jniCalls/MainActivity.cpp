#include <jni.h>
#include "simpleARClass.h"
#include "myJNIHelper.h"

#ifdef __cplusplus
extern "C" {
#endif

// global pointer is used in JNI calls to reference to same object of type Cube
SimpleARClass *gSimpleARObject =NULL;

// global pointer to instance of MyJNIHelper that is used to read from assets
MyJNIHelper * gHelperObject=NULL;

/**
 * Create the persistent native object and also initialize the single helper object
 */
JNIEXPORT void JNICALL
Java_com_example_arwall_MainActivity_CreateObjectNative(JNIEnv *env,
                                                                                     jobject instance,
                                                                                     jobject assetManager,
                                                                                     jstring pathToInternalDir) {

    gHelperObject = new MyJNIHelper(env, instance, assetManager, pathToInternalDir);
    gSimpleARObject = new SimpleARClass();
}

JNIEXPORT void JNICALL
Java_com_example_arwall_MainActivity_DeleteObjectNative(JNIEnv *env,
                                                                                     jobject instance) {
    if (gSimpleARObject != NULL) {
        delete gSimpleARObject;
    }
    gSimpleARObject = NULL;

    if (gHelperObject != NULL) {
        delete gHelperObject;
    }
    gHelperObject = NULL;
}

JNIEXPORT void JNICALL
Java_com_example_arwall_MainActivity_SetCameraParamsNative(
        JNIEnv *env, jobject instance, jint previewWidth, jint previewHeight, jfloat cameraFOV) {

    if (gSimpleARObject == NULL) {
        return;
    }
    gSimpleARObject->SetCameraParams((int) previewWidth, (int) previewHeight, (float) cameraFOV);
}

#ifdef __cplusplus
}
#endif
