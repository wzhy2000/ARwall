#include <jni.h>
#include "simpleARClass.h"

#ifdef __cplusplus
extern "C" {
#endif

extern SimpleARClass *gSimpleARObject;

JNIEXPORT void JNICALL
Java_com_example_arwall_MyGLRenderer_DrawFrameNative(JNIEnv *env,
                                                                      jobject instance) {

    if (gSimpleARObject == NULL) {
        return;
    }
    gSimpleARObject->Render();

}

JNIEXPORT void JNICALL
Java_com_example_arwall_MyGLRenderer_SurfaceCreatedNative(JNIEnv *env,
                                                                           jobject instance) {

    if (gSimpleARObject == NULL) {
        return;
    }
    gSimpleARObject->PerformGLInits();

}

JNIEXPORT void JNICALL
Java_com_example_arwall_MyGLRenderer_SurfaceChangedNative(JNIEnv *env,
                                                                           jobject instance,
                                                                           jint width,
                                                                           jint height) {

    if (gSimpleARObject == NULL) {
        return;
    }
    gSimpleARObject->SetViewport(width, height);

}

#ifdef __cplusplus
}
#endif

