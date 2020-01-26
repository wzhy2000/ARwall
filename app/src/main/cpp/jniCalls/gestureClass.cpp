#include <jni.h>
#include "simpleARClass.h"

#ifdef __cplusplus
extern "C" {
#endif

extern SimpleARClass *gSimpleARObject;

JNIEXPORT void JNICALL
Java_com_example_arwall_GestureClass_DoubleTapNative(JNIEnv *env, jobject instance) {

    if (gSimpleARObject == NULL) {
        return;
    }
    //gSimpleARObject->DoubleTapAction();

}

#ifdef __cplusplus
}
#endif
