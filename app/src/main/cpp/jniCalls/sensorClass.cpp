#include <jni.h>
#include "simpleARClass.h"
#include "myJNIHelper.h"

#ifdef __cplusplus
extern "C" {
#endif

extern SimpleARClass *gSimpleARObject;
JNIEXPORT void JNICALL
Java_com_example_arwall_SensorClass_SendGravityToNative(JNIEnv *env,
                                                                         jobject instance,
                                                                         jfloat gravityX,
                                                                         jfloat gravityY,
                                                                         jfloat gravityZ) {

    if(gSimpleARObject == NULL){
        return;
    }
    gSimpleARObject->UpdateGravity(gravityX, gravityY, gravityZ);

}

#ifdef __cplusplus
}
#endif
