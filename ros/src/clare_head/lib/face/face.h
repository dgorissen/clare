#ifndef FACE_H
#define FACE_H

#include "LedControl_SW_SPI.h"
#include "ledmasks.h"

class Face {
    private:
        LedControl_SW_SPI flc;
        int fleftEyeIdx = -1;
        int frightEyeIdx = -1;
        int fleftMouthIdx = -1;
        int frightMouthIdx = -1;

        const int DATA_IN_PIN = 2;
	    const int CS_PIN = 3;
	    const int CLK_PIN = 4;
	    const int NUM_DEVICES = 4;

        void setTuple(const frameType, const frameType, const int i, const int j);
        void setEyes(const frameType, const frameType);
        void setEyeExpression(const frameType expression[], const int size);
        void setMouth(const frameType, const frameType);
        void setMouthExpression(const frameType expression[], const int size);

    public:
        static const char * expressions;

        Face(int le, int re, int lm, int rm);
        void reset();
        void blink();
        void sadEyes();
        void angryEyes();
        void ouchEyes();
        void smileEyes();
        void lookLeft();
        void lookRight();
        void lookUp();
        void lookDown();
        void ughMouth();
        void smileMouth();
        void bigSmileMouth();
        void sadMouth();
        void sillyMouth();
        void kissMouth();
        void surpriseMouth();
        void crossEyes();
        void neutralEyes();
        void mmmMouth();
        void scepticalEyes();
        void neutralMouth();
        void slitEyes();
        void vampireMouth();
        
        void happyBlink();
        void happy();
        void bigHappy();
        void sad();
        void angry();
        void silly();
        void surprised();
        void ugh();
        void confused();
        void kiss();
        void mmm();
        void sceptical();
        void ohDear();
        void noExpression();
        void vampire();

        void loopExpressions(const int wait);
        void setExpression(const char* ex);

        static const char** listExpressions();

};

#endif