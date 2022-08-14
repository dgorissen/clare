#include "face.h"
#include "utils.h"
#include "facemasks.h"
#include "Arduino.h"

const int animTime = 50;


Face::Face(int le, int re, int lm, int rm) {
	fleftEyeIdx = le;
	frightEyeIdx = re;
	fleftMouthIdx = lm;
	frightMouthIdx = rm;

	flc = LedControl_SW_SPI();
	flc.begin(DATA_IN_PIN, CLK_PIN, CS_PIN, NUM_DEVICES);
}

void Face::reset() {
  for (int x = 0; x < NUM_DEVICES; x++) {
    // The MAX72XX is in power-saving mode on startup
	flc.shutdown(x, false);
    // Set the brightness to default value
	flc.setIntensity(x, 1);
    flc.clearDisplay(x);
  }
}

void Face::setTuple(const frameType fi, const frameType fj, const int i,
                    const int j) {
  for (int row = 0; row < 8; row++) {
    flc.setRow(i, row, binaryArray[fi.frameCount].array1[row]);
    flc.setRow(j, row, binaryArray[fj.frameCount].array1[row]);
    flc.setIntensity(i, fi.frameLuminance);
    flc.setIntensity(j, fj.frameLuminance);
  }
}

void Face::setEyes(const frameType leftEye, const frameType rightEye) {
  setTuple(leftEye, rightEye, fleftEyeIdx, frightEyeIdx);
}

void Face::setMouth(const frameType left, const frameType right) {
  setTuple(left, right, fleftMouthIdx, frightMouthIdx);
}

void Face::setEyeExpression(const frameType expression[], const int size) {
  for (int a = 0; a < size; a = a + 2) {
    setEyes(expression[a], expression[a + 1]);
    delayMillis(expression[a].frameDelay);
  }
}

void Face::setMouthExpression(const frameType expression[], const int size) {
  for (int a = 0; a < size; a = a + 2) {
    setMouth(expression[a], expression[a + 1]);
    delayMillis(expression[a].frameDelay);
  }
}

void Face::blink() {
  frameType neutralBlink[] = {
      {LeftNeutral1, 2000, 1},     {RightNeutral1, 2000, 1},
      {LeftNeutral2, animTime, 1}, {RightNeutral2, animTime, 1},
      {LeftNeutral3, animTime, 1}, {RightNeutral3, animTime, 1},
      {LeftNeutral4, animTime, 1}, {RightNeutral4, animTime, 1},
      {LeftNeutral3, animTime, 1}, {RightNeutral3, animTime, 1},
      {LeftNeutral2, animTime, 1}, {RightNeutral2, animTime, 1},
      {LeftNeutral1, 2000, 1},     {RightNeutral1, 2000, 1}
  };

  setEyeExpression(neutralBlink, arraySize(neutralBlink));
}

void Face::sadEyes() {
  frameType sadEyes[] = {
      {LeftNeutral1, animTime * 2, 1},
      {RightNeutral1, animTime * 2, 1},
      {LeftSad1, animTime, 1},
      {RightSad1, animTime, 1},
      {LeftSad2, animTime, 1},
      {RightSad2, animTime, 1},
      {LeftSad3, animTime, 1},
      {RightSad2, animTime, 1},
      {LeftSad4, 0, 1},
      {RightSad4, 0, 1},
  };
  setEyeExpression(sadEyes, arraySize(sadEyes));
}

void Face::angryEyes() {
  frameType angryEyes[] = {
      {LeftNeutral1, animTime * 2, 1}, {RightNeutral1, animTime * 2, 1},
      {LeftAngry1, animTime, 1},       {RightAngry1, animTime, 1},
      {LeftAngry2, animTime, 1},       {RightAngry2, animTime, 1},
      {LeftAngry3, animTime, 1},       {RightAngry3, animTime, 1},
      {LeftAngry4, animTime, 1},       {RightAngry4, animTime, 1},
      {LeftAngry5, 0, 1},           {RightAngry5, 0, 1},
  };
  setEyeExpression(angryEyes, arraySize(angryEyes));
}

void Face::lookRight() {
  frameType lookRight[] = {
      {LeftNeutral1, animTime * 2, 1},
      {RightNeutral1, animTime * 2, 1},
      {LeftRight1, animTime, 1},
      {LeftRight1, animTime, 1},
      {LeftRight2, animTime, 1},
      {LeftRight2, animTime, 1},
      {LeftRight3, 500, 1},
      {LeftRight3, 500, 1},
      {LeftRight2, animTime, 1},
      {LeftRight2, animTime, 1},
      {LeftRight1, animTime, 1},
      {LeftRight1, animTime, 1},
      {LeftNeutral1, animTime * 2, 1},
      {RightNeutral1, animTime * 2, 1},
  };
  setEyeExpression(lookRight, arraySize(lookRight));
}

void Face::lookLeft() {
  frameType lookLeft[] = {
      {LeftNeutral1, animTime * 2, 1},
      {RightNeutral1, animTime * 2, 1},
      {RightLeft1, animTime, 1},
      {RightLeft1, animTime, 1},
      {RightLeft2, animTime, 1},
      {RightLeft2, animTime, 1},
      {RightLeft3, 500, 1},
      {RightLeft3, 500, 1},
      {RightLeft2, animTime, 1},
      {RightLeft2, animTime, 1},
      {RightLeft1, animTime, 1},
      {RightLeft1, animTime, 1},
      {LeftNeutral1, animTime * 2, 1},
      {RightNeutral1, animTime * 2, 1},
  };
  setEyeExpression(lookLeft, arraySize(lookLeft));
}

void Face::crossEyes() {
  frameType lookLeft[] = {
      {LeftNeutral1, animTime * 2, 1},
      {RightNeutral1, animTime * 2, 1},
      {LeftRight1, animTime, 1},
      {RightLeft1, animTime, 1},
      {LeftRight2, animTime, 1},
      {RightLeft2, animTime, 1},
      {LeftRight3, 0, 1},
      {RightLeft3, 0, 1},
  };
  setEyeExpression(lookLeft, arraySize(lookLeft));
}

void Face::lookUp() {
  frameType lookUp[] = {
      {LeftNeutral1, animTime * 2, 1},
      {RightNeutral1, animTime * 2, 1},
      {LeftUp1, animTime, 1},
      {LeftUp1, animTime, 1},
      {LeftUp2, animTime, 1},
      {LeftUp2, animTime, 1},
      {LeftUp3, 500, 1},
      {LeftUp3, 500, 1},
      {LeftUp2, animTime, 1},
      {LeftUp2, animTime, 1},
      {LeftUp1, animTime, 1},
      {LeftUp1, animTime, 1},
      {LeftNeutral1, animTime * 2, 1},
      {RightNeutral1, animTime * 2, 1},
  };
  setEyeExpression(lookUp, arraySize(lookUp));
}

void Face::lookDown() {
  frameType lookDown[] = {
      {LeftNeutral1, animTime * 2, 1},
      {RightNeutral1, animTime * 2, 1},
      {LeftDown1, animTime, 1},
      {LeftDown1, animTime, 1},
      {LeftDown2, animTime, 1},
      {LeftDown2, animTime, 1},
      {LeftDown3, 500, 1},
      {LeftDown3, 500, 1},
      {LeftDown2, animTime, 1},
      {LeftDown2, animTime, 1},
      {LeftDown1, animTime, 1},
      {LeftDown1, animTime, 1},
      {LeftNeutral1, animTime * 2, 1},
      {RightNeutral1, animTime * 2, 1},
  };
  setEyeExpression(lookDown, arraySize(lookDown));
}

void Face::neutralEyes() {
  frameType neutralEyes[] = {
      {LeftNeutral1, 0, 1},
      {RightNeutral1, 0, 1},
  };
  setEyeExpression(neutralEyes, arraySize(neutralEyes));
}

void Face::ouchEyes() {
  frameType ouchEyes[] = {
      {LeftChevron, 0, 1},
      {RightChevron, 0, 1},
  };
  setEyeExpression(ouchEyes, arraySize(ouchEyes));
}

void Face::smileEyes() {
  frameType smileEyes[] = {
      {LeftSmile, 0, 1},
      {LeftSmile, 0, 1},
  };
  setEyeExpression(smileEyes, arraySize(smileEyes));
}

void Face::smileMouth() {
  frameType smile[] = {
      {LeftMouthSmile, 0, 1},
      {RightMouthSmile, 0, 1},
  };
  setMouthExpression(smile, arraySize(smile));
}

void Face::bigSmileMouth() {
  frameType smile[] = {
      {LeftMouthBigSmile, 0, 1},
      {RightMouthBigSmile, 0, 1},
  };
  setMouthExpression(smile, arraySize(smile));
}

void Face::sadMouth() {
  frameType sad[] = {
      {LeftMouthSad, 0, 1},
      {RightMouthSad, 0, 1},
  };
  setMouthExpression(sad, arraySize(sad));
}

void Face::kissMouth() {
  frameType kiss[] = {
      {LeftMouthX, 0, 1},
      {RightMouthX, 0, 1},
  };
  setMouthExpression(kiss, arraySize(kiss));
}

void Face::sillyMouth() {
  frameType silly[] = {
      {LeftMouthSilly, 0, 1},
      {RightMouthSilly, 0, 1},
  };
  setMouthExpression(silly, arraySize(silly));
}

void Face::surpriseMouth() {
  frameType surprise[] = {
      {LeftMouthO, 0, 1},
      {RightMouthO, 0, 1},
  };
  setMouthExpression(surprise, arraySize(surprise));
}

void Face::ughMouth() {
  frameType ugh[] = {
      {LeftMouthSquiggle, 0, 1},
      {RightMouthSquiggle, 0, 1},
  };
  setMouthExpression(ugh, arraySize(ugh));
}

void Face::mmmMouth(){
  frameType mm[] = {
      {LeftMouthSlash, 0, 1},
      {RightMouthSlash, 0, 1},
  };
  setMouthExpression(mm, arraySize(mm));
}

void Face::scepticalEyes(){
  frameType eyes[] = {
      {LeftEyeSceptical, 0, 1},
      {RightEyeSceptical, 0, 1},
  };
  setEyeExpression(eyes, arraySize(eyes));
}

void Face::neutralMouth(){
  frameType mm[] = {
      {LeftMouthNeutral, 0, 1},
      {RightMouthNeutral, 0, 1},
  };
  setMouthExpression(mm, arraySize(mm));
}

void Face::slitEyes(){
  frameType eyes[] = {
      {LeftEyeSlit, 0, 1},
      {RightEyeSlit, 0, 1},
  };
  setEyeExpression(eyes, arraySize(eyes));
}

void Face::vampireMouth(){
  frameType mm[] = {
      {LeftMouthVamp, 0, 1},
      {RightMouthVamp, 0, 1},
  };
  setMouthExpression(mm, arraySize(mm));
}


void Face::happyBlink(){
    smileMouth();
    blink();
}

void Face::happy(){
    smileEyes();
    smileMouth();
}

void Face::bigHappy(){
    smileEyes();
    bigSmileMouth();
}

void Face::sad(){
    sadEyes();
    sadMouth();
}

void Face::angry(){
    angryEyes();
    sadMouth();
}

void Face::silly(){
    smileEyes();
    sillyMouth();
}

void Face::surprised(){
    neutralEyes();
    surpriseMouth();
}

void Face::ugh(){
    ouchEyes();
    ughMouth();
}

void Face::confused(){
    crossEyes();
    ughMouth();
}

void Face::kiss(){
    smileEyes();
    kissMouth();
}
void Face::mmm(){
    neutralEyes();
    mmmMouth();
}

void Face::sceptical(){
    scepticalEyes();
    neutralMouth();
}

void Face::ohDear(){
    lookUp();
    neutralMouth();
}

void Face::noExpression(){
    slitEyes();
    neutralMouth();
}

void Face::vampire(){
	angryEyes();
	vampireMouth();
}

void Face::loopExpressions(const int wait) {
  this->happy();
  delay(wait);
  this->happyBlink();
  delay(wait);
  this->bigHappy();
  delay(wait);
  this->sad();
  delay(wait);
  this->silly();
  delay(wait);
  this->angry();
  delay(wait);
  this->confused();
  delay(wait);
  this->ugh();
  delay(wait);
  this->surprised();
  delay(wait);
  this->kiss();
  delay(wait);
  this->mmm();
  delay(wait);
  this->sceptical();
  delay(wait);
  this->ohDear();
  delay(wait);
  this->noExpression();
  delay(wait);
  this->vampire();
  delay(wait);
}


const char * Face::expressions = "angry,angryeyes,bighappy,bigsmilemouth,blink,confused,crosseyes,happy,happyblink,kiss,kissmouth,lookdown,lookleft,lookright,lookup,mmm,mmmmouth,neutraleyes,neutralmouth,noexpression,ohdear,oucheyes,sad,sadeyes,sadmouth,sceptical,scepticaleyes,silly,sillymouth,sliteyes,smileeyes,smilemouth,surprisemouth,surprised,ugh,ughmouth,vampire,vampiremouth";

void Face::setExpression(const char* ex){
  if (stricmp(ex, "angry") == 0) {
          this->angry();
  } else if (stricmp(ex, "angryEyes") == 0) {
        this->angryEyes();
  } else if (stricmp(ex, "bigHappy") == 0) {
        this->bigHappy();
  } else if (stricmp(ex, "bigSmileMouth") == 0) {
        this->bigSmileMouth();
  } else if (stricmp(ex, "blink") == 0) {
        this->blink();
  } else if (stricmp(ex, "confused") == 0) {
        this->confused();
  } else if (stricmp(ex, "crossEyes") == 0) {
        this->crossEyes();
  } else if (stricmp(ex, "happy") == 0) {
        this->happy();
  } else if (stricmp(ex, "happyBlink") == 0) {
        this->happyBlink();
  } else if (stricmp(ex, "kiss") == 0) {
        this->kiss();
  } else if (stricmp(ex, "kissMouth") == 0) {
        this->kissMouth();
  } else if (stricmp(ex, "lookDown") == 0) {
        this->lookDown();
  } else if (stricmp(ex, "lookLeft") == 0) {
        this->lookLeft();
  } else if (stricmp(ex, "lookRight") == 0) {
        this->lookRight();
  } else if (stricmp(ex, "lookUp") == 0) {
        this->lookUp();
  } else if (stricmp(ex, "mmm") == 0) {
        this->mmm();
  } else if (stricmp(ex, "mmmMouth") == 0) {
        this->mmmMouth();
  } else if (stricmp(ex, "neutralEyes") == 0) {
        this->neutralEyes();
  } else if (stricmp(ex, "neutralMouth") == 0) {
        this->neutralMouth();
  } else if (stricmp(ex, "noExpression") == 0) {
        this->noExpression();
  } else if (stricmp(ex, "ohDear") == 0) {
        this->ohDear();
  } else if (stricmp(ex, "ouchEyes") == 0) {
        this->ouchEyes();
  } else if (stricmp(ex, "sad") == 0) {
        this->sad();
  } else if (stricmp(ex, "sadEyes") == 0) {
        this->sadEyes();
  } else if (stricmp(ex, "sadMouth") == 0) {
        this->sadMouth();
  } else if (stricmp(ex, "sceptical") == 0) {
        this->sceptical();
  } else if (stricmp(ex, "scepticalEyes") == 0) {
        this->scepticalEyes();
  } else if (stricmp(ex, "silly") == 0) {
        this->silly();
  } else if (stricmp(ex, "sillyMouth") == 0) {
        this->sillyMouth();
  } else if (stricmp(ex, "slitEyes") == 0) {
        this->slitEyes();
  } else if (stricmp(ex, "smileEyes") == 0) {
        this->smileEyes();
  } else if (stricmp(ex, "smileMouth") == 0) {
        this->smileMouth();
  } else if (stricmp(ex, "surpriseMouth") == 0) {
        this->surpriseMouth();
  } else if (stricmp(ex, "surprised") == 0) {
        this->surprised();
  } else if (stricmp(ex, "ugh") == 0) {
        this->ugh();
  } else if (stricmp(ex, "ughMouth") == 0) {
        this->ughMouth();
  } else if (stricmp(ex, "vampire") == 0) {
        this->vampire();
  } else if (stricmp(ex, "vampireMouth") == 0) {
        this->vampireMouth();
  } else {
      // ignore
  }
}