#!/usr/bin/python3

# Quick code gen is faster than typing it all out...

funs = """
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
"""

def gen():
    exps = []
    for l in sorted(funs.splitlines()):
        exp = l.replace("void", "").replace("();", "").strip()
        if exp: exps.append(exp)

    code = f'void Face::setExpression(const char* ex){{\n' + \
    f'if (stricmp(ex, "{exps[0]}") == 0) {{\n' + \
    f'	this->{exps[0]}();\n'

    for i in range(1,len(exps)):
        s = f'}} else if (stricmp(ex, "{exps[i]}") == 0) {{\n' + \
        f'      this->{exps[i]}();\n'
        code += s

    code += "} else {\n     // ignore\n}"

    print(code)




if __name__ == "__main__":
    gen()