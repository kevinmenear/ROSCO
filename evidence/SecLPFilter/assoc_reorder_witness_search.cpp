// Same question, full-mantissa draws -- powers of two never distinguish the
// regrouping (DT*DT is exact there), so an exact-mantissa grid answers nothing.
#include <cstdio>
#include <cstring>
#include <cmath>
#include <cstdint>
static bool bd(double a,double b){uint64_t x,y;memcpy(&x,&a,8);memcpy(&y,&b,8);return x!=y;}
static uint64_t s=88172645463325252ull;
static double u(){ s^=s<<13; s^=s>>7; s^=s<<17; return (double)(s>>11)/9007199254740992.0; }
int main(){
  long a1h=0,b1h=0; double aDT=0,aCF=0,bDT=0,bCF=0;
  for (long t=0;t<40000000;++t){
    double DT = std::ldexp(0.5+0.5*u(), -1074 + (int)(u()*600));   // DT*DT subnormal band
    double CF = std::ldexp(0.5+0.5*u(), -60 + (int)(u()*1090));
    double pr = 2.0*(DT*DT)*(CF*CF), pm = (2.0*DT)*DT*(CF*CF);
    if (bd(pr,pm)) { if(!b1h){bDT=DT;bCF=CF;} ++b1h;
      if (bd(pr-8.0, pm-8.0)) { if(!a1h){aDT=DT;aCF=CF;} ++a1h; } }
  }
  printf("b1 form (bare product): %ld witness(es) in 4e7 draws\n", b1h);
  if(b1h) printf("   e.g. DT=%.17g CF=%.17g -> %.17g vs %.17g\n",bDT,bCF,
                 2.0*(bDT*bDT)*(bCF*bCF),(2.0*bDT)*bDT*(bCF*bCF));
  printf("a1 form (product - 8.0): %ld witness(es) in the SAME draws\n", a1h);
  if(a1h) printf("   e.g. DT=%.17g CF=%.17g -> %.17g vs %.17g\n",aDT,aCF,
                 2.0*(aDT*aDT)*(aCF*aCF)-8.0,(2.0*aDT)*aDT*(aCF*aCF)-8.0);
  return 0;
}
