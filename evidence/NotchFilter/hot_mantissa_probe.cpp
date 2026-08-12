#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <initializer_list>
static bool bd(double a,double b){uint64_t x,y;memcpy(&x,&a,8);memcpy(&y,&b,8);return x!=y;}
int main(){
  // Which MANTISSAS at an exponent in the subnormal-square band make
  // 2.0*(x*x) and (2.0*x)*x differ?  And do the round decimal rungs?
  for (int e : {-512,-520,-530,-535,-540,-545,-550}) {
    int hits=0; double first=0;
    for (int k=0;k<256;++k){ double m=1.0+k/256.0; double x=std::ldexp(m,e);
      if (bd(2.0*(x*x),(2.0*x)*x)){ if(!hits) first=m; ++hits; } }
    printf("exp %5d : %3d of 256 mantissas differ, first m=%.10g  (x=%.17g)\n",
           e,hits,first,std::ldexp(first,e));
  }
  for (double x : {1e-155,1e-156,1e-157,1e-158,1e-159,1e-160,1e-161,1e-162,1.4916681462400413e-154,2.2227587494850775e-162})
    printf("round rung %.17g : differs=%d\n", x, (int)bd(2.0*(x*x),(2.0*x)*x));
  // end to end, with K*K underflowed to zero
  double DTs[] = {1e300, 1e250, 1e204};
  for (double DT : DTs){ double K=2.0/DT;
    int hits=0; double fx=0;
    for (int k=0;k<256;++k){ double m=1.0+k/256.0; double om=std::ldexp(m,-535);
      double den=(K*K)+2.0*om*0.25*K+(om*om);
      double b=(2.0*(om*om)-2.0*(K*K))/den, mo=((2.0*om)*om-2.0*(K*K))/den;
      if (bd(b,mo)){ if(!hits) fx=om; ++hits; } }
    printf("DT=%.3g  K*K=%.3g : %3d of 256 omega mantissas kill the omega-mutant, first omega=%.17g\n",
           DT,K*K,hits,fx);
  }
  return 0;
}
