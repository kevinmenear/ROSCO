#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <vector>
static double orig(double K,double om,double bd){ return (2.0*(om*om) - 2.0*(K*K)) / ((K*K) + 2.0*om*bd*K + (om*om)); }
static double mutK(double K,double om,double bd){ return (2.0*(om*om) - (2.0*K)*K) / ((K*K) + 2.0*om*bd*K + (om*om)); }
static double mutO(double K,double om,double bd){ return ((2.0*om)*om - 2.0*(K*K)) / ((K*K) + 2.0*om*bd*K + (om*om)); }
static bool bitdiff(double a,double b){uint64_t x,y;memcpy(&x,&a,8);memcpy(&y,&b,8);return x!=y;}
int main(){
  std::vector<double> L={0.0,1.0,-1.0,0.5,-0.5,
    1.4916681462400413e-154,-1.4916681462400413e-154,
    2.2227587494850775e-162,-2.2227587494850775e-162,
    1.3407807929942596e+154,-1.3407807929942596e+154,
    2.2250738585072014e-308,-2.2250738585072014e-308,5e-324,-5e-324,
    1.7976931348623157e+308,-1.7976931348623157e+308,
    1e-300,-1e-300,1e-155,-1e-155,1e-100,-1e-100,1e-10,-1e-10,
    1e10,-1e10,1e100,-1e100,1e155,-1e155,1e300,-1e300};
  long nK=0,nO=0; bool pk=false,po=false;
  for(double DT: L){ if(DT==0.0) continue; double K=2.0/DT;
    for(double om: L) for(double bd: {0.0,0.25,1.0,-1.0,1e-155,1e155}){
      double a=orig(K,om,bd);
      if(bitdiff(a,mutK(K,om,bd))){ ++nK; if(!pk){printf("K-mutant witness: DT=%.17g omega=%.17g betaDen=%.17g -> orig %.17g mut %.17g\n",DT,om,bd,a,mutK(K,om,bd)); pk=true;} }
      if(bitdiff(a,mutO(K,om,bd))){ ++nO; if(!po){printf("omega-mutant witness: DT=%.17g omega=%.17g betaDen=%.17g -> orig %.17g mut %.17g\n",DT,om,bd,a,mutO(K,om,bd)); po=true;} }
    }
  }
  printf("K-mutant distinguished by %ld ladder triples; omega-mutant by %ld\n",nK,nO);
  return 0;
}
