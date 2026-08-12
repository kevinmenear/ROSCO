#include <cstdio>
#include <cstring>
static bool d(double a,double b){unsigned long long x,y;memcpy(&x,&a,8);memcpy(&y,&b,8);return x!=y;}
int main(){
  double DTs[]={1e-156,1e-158};
  double CFs[]={0.0,1.0,-1.0,1e3,-1e3,1.5708,1e300,1e-3,2.0,1e6};
  const char* n[]={"0.0","1.0","-1.0","1e3","-1e3","1.5708","1e300","1e-3","2.0","1e6"};
  for(double DT:DTs){ printf("DT=%.0e :",DT);
    for(int i=0;i<10;i++){ if(d(2.0*(DT*DT)*(CFs[i]*CFs[i]),(2.0*DT)*DT*(CFs[i]*CFs[i]))) printf(" %s",n[i]); }
    printf("\n"); }
  // the shipped default: does the plain ladder rung with CF at +/-1e3 kill it?
  double DT=1e-156, CF=1e3;
  printf("b1 ref=%.17g mut=%.17g\n", 2.0*(DT*DT)*(CF*CF), (2.0*DT)*DT*(CF*CF));
  return 0;
}
