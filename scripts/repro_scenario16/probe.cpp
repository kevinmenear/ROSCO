// Standalone replay of the C++ ratelimit final update, from exact operand bits.
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <algorithm>
#include <cstdlib>
static double D(const char*h){uint64_t u=strtoull(h,nullptr,16);double d;memcpy(&d,&u,8);return d;}
static uint64_t B(double d){uint64_t u;memcpy(&u,&d,8);return u;}
int main(int c,char**v){
  double inp=D(v[1]),L=D(v[2]),DT=D(v[3]),minR=D(v[4]),maxR=D(v[5]);
  double rate=(inp-L)/DT;
  rate=std::min(std::max(rate,minR),maxR);      // saturate_c
  double res=L+rate*DT;
  printf("C++     rate=0x%016llX  result=0x%016llX\n",
         (unsigned long long)B(rate),(unsigned long long)B(res));
  return 0;
}
