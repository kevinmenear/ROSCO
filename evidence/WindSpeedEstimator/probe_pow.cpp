#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdint>
int main(){
  for(int i=1;i<=8;i++){
    double x = 1.0 + 0.37*i + 0.001*i*i;
    double y = std::pow(x,3.0); uint64_t b; std::memcpy(&b,&y,8);
    std::printf("CPP pow(x,3.0)= %.17E  bits=%016llX\n", y, (unsigned long long)b);
    y = (x*x)*x; std::memcpy(&b,&y,8);
    std::printf("CPP (x*x)*x   = %.17E  bits=%016llX\n", y, (unsigned long long)b);
  }
}
