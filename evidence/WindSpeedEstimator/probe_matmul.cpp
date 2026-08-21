#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdint>
static void pr(const char*t,int n,int i,int j,double v){uint64_t b;std::memcpy(&b,&v,8);
  std::printf("%s%2d%2d%2d %016llX\n",t,n,i,j,(unsigned long long)b);}
int main(){
  for(int n=1;n<=3;n++){
    double F[9],P[9],H[3],K[3],I3[9];
    for(int j=1;j<=3;j++){
      for(int i=1;i<=3;i++){
        F[(j-1)*3+(i-1)] = std::sin(1.0*(i+3*j+9*n))*1.234;
        P[(j-1)*3+(i-1)] = std::cos(2.0*(i+5*j+7*n))*0.577;
      }
      K[j-1] = std::sin(0.7*(j+11*n));
      H[j-1] = 0.0;
    }
    H[0]=1.0; double R_m=0.02;
    for(int k=0;k<9;k++) I3[k]=0.0;
    for(int j=0;j<3;j++) I3[j*3+j]=1.0;
    double M1[9],M2[9],M3[9],Pn[9],Kn[3],S;
    for(int j=0;j<3;j++)for(int i=0;i<3;i++){double s=0.0;for(int k=0;k<3;k++)s=s+F[k*3+i]*P[j*3+k];M1[j*3+i]=s;}
    for(int j=0;j<3;j++)for(int i=0;i<3;i++){double s=0.0;for(int k=0;k<3;k++)s=s+P[k*3+i]*F[k*3+j];M2[j*3+i]=s;}
    for(int j=0;j<3;j++)for(int i=0;i<3;i++){double s=0.0;s=s+K[i]*R_m*K[j];M3[j*3+i]=s;}
    {double PHt[3];for(int i=0;i<3;i++){double s=0.0;for(int k=0;k<3;k++)s=s+P[k*3+i]*H[k];PHt[i]=s;}
     double s=0.0;for(int k=0;k<3;k++)s=s+H[k]*PHt[k];S=s+R_m;
     for(int i=0;i<3;i++)Kn[i]=PHt[i]/S;}
    {double KH[9];for(int j=0;j<3;j++)for(int i=0;i<3;i++){double s=0.0;s=s+K[i]*H[j];KH[j*3+i]=s;}
     double M[9];for(int k=0;k<9;k++)M[k]=I3[k]-KH[k];
     for(int j=0;j<3;j++)for(int i=0;i<3;i++){double s=0.0;for(int k=0;k<3;k++)s=s+M[k*3+i]*P[j*3+k];Pn[j*3+i]=s;}}
    for(int j=1;j<=3;j++){
      for(int i=1;i<=3;i++){
        pr("M1 ",n,i,j,M1[(j-1)*3+(i-1)]);
        pr("M2 ",n,i,j,M2[(j-1)*3+(i-1)]);
        pr("M3 ",n,i,j,M3[(j-1)*3+(i-1)]);
        pr("Pn ",n,i,j,Pn[(j-1)*3+(i-1)]);
      }
      pr("Kn ",n,j,1,Kn[j-1]);
    }
    pr("S  ",n,1,1,S);
  }
}
