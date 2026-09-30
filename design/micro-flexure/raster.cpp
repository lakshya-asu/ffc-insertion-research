// Orthographic z-buffer for CAD review. No lighting/contact simulation.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <vector>

int main(int argc, char** argv) {
  if (argc != 3) return 2;
  const int w = std::stoi(argv[1]), h = std::stoi(argv[2]);
  uint32_t n;
  if (!std::cin.read(reinterpret_cast<char*>(&n), 4)) return 3;
  std::vector<float> depth(w*h, std::numeric_limits<float>::infinity());
  std::vector<unsigned char> rgb(w*h*3);
  for (int i=0; i<w*h; ++i) {rgb[i*3]=243; rgb[i*3+1]=242; rgb[i*3+2]=238;}
  for (uint32_t i=0; i<n; ++i) {
    float p[12];
    if (!std::cin.read(reinterpret_cast<char*>(p), sizeof(p))) return 4;
    const float den=(p[4]-p[7])*(p[0]-p[6])+(p[6]-p[3])*(p[1]-p[7]);
    if (std::abs(den)<1e-9f) continue;
    int x0=std::max(0,int(std::floor(std::min({p[0],p[3],p[6]}))));
    int x1=std::min(w-1,int(std::ceil(std::max({p[0],p[3],p[6]}))));
    int y0=std::max(0,int(std::floor(std::min({p[1],p[4],p[7]}))));
    int y1=std::min(h-1,int(std::ceil(std::max({p[1],p[4],p[7]}))));
    for(int y=y0;y<=y1;++y) for(int x=x0;x<=x1;++x) {
      const float a=((p[4]-p[7])*(x+.5f-p[6])+(p[6]-p[3])*(y+.5f-p[7]))/den;
      const float b=((p[7]-p[1])*(x+.5f-p[6])+(p[0]-p[6])*(y+.5f-p[7]))/den;
      const float c=1-a-b;
      if (std::min({a,b,c})< -1e-6f) continue;
      const float z=a*p[2]+b*p[5]+c*p[8];
      const int k=y*w+x;
      if(z < depth[k]) {
        depth[k]=z;
        for(int ch=0;ch<3;++ch) rgb[k*3+ch]=(unsigned char)std::clamp(p[9+ch]*255.f,0.f,255.f);
      }
    }
  }
  std::cout<<"P6\n"<<w<<" "<<h<<"\n255\n";
  std::cout.write(reinterpret_cast<char*>(rgb.data()),rgb.size());
}
