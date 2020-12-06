#include <bubul.hpp>
#include <demo2d.hpp>

#include <random>
#include <vector>
#include <iterator>
#include <memory>
#include <algorithm>
#include <cmath>

#define NB_GAS_PARTICLES 200

using ref = std::shared_ptr<bubul::Particle>;

double bubul::Particle::dt = .01;

int main(int argc, char* argv[]) {
  std::random_device rd;  
  std::mt19937 random_device(rd());
  
  auto image = cv::Mat(1000, 1000, CV_8UC3, cv::Scalar(255,255,255));
  auto frame = demo2d::opencv::direct_orthonormal_frame(image.size(), .02*image.size().width, true);
  cv::namedWindow("Gas", cv::WINDOW_AUTOSIZE);

  auto drawer = bubul::particle_drawer<ref>(image, frame,
					    [](auto&)                               {return true;},
					    [](auto& ptr) -> const bubul::Particle& {return *ptr;},
					    [](auto&)                               {return .5;});

  // Here are the particles.
  std::vector<ref> particles;
  auto out = std::back_inserter(particles);
  for(unsigned int i=0; i < NB_GAS_PARTICLES; ++i)
    *(out++) = std::make_shared<bubul::Gas>(random_device, demo2d::Point(-14., -14.), demo2d::Point(0., 14.), 2.);

  // Let us add walls.
  for(double x = -15; x <= 15; x+=1.) {
    *(out++) = std::make_shared<bubul::adiabatic::Limit>(demo2d::Point(x, -15.));
    *(out++) = std::make_shared<bubul::adiabatic::Limit>(demo2d::Point(x,  15.));
  }
  for(double y = -14; y <= 14; y+=1.) {
    *(out++) = std::make_shared<bubul::adiabatic::Limit>(demo2d::Point(-15., y));
    *(out++) = std::make_shared<bubul::adiabatic::Limit>(demo2d::Point( 15., y));
  }
  
  int keycode = 0;
  bool do_simul = false;
  while(keycode != 27) {
    image = cv::Scalar(255,255,255);

    if(do_simul) {
      bubul::hit(particles.begin(), particles.end(),
		 [](auto& ptr) -> bubul::Particle& {return *ptr;});

      auto gas_end = particles.begin() + NB_GAS_PARTICLES;
      for(auto git = particles.begin(); git != gas_end; ++git) ++(*(*git));
    }
    std::copy(particles.begin(), particles.end(), drawer);
    
    cv::imshow("Gas", image);
    keycode = cv::waitKey(1) & 0xFF;
    if(keycode == 32)
      do_simul = !do_simul;
  }

  return 0;
}
 
