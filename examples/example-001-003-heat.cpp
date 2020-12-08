#include <bubul.hpp>
#include <demo2d.hpp>

#include <random>
#include <vector>
#include <iterator>
#include <memory>
#include <algorithm>
#include <cmath>

#define HOT_SPEED  10
#define COLD_SPEED  .1
using ref = std::shared_ptr<bubul::Particle>;

double bubul::Particle::dt = .01;

int main(int argc, char* argv[]) {
  std::random_device rd;  
  std::mt19937 random_device(rd());

  if(argc != 2) {
    std::cout << "Usage : " << argv[0] << " <nb-gas-particles>" << std::endl;
    return 0;
  }
  
  unsigned int nb_particles = std::stoi(argv[1]) / 2; // number of cold/hot.

  auto image = cv::Mat(600, 1200, CV_8UC3, cv::Scalar(255,255,255));
  auto frame = demo2d::opencv::direct_orthonormal_frame(image.size(), .015*image.size().width, true);
  cv::namedWindow("Heat", cv::WINDOW_AUTOSIZE);

  auto drawer = bubul::particle_drawer<ref>(image, frame,
					    [](auto&)                               {return true;},
					    [](auto& ptr) -> const bubul::Particle& {return *ptr;},
					    [](auto&)                               {return .5;});
  
  std::vector<ref> particles;
  auto out = std::back_inserter(particles);

  // We add hot and cold gas.
  for(unsigned int i=0; i < nb_particles; ++i)
    *(out++) = std::make_shared<bubul::Gas>(random_device, demo2d::Point(-29.5, -14.5), demo2d::Point( -.5, 14.5), HOT_SPEED);
  for(unsigned int i=0; i < nb_particles; ++i)
    *(out++) = std::make_shared<bubul::Gas>(random_device, demo2d::Point(   .5, -14.5), demo2d::Point(29.5, 14.5), COLD_SPEED);

  // We the diathermal walls.
  for(double y = -14; y <= 14; y+=1.) {
    *(out++) = std::make_shared<bubul::diathermal::HLimit>(demo2d::Point(-.6, y));
    *(out++) = std::make_shared<bubul::diathermal::HLimit>(demo2d::Point( .6, y));
  }

  auto nb_mobile_particles = particles.size();
  
  // We add adiabatic walls.
  for(double x = -30; x <= 30; x+=1.) {
    *(out++) = std::make_shared<bubul::adiabatic::Limit>(demo2d::Point(x, -15.));
    *(out++) = std::make_shared<bubul::adiabatic::Limit>(demo2d::Point(x,  15.));
  }
  for(double y = -14; y <= 14; y+=1.) {
    *(out++) = std::make_shared<bubul::adiabatic::Limit>(demo2d::Point(-30., y));
    *(out++) = std::make_shared<bubul::adiabatic::Limit>(demo2d::Point( 30., y));
  }

  std::cout << std::endl
	    << std::endl
	    << "<ESC>   quit"    << std::endl
	    << "<space> restart" << std::endl
	    << std::endl;
  
  int keycode = 0;
  auto git = particles.begin();
  auto gas_end = particles.begin();
  
  while(keycode != 27) {
    image = cv::Scalar(255,255,255);

    bubul::hit(particles.begin(), particles.end(),
	       [](auto& ptr) -> bubul::Particle& {return *ptr;});

    gas_end = particles.begin() + nb_mobile_particles;
    for(git = particles.begin(); git != gas_end; ++git) ++(*(*git));
      
    std::copy(particles.begin(), particles.end(), drawer);
    
    cv::imshow("Heat", image);
    keycode = cv::waitKey(1) & 0xFF;
    
    if((char)(keycode) == ' ') {
      gas_end = particles.begin() + nb_particles;
      for(git = particles.begin(); git != gas_end; ++git)
	*((*git)) = bubul::Gas(random_device, demo2d::Point(-29.5, -14.5), demo2d::Point( -.5, 14.5), HOT_SPEED);
      gas_end += nb_particles;
      for(; git != gas_end; ++git)
	*((*git)) = bubul::Gas(random_device, demo2d::Point(   .5, -14.5), demo2d::Point(29.5, 14.5), COLD_SPEED);
    }
  }

  return 0;
}
 
