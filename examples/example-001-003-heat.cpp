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
  
  unsigned int nb_hot_particles    = std::stoi(argv[1]); 
  unsigned int nb_cold1_particles  = nb_hot_particles;
  unsigned int nb_cold2_particles  = 2*nb_hot_particles;
  
  auto image = cv::Mat(1000, 1200, CV_8UC3, cv::Scalar(255,255,255));
  auto frame = demo2d::opencv::direct_orthonormal_frame(image.size(), .015*image.size().width, true);
  cv::namedWindow("Heat", cv::WINDOW_AUTOSIZE);

  auto drawer = bubul::particle_drawer<ref>(image, frame,
					    [](auto&)                               {return true;},
					    [](auto& ptr) -> const bubul::Particle& {return *ptr;},
					    [](auto&)                               {return .5;});
  
  std::vector<ref> particles;
  auto out = std::back_inserter(particles);

  // We add hot and cold gas.
  for(unsigned int i=0; i < nb_hot_particles; ++i)
    *(out++) = std::make_shared<bubul::Gas>(random_device, demo2d::Point(-29.5,    .5), demo2d::Point(-1.5, 24.5), HOT_SPEED);
  for(unsigned int i=0; i < nb_cold1_particles; ++i)
    *(out++) = std::make_shared<bubul::Gas>(random_device, demo2d::Point(  1.5,   1.5), demo2d::Point(29.5, 24.5), COLD_SPEED);
  for(unsigned int i=0; i < nb_cold2_particles; ++i)
    *(out++) = std::make_shared<bubul::Gas>(random_device, demo2d::Point(-29.5, -24.5), demo2d::Point(29.5, -1.5), COLD_SPEED);

  // We the diathermal walls.
  for(double y = 1; y <= 24; y += 1.) {
    *(out++) = std::make_shared<bubul::diathermal::HLimit>(demo2d::Point(-1, y));
    *(out++) = std::make_shared<bubul::diathermal::HLimit>(demo2d::Point( 0, y));
    *(out++) = std::make_shared<bubul::diathermal::HLimit>(demo2d::Point( 1, y));
  }
  for(double x = 10; x < 20; x += 1.) {
    *(out++) = std::make_shared<bubul::diathermal::VLimit>(demo2d::Point(x, -1));
    *(out++) = std::make_shared<bubul::diathermal::VLimit>(demo2d::Point(x,  0));
    *(out++) = std::make_shared<bubul::diathermal::VLimit>(demo2d::Point(x,  1));
  }
  
  unsigned int nb_mobile_particles = particles.size();

  // We add adiabatic walls.
  for(double x = -30; x <= 30; x += 1.) {
    *(out++) = std::make_shared<bubul::adiabatic::Limit>(demo2d::Point(x, -25.));
    if(!(10 <= x &&  x < 20))
      *(out++) = std::make_shared<bubul::adiabatic::Limit>(demo2d::Point(x,   0.));
    *(out++) = std::make_shared<bubul::adiabatic::Limit>(demo2d::Point(x,  25.));
  }
  
  *(out++) = std::make_shared<bubul::adiabatic::Limit>(demo2d::Point( 9,  1.));
  *(out++) = std::make_shared<bubul::adiabatic::Limit>(demo2d::Point(20,  1.));
  *(out++) = std::make_shared<bubul::adiabatic::Limit>(demo2d::Point( 9, -1.));
  *(out++) = std::make_shared<bubul::adiabatic::Limit>(demo2d::Point(20, -1.));
  
  for(double y = -24; y <= 24; y += 1.) {
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
    }
  }

  return 0;
}
 
