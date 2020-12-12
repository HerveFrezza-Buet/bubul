#include <bubul.hpp>
#include <demo2d.hpp>

#include <random>
#include <vector>
#include <iterator>
#include <memory>
#include <algorithm>
#include <cmath>

#define SPEED 10

#define PI 3.14159365435

using ref = std::shared_ptr<bubul::Particle>;

double bubul::Particle::dt    = .01;
double bubul::Weighted::floor = -20;

int main(int argc, char* argv[]) {
  std::random_device rd;  
  std::mt19937 random_device(rd());

  if(argc != 2) {
    std::cout << "Usage : " << argv[0] << " <nb-gas-particles>" << std::endl;
    return 0;
  }

  unsigned int nb_particles = std::stoi(argv[1]);
  
  auto image = cv::Mat(800, 500, CV_8UC3, cv::Scalar(255,255,255));
  auto frame = demo2d::opencv::direct_orthonormal_frame(image.size(), .035*image.size().width, true);
  cv::namedWindow("Gas", cv::WINDOW_AUTOSIZE);

  auto drawer = bubul::particle_drawer<ref>(image, frame,
					    [](auto&)                               {return true;},
					    [](auto& ptr) -> const bubul::Particle& {return *ptr;},
					    [](auto&)                               {return .5;});

  // Here are the particles.
  std::vector<ref> particles;
  auto out = std::back_inserter(particles);
  
  for(unsigned int i=0; i < nb_particles; ++i)
    *(out++) = std::make_shared<bubul::Weighted>(random_device, demo2d::Point(-9.5, -19.5), demo2d::Point(9.5, 19.5), SPEED);

  // Let us add walls.
  for(double x = -10; x <= 10; x+=1.) {
    *(out++) = std::make_shared<bubul::adiabatic::Limit>(demo2d::Point(x, -20.));
    *(out++) = std::make_shared<bubul::adiabatic::Limit>(demo2d::Point(x,  20.));
  }
  for(double y = -19; y <= 19; y+=1.) {
    *(out++) = std::make_shared<bubul::adiabatic::Limit>(demo2d::Point(-10., y));
    *(out++) = std::make_shared<bubul::adiabatic::Limit>(demo2d::Point( 10., y));
  }


  std::cout << std::endl
	    << std::endl
	    << "<ESC>   quit"       << std::endl
	    << "<space> pause/play" << std::endl
	    << "u       uniform"    << std::endl
	    << std::endl;

  int keycode = 0;
  bool do_simul = false;
  auto git = particles.begin();
  auto gas_end = particles.begin();
  
  while(keycode != 27) {
    image = cv::Scalar(255,255,255);


    if(do_simul) {
      bubul::hit(particles.begin(), particles.end(),
		 [](auto& ptr) -> bubul::Particle& {return *ptr;});

      gas_end = particles.begin() + nb_particles;
      for(git = particles.begin(); git != gas_end; ++git) ++(*(*git));
    }
    std::copy(particles.begin(), particles.end(), drawer);
    
    cv::imshow("Gas", image);
    keycode = cv::waitKey(1) & 0xFF;
    
    switch((char)(keycode)) {
    case ' ':
      do_simul = !do_simul;
      break;
    case 'u':
      gas_end = particles.begin() + nb_particles;
      for(git = particles.begin(); git != gas_end; ++git) {
	(*git)->set_position(demo2d::uniform(random_device, 
					     demo2d::Point(-9.5, -19.5),
					     demo2d::Point( 9.5,  19.5)));
	(*git)->set_speed(SPEED*demo2d::Point::unitary(std::uniform_real_distribution<double>(0, 2*PI)(random_device)));			     
      }
      
      break;
    default:
      break;
    }
    
  }

  return 0;
}
 
