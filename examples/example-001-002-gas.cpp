#include <bubul.hpp>
#include <demo2d.hpp>

#include <random>
#include <vector>
#include <iterator>
#include <memory>
#include <algorithm>
#include <cmath>
#include <thread>

#define TWOPI 6.283185307179586
#define SPEED 10

using ref = std::shared_ptr<bubul::Particle>;

bubul::param::Time bubul::Particle::time = .01;

int main(int argc, char* argv[]) {
  std::random_device rd;  
  std::mt19937 random_device(rd());

  unsigned int nb_threads = std::thread::hardware_concurrency();
  

  if(argc != 3) {
    std::cout << "Usage : " << argv[0] << " [circle|square] <nb-gas-particles>" << std::endl;
    return 0;
  }

  unsigned int nb_particles = std::stoi(argv[2]);
  std::string wall_shape = argv[1];
  
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
  for(unsigned int i=0; i < nb_particles; ++i)
    *(out++) = std::make_shared<bubul::Gas>(random_device, demo2d::Point(-14.5, -14.5), demo2d::Point(14.5, 14.5), SPEED);

  // Let us add walls.
  if(wall_shape == "square") {
    for(double x = -15; x <= 15; x+=1.) {
      *(out++) = std::make_shared<bubul::adiabatic::Limit>(demo2d::Point(x, -15.));
      *(out++) = std::make_shared<bubul::adiabatic::Limit>(demo2d::Point(x,  15.));
    }
    for(double y = -14; y <= 14; y+=1.) {
      *(out++) = std::make_shared<bubul::adiabatic::Limit>(demo2d::Point(-15., y));
      *(out++) = std::make_shared<bubul::adiabatic::Limit>(demo2d::Point( 15., y));
    }
  }
  else {
#define NB_WALLS 150
#define WALL_RADIUS 22
    for(unsigned int i=0; i < NB_WALLS; ++i)
      *(out++) = std::make_shared<bubul::adiabatic::Limit>(WALL_RADIUS * demo2d::Point::unitary(i*TWOPI/(double)NB_WALLS));
  }

  particles[0]->set_color(50, 50, 150);

  std::cout << std::endl
	    << std::endl
	    << "<ESC>   quit"       << std::endl
	    << "<space> pause/play" << std::endl
	    << "l       left"       << std::endl
	    << "c       center"     << std::endl
	    << "u       uniform"    << std::endl
	    << "j       jet"        << std::endl
	    << "b       bump"       << std::endl
	    << std::endl;
  
  int keycode = 0;
  bool do_simul = false;
  auto git = particles.begin();
  auto gas_end = particles.begin();
  
  while(keycode != 27) {
    image = cv::Scalar(255,255,255);

    if(do_simul) {
      bubul::hit(nb_threads, particles.begin(), particles.end(),
		 [](auto& ptr) -> bubul::Particle& {return *ptr;});

      gas_end = particles.begin() + nb_particles;
     bubul::timestep(nb_threads, particles.begin(), gas_end,
		     [](auto& ptr) -> bubul::Particle& {return *ptr;});
    }
    std::copy(particles.begin(), particles.end(), drawer);
    
    cv::imshow("Gas", image);
    keycode = cv::waitKey(1) & 0xFF;
    
    switch((char)(keycode)) {
    case ' ':
      do_simul = !do_simul;
      break;
    case 'l':
      gas_end = particles.begin() + nb_particles;
      for(git = particles.begin(); git != gas_end; ++git)
	(*git)->set_position(demo2d::uniform(random_device,
					     demo2d::Point(-14., -14.),
					     demo2d::Point(-10.,  14.)));
      break;
    case 'c':
      gas_end = particles.begin() + nb_particles;
      for(git = particles.begin(); git != gas_end; ++git)
	(*git)->set_position(demo2d::uniform(random_device,
					     demo2d::Point(-5., -5.),
					     demo2d::Point( 5.,  5.)));
      break;
    case 'u':
      gas_end = particles.begin() + nb_particles;
      for(git = particles.begin(); git != gas_end; ++git)
	(*git)->set_position(demo2d::uniform(random_device,
					     demo2d::Point(-14.5, -14.5),
					     demo2d::Point( 14.5,  14.5)));
      break;
    case 'j':
      gas_end = particles.begin() + nb_particles;
      for(git = particles.begin(); git != gas_end; ++git) {
	(*git)->set_position(demo2d::uniform(random_device,
					     demo2d::Point(-3, -14.5),
					     demo2d::Point( 3,  12.0)));
	(*git)->set_speed(demo2d::Point(0, SPEED));
      }
      break;
    case 'b':
      gas_end = particles.begin() + nb_particles/2;
      git = particles.begin();
      for(; git != gas_end; ++git) {
	(*git)->set_position(demo2d::uniform(random_device,
					     demo2d::Point(-4, -14.5),
					     demo2d::Point( 2,  -8.5)));
	(*git)->set_speed(demo2d::Point(0, SPEED));
      }
      gas_end = particles.begin() + nb_particles;
      for(; git != gas_end; ++git) {
	(*git)->set_position(demo2d::uniform(random_device,
					     demo2d::Point(-2,  8.5),
					     demo2d::Point( 4, 14.5)));
	(*git)->set_speed(demo2d::Point(0, -SPEED));
      }
      break;
    default:
      break;
    }
    
  }

  return 0;
}
 
