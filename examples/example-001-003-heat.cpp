#include <bubul.hpp>
#include <demo2d.hpp>

#include <random>
#include <vector>
#include <iterator>
#include <memory>
#include <algorithm>
#include <cmath>
#include <thread>

#define HOT_SPEED  10
#define COLD_SPEED  0.01
using ref = std::shared_ptr<bubul::Particle>;

bubul::param::Time bubul::Particle::time = .01;

int main(int argc, char* argv[]) {
  std::random_device rd;  
  std::mt19937 random_device(rd());

  unsigned int nb_threads = std::thread::hardware_concurrency();

  if(argc != 2) {
    std::cout << "Usage : " << argv[0] << " <nb-hot-gas-particles (200)>" << std::endl;
    return 0;
  }
  
  unsigned int nb_hot_particles    = std::stoi(argv[1]); 
  unsigned int nb_cold1_particles  = nb_hot_particles;
  unsigned int nb_cold2_particles  = 3*nb_hot_particles;
  
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
    *(out++) = std::make_shared<bubul::Gas>(random_device, demo2d::Point(-29.5,   1.5), demo2d::Point(-1.5, 24.5), HOT_SPEED);
  for(unsigned int i=0; i < nb_cold1_particles; ++i)
    *(out++) = std::make_shared<bubul::Gas>(random_device, demo2d::Point(  1.5,   1.5), demo2d::Point(29.5, 24.5), COLD_SPEED);
  for(unsigned int i=0; i < nb_cold2_particles; ++i)
    *(out++) = std::make_shared<bubul::MGas>(random_device, demo2d::Point(-29.5, -24.5), demo2d::Point(29.5, -1.5), COLD_SPEED);

  // We the diathermal walls.
  auto l = particles.size();
  for(double y = 2; y <= 24; y += 1.) {
    *(out++) = std::make_shared<bubul::diathermal::HLimit>(demo2d::Point(-1, y));
    *(out++) = std::make_shared<bubul::diathermal::HLimit>(demo2d::Point( 0, y));
    *(out++) = std::make_shared<bubul::diathermal::HLimit>(demo2d::Point( 1, y));
  }
  unsigned int nb_w1 = particles.size() - l;
  l = particles.size();
  for(double x = 7; x < 30; x += 1.) {
    *(out++) = std::make_shared<bubul::diathermal::VLimit>(demo2d::Point(x, -1));
    *(out++) = std::make_shared<bubul::diathermal::VLimit>(demo2d::Point(x,  0));
    *(out++) = std::make_shared<bubul::diathermal::VLimit>(demo2d::Point(x,  1));
  }
  unsigned int nb_w2 = particles.size() - l;
  
  unsigned int nb_mobile_particles = particles.size();

  // We add adiabatic walls.
  for(double x = -30; x <= 30; x += 1.) {
    *(out++) = std::make_shared<bubul::adiabatic::Limit>(demo2d::Point(x, -25.));
    *(out++) = std::make_shared<bubul::adiabatic::Limit>(demo2d::Point(x,  25.));
  }
  for(double x = -29; x <= 6; x += 1.) {
    *(out++) = std::make_shared<bubul::adiabatic::Limit>(demo2d::Point(x, -1.));
    *(out++) = std::make_shared<bubul::adiabatic::Limit>(demo2d::Point(x,  0.));
    *(out++) = std::make_shared<bubul::adiabatic::Limit>(demo2d::Point(x,  1.));
  }
  
  for(double y = -24; y <= 24; y += 1.) {
    *(out++) = std::make_shared<bubul::adiabatic::Limit>(demo2d::Point(-30., y));
    *(out++) = std::make_shared<bubul::adiabatic::Limit>(demo2d::Point( 30., y));
  }

  // Energy bar
  double sum = bubul::E(1, particles.begin(), particles.end(), [](auto& ptr) -> bubul::Particle& {return *ptr;});
  bubul::plot::EnergyHBar bar({-30, 26}, 1.5, 60/sum);
  bar.display.push_back({"g1", cv::Scalar(255, 180, 190)});
  bar.display.push_back({"w1", cv::Scalar(  0,   0, 120)});
  bar.display.push_back({"g2", cv::Scalar(255, 180, 190)});
  bar.display.push_back({"w2", cv::Scalar(  0,   0, 120)});
  bar.display.push_back({"g3", cv::Scalar(200, 100, 110)});

  std::cout << std::endl
	    << std::endl
	    << "<ESC>   quit"    << std::endl
	    << "<space> restart" << std::endl
	    << std::endl;
  
  int keycode = 0;
  auto gas_end = particles.begin();
  
  while(keycode != 27) {
    image = cv::Scalar(255,255,255);
 
     bubul::hit(nb_threads, particles.begin(), particles.end(), [](auto& ptr) -> bubul::Particle& {return *ptr;});

     gas_end = particles.begin() + nb_mobile_particles;
     bubul::timestep(nb_threads, particles.begin(), gas_end, [](auto& ptr) -> bubul::Particle& {return *ptr;});
      
     std::copy(particles.begin(), particles.end(), drawer);

     bar.content["g1"] = bubul::E(nb_threads, particles.begin(),
				  particles.begin() + nb_hot_particles,
				  [](auto& ptr) -> bubul::Particle& {return *ptr;});
     bar.content["g2"] = bubul::E(nb_threads, particles.begin() + nb_hot_particles,
				  particles.begin() + nb_hot_particles + nb_cold1_particles,
				  [](auto& ptr) -> bubul::Particle& {return *ptr;});
     bar.content["g3"] = bubul::E(nb_threads, particles.begin() + nb_hot_particles + nb_cold1_particles,
				  particles.begin() + nb_hot_particles + nb_cold1_particles + nb_cold2_particles,
				  [](auto& ptr) -> bubul::Particle& {return *ptr;});
     bar.content["w1"] = bubul::E(1, particles.begin() + nb_hot_particles + nb_cold1_particles + nb_cold2_particles,
				  particles.begin() + nb_hot_particles + nb_cold1_particles + nb_cold2_particles + nb_w1,
				  [](auto& ptr) -> bubul::Particle& {return *ptr;});
     bar.content["w2"] = bubul::E(1, particles.begin() + nb_hot_particles + nb_cold1_particles + nb_cold2_particles + nb_w1,
				  particles.begin() + nb_hot_particles + nb_cold1_particles + nb_cold2_particles + nb_w1 + nb_w2,
				  [](auto& ptr) -> bubul::Particle& {return *ptr;});
    bar(image, frame);
    
    cv::imshow("Heat", image);
    keycode = cv::waitKey(1) & 0xFF;

    
    
    if((char)(keycode) == ' ') {
    }
  }

  return 0;
}
 
