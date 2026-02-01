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
#define SIDE 20

using ref = std::shared_ptr<bubul::Particle>;

bubul::param::Time bubul::Particle::time = .01;

ref new_gas(std::mt19937& gen) {
  
  return std::make_shared<bubul::Gas>(gen,
				      demo2d::Point(-SIDE + 1, -SIDE + 1),
				      demo2d::Point( SIDE - 1,  SIDE - 1),
				      SPEED);
}

ref new_wall(const demo2d::Point& pos) {
  return std::make_shared<bubul::adiabatic::Limit>(pos);
}

int main(int argc, char* argv[]) {
  std::random_device rd;  
  std::mt19937 gen(rd());

  unsigned int nb_threads = std::thread::hardware_concurrency();
  
  
  std::string main_window {"Click me ! <ESC> ends."};
  auto image = cv::Mat(1000, 1000, CV_8UC3, cv::Scalar(255,255,255));
  auto frame = demo2d::opencv::direct_orthonormal_frame(image.size(), .024*image.size().width, true);
  auto gui = demo2d::opencv::gui(main_window, frame); 
  gui.loop_ms = 1; 
  
  auto drawer = bubul::particle_drawer<ref>(image, frame,
					    [](auto&)                               {return true;},
					    [](auto& ptr) -> const bubul::Particle& {return *ptr;},
					    [](auto&)                               {return .5;});
  
  
  // Here are the particles.
  std::vector<ref> particles;
  std::size_t nb_wall_particles;

#define GAS_MAX 3000
  gui += {std::string("max = ") + std::to_string(GAS_MAX), [&particles, &nb_wall_particles, &gen](double slider_value) {
    std::size_t target_nb = (std::size_t)(GAS_MAX*slider_value) + 1;
    std::size_t nb = particles.size() - nb_wall_particles;
    if(target_nb > nb) {
      auto out  = std::back_inserter(particles);
      for(auto i = nb; i <= target_nb; ++i)
	*(out++) = new_gas(gen);
    }
    else if (target_nb < nb) 
      particles.resize(nb_wall_particles + target_nb);
    
  }};
  
  gui += {cv::EVENT_LBUTTONDOWN,
      [&particles, &nb_wall_particles, &gen](const demo2d::Point&){
	for(auto git = particles.begin() + nb_wall_particles; git != particles.end(); ++git) {
	  (*git)->set_position(demo2d::uniform(gen,
					       demo2d::Point(-SIDE + 1, -SIDE + 1),
					       demo2d::Point( SIDE - 1,  SIDE - 1)));
	  (*git)->set_speed(gen, SPEED);
	}
      }};

  auto out  = std::back_inserter(particles);
  
  for(double x = -SIDE; x <= SIDE; x+=1.) {
    *(out++) = new_wall({x, -SIDE});
    *(out++) = new_wall({x,  SIDE});
    }
  for(double y = -(SIDE-1); y <= (SIDE-1); y+=1.) {
    *(out++) = new_wall({-SIDE, y});
    *(out++) = new_wall({ SIDE, y});
  }
  nb_wall_particles = particles.size();

  *(out++) = new_gas(gen);

  while(gui) {
    bubul::hit(nb_threads, particles.begin(), particles.end(),
	       [](auto& ptr) -> bubul::Particle& {return *ptr;});
    bubul::timestep(nb_threads, particles.begin() + nb_wall_particles, particles.end(),
		     [](auto& ptr) -> bubul::Particle& {return *ptr;});

    image = cv::Scalar(255,255,255);
    std::copy(particles.begin(), particles.end(), drawer);

    gui << image;
  }


  return 0;
}
 
