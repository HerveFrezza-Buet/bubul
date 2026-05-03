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
#define SPEED 5

using ref = std::shared_ptr<bubul::Particle>;

bubul::param::Time bubul::Particle::time = .01;


// Sets the number of particles : 0 means 1, 1 means 2, etc...
void set_nb_particles(std::vector<ref>& particles,
		      auto& random_device,
		      unsigned int nb_walls,
		      unsigned int nb) {
  nb++;
  unsigned int current = particles.size() - nb_walls;
  if(nb < current)
    particles.resize(nb_walls + nb);
  else {
    unsigned int delta = nb - current;
    auto out = std::back_inserter(particles);
    for(unsigned int i = 0; i < delta; ++i)
      *(out++) = std::make_shared<bubul::Gas>(random_device, demo2d::Point(-14.5, -14.5), demo2d::Point(14.5, 14.5), SPEED);
  }
}

int main(int argc, char* argv[]) {
  std::random_device rd;  
  std::mt19937 random_device(rd());

  unsigned int nb_threads = std::thread::hardware_concurrency();
  

  if(argc != 3) {
    std::cout << "Usage : " << argv[0] << " img-side [circle|square|rectangle]" << std::endl;
    return 0;
  }

  unsigned int img_size = std::stoul(argv[1]);
  std::string wall_shape = argv[2];
  
  std::string main_window {"Gas"};
  auto image = cv::Mat(img_size, img_size, CV_8UC3, cv::Scalar(255,255,255));
  auto frame = demo2d::opencv::direct_orthonormal_frame(image.size(), .02*image.size().width, true);
  auto gui = demo2d::opencv::gui(main_window, frame); 
  gui.loop_ms = 1; 

  auto drawer = bubul::particle_drawer<ref>(image, frame,
					    [](auto&)                               {return true;},
					    [](auto& ptr) -> const bubul::Particle& {return *ptr;},
					    [](auto&)                               {return .5;});


  // Here are the particles.
  std::vector<ref> particles;
  
  // Let us add walls.
  auto out = std::back_inserter(particles);
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
  else if (wall_shape == "circle") {
#define NB_WALLS 150
#define WALL_RADIUS 22
    for(unsigned int i=0; i < NB_WALLS; ++i)
      *(out++) = std::make_shared<bubul::adiabatic::Limit>(WALL_RADIUS * demo2d::Point::unitary(i*TWOPI/(double)NB_WALLS));
  }
  else {
    for(double x = -25; x <= 25; x+=1.) {
      *(out++) = std::make_shared<bubul::adiabatic::Limit>(demo2d::Point(x, -15.));
      *(out++) = std::make_shared<bubul::adiabatic::Limit>(demo2d::Point(x,  15.));
    }
    for(double y = -14; y <= 14; y+=1.) {
      *(out++) = std::make_shared<bubul::adiabatic::Limit>(demo2d::Point(-25., y));
      *(out++) = std::make_shared<bubul::adiabatic::Limit>(demo2d::Point( 25., y));
    }
  }
  unsigned int nb_walls = particles.size();

  auto p = std::make_shared<bubul::Gas>(random_device, demo2d::Point(-14.5, -14.5), demo2d::Point(14.5, 14.5), SPEED);
  p->set_color(50, 50, 150);
  *(out++) = p;


  std::cout << std::endl
	    << std::endl
	    << "<ESC>               quit"          << std::endl
	    << "<space>             pause/play"    << std::endl
	    << "e                   toogles kinetic energy conservation" << std::endl
	    << std::endl
	    << "i                   invert speeds" << std::endl
	    << "l                   left"          << std::endl
	    << "c                   center"        << std::endl
	    << "u / left click      uniform"       << std::endl
	    << "j                   jet"           << std::endl
	    << "b                   bump"          << std::endl
	    << std::endl
	    << std::endl;
  
  bool do_simul = false;
  bool Ec_conserv = true;
  double alpha = 1;
  
  gui += {' ', [&do_simul](){do_simul = not do_simul;}};
  
  gui += {'e', [&Ec_conserv](){
    Ec_conserv = not Ec_conserv;
  }};

  gui += {'i', [&particles, nb_walls]() {
    for(auto git = particles.begin() + nb_walls; git != particles.end(); ++git)
      (*git)->rescale_speed(-1.0);
  }};
  
  gui += {'l', [&random_device, &particles, nb_walls, nb_threads]() {
    for(auto git = particles.begin() + nb_walls; git != particles.end(); ++git) {
      (*git)->set_position(demo2d::uniform(random_device,
					   demo2d::Point(-14., -14.),
					   demo2d::Point(-10.,  14.)));
      (*git)->set_speed(random_device, SPEED);
    }
  }};
  
  gui += {'c', [&random_device, &particles, nb_walls, nb_threads]() {
    for(auto git = particles.begin() + nb_walls; git != particles.end(); ++git)
      (*git)->set_position(demo2d::uniform(random_device,
					   demo2d::Point(-5., -5.),
					   demo2d::Point( 5.,  5.)));
  }};
  
  gui += {'u', [&random_device, &particles, nb_walls, nb_threads]() {
    for(auto git = particles.begin() + nb_walls; git != particles.end(); ++git) {
      (*git)->set_position(demo2d::uniform(random_device,
					   demo2d::Point(-14.5, -14.5),
					   demo2d::Point( 14.5,  14.5)));
      (*git)->set_speed(random_device, SPEED);
    }
  }};
  
  
  gui += {cv::EVENT_LBUTTONDOWN, [&random_device, &particles, nb_walls, nb_threads](const demo2d::Point&){
    for(auto git = particles.begin() + nb_walls; git != particles.end(); ++git) {
      (*git)->set_position(demo2d::uniform(random_device,
					   demo2d::Point(-14.5, -14.5),
					   demo2d::Point( 14.5,  14.5)));
      (*git)->set_speed(random_device, SPEED);
    }
  }};

  gui += {'j', [&random_device, &particles, nb_walls, nb_threads]() {
    for(auto git = particles.begin() + nb_walls; git != particles.end(); ++git) {
      (*git)->set_position(demo2d::uniform(random_device,
					   demo2d::Point(-3, -14.5),
					   demo2d::Point( 3,  12.0)));
      (*git)->set_speed(demo2d::Point(0, SPEED));
    }
  }};
  
  gui += {'b', [&random_device, &particles, nb_walls, nb_threads]() {
    auto git = particles.begin() + nb_walls;
    unsigned int nb_particles = std::distance(git, particles.end());
    auto gas_half = particles.begin() + nb_particles/2;
    for(; git != gas_half; ++git) {
      (*git)->set_position(demo2d::uniform(random_device,
					   demo2d::Point(-4, -14.5),
					   demo2d::Point( 2,  -8.5)));
      (*git)->set_speed(demo2d::Point(0, SPEED));
    }
    for(; git != particles.end(); ++git) {
      (*git)->set_position(demo2d::uniform(random_device,
					   demo2d::Point(-2,  8.5),
					   demo2d::Point( 4, 14.5)));
      (*git)->set_speed(demo2d::Point(0, -SPEED));
    }
  }};
  

  gui += {"nb particles", [&random_device, &particles, nb_walls](double v){
    set_nb_particles(particles, random_device,
		     nb_walls, (unsigned int)(1000*v + .5));
  }};
  
  gui += {"viscosity", [&alpha](double v){alpha = 1 - v;}};
  
  while(gui) {
    image = cv::Scalar(255,255,255);

    if(do_simul) {
      bubul::hit(particles.begin(), particles.end(),
		 [](auto& ptr) -> bubul::Particle& {return *ptr;},
		 alpha, Ec_conserv);

      bubul::timestep(nb_threads, particles.begin() + nb_walls, particles.end(), [](auto& ptr) -> bubul::Particle& {return *ptr;});
    }
    
    std::copy(particles.begin(), particles.end(), drawer);
    if(Ec_conserv)
      cv::putText(image, "Ec: forced", cv::Point(10, 20), cv::FONT_HERSHEY_PLAIN, 1., cv::Scalar(0,0,0), 1);
    else
      cv::putText(image, "Ec: not forced", cv::Point(10, 20), cv::FONT_HERSHEY_PLAIN, 1., cv::Scalar(0,0,0), 1);
    gui << image;
    
    
  }

  return 0;
}
 
