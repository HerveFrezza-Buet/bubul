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

double global_Ec(unsigned int nb_threads, auto begin, auto end) {
  return bubul::Ec(nb_threads, begin, end, [](auto& ptr) -> bubul::Particle& {return *ptr;});
}

int main(int argc, char* argv[]) {
  std::random_device rd;  
  std::mt19937 random_device(rd());

  unsigned int nb_threads = std::thread::hardware_concurrency();
  

  if(argc != 4) {
    std::cout << "Usage : " << argv[0] << " img-side [circle|square|rectangle] <nb-gas-particles>" << std::endl;
    return 0;
  }

  unsigned int img_size = std::stoul(argv[1]);
  unsigned int nb_particles = std::stoi(argv[3]);
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

  particles[0]->set_color(50, 50, 150);

  std::cout << std::endl
	    << std::endl
	    << "<ESC>   quit"          << std::endl
	    << "<space> pause/play"    << std::endl
	    << "e       toogles cinetic energy conservation" << std::endl
	    << "i       invert speeds" << std::endl
	    << "l       left"          << std::endl
	    << "c       center"        << std::endl
	    << "u       uniform"       << std::endl
	    << "j       jet"           << std::endl
	    << "b       bump"          << std::endl
	    << std::endl;
  
  bool do_simul = false;
  bool Ec_conserv = true;
  auto gas_begin = particles.begin();
  auto gas_end   = particles.begin() + nb_particles;
  double alpha = 1;
  double Ec;
  
  gui += {' ', [&do_simul](){do_simul = not do_simul;}};
  
  gui += {'e', [&Ec, &Ec_conserv, gas_begin, gas_end, nb_threads](){
    Ec_conserv = not Ec_conserv;
    if(Ec_conserv)
      Ec = global_Ec(nb_threads, gas_begin, gas_end);
  }};

  gui += {'i', [gas_begin, gas_end, nb_threads]() {
    for(auto git = gas_begin; git != gas_end; ++git)
      (*git)->rescale_speed(-1.0);
  }};
  
  gui += {'l', [&Ec, &random_device, gas_begin, gas_end, nb_threads]() {
    for(auto git = gas_begin; git != gas_end; ++git) {
      (*git)->set_position(demo2d::uniform(random_device,
					   demo2d::Point(-14., -14.),
					   demo2d::Point(-10.,  14.)));
      (*git)->set_speed(random_device, SPEED);
    }
    Ec = global_Ec(nb_threads, gas_begin, gas_end);
  }};
  
  gui += {'c', [&random_device, gas_begin, gas_end, nb_threads]() {
    for(auto git = gas_begin; git != gas_end; ++git)
      (*git)->set_position(demo2d::uniform(random_device,
					   demo2d::Point(-5., -5.),
					   demo2d::Point( 5.,  5.)));
  }};
  
  gui += {'u', [&Ec, &random_device, gas_begin, gas_end, nb_threads]() {
    for(auto git = gas_begin; git != gas_end; ++git) {
      (*git)->set_position(demo2d::uniform(random_device,
					   demo2d::Point(-14.5, -14.5),
					   demo2d::Point( 14.5,  14.5)));
      (*git)->set_speed(random_device, SPEED);
    }
    Ec = global_Ec(nb_threads, gas_begin, gas_end);
  }};
  
  gui += {'j', [&Ec, &random_device, gas_begin, gas_end, nb_threads]() {
    for(auto git = gas_begin; git != gas_end; ++git) {
      (*git)->set_position(demo2d::uniform(random_device,
					   demo2d::Point(-3, -14.5),
					   demo2d::Point( 3,  12.0)));
      (*git)->set_speed(demo2d::Point(0, SPEED));
    }
    Ec = global_Ec(nb_threads, gas_begin, gas_end);
  }};
  
  gui += {'b', [&Ec, &random_device, gas_begin, gas_end, nb_particles, nb_threads]() {
    auto gas_half = gas_begin + nb_particles/2;
    auto git = gas_begin;
    for(; git != gas_half; ++git) {
      (*git)->set_position(demo2d::uniform(random_device,
					   demo2d::Point(-4, -14.5),
					   demo2d::Point( 2,  -8.5)));
      (*git)->set_speed(demo2d::Point(0, SPEED));
    }
    for(; git != gas_end; ++git) {
      (*git)->set_position(demo2d::uniform(random_device,
					   demo2d::Point(-2,  8.5),
					   demo2d::Point( 4, 14.5)));
      (*git)->set_speed(demo2d::Point(0, -SPEED));
    }
    Ec = global_Ec(nb_threads, gas_begin, gas_end);
  }};

  

  gui += {"viscosity", [&alpha](double v){alpha = 1 - v;}};
  
  Ec = global_Ec(nb_threads, gas_begin, gas_end);
  while(gui) {
    image = cv::Scalar(255,255,255);

    if(do_simul) {
      bubul::hit(particles.begin(), particles.end(),
		 [](auto& ptr) -> bubul::Particle& {return *ptr;},
		 alpha);
      if(Ec_conserv and alpha > 0)
	bubul::adjust_Ec(nb_threads, Ec, gas_begin, gas_end, [](auto& ptr) -> bubul::Particle& {return *ptr;});

      bubul::timestep(nb_threads, gas_begin, gas_end, [](auto& ptr) -> bubul::Particle& {return *ptr;});
    }
    
    std::copy(particles.begin(), particles.end(), drawer);
    gui << image;
    
    
  }

  return 0;
}
 
