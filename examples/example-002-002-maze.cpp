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
#define SIDE 20
#define STEP 8
#define BBOX_MARGIN 1

#define MIN_X (-SIDE + 1)
#define MAX_X (SIDE - STEP - 1)
#define MIN_Y (-SIDE + 1)
#define MAX_Y (SIDE - 1)

#define GAS_MAX 1500

using ref = std::shared_ptr<bubul::Particle>;

bubul::param::Time bubul::Particle::time = .01;

ref new_gas(std::mt19937& gen) {
  return std::make_shared<bubul::Gas>(gen,
				      demo2d::Point(MIN_X, MIN_Y),
				      demo2d::Point(MAX_X, MAX_Y),
				      SPEED);
}

ref new_wall(const demo2d::Point& pos) {
  return std::make_shared<bubul::adiabatic::Limit>(pos);
}

double maze_pos(std::size_t p) {return -SIDE + (double)(STEP) * p;}

template<typename OutIter>
void maze(OutIter out) {
  
  for(double x = -SIDE; x <= SIDE; x++) {
    *(out++) = new_wall({x, -SIDE});
    *(out++) = new_wall({x,  SIDE});
  }
  
  for(double y = -(SIDE-1); y <= (SIDE-1); y++) 
    *(out++) = new_wall({-SIDE, y});
  
  {
    auto c = maze_pos(4);
    auto l  = maze_pos(4);
    for(double y = maze_pos(1); y < l; y++) 
      *(out++) = new_wall({c, y});
  }

  ////////

  {
    double c = maze_pos(5);
    double l = maze_pos(5);
    for(double y = maze_pos(4); y < l; y++) 
      *(out++) = new_wall({c, y});
    l = maze_pos(1);
    for(double y = maze_pos(0); y < l; y++) 
      *(out++) = new_wall({c, y});
  }

  {
    double l = maze_pos(1);
    double c = maze_pos(5);
    for(double x = maze_pos(4)+1; x <= c; x++) 
      *(out++) = new_wall({x, l});
    l = maze_pos(4);
    for(double x = maze_pos(3); x < c; x++) 
      *(out++) = new_wall({x, l});
    
  }

  ////////
  
  {
    auto c = maze_pos(1);
    auto l = maze_pos(4);
    for(double y = maze_pos(1); y <= l; y++) 
      *(out++) = new_wall({c, y});
  }
  
  {
    auto l = maze_pos(3);
    auto c = maze_pos(1);
    for(double x = maze_pos(0) + 1; x < c; x++) 
      *(out++) = new_wall({x, l});
  }
  
  {
    auto l = maze_pos(1);
    auto c = maze_pos(2);
    for(double x = maze_pos(1) + 1; x <= c; x++) 
      *(out++) = new_wall({x, l});
  }
  
  {
    auto l = maze_pos(2);
    auto c = maze_pos(4);
    for(double x = maze_pos(2) + 1; x < c; x++) 
      *(out++) = new_wall({x, l});
  }
  
  {
    auto c = maze_pos(3);
    auto l = maze_pos(1);
    for(double y = maze_pos(0) + 1; y <= l; y++) 
      *(out++) = new_wall({c, y});
  }
  
  {
    auto c = maze_pos(2);
    auto l = maze_pos(3);
    for(double y = maze_pos(2); y < l; y++) 
      *(out++) = new_wall({c, y});
  }
  
  {
    auto c = maze_pos(2);
    auto l = maze_pos(5);
    for(double y = maze_pos(4) + 1; y < l; y++) 
      *(out++) = new_wall({c, y});
  }
  
  {
    auto l = maze_pos(3);
    auto c = maze_pos(3);
    for(double x = maze_pos(2); x <= c; x++) 
      *(out++) = new_wall({x, l});
  }
}

#define ARROW_BOTTOM_MARGIN 2
#define ARROW_BODY_MARGIN 3
#define ARROW_HEIGHT 4
#define ARROW_EXTRA 1.5
#define ARROW_YEXTRA 2
std::vector<std::vector<cv::Point>> arrow(demo2d::opencv::Frame& frame) {
  std::vector<std::vector<cv::Point>> res {1};
  auto& poly = res[0];

  auto out = std::back_inserter(poly);

  double xmin = maze_pos(4) + ARROW_BODY_MARGIN;
  double xmax = maze_pos(5)+1 - ARROW_BODY_MARGIN;
  double axis = .5*(xmin + xmax);
  double ymin = maze_pos(1) + ARROW_BOTTOM_MARGIN;
  double ymax = maze_pos(3) + ARROW_YEXTRA;
  
  *(out++) = frame(demo2d::Point(xmin, ymin));
  *(out++) = frame(demo2d::Point(xmax, ymin));
  *(out++) = frame(demo2d::Point(xmax, ymax));
  *(out++) = frame(demo2d::Point(xmax + ARROW_EXTRA, ymax));
  *(out++) = frame(demo2d::Point(axis, ymax + ARROW_HEIGHT));
  *(out++) = frame(demo2d::Point(xmin - ARROW_EXTRA, ymax));
  *(out++) = frame(demo2d::Point(xmin, ymax));

  return res;
}

int main(int argc, char* argv[]) {
  std::random_device rd;  
  std::mt19937 gen(rd());

  unsigned int nb_threads = std::thread::hardware_concurrency();

  demo2d::sample::BBox source {demo2d::Point(maze_pos(4) + BBOX_MARGIN, maze_pos(4) + BBOX_MARGIN),
			       demo2d::Point(maze_pos(5) - BBOX_MARGIN, maze_pos(5) - BBOX_MARGIN)};

  demo2d::sample::BBox sink {demo2d::Point(maze_pos(4) + BBOX_MARGIN, maze_pos(0) + BBOX_MARGIN),
			     demo2d::Point(maze_pos(5) - BBOX_MARGIN, maze_pos(1) - BBOX_MARGIN)};

  bool pump = false;
  bool ink = false;
  
  
  std::string main_window {"Click me ! Hit <space> or 'c' ! <ESC> ends."};
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

  gui += {32, [&pump, &ink](){pump = !pump; ink = false;}};
  gui += {'c', [&ink](){ink = true;}};
  
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
					       demo2d::Point(MIN_X, MIN_Y),
					       demo2d::Point(MAX_X, MAX_Y)));
	  (*git)->set_speed(gen, SPEED);
	  (*git)->set_color(190, 180, 255);
	}
      }};

  maze(std::back_inserter(particles));
  nb_wall_particles = particles.size();
  particles.push_back(new_gas(gen));

  auto polygons = arrow(frame);
  while(gui) {
    bubul::hit(nb_threads, particles.begin(), particles.end(),
	       [](auto& ptr) -> bubul::Particle& {return *ptr;});
    bubul::timestep(nb_threads, particles.begin() + nb_wall_particles, particles.end(),
		     [](auto& ptr) -> bubul::Particle& {return *ptr;});

    image = cv::Scalar(255,255,255);

    if(pump) {
      demo2d::opencv::draw(image, frame, source, cv::Scalar(200, 255, 200), -1);
      demo2d::opencv::draw(image, frame, sink,   cv::Scalar(200, 100, 110), -1);
      cv::fillPoly(image, polygons, cv::Scalar(200, 200, 200), cv::LINE_8);
      for(auto p : particles)
	if(sink.contains(p->position())) {
	  p->set_position(source.uniform(gen));
	  if(ink)
	    p->set_color(0, 200, 0);
	}
    }
    
    std::copy(particles.begin(), particles.end(), drawer);

    gui << image;
  }


  return 0;
}
 
