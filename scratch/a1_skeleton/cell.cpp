#include "cell.h"
#include <random>
#include <chrono>

Cell::Cell() :
  side_(cell::DEFAULT_CELL_SIZE),
  state_(cell::State::UNKNOWN)
{
  long int seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine gen(seed);
  std::uniform_real_distribution<double> distX(-cell::MAP_SIZE, cell::MAP_SIZE);
  std::uniform_real_distribution<double> distY(-cell::MAP_SIZE, cell::MAP_SIZE);

  centreX_=distX(gen);
  centreY_=distY(gen);

}

void Cell::setSide(double side)
{
    side_ = side;
}

void Cell::setCentre(double x, double y)
{
    centreX_=x;
    centreY_=y;
}

void Cell::getCentre(double &x, double &y)
{
    x=centreX_;
    y=centreY_;
}


double Cell::getSide()
{
    return side_;
}


cell::State Cell::getState()
{
    return state_;
}


void Cell::setState(cell::State state)
{
    state_= state;
}

double Cell::area(void)
{
    return side_ * side_;
}


double Cell::perimeter(void)
{
    return 4 * side_;
}
