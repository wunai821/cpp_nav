#include "rm_nav/app/app.hpp"

int main(int argc, char** argv) {
  rm_nav::app::App app;
  return app.Run(argc, argv);
}
