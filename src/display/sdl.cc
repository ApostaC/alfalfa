#include <stdexcept>
#include "sdl.hh"

using namespace std;

SDLDisplay::SDLDisplay(const uint16_t display_width, const uint16_t display_height)
  : display_width_(display_width), display_height_(display_height)
{
  if (SDL_Init(SDL_INIT_VIDEO) != 0) { 
    throw runtime_error(SDL_GetError());
  }

  window_ = SDL_CreateWindow(
    "SDL Display",
    SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
    display_width, display_height,
    SDL_WINDOW_RESIZABLE | SDL_WINDOW_OPENGL);

  if (window_ == nullptr) {
    throw runtime_error(SDL_GetError());
  }

  renderer_ = SDL_CreateRenderer(
    window_, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

  if (renderer_ == nullptr) {
    throw runtime_error(SDL_GetError());
  }

  texture_ = SDL_CreateTexture(
    renderer_, SDL_PIXELFORMAT_IYUV, SDL_TEXTUREACCESS_STREAMING,
    display_width, display_height);

  if (texture_ == nullptr) {
    throw runtime_error(SDL_GetError());
  }

  event_ = make_unique<SDL_Event>();
}

SDLDisplay::~SDLDisplay()
{
  SDL_DestroyTexture(texture_);
  SDL_DestroyRenderer(renderer_);
  SDL_DestroyWindow(window_);
  SDL_Quit();
}

void SDLDisplay::show_frame(const BaseRaster & raw_img)
{
  if (raw_img.display_width() != display_width_ or
      raw_img.display_height() != display_height_) {
    throw runtime_error("SDLDisplay: image dimensions don't match!");
  }
  
  SDL_UpdateYUVTexture(texture_, nullptr,
    &(raw_img.Y().at(0,0)), raw_img.Y().width(),
    &(raw_img.U().at(0,0)), raw_img.U().width(),
    &(raw_img.V().at(0,0)), raw_img.V().width());
  SDL_RenderClear(renderer_);
  SDL_RenderCopy(renderer_, texture_, nullptr, nullptr);
  SDL_RenderPresent(renderer_);
}

bool SDLDisplay::signal_quit()
{
  while (SDL_PollEvent(event_.get())) {
    if (event_->type == SDL_QUIT)
      return true;
  }

  return false;
}
