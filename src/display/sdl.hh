#ifndef SDL_HH
#define SDL_HH

extern "C" {
#include <SDL2/SDL.h>
}

#include <memory>
#include "raster.hh"

class SDLDisplay
{
public:
  SDLDisplay(const uint16_t display_width, const uint16_t display_height);
  ~SDLDisplay();

  // display a frame
  void show_frame(const BaseRaster & raster);

  // if signaled to quit
  bool signal_quit();

  // forbid copy and quit operators
  SDLDisplay(const SDLDisplay &) = delete;
  const SDLDisplay & operator=(const SDLDisplay &) = delete;
  SDLDisplay(SDLDisplay && other) = delete;
  SDLDisplay & operator=(SDLDisplay && other) = delete;

private:
  uint16_t display_width_;
  uint16_t display_height_;

  SDL_Window * window_ {nullptr};
  SDL_Renderer * renderer_ {nullptr};
  SDL_Texture * texture_ {nullptr};
  std::unique_ptr<SDL_Event> event_ {nullptr};
};

#endif
