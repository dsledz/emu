#pragma once

namespace EMU {

template <class palette_t>
class PaletteMap {
 public:
  PaletteMap(void);

 private:
};

template <class tile_t>
class TileMap {
 public:
  TileMap(void);

  const tile_t &at(unsigned idx) {}

 private:
};

template <class sprite_t>
class SpriteMap {
 public:
  SpriteMap(void);

 private:
};

template <uint32_t size, class color_t = RGBColor, uint32_t ele_size = 1>
class ColorMap {
 public:
  typedef color_t (*convert_cb)(uint8_t *ele);

  void init(uint8_t *b, convert_cb cb) {
    for (unsigned i = 0; i < m_colors.size(); i++) {
      m_colors[i] = cb(b);
      b += ele_size;
    }
  }

  const color_t &operator[](unsigned idx) const { return m_colors.at(idx); }

 private:
  std::array<color_t, size> m_colors;
};

/**
 * Class for simple (80s) graphics libaries
 */
template <class palette_map_t, class tile_map_t, class sprite_map_t,
          class color_map_t>
class SimpleGfx {
 public:
  SimpleGfx(void) : m_colors(), m_palettes(), m_tiles(), m_sprites() {}
  ~SimpleGfx(void);

  /** Initialize the graphics data */
  void init_gfx(RomSet *romset) {
    /** Load the color map */
    m_colors.init(romset);

    m_palettes.init(romset);

    m_tiles.init(romset);

    m_sprites.init(romset);
  }

  /** Update the screen */
  void update(FrameBuffer *screen) {
    screen->clear();

    draw_tiles(screen);
    draw_sprites(screen);

    screen->flip();
  }

 private:
  void draw_tiles(FrameBuffer *screen) {
    /**
     * for tx = 0 to 36:
     *     for ty = 0 to 28:
     *         lookup tile
     *         draw tile
     *
     */
  }

  void draw_sprites(FrameBuffer *screen) {
    /**
     * for each sprite
     *     lookup sprite
     *     draw sprite
     */
  }

  color_map_t m_colors;
  palette_map_t m_palettes;
  tile_map_t m_tiles;
  sprite_map_t m_sprites;
};
};
