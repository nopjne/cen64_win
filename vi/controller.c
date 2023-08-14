//
// vi/controller.c: Video interface controller.
//
// CEN64: Cycle-Accurate Nintendo 64 Emulator.
// Copyright (C) 2015, Tyler J. Stachecki.
//
// This file is subject to the terms and conditions defined in
// 'LICENSE', which is part of this source code package.
//

#include "common.h"
#include "context.h"
#include "bus/address.h"
#include "bus/controller.h"
#include "device/device.h"
#include "os/main.h"
#include "timer.h"
#include "ri/controller.h"
#include "vi/controller.h"
#include "vi/render.h"
#include "vi/window.h"
#include "vr4300/interface.h"

#define VI_COUNTER_START ((62500000.0 / 60.0) + 1)
#define VI_BLANKING_DONE (unsigned) ((VI_COUNTER_START - VI_COUNTER_START / 525.0 * 39))

#ifdef DEBUG_MMIO_REGISTER_ACCESS
const char *vi_register_mnemonics[NUM_VI_REGISTERS] = {
#define X(reg) #reg,
#include "vi/registers.md"
#undef X
};
#endif

extern struct cen64_options* g_options;
extern FILE* button_replay_file;
extern FILE* button_trace_file;

#if 0
#pragma push(pack,0)
typedef struct tagBITMAPFILEHEADER {
    WORD  bfType;
    DWORD bfSize;
    WORD  bfReserved1;
    WORD  bfReserved2;
    DWORD bfOffBits;
} BITMAPFILEHEADER, * LPBITMAPFILEHEADER, * PBITMAPFILEHEADER;

typedef struct tagBITMAPINFOHEADER {
    DWORD biSize;
    LONG  biWidth;
    LONG  biHeight;
    WORD  biPlanes;
    WORD  biBitCount;
    DWORD biCompression;
    DWORD biSizeImage;
    LONG  biXPelsPerMeter;
    LONG  biYPelsPerMeter;
    DWORD biClrUsed;
    DWORD biClrImportant;
} BITMAPINFOHEADER, * LPBITMAPINFOHEADER, * PBITMAPINFOHEADER;

typedef struct {
    DWORD        bV5Size;
    LONG         bV5Width;
    LONG         bV5Height;
    WORD         bV5Planes;
    WORD         bV5BitCount;
    DWORD        bV5Compression;
    DWORD        bV5SizeImage;
    LONG         bV5XPelsPerMeter;
    LONG         bV5YPelsPerMeter;
    DWORD        bV5ClrUsed;
    DWORD        bV5ClrImportant;
    DWORD        bV5RedMask;
    DWORD        bV5GreenMask;
    DWORD        bV5BlueMask;
    DWORD        bV5AlphaMask;
    DWORD        bV5CSType;
    CIEXYZTRIPLE bV5Endpoints;
    DWORD        bV5GammaRed;
    DWORD        bV5GammaGreen;
    DWORD        bV5GammaBlue;
    DWORD        bV5Intent;
    DWORD        bV5ProfileData;
    DWORD        bV5ProfileSize;
    DWORD        bV5Reserved;
} BITMAPV5HEADER, * LPBITMAPV5HEADER, * PBITMAPV5HEADER;

typedef struct tagRGBQUAD {
    BYTE rgbBlue;
    BYTE rgbGreen;
    BYTE rgbRed;
    BYTE rgbReserved;
} RGBQUAD;
#pragma pop
#endif

// Reads a word from the VI MMIO register space.
int read_vi_regs(void *opaque, uint32_t address, uint32_t *word) {
  struct vi_controller *vi = (struct vi_controller *) opaque;
  unsigned offset = address - VI_REGS_BASE_ADDRESS;
  enum vi_register reg = (offset >> 2);

  vi->regs[VI_CURRENT_REG] = 0;

  // Prevent division by zero (field number doesn't count).
  if (vi->regs[VI_V_SYNC_REG] >= 0x2) {
    vi->regs[VI_CURRENT_REG] =
      (VI_COUNTER_START - (vi->counter)) /
      (VI_COUNTER_START / (vi->regs[VI_V_SYNC_REG] >> 1));

    vi->regs[VI_CURRENT_REG] = (vi->regs[VI_CURRENT_REG] << 1);

    // Interlaced fields should get the current field number.
    // Non-interlaced modes should always get a constant field.
    if (!(vi->regs[VI_V_SYNC_REG] & 0x1))
      vi->regs[VI_CURRENT_REG] |= vi->field;
  }

  *word = vi->regs[reg];
  debug_mmio_read(vi, vi_register_mnemonics[reg], *word);
  return 0;
}

static char filename[256];
BITMAPV5HEADER DIBheader;
BITMAPFILEHEADER header;
uint8_t tempBuffer[FRAMEBUF_SZ];
DWORD Bitmask[4];
// Advances the controller by one clock cycle.
void vi_cycle(struct vi_controller *vi) {
  cen64_gl_window window;
  size_t copy_size;

  unsigned counter;
  struct render_area *ra = &vi->render_area;
  struct bus_controller *bus;
  float hcoeff, vcoeff;

  counter = --(vi->counter);

  // Wrap the counter around when it hits zero.
  if (unlikely(counter == 0))
    vi->counter = VI_COUNTER_START;

  // Throw an interrupt when VI_INTR_REG == VI_CURRENT_REG.
  // We use a counter so we don't have to recalc each cycle.
  if (unlikely(counter == vi->intr_counter))
    signal_rcp_interrupt(vi->bus->vr4300, MI_INTR_VI);

  // NTSC reserves the first 39 lines for vertical blanking
  // according to the literature I've read. Normally after this
  // time, the VI would slowly push out the analog signal. For
  // now, let's just toss the GPU the framebuffer the moment
  // we step out of the vertical blanking interval.
  if (likely(counter != VI_BLANKING_DONE))
    return;

  vi->field = !vi->field;
  window = vi->window;

  // Calculate the bounding positions.
  ra->x.start = vi->regs[VI_H_START_REG] >> 16 & 0x3FF;
  ra->x.end = vi->regs[VI_H_START_REG] & 0x3FF;
  ra->y.start = vi->regs[VI_V_START_REG] >> 16 & 0x3FF;
  ra->y.end = vi->regs[VI_V_START_REG] & 0x3FF;

  hcoeff = (float) (vi->regs[VI_X_SCALE_REG] & 0xFFF) / (1 << 10);
  vcoeff = (float) (vi->regs[VI_Y_SCALE_REG] & 0xFFF) / (1 << 10);

  // Interact with the user interface?
  if (likely(window)) {
    cen64_mutex_lock(&window->event_mutex);

    if (unlikely(window->exit_requested)) {
        if (g_options->button_state_trace_enabled != false) {
            static uint8_t lastFrameInput[4];
            if (button_trace_file == NULL) {
                char string[265];
                sprintf(string, "%s_button_trace.cbt", g_options->cart_file_name);
                button_trace_file = fopen(string, "wb");
            }

            if (memcmp(lastFrameInput, bus->si->input, sizeof(bus->si->input)) != 0) {
                uint64_t frameCount = vi->frame_count;
                size_t written = fwrite(&frameCount, sizeof(frameCount), 1, button_trace_file);
                size_t written1 = fwrite(vi->bus->si->input, sizeof(vi->bus->si->input), 1, button_trace_file);
                assert(written != 0);
                assert(written1 != 0);
                memcpy(lastFrameInput, vi->bus->si->input, sizeof(vi->bus->si->input));
                fflush(button_trace_file);
            }
        }
      cen64_mutex_unlock(&window->event_mutex);
      device_exit(vi->bus);
    }

    cen64_mutex_unlock(&window->event_mutex);
    cen64_mutex_lock(&window->render_mutex);

    // Calculate the height and width of the frame.
    window->frame_vres = ra->height =((ra->y.end - ra->y.start) >> 1) * vcoeff;
    window->frame_hres = ra->width = ((ra->x.end - ra->x.start)) * hcoeff;
    window->frame_hskip = ra->hskip = vi->regs[VI_WIDTH_REG] - ra->width;
    window->frame_type = vi->regs[VI_STATUS_REG] & 0x3;

    if (window->frame_hres <= 0 || window->frame_vres <= 0)
      window->frame_type = 0;

    // Copy the frame data into a temporary buffer.
    copy_size = sizeof(bus->ri->ram) - (vi->regs[VI_ORIGIN_REG] & MAX_RDRAM_SIZE_MASK);

    if (copy_size > sizeof(vi->window->frame_buffer))
      copy_size = sizeof(vi->window->frame_buffer);

    memcpy(&bus, vi, sizeof(bus));
    memcpy(vi->window->frame_buffer,
      bus->ri->ram + (vi->regs[VI_ORIGIN_REG] & MAX_RDRAM_SIZE_MASK),
      copy_size);

    cen64_mutex_unlock(&vi->window->render_mutex);
    cen64_gl_window_push_frame(window);
  }
  else if (((++vi->frame_count) % 60) == 0) {
    cen64_time current_time;
    float ns;

    get_time(&current_time);
    ns = compute_time_difference(&current_time, &vi->last_update_time);
    vi->last_update_time = current_time;

    printf("VI/s: %.2f\n", (60 / (ns / NS_PER_SEC)));
  }

  // If comparison is enabled check if there is a reference.
  if (g_options->ref_compare_enabled != false) {
        sprintf(filename, "ref\\%s_frame%05d.bmp", g_options->cart_file_name, vi->snes_frame);
        FILE* file = fopen(filename, "rb");
        if (file != NULL) {
            // Compare the bmp, skip to the data.
            bool outputOnMismatch = false;
            fread(&header, sizeof(header), 1, file);
            fread(&DIBheader, sizeof(DIBheader), 1, file);
            fread(&Bitmask, sizeof(Bitmask), 1, file);
            if ((sizeof(header) + sizeof(DIBheader) + sizeof(Bitmask)) == header.bfOffBits) {
                uint32_t datasize = (vi->render_area.width * vi->render_area.height * 2);
                if (datasize == (DIBheader.bV5Width * (-DIBheader.bV5Height) * (DIBheader.bV5BitCount / 8))) {
                    fread(tempBuffer, datasize, 1, file);
                    if (memcmp(tempBuffer, window->frame_buffer, datasize) != 0) {
                        outputOnMismatch = true;
                    }
                } else {
                    // mismatch on image dimensions.
                }
            }
            fclose(file);

            // Incase there is a difference, dump the reference, the currently rendered frame and the diff to a .bmp file of
            // 3x height.
            if (outputOnMismatch != false) {
                sprintf(filename, "%s_mismatch%i.bmp", g_options->cart_file_name, vi->snes_frame);
                file = fopen(filename, "wb");
                if (file != NULL) {
                    // Generate header and output.
                    BITMAPV5HEADER DIBheader = { 0 };
                    BITMAPFILEHEADER header = { 0 };
                    DWORD Bitmask[4];
                    header.bfType = 'MB';
                    header.bfSize = sizeof(header);
                    header.bfOffBits = sizeof(header) + sizeof(DIBheader) + sizeof(Bitmask);
                    DIBheader.bV5Size = sizeof(DIBheader);
                    DIBheader.bV5Width = vi->render_area.width;
                    DIBheader.bV5Height = -((int)vi->render_area.height * 3);
                    DIBheader.bV5Planes = 1;
                    DIBheader.bV5BitCount = 16;
                    DIBheader.bV5Compression = BI_BITFIELDS;
                    DIBheader.bV5SizeImage = 0;
                    //                         ggbbbbbarrrrrggg
                    DIBheader.bV5BlueMask = 0b0011111000000000;
                    DIBheader.bV5GreenMask = 0b1100000000000111;
                    DIBheader.bV5RedMask = 0b0000000011111000;
                    DIBheader.bV5AlphaMask = 0b0000000100000000;

                    Bitmask[0] = 0b0000000011111000;
                    Bitmask[1] = 0b1100000000000111;
                    Bitmask[2] = 0b0011111000000000;
                    Bitmask[3] = 0b0000000100000000;
                    fwrite(&header, sizeof(header), 1, file);
                    fwrite(&DIBheader, sizeof(DIBheader), 1, file);
                    fwrite(&Bitmask, sizeof(Bitmask), 1, file);
                    fwrite(&tempBuffer, (vi->render_area.width* vi->render_area.height * 2), 1, file);
                    fwrite(&window->frame_buffer, (vi->render_area.width * vi->render_area.height * 2), 1, file);
                    
                    for (uint32_t i = 0; i < (vi->render_area.width * vi->render_area.height); i += 1) {
                        uint16_t color = 0;
                        uint16_t *ref = tempBuffer;
                        uint16_t *render = window->frame_buffer;
                        if (ref[i] != render[i]) {
                            color = 0xFFFF & ~0b0000000100000000;
                        }

                        fwrite(&color, sizeof(color), 1, file);
                    }
                    fclose(file);
                }
                outputOnMismatch = false;
            }
        }
        
  }

  bool dump_enabled = g_options->ref_dump_enabled;
  uint32_t dump_interval = 88;
  // If dumping is enabled output the frame_buffer.
  if ((dump_enabled != false) && ((vi->snes_frame % dump_interval) == 0) && (vi->render_area.width != 0)) {
      sprintf(filename, "%s_frame%05d.bmp", g_options->cart_file_name, vi->snes_frame);
      FILE* file = fopen(filename, "wb");
      if (file != NULL) {
          // Generate header and output.
          header.bfType = 'MB';
          header.bfSize = sizeof(header);
          header.bfOffBits = sizeof(header) + sizeof(DIBheader) + sizeof(Bitmask);
          DIBheader.bV5Size = sizeof(DIBheader);
          DIBheader.bV5Width = vi->render_area.width;
          DIBheader.bV5Height = -((int)vi->render_area.height);
          DIBheader.bV5Planes = 1;
          DIBheader.bV5BitCount = 16;
          DIBheader.bV5Compression = BI_BITFIELDS;
          DIBheader.bV5SizeImage = 0;
          //                         ggbbbbbarrrrrggg
          DIBheader.bV5BlueMask =  0b0011111000000000;
          DIBheader.bV5GreenMask = 0b1100000000000111;
          DIBheader.bV5RedMask =   0b0000000011111000;
          DIBheader.bV5AlphaMask = 0b0000000100000000;
          
          Bitmask[0] = 0b0000000011111000;
          Bitmask[1] = 0b1100000000000111;
          Bitmask[2] = 0b0011111000000000;
          Bitmask[3] = 0b0000000100000000;
          fwrite(&header, sizeof(header), 1, file);
          fwrite(&DIBheader, sizeof(DIBheader), 1, file);
          fwrite(&Bitmask, sizeof(Bitmask), 1, file);
          fwrite(&window->frame_buffer, (vi->render_area.width * vi->render_area.height * 2), 1, file);
          fclose(file);
      }
  }

  // Record input
  bool RecordInput = g_options->button_state_trace_enabled;
  if (RecordInput != false) {
    static uint8_t lastFrameInput[4];
    if (button_trace_file == NULL) {
        char string[265];
        sprintf(string, "%s_button_trace.cbt", g_options->cart_file_name);
        button_trace_file = fopen(string, "wb");
    }

    if (memcmp(lastFrameInput, bus->si->input, sizeof(bus->si->input)) != 0) {
        uint64_t frameCount = vi->snes_frame;
        size_t written = fwrite(&frameCount, sizeof(frameCount), 1, button_trace_file);
        size_t written1 = fwrite(vi->bus->si->input, sizeof(vi->bus->si->input), 1, button_trace_file);
        assert(written != 0);
        assert(written1 != 0);
        memcpy(lastFrameInput, vi->bus->si->input, sizeof(vi->bus->si->input));
        fflush(button_trace_file);
    }
  }

  // Replay input
  bool ReplayInput = g_options->button_state_replay_enabled;
  if (ReplayInput != false) {
      static uint64_t frameCount = 0;
      if (button_replay_file == NULL) {
          char string[265];
          sprintf(string, "ref\\%s_button_trace.cbt", g_options->cart_file_name);
          button_replay_file = fopen(string, "rb");
          if (button_replay_file == NULL) {
            g_options->button_state_replay_enabled = false;
          } else {
            fread(&frameCount, sizeof(frameCount), 1, button_replay_file);
          }
      }

      if (button_replay_file != NULL) {
        static uint8_t lastFrameInput[4];
        if ((frameCount - 1) <= vi->snes_frame) {
          if (feof(button_replay_file)) {
              ExitProcess(0);
          }
          fread(&(vi->bus->si->input[0]), sizeof(vi->bus->si->input), 1, button_replay_file);
          fread(&frameCount, sizeof(frameCount), 1, button_replay_file);
        }
      }
  }

  vi->frame_count += 1;
}

// Initializes the VI.
int vi_init(struct vi_controller *vi,
  struct bus_controller *bus, bool no_interface) {
  vi->counter = VI_COUNTER_START;
  vi->bus = bus;

  if (!no_interface) {
    if (vi_create_window(vi))
      return -1;

    gl_window_init(vi);
  }

  return 0;
}

// Writes a word to the VI MMIO register space.
int write_vi_regs(void *opaque, uint32_t address, uint32_t word, uint32_t dqm) {
  struct vi_controller *vi = (struct vi_controller *) opaque;
  unsigned offset = address - VI_REGS_BASE_ADDRESS;
  enum vi_register reg = (offset >> 2);

  debug_mmio_write(vi, vi_register_mnemonics[reg], word, dqm);

  if (reg == VI_CURRENT_REG)
    clear_rcp_interrupt(vi->bus->vr4300, MI_INTR_VI);

  else if (reg == VI_INTR_REG) {

    // TODO: This seems... all kinds of wrong for interlaced modes.
    // Do we fire two interrupts in interlaced modes? Have to test.
    // I'm not an NTSC signal expert, so this'll have to do for now.
    vi->intr_counter = VI_COUNTER_START - VI_COUNTER_START / 525 * (word >> 1);

    vi->regs[reg] &= ~dqm;
    vi->regs[reg] |= word;
  }

  else if (VI_ORIGIN_REG) {
      // This is a hack to hook the Sodium64 frame ptr swap.
      vi->snes_frame += 1;

      // Continue regular execution.
      vi->regs[reg] &= ~dqm;
      vi->regs[reg] |= word;
  }

  else {
    vi->regs[reg] &= ~dqm;
    vi->regs[reg] |= word;
  }

  return 0;
}

