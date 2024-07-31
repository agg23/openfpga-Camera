// For anyone looking at this, this method of creating UI is kind of a hack, and pieced together from other, more
// complete experiments of mine. I dislike all of this code, but it works for the purpose
module ui (
    input wire clk,

    input wire [9:0] video_fetch_x,
    input wire [9:0] video_fetch_y,

    // Settings
    input wire [2:0] string_index,

    output wire active,
    output reg [23:0] vid_out
);
  localparam UI_SCALE = 1;
  localparam X_SCALE_BIT = UI_SCALE;
  localparam Y_SCALE_BIT = UI_SCALE;

  localparam DISPLAY_WIDTH = 160;
  localparam DISPLAY_HEIGHT = 144;
  localparam TEXT_PADDING_Y = 10;

  localparam TEXT_HEIGHT = UI_SCALE * 8 + 2 * TEXT_PADDING_Y;

  localparam START_Y = (DISPLAY_HEIGHT - TEXT_HEIGHT) / 2;

  localparam UI_START_Y = START_Y + TEXT_PADDING_Y;
  localparam UI_END_Y = UI_START_Y + 8 * UI_SCALE;
  localparam END_Y = START_Y + TEXT_PADDING_Y * 2 + 8 * UI_SCALE;

  // Strings
  // Not detected
  localparam NOT_DETECTED_TEXT_LENGTH = 19;
  localparam NOT_DETECTED_TEXT_WIDTH = NOT_DETECTED_TEXT_LENGTH * UI_SCALE * 8;

  localparam NOT_DETECTED_START_X = (DISPLAY_WIDTH - NOT_DETECTED_TEXT_WIDTH) / 2;
  localparam NOT_DETECTED_END_X = NOT_DETECTED_START_X + NOT_DETECTED_TEXT_WIDTH;

  reg [7:0] not_detected_text[NOT_DETECTED_TEXT_LENGTH] = '{
      "C",
      "a",
      "m",
      "e",
      "r",
      "a",
      " ",
      "n",
      "o",
      "t",
      " ",
      "d",
      "e",
      "t",
      "e",
      "c",
      "t",
      "e",
      "d"
  };

  // SRAM exported
  localparam SRAM_EXPORTED_TEXT_LENGTH = 13;
  localparam SRAM_EXPORTED_TEXT_WIDTH = SRAM_EXPORTED_TEXT_LENGTH * UI_SCALE * 8;

  localparam SRAM_EXPORTED_START_X = (DISPLAY_WIDTH - SRAM_EXPORTED_TEXT_WIDTH) / 2;
  localparam SRAM_EXPORTED_END_X = SRAM_EXPORTED_START_X + SRAM_EXPORTED_TEXT_WIDTH;

  reg [7:0] sram_exported_text[SRAM_EXPORTED_TEXT_LENGTH] = '{
      "S",
      "R",
      "A",
      "M",
      " ",
      "e",
      "x",
      "p",
      "o",
      "r",
      "t",
      "e",
      "d"
  };

  // Comb
  reg [7:0] character = 0;

  always_comb begin
    reg [9:0] local_addr;
    character = 0;

    case (string_index)
      0: begin
        if (character_addr < NOT_DETECTED_TEXT_LENGTH) begin
          character = not_detected_text[character_addr[4:0]];
        end
      end
      1: begin
        if (character_addr < SRAM_EXPORTED_TEXT_LENGTH) begin
          character = sram_exported_text[character_addr[4:0]];
        end
      end
    endcase
  end

  wire [9:0] ui_start_x;
  wire [9:0] ui_end_x;

  always_comb begin
    ui_start_x = 0;
    ui_end_x = 1;

    case (string_index)
      0: begin
        ui_start_x = NOT_DETECTED_START_X;
        ui_end_x   = NOT_DETECTED_END_X;
      end
      0: begin
        ui_start_x = SRAM_EXPORTED_START_X;
        ui_end_x   = SRAM_EXPORTED_END_X;
      end
    endcase
  end

  wire x_active = video_fetch_x >= ui_start_x && video_fetch_x < ui_end_x;
  wire y_active = video_fetch_y >= UI_START_Y && video_fetch_y < UI_END_Y;

  assign active = video_fetch_y >= START_Y && video_fetch_y < END_Y;

  wire [9:0] x_offset = video_fetch_x - ui_start_x;
  wire [9:0] y_offset = video_fetch_y - UI_START_Y;

  wire [3:0] char_x_pixel = video_fetch_x >= ui_start_x ? x_offset[X_SCALE_BIT+1:0] : 0;
  wire [3:0] char_y_pixel = video_fetch_y >= UI_START_Y ? y_offset[X_SCALE_BIT+1:0] : 0;

  wire [2:0] character_row = char_y_pixel[Y_SCALE_BIT+1:Y_SCALE_BIT-1];

  wire [9:0] x_offset_prefetch = x_offset + 1;
  // wire [9:0] character_addr = {4'b0, x_offset_prefetch[9:4]};
  wire [9:0] character_addr = {5'b0, x_offset_prefetch[8:3]};

  assign vid_out = x_active && y_active && font_line[7-char_x_pixel[X_SCALE_BIT+1:X_SCALE_BIT-1]] ? 24'hFFFFFF : 0;

  wire [7:0] font_line;

  char_rom char_rom (
      .clk(clk),

      .character(character - 32),
      .row(character_row),
      .data_out(font_line)
  );
endmodule
