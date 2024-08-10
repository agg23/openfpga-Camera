// For anyone looking at this, this method of creating UI is kind of a hack, and pieced together from other, more
// complete experiments of mine. I dislike all of this code, but it works for the purpose
module ui (
    input wire clk,
    input wire ce_pix,

    input wire [9:0] video_fetch_x,
    input wire [9:0] video_fetch_y,

    // Settings
    input wire [2:0] string_index,
    input wire [7:0] save_index_bcd,

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

  // SRAM exporting
  localparam SRAM_EXPORTING_TEXT_LENGTH = 14;
  localparam SRAM_EXPORTING_TEXT_WIDTH = SRAM_EXPORTING_TEXT_LENGTH * UI_SCALE * 8;

  localparam SRAM_EXPORTING_START_X = (DISPLAY_WIDTH - SRAM_EXPORTING_TEXT_WIDTH) / 2;
  localparam SRAM_EXPORTING_END_X = SRAM_EXPORTING_START_X + SRAM_EXPORTING_TEXT_WIDTH;

  reg [7:0] sram_exporting_text[SRAM_EXPORTING_TEXT_LENGTH] = '{
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
      "i",
      "n",
      "g"
  };

  // Saved to SRAM_
  localparam SAVED_TO_TEXT_LENGTH = 12;
  localparam SAVED_TO_NUMBER_TEXT_LENGTH = 2;
  localparam SAVED_TO_SUFFIX_TEXT_LENGTH = 4;
  localparam SAVED_TO_TEXT_WIDTH = (SAVED_TO_TEXT_LENGTH + SAVED_TO_NUMBER_TEXT_LENGTH + SAVED_TO_SUFFIX_TEXT_LENGTH) * UI_SCALE * 8;

  localparam SAVED_TO_START_X = (DISPLAY_WIDTH - SAVED_TO_TEXT_WIDTH) / 2;
  localparam SAVED_TO_END_X = SAVED_TO_START_X + SAVED_TO_TEXT_WIDTH;

  reg [7:0] saved_to_text[SAVED_TO_TEXT_LENGTH] = '{
      "S",
      "a",
      "v",
      "e",
      "d",
      ":",
      " ",
      "S",
      "R",
      "A",
      "M",
      "_"
  };

  reg [7:0] saved_to_suffix_text[SAVED_TO_SUFFIX_TEXT_LENGTH] = '{".", "s", "a", "v"};

  // Fail: No free slots
  localparam FAILED_EXPORT_TEXT_LENGTH = 19;
  localparam FAILED_EXPORT_TEXT_WIDTH = FAILED_EXPORT_TEXT_LENGTH * UI_SCALE * 8;

  localparam FAILED_EXPORT_START_X = (DISPLAY_WIDTH - FAILED_EXPORT_TEXT_WIDTH) / 2;
  localparam FAILED_EXPORT_END_X = FAILED_EXPORT_START_X + FAILED_EXPORT_TEXT_WIDTH;

  reg [7:0] failed_export_text[FAILED_EXPORT_TEXT_LENGTH] = '{
      "F",
      "a",
      "i",
      "l",
      ":",
      " ",
      "N",
      "o",
      " ",
      "f",
      "r",
      "e",
      "e",
      " ",
      "s",
      "l",
      "o",
      "t",
      "s"
  };

  // Comb
  reg [7:0] character = 0;

  reg [9:0] ui_start_x;
  reg [9:0] ui_end_x;

  always_comb begin
    reg [9:0] local_addr;
    character  = 0;
    ui_start_x = 0;
    ui_end_x   = 1;

    case (string_index)
      0: begin
        ui_start_x = NOT_DETECTED_START_X;
        ui_end_x   = NOT_DETECTED_END_X;

        if (character_addr < NOT_DETECTED_TEXT_LENGTH) begin
          character = not_detected_text[character_addr[4:0]];
        end
      end
      1: begin
        ui_start_x = SRAM_EXPORTING_START_X;
        ui_end_x   = SRAM_EXPORTING_END_X;

        if (character_addr < SRAM_EXPORTING_TEXT_LENGTH) begin
          character = sram_exporting_text[character_addr[4:0]];
        end
      end
      2: begin
        ui_start_x = SAVED_TO_START_X;
        ui_end_x   = SAVED_TO_END_X;

        if (character_addr < SAVED_TO_TEXT_LENGTH) begin
          character = saved_to_text[character_addr[4:0]];
        end else if (character_addr == SAVED_TO_TEXT_LENGTH) begin
          character = {4'h0, save_index_bcd[7:4]} + 8'h30;
        end else if (character_addr == SAVED_TO_TEXT_LENGTH + 1) begin
          character = {4'h0, save_index_bcd[3:0]} + 8'h30;
        end else if (character_addr < SAVED_TO_TEXT_LENGTH + SAVED_TO_NUMBER_TEXT_LENGTH + SAVED_TO_SUFFIX_TEXT_LENGTH) begin
          character = saved_to_suffix_text[character_addr[4:0] - (SAVED_TO_TEXT_LENGTH + SAVED_TO_NUMBER_TEXT_LENGTH + SAVED_TO_SUFFIX_TEXT_LENGTH)];
        end
      end
      3: begin
        ui_start_x = FAILED_EXPORT_START_X;
        ui_end_x   = FAILED_EXPORT_END_X;

        if (character_addr < FAILED_EXPORT_TEXT_LENGTH) begin
          character = failed_export_text[character_addr[4:0]];
        end
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

  assign vid_out = x_active && y_active && font_line_reg[7-char_x_pixel[X_SCALE_BIT+1:X_SCALE_BIT-1]] ? 24'hFFFFFF : 0;

  reg  [7:0] font_line_reg;
  wire [7:0] font_line;

  char_rom char_rom (
      .clk(clk),

      .character(character - 32),
      .row(character_row),
      .data_out(font_line)
  );

  always @(posedge clk) begin
    if (ce_pix) begin
      font_line_reg <= font_line;
    end
  end
endmodule
