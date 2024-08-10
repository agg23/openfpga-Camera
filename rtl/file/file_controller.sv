module file_controller (
    input wire clk,
    input wire reset,

    input  wire dump_sram,
    output wire dumping,

    output wire [7:0] save_index_bcd,
    output reg no_slots_error,

    // input wire bridge_rd,
    input  wire [31:0] bridge_8bit_addr,
    output wire [ 7:0] bridge_8bit_rd_data,

    output reg target_dataslot_write,
    output reg target_dataslot_openfile,
    output wire [15:0] target_dataslot_id,
    output wire [31:0] target_dataslot_slotoffset,
    output wire [31:0] target_dataslot_bridgeaddr,
    output wire [31:0] target_dataslot_length,

    input wire target_dataslot_ack,
    input wire target_dataslot_done,
    input wire [2:0] target_dataslot_err
);
  assign target_dataslot_id = 5;
  assign target_dataslot_slotoffset = 0;
  assign target_dataslot_bridgeaddr = use_path ? 32'h3000_0000 : 32'h2000_0000;
  // TODO: Grab actual size from cart
  assign target_dataslot_length = 32'h2_0000;

  assign dumping = state != STATE_NONE;

  // If set, use write_path
  reg use_path = 0;

  reg [7:0] current_save_index = 0;

  localparam STATE_NONE = 0;
  localparam STATE_START_OPENING_FILE = 1;
  localparam STATE_OPENING_FILE = 2;
  localparam STATE_EVALUATE_FILE = 3;
  localparam STATE_START_DUMP = 4;
  localparam STATE_DUMPING = 5;

  reg  [2:0] state = STATE_NONE;

  wire [7:0] path_character;

  write_path write_path (
      .clk(clk),

      .save_index(current_save_index),
      .save_index_bcd(save_index_bcd),

      .address(bridge_8bit_addr),
      .q(path_character)
  );

  open_file_struct open_file_struct (
      .address(bridge_8bit_addr),
      .path_character(path_character),

      .file_size(target_dataslot_length),

      .q(bridge_8bit_rd_data)
  );

  always @(posedge clk) begin
    if (reset) begin
      no_slots_error <= 0;
    end else begin
      use_path <= 0;

      target_dataslot_write <= 0;
      target_dataslot_openfile <= 0;

      case (state)
        STATE_NONE: begin
          if (dump_sram) begin
            state <= STATE_START_OPENING_FILE;
          end
        end
        STATE_START_OPENING_FILE: begin
          target_dataslot_openfile <= 1;
          use_path <= 1;

          if (target_dataslot_ack) begin
            state <= STATE_OPENING_FILE;
          end
        end
        STATE_OPENING_FILE: begin
          use_path <= 1;

          if (target_dataslot_done) begin
            state <= STATE_EVALUATE_FILE;
          end
        end
        STATE_EVALUATE_FILE: begin
          // Look for specifically created and opened. Normal opened is not OK, nor is any error case
          if (target_dataslot_err == 1) begin
            state <= STATE_START_DUMP;
          end else begin
            // This file slot is taken. Increment count
            state <= STATE_START_OPENING_FILE;

            current_save_index <= current_save_index + 8'h1;

            if (current_save_index == 8'd99) begin
              // If we've exhausted all the way up to 99, there are no more slots left. Fail
              no_slots_error <= 1;

              state <= STATE_NONE;
            end
          end
        end
        STATE_START_DUMP: begin
          target_dataslot_write <= 1;

          if (target_dataslot_ack) begin
            state <= STATE_DUMPING;
          end
        end
        STATE_DUMPING: begin
          if (target_dataslot_done) begin
            state <= STATE_NONE;
          end
        end
      endcase
    end
  end

endmodule
