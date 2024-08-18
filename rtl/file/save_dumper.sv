module save_dumper (
    input wire clk_sys,

    input wire reset,

    input wire bridge_rd,
    input wire [31:0] bridge_8bit_addr,
    output reg [7:0] bridge_8bit_rd_data,

    input wire disable_camera,

    output wire [15:0] cart_address,

    output wire [7:4] cart_tran_bank0_out,

    input  wire [7:0] cart_tran_bank1_in,
    output reg  [7:0] cart_tran_bank1_out,
    output reg        cart_tran_bank1_dir
);
  reg read_req = 0;
  reg cart_ack = 0;
  reg set_bank_req = 0;
  reg disable_camera_req = 0;

  reg cart_read = 0;
  reg cart_write = 0;

  // CS is high when writing
  assign cart_tran_bank0_out = {1'b0, ~cart_write, ~cart_read, cart_write};

  localparam STATE_NONE = 0;
  localparam STATE_READ_REQ = 1;
  localparam STATE_READ_DATA = 2;
  localparam STATE_CAMERA_BANK_REQ = 3;
  localparam STATE_CAMERA_BANK_WAIT = 4;
  localparam STATE_CAMERA_DISABLE_REQ = 5;

  reg [2:0] state = STATE_NONE;

  localparam STATE_CART_NONE = 0;
  localparam STATE_CART_READ = 1;
  localparam STATE_CART_DELAY = 2;
  localparam STATE_CART_READ_COMPLETE = 3;
  localparam STATE_CART_SET_BANK = 4;
  localparam STATE_CART_DISABLE_CAMERA = 5;
  localparam STATE_CART_WRITE_COMPLETE = 6;

  reg [2:0] cart_state = STATE_CART_NONE;

  reg [7:0] current_bank = 0;

  reg [7:0] cart_delay = 0;
  reg [7:0] cart_delay_state = 0;

  reg prev_bridge_rd = 0;
  reg prev_disable_camera = 0;

  always @(posedge clk_sys) begin
    if (reset) begin
      current_bank <= 0;
    end else begin
      prev_bridge_rd <= bridge_rd;
      prev_disable_camera <= disable_camera;

      read_req <= 0;
      cart_ack <= 0;
      set_bank_req <= 0;
      disable_camera_req <= 0;

      // Bridge state machine
      case (state)
        STATE_NONE: begin
          if (bridge_rd && ~prev_bridge_rd) begin
            state <= STATE_READ_REQ;
          end else if (disable_camera && ~prev_disable_camera) begin
            state <= STATE_CAMERA_BANK_REQ;
          end
        end
        STATE_READ_REQ: begin
          state <= STATE_READ_DATA;

          read_req <= 1;
        end
        STATE_READ_DATA: begin
          if (cart_ack) begin
            state <= STATE_NONE;

            if (bridge_8bit_addr[12:0] == 13'h1FFF) begin
              // End of bank, increment bank
              current_bank <= current_bank + 8'h1;

              set_bank_req <= 1;
            end
          end
        end
        STATE_CAMERA_BANK_REQ: begin
          state <= STATE_CAMERA_BANK_WAIT;

          set_bank_req <= 1;

          current_bank <= 8'h10;
        end
        STATE_CAMERA_BANK_WAIT: begin
          if (cart_ack) begin
            state <= STATE_CAMERA_DISABLE_REQ;

            current_bank <= 0;
          end
        end
        STATE_CAMERA_DISABLE_REQ: begin
          disable_camera_req <= 1;

          state <= STATE_NONE;
        end
      endcase

      // Cart state machine
      case (cart_state)
        STATE_CART_NONE: begin
          if (read_req) begin
            cart_state <= STATE_CART_READ;
          end else if (set_bank_req) begin
            cart_state <= STATE_CART_SET_BANK;
          end else if (disable_camera_req) begin
            cart_state <= STATE_CART_DISABLE_CAMERA;
          end
        end
        STATE_CART_READ: begin
          cart_state <= STATE_CART_DELAY;
          cart_delay_state <= STATE_CART_READ_COMPLETE;
          cart_delay <= 8'hF;

          cart_read <= 1;
          cart_address <= bridge_8bit_addr[12:0] + 16'hA000;
        end
        STATE_CART_DELAY: begin
          cart_delay <= cart_delay - 8'h1;

          if (cart_delay == 8'h0) begin
            cart_state <= cart_delay_state;
          end
        end
        STATE_CART_READ_COMPLETE: begin
          cart_state <= STATE_CART_NONE;

          cart_read <= 0;
          cart_ack <= 1;

          bridge_8bit_rd_data <= cart_tran_bank1_in;
        end

        STATE_CART_SET_BANK: begin
          cart_state <= STATE_CART_DELAY;
          cart_delay_state <= STATE_CART_WRITE_COMPLETE;
          cart_delay <= 8'h3F;

          cart_write <= 1;
          cart_address <= 16'h4000;

          cart_tran_bank1_out <= current_bank;

          // Write bank data
          cart_tran_bank1_dir <= 1;
        end
        STATE_CART_DISABLE_CAMERA: begin
          // Disable the camera, if it was capturing
          cart_state <= STATE_CART_DELAY;
          cart_delay_state <= STATE_CART_WRITE_COMPLETE;
          cart_delay <= 8'h3F;

          cart_write <= 1;
          cart_address <= 16'hA000;

          // Bit 0 == 0 disables capture
          cart_tran_bank1_out <= 8'h0;

          // Write bank data
          cart_tran_bank1_dir <= 1;
        end
        STATE_CART_WRITE_COMPLETE: begin
          cart_state <= STATE_CART_NONE;

          cart_write <= 0;
          cart_tran_bank1_dir <= 0;

          cart_ack <= 1;
        end
      endcase
    end
  end

endmodule
