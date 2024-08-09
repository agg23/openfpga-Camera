module save_dumper (
    input wire clk_74a,

    input wire bridge_rd,
    input wire [31:0] bridge_8bit_addr,
    output reg [7:0] bridge_8bit_rd_data,

    output wire [15:0] cart_address,

    output wire [7:4] cart_tran_bank0_out,

    input  wire [7:0] cart_tran_bank1_in,
    output wire [7:0] cart_tran_bank1_out,
    output reg        cart_tran_bank1_dir
);
  assign cart_tran_bank1_out = current_bank;

  reg read_req = 0;
  reg read_ack = 0;
  reg set_bank_req = 0;
  reg [15:0] read_addr = 0;

  reg cart_read = 0;
  reg cart_write = 0;

  // CS is high when writing
  assign cart_tran_bank0_out = {1'b0, ~cart_write, ~cart_read, cart_write};

  localparam STATE_NONE = 0;
  localparam STATE_READ_REQ = 1;
  localparam STATE_READ_DATA = 2;

  reg [1:0] state = STATE_NONE;

  localparam STATE_CART_NONE = 0;
  localparam STATE_CART_READ = 1;
  localparam STATE_CART_DELAY = 2;
  localparam STATE_CART_READ_COMPLETE = 3;
  localparam STATE_CART_SET_BANK = 4;
  localparam STATE_CART_SET_BANK_COMPLETE = 5;

  reg [2:0] cart_state = STATE_CART_NONE;

  reg [7:0] current_bank = 0;

  reg [7:0] cart_delay = 0;
  reg [7:0] cart_delay_state = 0;

  reg prev_bridge_rd = 0;

  always @(posedge clk_74a) begin
    prev_bridge_rd <= bridge_rd;

    read_req <= 0;
    read_ack <= 0;
    set_bank_req <= 0;

    // Bridge state machine
    case (state)
      STATE_NONE: begin
        if (bridge_rd && ~prev_bridge_rd) begin
          state <= STATE_READ_REQ;
        end
      end
      STATE_READ_REQ: begin
        state <= STATE_READ_DATA;

        read_req <= 1;

        read_addr <= bridge_8bit_addr[15:0];
      end
      STATE_READ_DATA: begin
        if (read_ack) begin
          state <= STATE_NONE;

          if (bridge_8bit_addr[12:0] == 13'h1FFF) begin
            // End of bank, increment bank
            current_bank <= current_bank + 8'h1;

            set_bank_req <= 1;
          end
        end
      end
    endcase

    // Cart state machine
    case (cart_state)
      STATE_CART_NONE: begin
        if (read_req) begin
          cart_state <= STATE_CART_READ;
        end else if (set_bank_req) begin
          cart_state <= STATE_CART_SET_BANK;
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
        read_ack <= 1;

        bridge_8bit_rd_data <= cart_tran_bank1_in;
      end

      STATE_CART_SET_BANK: begin
        cart_state <= STATE_CART_DELAY;
        cart_delay_state <= STATE_CART_SET_BANK_COMPLETE;
        cart_delay <= 8'h3F;

        cart_write <= 1;
        cart_address <= 16'h4000;

        // Write bank data
        cart_tran_bank1_dir <= 1;
      end
      STATE_CART_SET_BANK_COMPLETE: begin
        cart_state <= STATE_CART_NONE;

        cart_write <= 0;
        cart_tran_bank1_dir <= 0;
      end
    endcase
  end

endmodule
