module cart_verifier (
    input wire clk,

    input wire verification_req,

    input wire [7:0] cart_data,
    output reg cart_rd,
    output reg [15:0] cart_addr,

    output wire verifying,
    output reg  verification_complete,
    output reg  verification_passed
);

  // localparam CAMERA_TEXT_LENGTH = 13;
  // reg [7:0] camera_text[CAMERA_TEXT_LENGTH] = '{
  //     "G",
  //     "A",
  //     "M",
  //     "E",
  //     "B",
  //     "O",
  //     "Y",
  //     "C",
  //     "A",
  //     "M",
  //     "E",
  //     "R",
  //     "A"
  // };

  // localparam CAMERA_TEXT_ADDRESS = 16'h134;

  localparam CAMERA_TEXT_LENGTH = 1;
  reg [7:0] camera_text[CAMERA_TEXT_LENGTH] = '{
      // Pocket Camera
      8'hFC
  };

  localparam CAMERA_TEXT_ADDRESS = 16'h147;


  reg [7:0] read_delay;
  reg [3:0] address_offset;

  localparam STATE_NONE = 0;
  localparam STATE_READ = 1;
  localparam STATE_WAIT_READ = 2;
  localparam STATE_SUCCESS = 3;
  localparam STATE_ERROR = 4;

  reg [7:0] state = STATE_NONE;

  assign verifying = state == STATE_READ || state == STATE_WAIT_READ;

  always @(posedge clk) begin
    case (state)
      STATE_NONE: begin
        verification_complete <= 0;
        verification_passed <= 0;
        cart_rd <= 0;
        cart_addr <= 0;

        if (verification_req) begin
          state <= STATE_READ;
        end
      end
      STATE_READ: begin
        state <= STATE_WAIT_READ;

        cart_addr <= CAMERA_TEXT_ADDRESS + {12'h0, address_offset};
        cart_rd <= 1;

        read_delay <= 8'h3F;
      end
      STATE_WAIT_READ: begin
        read_delay <= read_delay - 8'h1;

        if (read_delay == 0) begin
          address_offset <= address_offset + 4'h1;

          if (cart_data == camera_text[address_offset]) begin
            // Continue processing
            if (address_offset == CAMERA_TEXT_LENGTH - 1) begin
              state <= STATE_SUCCESS;
            end else begin
              state <= STATE_READ;
            end
          end else begin
            // Failed to match
            state <= STATE_ERROR;
          end
        end
      end
      STATE_SUCCESS: begin
        verification_complete <= 1;
        verification_passed <= 1;

        cart_rd <= 0;
      end
      STATE_ERROR: begin
        verification_complete <= 1;

        cart_rd <= 0;
      end
    endcase
  end

endmodule
