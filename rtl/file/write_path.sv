module write_path (
    input wire clk,

    input  wire [6:0] save_index,
    output wire [7:0] save_index_bcd,

    input  wire [7:0] address,
    output reg  [7:0] q
);

  localparam PATH_PREFIX_LENGTH = 26;
  localparam PATH_SUFFIX_LENGTH = 4;

  reg [7:0] path_prefix[PATH_PREFIX_LENGTH] = '{
      "/",
      "S",
      "a",
      "v",
      "e",
      "s",
      "/",
      "c",
      "a",
      "m",
      "e",
      "r",
      "a",
      "/",
      "c",
      "o",
      "m",
      "m",
      "o",
      "n",
      "/",
      "S",
      "R",
      "A",
      "M",
      "_"
  };

  reg [7:0] path_suffix[PATH_SUFFIX_LENGTH] = '{".", "s", "a", "v"};

  double_bcd double_bcd (
      .binary(save_index),
      .bcd(save_index_bcd)
  );

  always @(posedge clk) begin
    q <= 0;

    if (address < PATH_PREFIX_LENGTH) begin
      q <= path_prefix[address];
    end else if (address < PATH_PREFIX_LENGTH + 2) begin
      // Save index
      case (address - PATH_PREFIX_LENGTH)
        0: begin
          q <= {4'h0, save_index_bcd[7:4]} + 8'h30;
        end
        1: begin
          q <= {4'h0, save_index_bcd[3:0]} + 8'h30;
        end
      endcase
    end else if (address < PATH_PREFIX_LENGTH + 2 + PATH_SUFFIX_LENGTH) begin
      q <= path_suffix[address-PATH_PREFIX_LENGTH+2];
    end
  end

endmodule
