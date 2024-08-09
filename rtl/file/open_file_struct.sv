module open_file_struct (
    input wire [8:0] address,
    input wire [7:0] path_character,

    input wire [31:0] file_size,

    output reg [7:0] q
);

  always_comb begin
    q = 0;

    if (address <= 9'hFF) begin
      q = path_character;
    end else if (address == 9'h103) begin
      // Big endian operation flags
      // Create file if it does not exist
      // Resize file to expected size
      q = 8'h3;
    end else if (address == 9'h104) begin
      q = file_size[31:24];
    end else if (address == 9'h105) begin
      q = file_size[23:16];
    end else if (address == 9'h106) begin
      q = file_size[15:8];
    end else if (address == 9'h107) begin
      q = file_size[7:0];
    end
  end

endmodule
