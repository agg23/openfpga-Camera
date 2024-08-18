module gb_camera (
    input enable,
    input reset,

    input clk_sys,
    input ce_cpu,

    input        savestate_load,
    input [15:0] savestate_data,
    inout [15:0] savestate_back_b,

    input [3:0] ram_mask,
    input [8:0] rom_mask,

    // Address in from the CPU
    input [14:0] cart_addr,
    input        cart_a15,

    input nCS,

    input [7:0] cart_mbc_type,

    input        cart_rd,
    input        cart_wr,
    // Data in from CPU
    input  [7:0] cart_di,
    output       cart_oe,

    // Cart RAM data in from the virtual cart
    // input   [7:0] cram_di,
    // Cart RAM data out from the mapper
    output [7:0] cram_do,
    // Cart RAM address out to hit the virtual cart
    // inout  [16:0] cram_addr_b,

    output [22:0] mbc_addr,

    output [7:4] cart_tran_bank0_out,
    input  [7:0] cart_tran_bank1_in,
    output [7:0] cart_tran_bank1_out,
    output       cart_tran_bank1_dir,
    output [7:0] cart_tran_bank2_out,
    output [7:0] cart_tran_bank3_out
);

  wire [15:0] savestate_back;

  assign savestate_back_b = enable ? savestate_back : 16'hZ;

  // --------------------- CPU register interface ------------------

  // clock low, ~wr, ~rd, ~cs (mreq) = wr?
  // Try substituting ~cart_a15 for nCS
  assign cart_tran_bank0_out = {clk_cart, ~cart_wr, ~cart_rd, ~cart_a15};

  assign cart_tran_bank1_out = cart_di;
  assign cram_do = cart_tran_bank1_in;
  assign cart_tran_bank1_dir = cart_wr;
  assign {cart_tran_bank2_out, cart_tran_bank3_out} = {cart_a15, cart_addr};

  reg [5:0] rom_bank_reg;

  reg [1:0] clk_divider = 0;
  reg clk_cart = 0;

  always @(posedge clk_sys) begin
    if (~enable) begin
      rom_bank_reg <= 6'd1;
    end else if (ce_cpu) begin
      clk_divider <= clk_divider + 2'h1;

      clk_cart <= clk_divider < 2'h2;

      if (cart_wr & ~cart_a15) begin
        case (cart_addr[14:13])
          2'b01: rom_bank_reg <= cart_di[5:0];  //write to ROM bank register
        endcase
      end
    end
  end

  // 0x0000-0x3FFF = Bank 0
  wire [5:0] rom_bank = (~cart_addr[14]) ? 6'd0 : rom_bank_reg;

  // mask address lines to enable proper mirroring
  wire [5:0] rom_bank_m = rom_bank & rom_mask[5:0];  //64

  assign mbc_addr = {3'b000, rom_bank_m, cart_addr[13:0]};  // 16k ROM Bank 0-63

  wire is_cram_addr = ~nCS & ~cart_addr[14];

  wire cram_rd = cart_rd & is_cram_addr;

  assign cart_oe = (cart_rd & ~cart_a15) | cram_rd;

endmodule
