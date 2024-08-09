module mappers (
    input reset,

    input clk_sys,
    input clk_cart,
    input ce_cpu,
    input ce_cpu2x,
    input speed,

    input mbc1,
    input mbc1m,
    input mbc2,
    input mbc3,
    input mbc30,
    input mbc5,
    input mbc6,
    input mbc7,
    input mmm01,
    input huc1,
    input huc3,
    input gb_camera,
    input tama,
    input rocket,
    input sachen,
    input wisdom_tree,
    input mani161,

    input megaduck,

    input isGBC_game,

    input [15:0] joystick_analog_0,

    input         ce_32k,
    input  [32:0] RTC_time,
    output [31:0] RTC_timestampOut,
    output [47:0] RTC_savedtimeOut,
    output        RTC_inuse,

    input         savestate_load,
    input  [15:0] savestate_data,
    output [15:0] savestate_back,
    input  [63:0] savestate_data2,
    output [63:0] savestate_back2,

    input       has_ram,
    input [3:0] ram_mask,
    input [8:0] rom_mask,

    input [14:0] cart_addr,
    input        cart_a15,

    input [7:0] cart_mbc_type,

    input        cart_rd,
    input        cart_wr,
    input  [7:0] cart_di,
    output       cart_oe,

    input nCS,

    // input         cram_rd,
    // input  [ 7:0] cram_di,   // input from Cart RAM q
    output [7:0] cram_do,  // output to CPU
    // output [16:0] cram_addr,

    output [22:0] mbc_addr,
    // output        ram_enabled,
    output        has_battery,
    output        rumbling,

    output [7:4] cart_tran_bank0_out,
    input  [7:0] cart_tran_bank1_in,
    output [7:0] cart_tran_bank1_out,
    output       cart_tran_bank1_dir,
    output [7:0] cart_tran_bank2_out,
    output [7:0] cart_tran_bank3_out
);

  tri0 has_battery_b;
  tri0 [15:0] savestate_back_b;
  tri0 [63:0] savestate_back2_b;
  tri0 [31:0] RTC_timestampOut_b;
  tri0 [47:0] RTC_savedtimeOut_b;
  tri0 RTC_inuse_b;


  wire ce = speed ? ce_cpu2x : ce_cpu;
  wire no_mapper = ~gb_camera;
  wire no_mapper_single_bank = no_mapper & ~rom_mask[1];
  wire no_mapper_multi_bank = no_mapper & rom_mask[1];  // size > 32KB
  wire rom_override = (rocket);
  wire cart_oe_override = (mbc3 | mbc7 | huc1 | huc3 | gb_camera | tama);

  gb_camera map_gb_camera (
      .enable(gb_camera),

      .clk_sys (clk_sys),
      .clk_cart(clk_cart),
      .ce_cpu  (ce),

      .savestate_load  (savestate_load),
      .savestate_data  (savestate_data),
      .savestate_back_b(savestate_back_b),

      .ram_mask(ram_mask),
      .rom_mask(rom_mask),

      .cart_addr(cart_addr),
      .cart_a15 (cart_a15),

      .nCS(nCS),

      .cart_mbc_type(cart_mbc_type),

      .cart_rd(cart_rd),
      .cart_wr(cart_wr),
      .cart_di(cart_di),
      .cart_oe(cart_oe),

      // .cram_rd    (cram_rd),
      // .cram_di    (cram_di),
      .cram_do(cram_do),

      .mbc_addr   (mbc_addr),
      // .ram_enabled_b(ram_enabled_b),
      .has_battery_b(has_battery_b),

      .cart_tran_bank0_out(cart_tran_bank0_out),
      .cart_tran_bank1_in (cart_tran_bank1_in),
      .cart_tran_bank1_out(cart_tran_bank1_out),
      .cart_tran_bank1_dir(cart_tran_bank1_dir),
      .cart_tran_bank2_out(cart_tran_bank2_out),
      .cart_tran_bank3_out(cart_tran_bank3_out)
  );

  // assign {cram_do} = {cram_do_b};
  assign {savestate_back, savestate_back2} = {savestate_back_b, savestate_back2_b};
  assign {RTC_timestampOut, RTC_savedtimeOut, RTC_inuse} = {
    RTC_timestampOut_b, RTC_savedtimeOut_b, RTC_inuse_b
  };
  // assign {cram_wr_do, cram_wr} = {cram_wr_do_b, cram_wr_b};

  // assign mbc_addr = no_mapper_single_bank ? {8'd0, cart_addr[14:0]} : mbc_addr_b;
  // assign cram_addr = no_mapper_single_bank ? {4'd0, cart_addr[12:0]} : cram_addr_b;
  assign has_battery = no_mapper_single_bank ? (cart_mbc_type == 8'h09) : has_battery_b;
  // assign ram_enabled = no_mapper_single_bank ? has_ram : ram_enabled_b;

endmodule
