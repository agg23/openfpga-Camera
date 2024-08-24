`default_nettype none

`define isgbc 1

module core_top (

  //
  // physical connections
  //

  ///////////////////////////////////////////////////
  // clock inputs 74.25mhz. not phase aligned, so treat these domains as asynchronous

  input   wire            clk_74a, // mainclk1
  input   wire            clk_74b, // mainclk1 

  ///////////////////////////////////////////////////
  // cartridge interface
  // switches between 3.3v and 5v mechanically
  // output enable for multibit translators controlled by pic32

  // GBA AD[15:8]
  inout   wire    [7:0]   cart_tran_bank2,
  output  wire            cart_tran_bank2_dir,

  // GBA AD[7:0]
  inout   wire    [7:0]   cart_tran_bank3,
  output  wire            cart_tran_bank3_dir,

  // GBA A[23:16]
  inout   wire    [7:0]   cart_tran_bank1,
  output  reg             cart_tran_bank1_dir,

  // GBA [7] PHI#
  // GBA [6] WR#
  // GBA [5] RD#
  // GBA [4] CS1#/CS#
  //     [3:0] unwired
  inout   wire    [7:4]   cart_tran_bank0,
  output  wire            cart_tran_bank0_dir,

  // GBA CS2#/RES#
  inout   wire            cart_tran_pin30,
  output  wire            cart_tran_pin30_dir,
  // when GBC cart is inserted, this signal when low or weak will pull GBC /RES low with a special circuit
  // the goal is that when unconfigured, the FPGA weak pullups won't interfere.
  // thus, if GBC cart is inserted, FPGA must drive this high in order to let the level translators
  // and general IO drive this pin.
  output  wire            cart_pin30_pwroff_reset,

  // GBA IRQ/DRQ
  inout   wire            cart_tran_pin31,
  output  wire            cart_tran_pin31_dir,

  // infrared
  input   wire            port_ir_rx,
  output  wire            port_ir_tx,
  output  wire            port_ir_rx_disable, 

  // GBA link port
  inout   wire            port_tran_si,
  output  wire            port_tran_si_dir,
  inout   wire            port_tran_so,
  output  wire            port_tran_so_dir,
  inout   wire            port_tran_sck,
  output  wire            port_tran_sck_dir,
  inout   wire            port_tran_sd,
  output  wire            port_tran_sd_dir,
   
  ///////////////////////////////////////////////////
  // cellular psram 0 and 1, two chips (64mbit x2 dual die per chip)

  output  wire    [21:16] cram0_a,
  inout   wire    [15:0]  cram0_dq,
  input   wire            cram0_wait,
  output  wire            cram0_clk,
  output  wire            cram0_adv_n,
  output  wire            cram0_cre,
  output  wire            cram0_ce0_n,
  output  wire            cram0_ce1_n,
  output  wire            cram0_oe_n,
  output  wire            cram0_we_n,
  output  wire            cram0_ub_n,
  output  wire            cram0_lb_n,

  output  wire    [21:16] cram1_a,
  inout   wire    [15:0]  cram1_dq,
  input   wire            cram1_wait,
  output  wire            cram1_clk,
  output  wire            cram1_adv_n,
  output  wire            cram1_cre,
  output  wire            cram1_ce0_n,
  output  wire            cram1_ce1_n,
  output  wire            cram1_oe_n,
  output  wire            cram1_we_n,
  output  wire            cram1_ub_n,
  output  wire            cram1_lb_n,

  ///////////////////////////////////////////////////
  // sdram, 512mbit 16bit

  output  wire    [12:0]  dram_a,
  output  wire    [1:0]   dram_ba,
  inout   wire    [15:0]  dram_dq,
  output  wire    [1:0]   dram_dqm,
  output  wire            dram_clk,
  output  wire            dram_cke,
  output  wire            dram_ras_n,
  output  wire            dram_cas_n,
  output  wire            dram_we_n,

  ///////////////////////////////////////////////////
  // sram, 1mbit 16bit

  output  wire    [16:0]  sram_a,
  inout   wire    [15:0]  sram_dq,
  output  wire            sram_oe_n,
  output  wire            sram_we_n,
  output  wire            sram_ub_n,
  output  wire            sram_lb_n,

  ///////////////////////////////////////////////////
  // vblank driven by dock for sync in a certain mode

  input   wire            vblank,

  ///////////////////////////////////////////////////
  // i/o to 6515D breakout usb uart

  output  wire            dbg_tx,
  input   wire            dbg_rx,

  ///////////////////////////////////////////////////
  // i/o pads near jtag connector user can solder to

  output  wire            user1,
  input   wire            user2,

  ///////////////////////////////////////////////////
  // RFU internal i2c bus 

  inout   wire            aux_sda,
  output  wire            aux_scl,

  ///////////////////////////////////////////////////
  // RFU, do not use
  output  wire            vpll_feed,

  //
  // logical connections
  //

  ///////////////////////////////////////////////////
  // video, audio output to scaler
  output  wire    [23:0]  video_rgb,
  output  wire            video_rgb_clock,
  output  wire            video_rgb_clock_90,
  output  wire            video_de,
  output  wire            video_skip,
  output  wire            video_vs,
  output  wire            video_hs,
      
  output  wire            audio_mclk,
  input   wire            audio_adc,
  output  wire            audio_dac,
  output  wire            audio_lrck,

  ///////////////////////////////////////////////////
  // bridge bus connection
  // synchronous to clk_74a
  output  wire            bridge_endian_little,
  input   wire    [31:0]  bridge_addr,
  input   wire            bridge_rd,
  output  reg     [31:0]  bridge_rd_data,
  input   wire            bridge_wr,
  input   wire    [31:0]  bridge_wr_data,

  ///////////////////////////////////////////////////
  // controller data
  // 
  // key bitmap:
  //   [0]    dpad_up
  //   [1]    dpad_down
  //   [2]    dpad_left
  //   [3]    dpad_right
  //   [4]    face_a
  //   [5]    face_b
  //   [6]    face_x
  //   [7]    face_y
  //   [8]    trig_l1
  //   [9]    trig_r1
  //   [10]   trig_l2
  //   [11]   trig_r2
  //   [12]   trig_l3
  //   [13]   trig_r3
  //   [14]   face_select
  //   [15]   face_start
  //   [31:28] type
  // joy values - unsigned
  //   [ 7: 0] lstick_x
  //   [15: 8] lstick_y
  //   [23:16] rstick_x
  //   [31:24] rstick_y
  // trigger values - unsigned
  //   [ 7: 0] ltrig
  //   [15: 8] rtrig
  //
  input   wire    [31:0]  cont1_key,
  input   wire    [31:0]  cont2_key,
  input   wire    [31:0]  cont3_key,
  input   wire    [31:0]  cont4_key,
  input   wire    [31:0]  cont1_joy,
  input   wire    [31:0]  cont2_joy,
  input   wire    [31:0]  cont3_joy,
  input   wire    [31:0]  cont4_joy,
  input   wire    [15:0]  cont1_trig,
  input   wire    [15:0]  cont2_trig,
  input   wire    [15:0]  cont3_trig,
  input   wire    [15:0]  cont4_trig
);

// not using the IR port, so turn off both the LED, and
// disable the receive circuit to save power
assign port_ir_tx = 0;
assign port_ir_rx_disable = 1;

// bridge endianness
assign bridge_endian_little = 0;

// cart is unused, so set all level translators accordingly
// directions are 0:IN, 1:OUT
// assign cart_tran_bank3[7:2]    = 6'hzz;
// assign cart_tran_bank3[0]      = 1'hz;
// assign cart_tran_bank2         = 8'hzz;
// assign cart_tran_bank1         = 8'hzz;

assign cart_tran_bank0_dir     = 1'b1; // Output control flags
assign cart_tran_bank2_dir     = 1'b1; // Output addresses
assign cart_tran_bank3_dir     = 1'b1; // Output addresses

assign cart_tran_pin30         = 1'bZ; // Set ~reset high
assign cart_tran_pin30_dir     = 1'bZ;
assign cart_pin30_pwroff_reset = reset_n_s ? 1'b1 : 1'bZ;  // Enable GBC cart access

assign cart_tran_pin31         = 1'bz;      // input
assign cart_tran_pin31_dir     = 1'b0;  // input

assign port_tran_sd     = 1'bz;
assign port_tran_sd_dir = 1'b0;     // SD is input and not used
assign video_skip       = 1'b0;

// tie off the rest of the pins we are not using
assign cram0_a     = 'h0;
assign cram0_dq    = {16{1'bZ}};
assign cram0_clk   = 0;
assign cram0_adv_n = 1;
assign cram0_cre   = 0;
assign cram0_ce0_n = 1;
assign cram0_ce1_n = 1;
assign cram0_oe_n  = 1;
assign cram0_we_n  = 1;
assign cram0_ub_n  = 1;
assign cram0_lb_n  = 1;

assign cram1_a     = 'h0;
assign cram1_dq    = {16{1'bZ}};
assign cram1_clk   = 0;
assign cram1_adv_n = 1;
assign cram1_cre   = 0;
assign cram1_ce0_n = 1;
assign cram1_ce1_n = 1;
assign cram1_oe_n  = 1;
assign cram1_we_n  = 1;
assign cram1_ub_n  = 1;
assign cram1_lb_n  = 1;

assign sram_a      = 'h0;
assign sram_dq     = {16{1'bZ}};
assign sram_oe_n   = 1;
assign sram_we_n   = 1;
assign sram_ub_n   = 1;
assign sram_lb_n   = 1;

assign dbg_tx      = 1'bZ;
assign user1       = 1'bZ;
assign aux_scl     = 1'bZ;
assign vpll_feed   = 1'bZ;

//
// host/target command handler
//
wire            reset_n;                // driven by host commands, can be used as core-wide reset
wire    [31:0]  cmd_bridge_rd_data;
    
// bridge host commands
// synchronous to clk_74a
wire            status_boot_done  = pll_core_locked_s; 
wire            status_setup_done = pll_core_locked_s; // rising edge triggers a target command
wire            status_running    = reset_n; // we are running as soon as reset_n goes high

wire            dataslot_requestread;
wire    [15:0]  dataslot_requestread_id;
wire            dataslot_requestread_ack = 1;
wire            dataslot_requestread_ok  = 1;

wire            dataslot_requestwrite;
wire    [15:0]  dataslot_requestwrite_id;
wire    [31:0]  dataslot_requestwrite_size;
wire            dataslot_requestwrite_ack = 1;
wire            dataslot_requestwrite_ok  = 1;

wire            dataslot_update;
wire    [15:0]  dataslot_update_id;
wire    [31:0]  dataslot_update_size;

wire            dataslot_allcomplete;

wire     [31:0] rtc_epoch_seconds;
wire     [31:0] rtc_date_bcd;
wire     [31:0] rtc_time_bcd;
wire            rtc_valid;

wire            savestate_supported;
wire    [31:0]  savestate_addr;
wire    [31:0]  savestate_size;
wire    [31:0]  savestate_maxloadsize;

wire            savestate_start;
wire            savestate_start_ack;
wire            savestate_start_busy;
wire            savestate_start_ok;
wire            savestate_start_err;

wire            savestate_load;
wire            savestate_load_ack;
wire            savestate_load_busy;
wire            savestate_load_ok;
wire            savestate_load_err;

wire            osnotify_inmenu;

// bridge target commands
// synchronous to clk_74a

reg             target_dataslot_read;       
wire            target_dataslot_write;
wire            target_dataslot_write_s;
reg             target_dataslot_getfile;    // require additional param/resp structs to be mapped
wire            target_dataslot_openfile;   // require additional param/resp structs to be mapped
wire            target_dataslot_openfile_s;

wire            target_dataslot_ack;
wire            target_dataslot_ack_s;        
wire            target_dataslot_done;
wire            target_dataslot_done_s;
wire    [2:0]   target_dataslot_err;
wire    [2:0]   target_dataslot_err_s;

wire    [15:0]  target_dataslot_id;
wire    [15:0]  target_dataslot_id_s;
wire    [31:0]  target_dataslot_slotoffset;
wire    [31:0]  target_dataslot_slotoffset_s;
wire    [31:0]  target_dataslot_bridgeaddr;
wire    [31:0]  target_dataslot_bridgeaddr_s;
wire    [31:0]  target_dataslot_length;
wire    [31:0]  target_dataslot_length_s;
    
// bridge data slot access
// synchronous to clk_74a

wire    [9:0]   datatable_addr;
wire            datatable_wren;
wire    [31:0]  datatable_data;
wire    [31:0]  datatable_q;

wire            bw_en;

core_bridge_cmd icb (
  .clk                        ( clk_74a                    ),
  .reset_n                    ( reset_n                    ),

  .bridge_endian_little       ( bridge_endian_little       ),
  .bridge_addr                ( bridge_addr                ),
  .bridge_rd                  ( bridge_rd                  ),
  .bridge_rd_data             ( cmd_bridge_rd_data         ),
  .bridge_wr                  ( bridge_wr                  ),
  .bridge_wr_data             ( bridge_wr_data             ),
  
  .status_boot_done           ( status_boot_done           ),
  .status_setup_done          ( status_setup_done          ),
  .status_running             ( status_running             ),

  .dataslot_requestread       ( dataslot_requestread       ),
  .dataslot_requestread_id    ( dataslot_requestread_id    ),
  .dataslot_requestread_ack   ( dataslot_requestread_ack   ),
  .dataslot_requestread_ok    ( dataslot_requestread_ok    ),

  .dataslot_requestwrite      ( dataslot_requestwrite      ),
  .dataslot_requestwrite_id   ( dataslot_requestwrite_id   ),
  .dataslot_requestwrite_size ( dataslot_requestwrite_size ),
  .dataslot_requestwrite_ack  ( dataslot_requestwrite_ack  ),
  .dataslot_requestwrite_ok   ( dataslot_requestwrite_ok   ),

  .dataslot_update            ( dataslot_update            ),
  .dataslot_update_id         ( dataslot_update_id         ),
  .dataslot_update_size       ( dataslot_update_size       ),
  
  .dataslot_allcomplete       ( dataslot_allcomplete       ),

  .rtc_epoch_seconds          ( rtc_epoch_seconds          ),
  .rtc_date_bcd               ( rtc_date_bcd               ),
  .rtc_time_bcd               ( rtc_time_bcd               ),
  .rtc_valid                  ( rtc_valid                  ),
  
  .savestate_supported        ( savestate_supported        ),
  .savestate_addr             ( savestate_addr             ),
  .savestate_size             ( savestate_size             ),
  .savestate_maxloadsize      ( savestate_maxloadsize      ),

  .savestate_start            ( savestate_start            ),
  .savestate_start_ack        ( savestate_start_ack        ),
  .savestate_start_busy       ( savestate_start_busy       ),
  .savestate_start_ok         ( savestate_start_ok         ),
  .savestate_start_err        ( savestate_start_err        ),

  .savestate_load             ( savestate_load             ),
  .savestate_load_ack         ( savestate_load_ack         ),
  .savestate_load_busy        ( savestate_load_busy        ),
  .savestate_load_ok          ( savestate_load_ok          ),
  .savestate_load_err         ( savestate_load_err         ),

  .osnotify_inmenu            ( osnotify_inmenu            ),
  
  .target_dataslot_read       ( target_dataslot_read       ),
  .target_dataslot_write      ( target_dataslot_write      ),
  .target_dataslot_getfile    ( target_dataslot_getfile    ),
  .target_dataslot_openfile   ( target_dataslot_openfile   ),
  
  .target_dataslot_ack        ( target_dataslot_ack        ),
  .target_dataslot_done       ( target_dataslot_done       ),
  .target_dataslot_err        ( target_dataslot_err        ),

  .target_dataslot_id         ( target_dataslot_id         ),
  .target_dataslot_slotoffset ( target_dataslot_slotoffset ),
  .target_dataslot_bridgeaddr ( target_dataslot_bridgeaddr ),
  .target_dataslot_length     ( target_dataslot_length     ),

  .target_buffer_param_struct ( target_dataslot_bridgeaddr ),
  .target_buffer_resp_struct  ( target_dataslot_bridgeaddr ),
  
  .datatable_addr             ( datatable_addr             ),
  .datatable_wren             ( datatable_wren             ),
  .datatable_data             ( datatable_data             ),
  .datatable_q                ( datatable_q                ),

  .bw_en                      ( bw_en                      )

);

//! ------------------------------------------------------------------------
//! Reset Handler (Thanks boogerman!)
//! ------------------------------------------------------------------------
reg  [31:0] reset_counter;
reg         reset_timer;
reg         core_reset   = 0;

always_ff @(posedge clk_74a) begin
  if(reset_timer) begin
    reset_counter <= 32'd8000;
    core_reset    <= 0;
  end
  else begin
    if (reset_counter == 32'h0) begin
      core_reset <= 0;
    end
    else begin
      reset_counter <= reset_counter - 1;
      core_reset    <= 1;
    end
  end
end

// for bridge write data, we just broadcast it to all bus devices
// for bridge read data, we have to mux it
// add your own devices here
always_comb begin
  casex(bridge_addr)
    32'h2xxxxxxx: begin bridge_rd_data = dump_bridge_rd_data;  end
    32'h3xxxxxxx: begin bridge_rd_data = dump_bridge_rd_data;  end
    32'hF8xxxxxx: begin bridge_rd_data = cmd_bridge_rd_data;   end
    32'hF1000000: begin bridge_rd_data = int_bridge_read_data; end
    32'hF2000000: begin bridge_rd_data = int_bridge_read_data; end
    default:      begin bridge_rd_data = 0;                    end
  endcase
end

reg [31:0] boot_settings = 32'h0;
reg [31:0] run_settings  = 32'h0;
logic [31:0] int_bridge_read_data;

reg core_settings_dump = 0;

always_ff @(posedge clk_74a) begin
  reset_timer <= 0; //! Always default this to zero
  core_settings_dump <= 0;

  if(bridge_wr) begin
    case (bridge_addr)
      32'hF0000000: begin /*         RESET ONLY          */ reset_timer <= 1; end //! Reset Core Command
      32'hF0000004: begin
        core_settings_dump <= 1;
      end
      32'hF1000000: begin boot_settings  <= bridge_wr_data; reset_timer <= 1; end //! System Settings
      32'hF2000000: begin run_settings   <= bridge_wr_data;                   end //! Runtime settings
    endcase
  end

  if(bridge_rd) begin
    case (bridge_addr)
      32'hF1000000: begin int_bridge_read_data  <= boot_settings;  end //! System Settings
      32'hF2000000: begin int_bridge_read_data  <= run_settings;   end //! Runtime settings
    endcase
  end
end

logic clk_sys, clk_ram, clk_ram_90, clk_vid, clk_vid_90;
logic pll_core_locked, pll_core_locked_s, reset_n_s, reset_n_s74, external_reset_s;
logic [31:0] cont1_key_s, cont2_key_s, cont3_key_s, cont4_key_s;
logic [31:0] boot_settings_s, run_settings_s;

synch_3               s01 (pll_core_locked, pll_core_locked_s,  clk_ram);
synch_3               s02_74 (reset_n,         reset_n_s74,        clk_74a);
synch_3               s02 (reset_n,         reset_n_s,          clk_sys);
synch_3               s03 (core_reset,      external_reset_s,   clk_sys);
synch_3 #(.WIDTH(32)) s04 (cont1_key,       cont1_key_s,        clk_sys);
synch_3 #(.WIDTH(32)) s05 (cont2_key,       cont2_key_s,        clk_sys);
synch_3 #(.WIDTH(32)) s06 (cont3_key,       cont3_key_s,        clk_sys);
synch_3 #(.WIDTH(32)) s07 (cont4_key,       cont4_key_s,        clk_sys);
synch_3 #(.WIDTH(32)) s08 (boot_settings,   boot_settings_s,    clk_sys);
synch_3 #(.WIDTH(32)) s09 (run_settings,    run_settings_s,     clk_sys);

logic sgb_en, rumble_en, originalcolors, ff_snd_en, ff_en, sgb_border_en, gba_en;
logic [1:0] tint;

always_comb begin
  // These settings trigger a reset
  sgb_en         = boot_settings_s[0];
  gba_en         = boot_settings_s[1];

  // These settings don't
  rumble_en      = run_settings_s[0];
  originalcolors = run_settings_s[1];
  ff_snd_en      = run_settings_s[2];
  ff_en          = run_settings_s[3];
  sgb_border_en  = run_settings_s[4];
  tint           = run_settings_s[6:5];
end

mf_pllbase mp1
(
  .refclk   ( clk_74a         ),
  .rst      ( 0               ),
  
  .outclk_0 ( clk_ram         ),
  .outclk_1 ( clk_sys         ),
  .outclk_2 ( clk_vid         ),
  .outclk_3 ( clk_vid_90      ),
  
  .locked   ( pll_core_locked )
);

data_loader #(
  .ADDRESS_MASK_UPPER_4   ( 4'h1  ),
  .OUTPUT_WORD_SIZE       ( 2     ),
  .WRITE_MEM_CLOCK_DELAY  ( 20    )
) data_loader (
  .clk_74a              ( clk_74a               ),
  .clk_memory           ( clk_sys               ),

  .bridge_wr            ( bridge_wr             ),
  .bridge_endian_little ( bridge_endian_little  ),
  .bridge_addr          ( bridge_addr           ),
  .bridge_wr_data       ( bridge_wr_data        ),

  .write_en             ( ioctl_wr              ),
  .write_addr           ( ioctl_addr            ),
  .write_data           ( ioctl_dout            )
);

//////// Control ////////

wire verifier_cart_rd;
wire [15:0] verifier_cart_addr;

wire verifying;
wire verification_complete;
wire verification_passed;

reg prev_reset_n = 0;

reg [22:0] verification_req_delay = 0;

always @(posedge clk_sys) begin
  prev_reset_n <= reset_n_s;

  if (reset_n_s && ~prev_reset_n) begin
    verification_req_delay <= 23'h7F_FFFF;
  end

  if (verification_req_delay > 0) begin
    verification_req_delay <= verification_req_delay - 23'h1;
  end
end

cart_verifier cart_verifier (
  .clk(clk_sys),

  .verification_req(verification_req_delay == 23'h1),

  .cart_data(cart_tran_bank1_reg_in),
  .cart_rd(verifier_cart_rd),
  .cart_addr(verifier_cart_addr),

  .verifying(verifying),
  .verification_complete(verification_complete),
  .verification_passed(verification_passed)
);

wire [3:0] gb_cart_tran_bank0_out;
wire [7:0] gb_cart_tran_bank1_out;
wire [7:0] gb_cart_tran_bank2_out;
wire [7:0] gb_cart_tran_bank3_out;

wire gb_cart_tran_bank1_dir;

// wire [7:0] gb_cart_tran_bank1_drive, dump_cart_tran_bank1_drive;

// assign gb_cart_tran_bank1_drive = ~dumping ? gb_cart_tran_bank1 : 8'hZZ;
// assign dump_cart_tran_bank1_drive = dumping ? dump_cart_tran_bank1 : 8'hZZ;

wire [3:0] cart_control = dumping ? dump_cart_tran_bank0_out : verifying ? {2'b01, ~verifier_cart_rd, 1'b0} : gb_cart_tran_bank0_out;

reg cart_wr_n_delay = 1;
reg cart_wr_n = 1;

reg gb_cart_tran_bank1_dir_delay = 0;

reg [7:4] cart_tran_bank0_reg_in;
reg [7:0] cart_tran_bank1_reg_in;
reg [7:0] cart_tran_bank2_reg_in;
reg [7:0] cart_tran_bank3_reg_in;

reg [7:4] cart_tran_bank0_reg_out;
reg [7:0] cart_tran_bank1_reg_out;
reg [7:0] cart_tran_bank2_reg_out;
reg [7:0] cart_tran_bank3_reg_out;

always @(posedge clk_sys) begin
  cart_wr_n_delay <= cart_control[2];
  cart_wr_n <= cart_wr_n_delay;

  gb_cart_tran_bank1_dir_delay <= gb_cart_tran_bank1_dir;
end

always @(posedge clk_ram) begin
  // If main wr_n control signal is high, always be high, otherwise, go low after a delay
  cart_tran_bank0_reg_out <= {cart_control[3], cart_control[2] | cart_wr_n, cart_control[1], cart_control[0]};
  cart_tran_bank1_reg_out <= dumping ? dump_cart_tran_bank1_out : gb_cart_tran_bank1_out;
  cart_tran_bank2_reg_out <= dumping ? dump_cart_addr[15:8] : verifying ? verifier_cart_addr[15:8] : gb_cart_tran_bank2_out;
  cart_tran_bank3_reg_out <= dumping ? dump_cart_addr[7:0] : verifying ? verifier_cart_addr[7:0] : gb_cart_tran_bank3_out;

  cart_tran_bank0_reg_in <= cart_tran_bank0;
  cart_tran_bank1_reg_in <= cart_tran_bank1;
  cart_tran_bank2_reg_in <= cart_tran_bank2;
  cart_tran_bank3_reg_in <= cart_tran_bank3;

  // Keep bank from swapping directions until a tick after the control signal actually changes
  cart_tran_bank1_dir <= dumping ? dump_cart_tran_bank1_dir : ~verifying && (gb_cart_tran_bank1_dir || gb_cart_tran_bank1_dir_delay);
end

assign cart_tran_bank0 = cart_tran_bank0_dir ? cart_tran_bank0_reg_out : 4'hZ;
assign cart_tran_bank1 = cart_tran_bank1_dir ? cart_tran_bank1_reg_out : 8'hZZ;
assign cart_tran_bank2 = cart_tran_bank2_dir ? cart_tran_bank2_reg_out : 8'hZZ;
assign cart_tran_bank3 = cart_tran_bank3_dir ? cart_tran_bank3_reg_out : 8'hZZ;

wire no_slots_error;
wire display_cart_error = verification_complete && ~verification_passed;
wire display_ui = display_cart_error || dumping || ui_timer > 0;

//////// SRAM Dumping ////////

wire dumping;

wire [15:0] dump_cart_addr;
wire [7:0] dump_cart_tran_bank0_out;
wire [7:0] dump_cart_tran_bank1_out;
wire dump_cart_tran_bank1_dir;

wire [31:0] dump_bridge_rd_data;

wire dump_rd;
wire [15:0] dump_addr;
wire [7:0] dump_data;
wire [7:0] path_data;

wire is_dump_address = bridge_addr[31:28] == 4'h2;
wire is_path_address = bridge_addr[31:28] == 4'h3;

data_unloader #(
  .ADDRESS_SIZE         (16),
  .READ_MEM_CLOCK_DELAY (34),
  .INPUT_WORD_SIZE      (1)
) dump_data_unloader (
  .clk_74a              (clk_74a),
  .clk_memory           (clk_sys),

  .bridge_rd            (bridge_rd),
  .bridge_endian_little (bridge_endian_little),
  .bridge_addr          (bridge_addr),
  .bridge_rd_data       (dump_bridge_rd_data),

  .active_address       (is_dump_address || is_path_address),

  .read_en              (dump_rd),
  .read_addr            (dump_addr),
  .read_data            (is_dump_address ? dump_data : path_data)
);

save_dumper save_dumper (
  .clk_sys(clk_sys),

  .reset(~dumping && prev_dumping),

  .bridge_rd(dump_rd && is_dump_address),
  .bridge_8bit_addr({16'h0, dump_addr}),
  .bridge_8bit_rd_data(dump_data),

  .disable_camera(dumping),

  .cart_address(dump_cart_addr),

  .cart_tran_bank0_out(dump_cart_tran_bank0_out),

  .cart_tran_bank1_in(cart_tran_bank1_reg_in),
  .cart_tran_bank1_out(dump_cart_tran_bank1_out),
  .cart_tran_bank1_dir(dump_cart_tran_bank1_dir)
);

wire [7:0] save_index_bcd;

reg [2:0] dump_sram_counter = 0;
wire dump_sram = dump_sram_counter > 0;

wire dump_sram_s;

synch_3 file_controller_sync (dump_sram, dump_sram_s, clk_sys);

file_controller file_controller (
  .clk(clk_sys),
  .reset(~reset_n_s),

  .dump_sram(dump_sram_s),
  .dumping(dumping),

  .save_index_bcd(save_index_bcd),
  .no_slots_error(no_slots_error),

  .bridge_8bit_addr({16'h0, dump_addr}),
  .bridge_8bit_rd_data(path_data),

  .target_dataslot_write(target_dataslot_write_s),
  .target_dataslot_openfile(target_dataslot_openfile_s),
  .target_dataslot_id(target_dataslot_id_s),
  .target_dataslot_slotoffset(target_dataslot_slotoffset_s),
  .target_dataslot_bridgeaddr(target_dataslot_bridgeaddr_s),
  .target_dataslot_length(target_dataslot_length_s),

  .target_dataslot_ack(target_dataslot_ack_s),
  .target_dataslot_done(target_dataslot_done_s),
  .target_dataslot_err(target_dataslot_err_s)
);

synch_3 #(.WIDTH(114)) file_bridge_out_sync (
  {target_dataslot_write_s, target_dataslot_openfile_s, target_dataslot_id_s, target_dataslot_slotoffset_s, target_dataslot_bridgeaddr_s, target_dataslot_length_s},
  {target_dataslot_write, target_dataslot_openfile, target_dataslot_id, target_dataslot_slotoffset, target_dataslot_bridgeaddr, target_dataslot_length},
  clk_74a
);

synch_3 #(.WIDTH(5)) file_bridge_in_sync (
  {target_dataslot_ack, target_dataslot_done, target_dataslot_err},
  {target_dataslot_ack_s, target_dataslot_done_s, target_dataslot_err_s},
  clk_sys
);

wire dumping_s;
wire display_cart_error_s;

synch_3 #(.WIDTH(2)) control_sync (
  {display_cart_error, dumping},
  {display_cart_error_s, dumping_s},
  clk_74a
);

wire both_triggers = cont1_key[8] && cont1_key[9];

reg prev_both_triggers = 0;
reg released_all_triggers_since_last_dump = 1;

always @(posedge clk_74a) begin
  prev_both_triggers <= both_triggers;

  if (dump_sram_counter > 0) begin
    dump_sram_counter <= dump_sram_counter - 3'h1;
  end

  if (~cont1_key[8] && ~cont1_key[9]) begin
    // Triggers are not pressed
    released_all_triggers_since_last_dump <= 1;
  end

  if (~dumping_s && ~display_cart_error_s) begin
    // Only dump if both triggers are pressed, or Core Settings button is pressed
    if ((released_all_triggers_since_last_dump && ~prev_both_triggers && both_triggers) || core_settings_dump) begin
      dump_sram_counter <= 3'h7;
      released_all_triggers_since_last_dump <= 0;
    end
  end
end

reg [27:0] ui_timer = 0;
reg prev_dumping = 0;
reg prev_no_slots_error = 0;

always @(posedge clk_sys) begin
  prev_dumping <= dumping;
  prev_no_slots_error <= no_slots_error;

  if (ui_timer > 0) begin
    ui_timer <= ui_timer - 27'h1;
  end

  if ((~dumping && prev_dumping) || (no_slots_error && ~prev_no_slots_error)) begin
    ui_timer <= 28'hFFF_FFFF;
  end
end

//////// Start GB/GBC Stuff ////////

reg ioctl_download = 0;

always_ff @(posedge clk_74a) begin
  if      (dataslot_requestwrite) ioctl_download <= 1;
  else if (dataslot_allcomplete)  ioctl_download <= 0;
end

logic [14:0] cart_addr;
logic [22:0] mbc_addr;
logic cart_a15, cart_rd, cart_wr, cart_oe, nCS;
logic [7:0] cart_di, cart_do;
logic        ioctl_wr, ioctl_wait;
logic [24:0] ioctl_addr;
logic [15:0] ioctl_dout;
logic boot_download, cart_download, palette_download, sgb_border_download, cgb_boot_download, dmg_boot_download, sgb_boot_download;

always_comb begin
  cart_download       = 0;
  sgb_border_download = 0;
  palette_download    = 0;
  cgb_boot_download   = 0;
  dmg_boot_download   = 0;
  sgb_boot_download   = 0;

  if(ioctl_download) begin
    case (dataslot_requestwrite_id)
      1: begin cart_download        = 1'b1; end
      2: begin sgb_border_download  = 1'b1; end
      3: begin palette_download     = 1'b1; end
      4: begin cgb_boot_download    = 1'b1; end
      5: begin dmg_boot_download    = 1'b1; end
      6: begin sgb_boot_download    = 1'b1; end
    endcase
  end

  boot_download  = cgb_boot_download | dmg_boot_download | sgb_boot_download;
end

reg isGBC     = `isgbc;

wire  [1:0] sdram_ds     =  cart_download ? 2'b11 : {mbc_addr[0], ~mbc_addr[0]};
wire [15:0] sdram_do;
wire [15:0] sdram_di     =  cart_download ? ioctl_dout : 16'd0;
wire [23:0] sdram_addr   =  cart_download ? ioctl_addr[24:1] : {2'b00, mbc_addr[22:1]};
wire sdram_oe            = ~cart_download & cart_rd & ~cram_rd;
wire sdram_we            =  cart_download & dn_write;
wire sdram_refresh_force;
wire sdram_autorefresh   = !ff_on;

assign dram_cke = 1;

sdram sdram (
 // interface to the MT48LC16M16 chip
 .sd_data        ( dram_dq                ),
 .sd_addr        ( dram_a                 ),
 .sd_dqm         ( dram_dqm               ),
 .sd_cs          (                        ),
 .sd_ba          ( dram_ba                ),
 .sd_we          ( dram_we_n              ),
 .sd_ras         ( dram_ras_n             ),
 .sd_cas         ( dram_cas_n             ),
 .sd_clk         ( dram_clk               ),

  // system interface
 .clk            ( clk_ram                ),
 .sync           ( ce_cpu2x               ),
 .init           ( ~pll_core_locked_s     ),

 // cpu interface
 .din            ( sdram_di               ),
 .addr           ( sdram_addr             ),
 .ds             ( sdram_ds               ),
 .we             ( sdram_we               ),
 .oe             ( sdram_oe               ),
 .autorefresh    ( sdram_autorefresh      ),
 .refresh        ( sdram_refresh_force    ),
 .dout           ( sdram_do               )
);

wire dn_write;
wire cart_ready;
wire cram_rd;
wire [7:0] rom_do = (mbc_addr[0]) ? sdram_do[15:8] : sdram_do[7:0];
wire [7:0] ram_mask_file, cart_ram_size;
wire isGBC_game, isSGB_game;
wire cart_has_save;
wire [31:0] RTC_timestampOut;
wire [47:0] RTC_savedtimeOut;
wire rumbling;
wire RTC_inuse;

// rumbler rumbler_module
// (
//   .clk          ( clk_sys             ),
//   .reset        ( reset               ),
//   .rumble_en    ( rumble_en           ),
//   .rumbling     ( rumbling            ),
//   .cart_wr      ( cart_tran_bank0[6]  ),
//   .cart_rumble  ( cart_tran_bank3[1]  )
// );

reg ce_32k; // 32768Hz clock for RTC
reg [9:0] ce_32k_div;
always_ff @(posedge clk_sys) begin
  ce_32k_div  <=  ce_32k_div + 1'b1;
  ce_32k      <= !ce_32k_div;
end

logic [32:0] rtc_data_s;

sync_fifo #(
  .WIDTH ( 33 )
) RTC_FIFO(
  .clk_write  ( clk_74a                         ),
  .clk_read   ( clk_sys                         ),

  .write_en   ( rtc_valid                       ),
  .data       ( {rtc_valid, rtc_epoch_seconds}  ),
  .data_s     ( rtc_data_s                      ),
  .write_en_s (                                 )
);

cart_top cart
(
  .reset                      ( reset             ),

  .clk_sys                    ( clk_sys           ),
  .ce_cpu                     ( ce_cpu            ),
  .ce_cpu2x                   ( ce_cpu2x          ),
  .speed                      ( speed             ),
  .megaduck                   ( 0                 ),
  .mapper_sel                 ( 0                 ),

  .cart_addr                  ( cart_addr         ),
  .cart_a15                   ( cart_a15          ),
  .cart_rd                    ( cart_rd           ),
  .cart_wr                    ( cart_wr           ),
  .cart_do                    ( cart_do           ),
  .cart_di                    ( cart_di           ),
  .cart_oe                    ( cart_oe           ),

  .nCS                        ( nCS               ),

  .mbc_addr                   ( mbc_addr          ),

  .dn_write                   ( dn_write          ),
  .cart_ready                 ( cart_ready        ),

  .cram_rd                    ( cram_rd           ),

  .cart_download              ( cart_download     ),

  .ram_mask_file              ( ram_mask_file     ),
  .ram_size                   ( cart_ram_size     ),
  .has_save                   ( cart_has_save     ),

  .isGBC_game                 ( isGBC_game        ),
  .isSGB_game                 ( isSGB_game        ),

  .ioctl_download             ( ioctl_download    ),
  .ioctl_wr                   ( ioctl_wr          ),
  .ioctl_addr                 ( ioctl_addr        ),
  .ioctl_dout                 ( ioctl_dout        ),
  .ioctl_wait                 ( ioctl_wait        ),

  .rom_di                     ( rom_do            ),

  .joystick_analog_0          ( 0                 ),

  .ce_32k                     ( ce_32k            ),
  .RTC_time                   ( rtc_data_s ),
  .RTC_timestampOut           ( RTC_timestampOut  ),
  .RTC_savedtimeOut           ( RTC_savedtimeOut  ),
  .RTC_inuse                  ( RTC_inuse ),

  .SaveStateExt_Din           ( 0                 ),
  .SaveStateExt_Adr           ( 0                 ),
  .SaveStateExt_wren          ( 0                 ),
  .SaveStateExt_rst           ( 0                 ),
  .SaveStateExt_Dout          (                   ),
  .savestate_load             ( 0                 ),
  .sleep_savestate            ( sleep_savestate   ),

  .Savestate_CRAMAddr         ( 0                 ),
  .Savestate_CRAMRWrEn        ( 0                 ),
  .Savestate_CRAMWriteData    ( 0                 ),
  .Savestate_CRAMReadData     (                   ),

  .rumbling                   ( rumbling          ),

  .cart_tran_bank0_out        ( gb_cart_tran_bank0_out    ),
  .cart_tran_bank1_in         ( cart_tran_bank1_reg_in    ),
  .cart_tran_bank1_out        ( gb_cart_tran_bank1_out    ),
  .cart_tran_bank1_dir        ( gb_cart_tran_bank1_dir    ),
  .cart_tran_bank2_out        ( gb_cart_tran_bank2_out    ),
  .cart_tran_bank3_out        ( gb_cart_tran_bank3_out    )
);

reg [127:0] palette = 128'h828214517356305A5F1A3B4900000000;

always_ff @(posedge clk_sys) begin
  if (palette_download & ioctl_wr) begin
    palette[127:0] <= {palette[111:0], ioctl_dout[7:0], ioctl_dout[15:8]};
  end
end

wire lcd_clkena;
wire [14:0] lcd_data;
wire [1:0] lcd_mode;
wire [1:0] lcd_data_gb;
wire lcd_on;
wire lcd_vsync;

wire DMA_on;

wire reset = ~reset_n_s | (external_reset_s) | cart_download | boot_download | verification_req_delay > 0 | verifying | (verification_complete && !verification_passed);
wire speed;

wire [15:0] GB_AUDIO_L;
wire [15:0] GB_AUDIO_R;

wire sc_int_clock_out, ser_clk_out, ser_clk_in;

always_comb begin
  port_tran_so_dir  = 1'b1;
  port_tran_si_dir  = 1'b0;
  ser_clk_in        = port_tran_sck;
  port_tran_sck_dir = sc_int_clock_out;

  if (sc_int_clock_out) begin
    port_tran_sck = ser_clk_out;
  end else begin
    port_tran_sck = 1'bZ;
  end                   
end

// the gameboy itself
gb gb
(
  .reset                  ( reset ),
  
  .clk_sys                ( clk_sys               ),
  .ce                     ( ce_cpu                ),   // the whole gameboy runs on 4mhnz
  .ce_2x                  ( ce_cpu2x              ),   // ~8MHz in dualspeed mode (GBC)
  
  .isGBC                  ( isGBC                 ),
  .real_cgb_boot          ( 1                     ),  
  .isSGB                  ( sgb_en & ~isGBC       ),
  .megaduck               ( 0                     ),

  .joy_p54                ( joy_p54               ),
  .joy_din                ( joy_do_sgb            ),

  // interface to the "external" game cartridge
  .ext_bus_addr           ( cart_addr             ),
  .ext_bus_a15            ( cart_a15              ),
  .cart_rd                ( cart_rd               ),
  .cart_wr                ( cart_wr               ),
  .cart_do                ( cart_do               ),
  .cart_di                ( cart_di               ),
  .cart_oe                ( cart_oe               ),

  .nCS                    ( nCS                   ),

  .boot_gba_en            ( gba_en                ),
  .fast_boot_en           ( 1                     ),

  .cgb_boot_download      ( cgb_boot_download     ),
  .dmg_boot_download      ( dmg_boot_download     ),
  .sgb_boot_download      ( sgb_boot_download     ),
  .ioctl_wr               ( ioctl_wr              ),
  .ioctl_addr             ( ioctl_addr            ),
  .ioctl_dout             ( ioctl_dout            ),

  // audio
  .audio_l                ( GB_AUDIO_L            ),
  .audio_r                ( GB_AUDIO_R            ),
  
  // interface to the lcd
  .lcd_clkena             ( lcd_clkena            ),
  .lcd_data               ( lcd_data              ),
  .lcd_data_gb            ( lcd_data_gb           ),
  .lcd_mode               ( lcd_mode              ),
  .lcd_on                 ( lcd_on                ),
  .lcd_vsync              ( lcd_vsync             ),
  
  .speed                  ( speed                 ),
  .DMA_on                 ( DMA_on                ),
  
  // serial port
  .sc_int_clock2          ( sc_int_clock_out      ),
  .serial_clk_in          ( ser_clk_in            ),
  .serial_data_in         ( port_tran_si          ),
  .serial_clk_out         ( ser_clk_out           ),
  .serial_data_out        ( port_tran_so          ),
  
  // Palette download will disable cheats option (HPS doesn't distinguish downloads),
  // so clear the cheats and disable second option (chheats enable/disable)
  .gg_reset               ( 0                     ),
  .gg_en                  ( 0                     ),
  .gg_code                ( 0                     ),
  .gg_available           (                       ),
  
  // savestates
  .increaseSSHeaderCount  ( 0                     ),
  .cart_ram_size          ( 0                     ),
  .save_state             ( 0                     ),
  .load_state             ( 0                     ),
  .savestate_number       ( 0                     ),
  .sleep_savestate        ( sleep_savestate       ),

  .SaveStateExt_Din       (                       ),
  .SaveStateExt_Adr       (                       ),
  .SaveStateExt_wren      (                       ),
  .SaveStateExt_rst       (                       ),
  .SaveStateExt_Dout      ( 0                     ),
  .SaveStateExt_load      (                       ),
  
  .Savestate_CRAMAddr     (                       ),
  .Savestate_CRAMRWrEn    (                       ),
  .Savestate_CRAMWriteData(                       ),
  .Savestate_CRAMReadData ( 0                     ),
  
  .SAVE_out_Din           (                       ),  // data read from savestate
  .SAVE_out_Dout          ( 0                     ),  // data written to savestate
  .SAVE_out_Adr           (                       ),  // all addresses are DWORD addresses!
  .SAVE_out_rnw           (                       ),  // read = 1, write = 0
  .SAVE_out_ena           (                       ),  // one cycle high for each action
  .SAVE_out_be            (                       ),            
  .SAVE_out_done          ( 0                     ),  // should be one cycle high when write is done or read value is valid
  
  .rewind_on              ( 0                     ),
  .rewind_active          ( 0                     )
);

// Sound

wire [15:0] audio_l, audio_r;

assign audio_l = (fast_forward && ~ff_snd_en) ? 16'd0 : GB_AUDIO_L;
assign audio_r = (fast_forward && ~ff_snd_en) ? 16'd0 : GB_AUDIO_R;

audio_mixer #(
  .DW     ( 16  ),
  .STEREO ( 1   )
) audio_mixer (
  .clk_74b      ( clk_74b     ),
  .clk_audio    ( clk_sys     ),

  .vol_att      ( 0           ),
  .mix          ( 0           ),

  .is_signed    ( 1           ),
  .core_l       ( audio_l     ),
  .core_r       ( audio_r     ),

  .audio_mclk   ( audio_mclk  ),
  .audio_lrck   ( audio_lrck  ),
  .audio_dac    ( audio_dac   )
);

// the lcd to vga converter
wire ce_pix;
wire [8:0] h_cnt, v_cnt;
wire h_end;

wire [2:0] ui_string_index = ui_timer > 0 ? 
                                no_slots_error ? 3'h3 : 3'h2 :
                                dumping ? 3'h1 : 3'h0;

lcd lcd
(
  // serial interface
  .clk_sys        ( clk_sys                 ),
  .ce             ( ce_cpu                  ),

  .lcd_clkena     ( sgb_lcd_clkena          ),
  .data           ( sgb_lcd_data            ),
  .mode           ( sgb_lcd_mode            ),  // used to detect begin of new lines and frames
  .on             ( sgb_lcd_on              ),
  .lcd_vs         ( sgb_lcd_vsync           ),
  .shadow         ( 0                       ),

  .isGBC          ( isGBC                   ),

  .tint           ( |tint & ~bw_en          ),
  .inv            ( 0                       ),
  .originalcolors ( originalcolors          ),
  .analog_wide    ( 0                       ),

  // Palettes
  .pal1           ( palette[127:104]        ),
  .pal2           ( palette[103:80]         ),
  .pal3           ( palette[79:56]          ),
  .pal4           ( palette[55:32]          ),

  .sgb_border_pix ( sgb_border_pix          ),
  .sgb_pal_en     ( sgb_pal_en              ),
  .sgb_en         ( sgb_border_en & sgb_en  ),
  .sgb_freeze     ( sgb_lcd_freeze          ),

  .display_ui     ( display_ui              ),
  .fullscreen_ui  ( display_cart_error      ),
  .ui_string_index( ui_string_index         ),
  .save_index_bcd ( save_index_bcd          ),

  .clk_vid        ( clk_ram                 ),
  .hs             ( video_hs_gb             ),
  .vs             ( video_vs_gb             ),
  .hbl            ( h_blank                 ),
  .vbl            ( v_blank                 ),
  .r              ( video_rgb_gb[23:16]     ),
  .g              ( video_rgb_gb[15:8]      ),
  .b              ( video_rgb_gb[7:0]       ),
  .ce_pix         ( ce_pix                  ),
  .h_cnt          ( h_cnt                   ),
  .v_cnt          ( v_cnt                   ),
  .h_end          ( h_end                   )
);

wire [1:0] joy_p54;
wire [3:0] joy_do_sgb;
wire [14:0] sgb_lcd_data;
wire [15:0] sgb_border_pix;
wire sgb_lcd_clkena, sgb_lcd_on, sgb_lcd_vsync, sgb_lcd_freeze;
wire [1:0] sgb_lcd_mode;
wire sgb_pal_en;

wire [7:0] joystick_0 = {cont1_key_s[15], cont1_key_s[14], cont1_key_s[5], cont1_key_s[4], cont1_key_s[0], cont1_key_s[1], cont1_key_s[2], cont1_key_s[3]};
wire [7:0] joystick_1 = {cont2_key_s[15], cont2_key_s[14], cont2_key_s[5], cont2_key_s[4], cont2_key_s[0], cont2_key_s[1], cont2_key_s[2], cont2_key_s[3]};
wire [7:0] joystick_2 = {cont3_key_s[15], cont3_key_s[14], cont3_key_s[5], cont3_key_s[4], cont3_key_s[0], cont3_key_s[1], cont3_key_s[2], cont3_key_s[3]};
wire [7:0] joystick_3 = {cont4_key_s[15], cont4_key_s[14], cont4_key_s[5], cont4_key_s[4], cont4_key_s[0], cont4_key_s[1], cont4_key_s[2], cont4_key_s[3]};

sgb sgb (
  .reset              ( reset                         ),
  .clk_sys            ( clk_sys                       ),
  .ce                 ( ce_cpu                        ),

  .clk_vid            ( clk_ram                       ),
  .ce_pix             ( ce_pix                        ),

  .joystick_0         ( joystick_0                    ),
  .joystick_1         ( joystick_1                    ),
  .joystick_2         ( joystick_2                    ),
  .joystick_3         ( joystick_3                    ),
  .joy_p54            ( joy_p54                       ),
  .joy_do             ( joy_do_sgb                    ),

  .sgb_en             ( sgb_en & isSGB_game & ~isGBC  ),
  .tint               ( tint[1] & ~bw_en              ),
  .isGBC_game         ( isGBC & isGBC_game            ),

  .lcd_on             ( lcd_on                        ),
  .lcd_clkena         ( lcd_clkena                    ),
  .lcd_data           ( lcd_data                      ),
  .lcd_data_gb        ( lcd_data_gb                   ),
  .lcd_mode           ( lcd_mode                      ),
  .lcd_vsync          ( lcd_vsync                     ),

  .h_cnt              ( h_cnt                         ),
  .v_cnt              ( v_cnt                         ),
  .h_end              ( h_end                         ),

  .border_download    ( sgb_border_download           ),
  .ioctl_wr           ( ioctl_wr                      ),
  .ioctl_addr         ( ioctl_addr                    ),
  .ioctl_dout         ( ioctl_dout                    ),

  .sgb_border_pix     ( sgb_border_pix                ),
  .sgb_pal_en         ( sgb_pal_en                    ),
  .sgb_lcd_data       ( sgb_lcd_data                  ),
  .sgb_lcd_on         ( sgb_lcd_on                    ),
  .sgb_lcd_freeze     ( sgb_lcd_freeze                ),
  .sgb_lcd_clkena     ( sgb_lcd_clkena                ),
  .sgb_lcd_mode       ( sgb_lcd_mode                  ),
  .sgb_lcd_vsync      ( sgb_lcd_vsync                 )
);

// Video
wire h_blank;
wire v_blank;
wire video_hs_gb;
wire video_vs_gb;
wire [23:0] video_rgb_gb;

reg video_de_reg;
reg video_hs_reg;
reg video_vs_reg;
reg [23:0] video_rgb_reg;

reg hs_prev;
reg [2:0] hs_delay;
reg vs_prev;
reg de_prev;

wire de = ~(h_blank || v_blank);

wire [7:0] lum;
assign lum = (video_rgb_gb[23:16]>>2) + (video_rgb_gb[23:16]>>5) + (video_rgb_gb[15:8]>>1) + (video_rgb_gb[15:8]>>4) + (video_rgb_gb[7:0]>>4) + (video_rgb_gb[7:0]>>5);

always_ff @(posedge clk_vid) begin
  video_hs_reg  <= 0;
  video_de_reg  <= 0;
  video_rgb_reg <= 24'h0;

  if (de) begin
    video_de_reg  <= 1;

    if (bw_en) begin
      video_rgb_reg <= {lum, lum, lum};
    end else begin
      video_rgb_reg <= video_rgb_gb;
    end
  end else if (de_prev && ~de) begin
    video_rgb_reg <= 24'h0;
  end

  if (hs_delay > 0) begin
    hs_delay <= hs_delay - 3'h1;
  end

  if (hs_delay == 1) begin
    video_hs_reg <= 1;
  end

  if (~hs_prev && video_hs_gb) begin
    // HSync went high. Delay by 3 cycles to prevent overlapping with VSync
    hs_delay <= 7;
  end

  // Set VSync to be high for a single cycle on the rising edge of the VSync coming out of the core
  video_vs_reg  <= ~vs_prev && video_vs_gb;
  hs_prev       <= video_hs_gb;
  vs_prev       <= video_vs_gb;
  de_prev       <= de;
end

assign video_rgb_clock    = clk_vid;
assign video_rgb_clock_90 = clk_vid_90;
assign video_de           = video_de_reg;
assign video_hs           = video_hs_reg;
assign video_vs           = video_vs_reg;

always_comb begin
  if(~video_de_reg) begin
    if(sgb_border_en & sgb_en) begin
      video_rgb[23:13] = 1;
      video_rgb[12:3]  = 0;
      video_rgb[2:0]   = 0;
    end else begin
      video_rgb[23:13] = 0;
      video_rgb[12:3]  = 0;
      video_rgb[2:0]   = 0;
    end
  end else begin
    video_rgb = video_rgb_reg;
  end
end

//////////////////////////////// CE ////////////////////////////////////

wire ce_cpu, ce_cpu2x;
wire cart_act     = cart_wr | cart_rd;
wire fastforward  = ff_en && cont1_key_s[9] && !ioctl_download;
wire ff_on;
wire sleep_savestate;
reg paused;

always_ff @(posedge clk_sys) begin
  paused <= sleep_savestate;
end

speedcontrol speedcontrol
(
  .clk_sys     ( clk_sys              ),
  .pause       ( paused | dumping     ),
  .speedup     ( fast_forward         ),
  .cart_act    ( cart_act             ),
  .DMA_on      ( DMA_on               ),
  .ce          ( ce_cpu               ),
  .ce_2x       ( ce_cpu2x             ),
  .refresh     ( sdram_refresh_force  ),
  .ff_on       ( ff_on                )
);

///////////////////////////// Fast Forward Latch /////////////////////////////////

reg fast_forward;
reg ff_latch;

always_ff @(posedge clk_sys) begin : ffwd
  reg last_ffw;
  reg ff_was_held;
  longint ff_count;

  last_ffw <= fastforward;

  if (fastforward)
    ff_count <= ff_count + 1;

  if (~last_ffw & fastforward) begin
    ff_latch <= 0;
    ff_count <= 0;
  end

  if ((last_ffw & ~fastforward)) begin // 32mhz clock, 0.2 seconds
    ff_was_held <= 0;

    if (ff_count < 4800000 && ~ff_was_held) begin
      ff_was_held <= 1;
      ff_latch    <= 1;
    end
  end

  fast_forward <= (fastforward | ff_latch);
end

endmodule
