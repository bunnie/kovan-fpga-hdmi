////////////////////////////////////////////////
// Copyright (c) 2012, Andrew "bunnie" Huang  
// (bunnie _aht_ bunniestudios "dote" com)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//     Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in
//     the documentation and/or other materials provided with the
//     distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////
  
`timescale 1 ns / 1 ps

module hdmi (
	     input wire       qvgaclk,
	     input wire [5:0] lcd_red,
	     input wire [5:0] lcd_green,
	     input wire [5:0] lcd_blue,
	     input wire       lcd_den,
	     input wire       lcd_hsync,
	     input wire       lcd_vsync,

	     output wire [3:0] TMDS,
	     output wire [3:0] TMDSB,

	     output wire dbg_hsync,
	     output wire dbg_vsync,
	     output wire dbg_de,
	     output wire [2:0] dbg_tmdsint,
	     output wire dbg_tmdsclk,

	     input wire  rstin
	     );

   wire [9:0] 		 red;
   wire [9:0] 		 green;
   wire [9:0] 		 blue;

   wire [7:0] 		 red_din;
   wire [7:0] 		 green_din;
   wire [7:0] 		 blue_din;
   wire [15:0] 		 line0a_dout;
   wire [15:0] 		 line1a_dout;
   wire 		 line0a_full;
   wire 		 line0a_empty;
   wire 		 line1a_full;
   wire 		 line1a_empty;

   wire [15:0] 		 line0b_dout;
   wire [15:0] 		 line1b_dout;
   wire 		 line0b_full;
   wire 		 line0b_empty;
   wire 		 line1b_full;
   wire 		 line1b_empty;
   wire [15:0] 		 linemux;
   
   wire [4:0] 		 fifo_red;
   wire [5:0] 		 fifo_green;
   wire [4:0] 		 fifo_blue;

   wire 		 hsync;
   wire 		 vsync;
   wire 		 de;
   
   wire [4:0] 		 tmds_data0, tmds_data1, tmds_data2;
   wire [2:0] 		 tmdsint;


   reg 			 active_line;
   reg 			 dup_line;

   reg 			 hsync_hd;
   reg 			 vsync_hd;
   reg 			 hsync_fall;
   reg 			 vsync_fall;
   reg 			 hsync_to_hdmi_fall;
   reg 			 hsync_to_hdmi_d;
   reg 			 de_hd;
   wire 		 hd_reset;
   reg [11:0] 		 hpix_cnt;
   reg [11:0] 		 line_cnt;
   reg 			 block_de;
   reg 			 hsync_to_hdmi;
   reg 			 vsync_to_hdmi;
   reg 			 de_to_hdmi;
   
   wire 		 pclk_hd;
   wire 		 qvgaclkx4_locked;

   wire 		 pclk;

   sync_reset  hd_reset_sync(
			  .clk(pclk_hd),
			  .glbl_reset(rstin || !qvgaclkx4_locked),
			  .reset(hd_reset) );

   wire 		 computed_gb;
   wire [3:0]		 computed_ctl_code;
   
   gbgen gbgen (
		.pclk(pclk),
		.rstin(hd_reset),
		.vsync(vsync_to_hdmi),
		.hsync(hsync_to_hdmi | block_de),
		.sync_pol(1'b0), // sync is active low for this mode
		.de(de),

		.gb(computed_gb),
		.code(computed_ctl_code)
		);

   always @(posedge pclk_hd) begin
      // synchronize control signals into the HD domain
      hsync_hd <= lcd_hsync;
      vsync_hd <= lcd_vsync;
      de_hd <= lcd_den;

      hsync_fall <= !lcd_hsync & hsync_hd;
      vsync_fall <= !lcd_vsync & vsync_hd;
   end // always @ (posedge pclk_hd)
   
   always @(posedge pclk_hd) begin
      if( hd_reset ) begin
	 active_line <= 1'b0;
      end else if( hsync_fall ) begin
	 active_line <= !active_line;
      end else begin
	 active_line <= active_line;
      end
   end // always @ (posedge pclk_hd)

   always @(posedge pclk_hd) begin
      if( hd_reset ) begin
	 dup_line <= 1'b0;
      end else if( hpix_cnt == 12'd16 ) begin // at pix 16, hsync to hdmi falls
	 dup_line <= !dup_line;
      end else begin
	 dup_line <= dup_line;
      end
   end // always @ (posedge pclk_hd)

   always @(posedge pclk_hd) begin
      hsync_to_hdmi_d <= hsync_to_hdmi;
      hsync_to_hdmi_fall <= !hsync_to_hdmi & hsync_to_hdmi_d;
   end
   
   always @(posedge pclk_hd) begin
      if( hd_reset ) begin
	 line_cnt <= 12'b0;
      end else if( vsync_fall ) begin
	 line_cnt <= 12'b0;
      end else if( hsync_to_hdmi_fall ) begin
	 line_cnt <= line_cnt + 12'b1;
      end
   end // always @ (posedge pclk_hd)

   always @(posedge pclk_hd) begin
      if( hd_reset ) begin
	 vsync_to_hdmi <= 1'b1; 
      end else if( line_cnt < 12'd3 ) begin
	 vsync_to_hdmi <= 1'b0;
      end else begin
	 vsync_to_hdmi <= 1'b1;
      end
   end
   
   always @(posedge pclk_hd) begin
      if( hd_reset ) begin
	 block_de <= 1'b1;
      end else if( (line_cnt < 12'd36) || (line_cnt > 12'd515) ) begin
	 block_de <= 1'b1;
      end else begin
	 block_de <= 1'b0;
      end
   end

   always @(posedge pclk_hd) begin
      if( hd_reset ) begin
	 hpix_cnt <= 12'b0;
      end else if( hsync_fall || (hpix_cnt >= 12'd799) ) begin
	 hpix_cnt <= 12'b0;
      end else begin
	 hpix_cnt <= hpix_cnt + 12'b1;
      end
   end

   always @(posedge pclk_hd) begin
      if( hd_reset ) begin
	 hsync_to_hdmi <= 1'b1;
	 de_to_hdmi <= 1'b0;
      end else begin
	 if( hpix_cnt < 12'd16 ) begin
	    hsync_to_hdmi <= 1'b1;
	    de_to_hdmi <= 1'b0;
	 end else if( hpix_cnt < 12'd112 ) begin
	    hsync_to_hdmi <= 1'b0;
	    de_to_hdmi <= 1'b0;
	 end else if( hpix_cnt < 12'd160 ) begin
	    hsync_to_hdmi <= 1'b1;
	    de_to_hdmi <= 1'b0;
	 end else if( hpix_cnt < 12'd800 ) begin
	    hsync_to_hdmi <= 1'b1;
	    de_to_hdmi <= 1'b1;
	 end else begin
	    hsync_to_hdmi <= 1'b1;
	    de_to_hdmi <= 1'b0;
	 end
      end // else: !if( hd_reset )
   end // always @ (posedge pclk_hd)
   
   
//			       .rd_rst(hsync_fall & active_line),
//			       .wr_rst(hsync_fall & active_line),
   qvga_line_fifo line_fifo0a (
			       .rd_rst(vsync_fall),
			       .wr_rst(vsync_fall),
			      .wr_clk(qvgaclk), // input wr_clk
			      .rd_clk(pclk_hd), // input rd_clk
			      .din({lcd_red[5:1],lcd_green[5:0],lcd_blue[5:1]}), // input [15 : 0] din
			      .wr_en(lcd_den & !active_line), // input wr_en
			      .rd_en( de_to_hdmi & active_line & hpix_cnt[0] & dup_line ), // input rd_en
			      .dout(line0a_dout[15:0]), // output [15 : 0] dout
			      .full(line0a_full), // output full
			      .empty(line0a_empty) // output empty
			      );

   qvga_line_fifo line_fifo0b (
			       .rd_rst(vsync_fall),
			       .wr_rst(vsync_fall),
			      .wr_clk(qvgaclk), // input wr_clk
			      .rd_clk(pclk_hd), // input rd_clk
			      .din({lcd_red[5:1],lcd_green[5:0],lcd_blue[5:1]}), // input [15 : 0] din
			      .wr_en(lcd_den & !active_line), // input wr_en
			      .rd_en( de_to_hdmi & active_line & hpix_cnt[0] & !dup_line ), // input rd_en
			      .dout(line0b_dout[15:0]), // output [15 : 0] dout
			      .full(line0b_full), // output full
			      .empty(line0b_empty) // output empty
			      );
   
   qvga_line_fifo line_fifo1a (
			       .rd_rst(vsync_fall),
			       .wr_rst(vsync_fall),
			      .wr_clk(qvgaclk), // input wr_clk
			      .rd_clk(pclk_hd), // input rd_clk
			      .din({lcd_red[5:1],lcd_green[5:0],lcd_blue[5:1]}), // input [15 : 0] din
			      .wr_en(lcd_den & active_line), // input wr_en
			      .rd_en( de_to_hdmi & !active_line & hpix_cnt[0] & dup_line ), // input rd_en
			      .dout(line1a_dout[15:0]), // output [15 : 0] dout
			      .full(line1a_full), // output full
			      .empty(line1a_empty) // output empty
			      );
   
   qvga_line_fifo line_fifo1b (
			       .rd_rst(vsync_fall),
			       .wr_rst(vsync_fall),
			      .wr_clk(qvgaclk), // input wr_clk
			      .rd_clk(pclk_hd), // input rd_clk
			      .din({lcd_red[5:1],lcd_green[5:0],lcd_blue[5:1]}), // input [15 : 0] din
			      .wr_en(lcd_den & active_line), // input wr_en
			      .rd_en( de_to_hdmi & !active_line & hpix_cnt[0] & !dup_line ), // input rd_en
			      .dout(line1b_dout[15:0]), // output [15 : 0] dout
			      .full(line1b_full), // output full
			      .empty(line1b_empty) // output empty
			      );

   assign linemux = active_line ? 
		    (dup_line ? line0a_dout : line0b_dout) : 
		    (dup_line ? line1a_dout : line1b_dout);
   
   assign fifo_blue[4:0] = linemux[4:0];
   assign fifo_green[5:0] = linemux[10:5];
   assign fifo_red[4:0] = linemux[15:11];
      
   assign red_din[7:0] = {fifo_red[4:0], 3'b0};
   assign green_din[7:0] = {fifo_green[5:0], 2'b0};
   assign blue_din[7:0] = {fifo_blue[4:0], 3'b0};

   assign hsync = hsync_to_hdmi;
   assign vsync = vsync_to_hdmi;
   assign de = de_to_hdmi & !block_de; // gate out de when it shouldn't be on per spec

   assign dbg_de = de;
   assign dbg_vsync = vsync;
   assign dbg_hsync = hsync;
   
   ///
   wire 		 pclkx10;
   wire 		 pclkx2;
   wire 		 serdesstrobe;
   wire 		 tmdsclk;

   wire 		 serdes_reset;
   
   //
   // Forward TMDS Clock Using OSERDES2 block
   //
   reg [4:0] 		 tmdsclkint = 5'b00000;
   reg 			 toggle = 1'b0;
 
   always @ (posedge pclkx2 or posedge serdes_reset) begin
      if (serdes_reset)
	toggle <= 1'b0;
      else
	toggle <= ~toggle;
   end

  always @ (posedge pclkx2) begin
    if (toggle)
      tmdsclkint <= 5'b11111;
    else
      tmdsclkint <= 5'b00000;
  end

   serdes_n_to_1 #(
		   .SF           (5))
   clkout (
	   .iob_data_out (tmdsclk),
	   .ioclk        (pclkx10),
	   .serdesstrobe (serdesstrobe),
	   .gclk         (pclkx2),
	   .reset        (serdes_reset),
    .datain       (tmdsclkint));
   
   OBUFDS TMDS3 (.I(tmdsclk), .O(TMDS[3]), .OB(TMDSB[3])) ;// clock
   assign dbg_tmdsclk = tmdsclk;
   assign dbg_tmdsint[2:0] = tmdsint[2:0];
   
   //
   // Forward TMDS Data: 3 channels
   //
   serdes_n_to_1 #(.SF(5)) oserdes0 (
				     .ioclk(pclkx10),
				     .serdesstrobe(serdesstrobe),
				     .reset(serdes_reset),
				     .gclk(pclkx2),
				     .datain(tmds_data0),
				     .iob_data_out(tmdsint[0])) ;

   serdes_n_to_1 #(.SF(5)) oserdes1 (
				     .ioclk(pclkx10),
				     .serdesstrobe(serdesstrobe),
				     .reset(serdes_reset),
				     .gclk(pclkx2),
				     .datain(tmds_data1),
				     .iob_data_out(tmdsint[1])) ;

   serdes_n_to_1 #(.SF(5)) oserdes2 (
				     .ioclk(pclkx10),
				     .serdesstrobe(serdesstrobe),
				     .reset(serdes_reset),
				     .gclk(pclkx2),
				     .datain(tmds_data2),
				     .iob_data_out(tmdsint[2])) ;

   OBUFDS TMDS0 (.I(tmdsint[0]), .O(TMDS[0]), .OB(TMDSB[0])) ;
   OBUFDS TMDS1 (.I(tmdsint[1]), .O(TMDS[1]), .OB(TMDSB[1])) ;
   OBUFDS TMDS2 (.I(tmdsint[2]), .O(TMDS[2]), .OB(TMDSB[2])) ;
   
   encodeb encb (
		 .clkin	(pclk),
		 .rstin	(serdes_reset),
		 .din		(blue_din),
		 .c0			(hsync),
		 .c1			(vsync),
		 .de			(de),
		 .dout		(blue),
 		 .vid_gb             (1'b0)) ;
 //		 .vid_gb             (computed_gb)) ;
   
   encodeg encg (
		 .clkin	(pclk),
		 .rstin	(serdes_reset),
		 .din		(green_din),
		 .c0			(1'b0), // bit 0
//		 .c0			(computed_ctl_code[0]), // bit 0
		 .c1			(computed_ctl_code[1]), // bit 1
		 .de			(de),
		 .dout		(green),
 		 .vid_gb             (1'b0)) ;
 //		 .vid_gb             (computed_gb)) ;

    
   encoder encr (
		 .clkin	(pclk),
		 .rstin	(serdes_reset),
		 .din		(red_din),
		 .c0			(computed_ctl_code[2]), // bit 2
		 .c1			(computed_ctl_code[3]), // bit 3
		 .de			(de),
		 .dout		(red),
 		 .vid_gb             (1'b0)) ;
 //		 .vid_gb             (computed_gb)) ;

   wire [29:0] s_data_x = {red[9:5], green[9:5], blue[9:5],
			   red[4:0], green[4:0], blue[4:0]};
   
   // was bypass_ena in here...
   wire [29:0] s_data = s_data_x;

  convert_30to15_fifo pixel2x (
    .rst     (serdes_reset),
    .clk     (pclk),
    .clkx2   (pclkx2),
    .datain  (s_data),
    .dataout ({tmds_data2, tmds_data1, tmds_data0}));


  //////////////////////////////////////////////////////////////////
  // Instantiate x4 DCM for qvga to vga clock scaling
  //////////////////////////////////////////////////////////////////
  qvga_clock_scale_x4 clock_scale_x4
   (// Clock in ports
    .clk6p3(qvgaclk),      // IN
    // Clock out ports
    .clk25p2(pclk_hd),     // OUT
    // Status and control signals
    .RESET(rstin),// IN
    .LOCKED(qvgaclkx4_locked) // OUT
    );
   assign pclk = pclk_hd;


   //////////////////////////////////////////////////////////////////
   // Instantiate a dedicate PLL for output port
   //////////////////////////////////////////////////////////////////
   wire clkfbout, clkfbin;
   wire pllclk0, pllclk2;
   wire plllckd;

  PLL_BASE # (
    .CLKIN_PERIOD(39.6825), // 25.2 MHz
    .CLKFBOUT_MULT(20), //set VCO to 10x of CLKIN
    .CLKOUT0_DIVIDE(2),
    .CLKOUT1_DIVIDE(20),
    .CLKOUT2_DIVIDE(10),
//	      .BANDWIDTH("LOW"), // normally not here
    .COMPENSATION("SOURCE_SYNCHRONOUS")
  ) PLL_OSERDES_0 (
    .CLKFBOUT(clkfbout),
    .CLKOUT0(pllclk0),
    .CLKOUT1(),
    .CLKOUT2(pllclk2),
    .CLKOUT3(),
    .CLKOUT4(),
    .CLKOUT5(),
    .LOCKED(plllckd),
    .CLKFBIN(clkfbin),
    .CLKIN(pclk_hd),
    .RST( rstin || !qvgaclkx4_locked )  
  );

   wire lcd_intbuf;
   
  //
  // This BUFG is needed in order to deskew between PLL clkin and clkout
  // So the tx0 pclkx2 and pclkx10 will have the same phase as the pclk input
  //
  BUFG clkfb_buf (.I(clkfbout), .O(clkfbin));

  //
  // regenerate pclkx2 for TX
  //
  BUFG pclkx2_buf (.I(pllclk2), .O(pclkx2));

  //
  // regenerate pclkx10 for TX
  //
  wire bufpll_lock;
  BUFPLL #(.DIVIDE(5)) ioclk_buf (.PLLIN(pllclk0), .GCLK(pclkx2), .LOCKED(plllckd),
           .IOCLK(pclkx10), .SERDESSTROBE(serdesstrobe), .LOCK(bufpll_lock));

   // reset off of master PLL lock, not BUFPLL lock
   assign serdes_reset = (~plllckd) || (hd_reset);

endmodule


