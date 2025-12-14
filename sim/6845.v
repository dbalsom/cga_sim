// ---------------------------------------------------------------------------
// MC6845 "Type 2" CRTC - Pin-accurate DIP-40 wrapper
//   Pinout matches the diagram provided:
//
//   1  VSS          40 VSYNC
//   2  /RESET       39 HSYNC
//   3  LPSTB        38 RA0
//   4  MA0          37 RA1
//   5  MA1          36 RA2
//   6  MA2          35 RA3
//   7  MA3          34 RA4
//   8  MA4          33 D0
//   9  MA5          32 D1
//  10  MA6          31 D2
//  11  MA7          30 D3
//  12  MA8          29 D4
//  13  MA9          28 D5
//  14  MA10         27 D6
//  15  MA11         26 D7
//  16  MA12         25 /CS
//  17  MA13         24 RS
//  18  DisplayEn    23 E
//  19  Cursor       22 R/W  (1=read, 0=write)
//  20  VCC          21 CLK
//
// Core behaviour is the same as the previous "mc6845_type2" model
// (Type-2 like Motorola, CGA oriented, no obscure demo edge cases).
//
// Pure Verilog-2005, no SystemVerilog.
// ---------------------------------------------------------------------------

module mc6845 (
    // Power pins (not functionally modeled)
    input  wire VSS,      // pin 1
    input  wire VCC,      // pin 20

    // Control / timing pins
    input  wire RESET_N,  // pin 2, active low
    input  wire LPSTB,    // pin 3
    input  wire CLK,      // pin 21

    // MPU interface (Motorola-style)
    input  wire CS_N,     // pin 25, active low
    input  wire RS,       // pin 24
    input  wire E,        // pin 23
    input  wire RW,       // pin 22, 1 = read, 0 = write
    inout  wire [7:0] D,  // pins 33(D0)..26(D7)

    // Address outputs
    output reg  [13:0] MA,  // pins 4..17 = MA0..MA13
    output reg  [4:0]  RA,  // pins 38..34 = RA0..RA4

    // Video timing outputs
    output reg  VSYNC,   // pin 40
    output reg  HSYNC,   // pin 39
    output reg  DISP_EN, // pin 18 (Display enable)
    output reg  CURSOR   // pin 19
);

    // -----------------------------------------------------------------------
    // Internal register file (with corrected widths)
    // -----------------------------------------------------------------------

    reg [4:0] reg_index;          // Address register (R0..R17)

    reg [7:0] r0_htotal;          // R0  Horizontal Total
    reg [7:0] r1_hdisp;           // R1  Horizontal Displayed
    reg [7:0] r2_hsync_pos;       // R2  HSync Position
    reg [7:0] r3_sync_width;      // R3  Sync Width (HS:3:0, VS:7:4)
    reg [6:0] r4_vtotal;          // R4  Vertical Total (7 bits)
    reg [4:0] r5_vadj;            // R5  Vertical Total Adjust (5 bits)
    reg [6:0] r6_vdisp;           // R6  Vertical Displayed (7 bits)
    reg [6:0] r7_vsync_pos;       // R7  VSync Position (7 bits)
    reg [5:0] r8_mode;            // R8  Interlace/Skew (unused here)
    reg [4:0] r9_max_scan;        // R9  Max Scan Line Address
    reg [6:0] r10_cursor_start;   // R10 Cursor Start (4:0 + mode 6:5)
    reg [4:0] r11_cursor_end;     // R11 Cursor End (4:0)
    reg [5:0] r12_start_hi;       // R12 Start Address high (MA[13:8])
    reg [7:0] r13_start_lo;       // R13 Start Address low  (MA[7:0])
    reg [5:0] r14_cursor_hi;      // R14 Cursor Address high
    reg [7:0] r15_cursor_lo;      // R15 Cursor Address low

    // Light pen registers (R16/R17)
    reg [5:0] r16_lp_hi;          // MA[13:8]
    reg [7:0] r17_lp_lo;          // MA[7:0]

    // For light pen calculation
    reg [13:0] lp_addr;

    // -----------------------------------------------------------------------
    // Derived signals
    // -----------------------------------------------------------------------

    wire [13:0] start_addr  = { r12_start_hi, r13_start_lo };
    wire [13:0] cursor_addr = { r14_cursor_hi, r15_cursor_lo };

    wire [3:0] hsw_nib = r3_sync_width[3:0];
    wire [3:0] vsw_nib = r3_sync_width[7:4];

    wire [4:0] hsw_eff = (hsw_nib == 4'd0) ? 5'd16 : {1'b0, hsw_nib};
    wire [4:0] vsw_eff = (vsw_nib == 4'd0) ? 5'd16 : {1'b0, vsw_nib};

    wire [4:0] cursor_start_line = r10_cursor_start[4:0];
    wire [4:0] cursor_end_line   = r11_cursor_end;
    wire [1:0] cursor_mode       = r10_cursor_start[6:5];

    // MPU data bus split
    wire [7:0] di;   // data from MPU
    reg  [7:0] dout; // data to MPU

    assign di = D;
    // Drive bus only during read cycles (RW=1), when selected and E high.
    assign D  = (!CS_N && RW && E) ? dout : 8'bz;

    // -----------------------------------------------------------------------
    // Timing counters
    // -----------------------------------------------------------------------

    reg [7:0] h_count;
    reg [7:0] row_count;
    reg [4:0] raster_count;
    reg       in_vadj;
    reg [4:0] vadj_count;
    reg [13:0] ma_line_start;
    reg [23:0] blink_counter;

    // -----------------------------------------------------------------------
    // Blink gate (simple, not field-locked)
    // -----------------------------------------------------------------------

    wire blink_gate =
        (cursor_mode == 2'b00) ? 1'b1   : // non-blink
        (cursor_mode == 2'b01) ? 1'b0   : // cursor off
        (cursor_mode == 2'b10) ? blink_counter[18] : // ~1/16
                                 blink_counter[19];  // ~1/32

    // -----------------------------------------------------------------------
    // Bus interface (E/CS_N/RS/RW)
    // -----------------------------------------------------------------------

    always @(posedge E or negedge RESET_N) begin
        if (!RESET_N) begin
            reg_index        <= 5'd0;
            r0_htotal        <= 8'd0;
            r1_hdisp         <= 8'd0;
            r2_hsync_pos     <= 8'd0;
            r3_sync_width    <= 8'd0;
            r4_vtotal        <= 7'd0;
            r5_vadj          <= 5'd0;
            r6_vdisp         <= 7'd0;
            r7_vsync_pos     <= 7'd0;
            r8_mode          <= 6'd0;
            r9_max_scan      <= 5'd0;
            r10_cursor_start <= 7'd0;
            r11_cursor_end   <= 5'd0;
            r12_start_hi     <= 6'd0;
            r13_start_lo     <= 8'd0;
            r14_cursor_hi    <= 6'd0;
            r15_cursor_lo    <= 8'd0;
            r16_lp_hi        <= 6'd0;
            r17_lp_lo        <= 8'd0;
            dout             <= 8'h00;
        end
        else begin
            if (!CS_N) begin
                if (!RW) begin
                    // Write cycle
                    if (!RS) begin
                        // Address register
                        reg_index <= di[4:0];
                    end
                    else begin
                        // Register file write
                        case (reg_index)
                            5'd0:  r0_htotal        <= di;
                            5'd1:  r1_hdisp         <= di;
                            5'd2:  r2_hsync_pos     <= di;
                            5'd3:  r3_sync_width    <= di;
                            5'd4:  r4_vtotal        <= di[6:0];
                            5'd5:  r5_vadj          <= di[4:0];
                            5'd6:  r6_vdisp         <= di[6:0];
                            5'd7:  r7_vsync_pos     <= di[6:0];
                            5'd8:  r8_mode          <= di[5:0];
                            5'd9:  r9_max_scan      <= di[4:0];
                            5'd10: r10_cursor_start <= di[6:0];
                            5'd11: r11_cursor_end   <= di[4:0];
                            5'd12: r12_start_hi     <= di[5:0];
                            5'd13: r13_start_lo     <= di;
                            5'd14: r14_cursor_hi    <= di[5:0];
                            5'd15: r15_cursor_lo    <= di;
                            default: ; // R16/17 read-only
                        endcase
                    end
                end
                else begin
                    // Read cycle
                    if (!RS) begin
                        dout <= {3'b000, reg_index};
                    end
                    else begin
                        case (reg_index)
                            5'd0:  dout <= r0_htotal;
                            5'd1:  dout <= r1_hdisp;
                            5'd2:  dout <= r2_hsync_pos;
                            5'd3:  dout <= r3_sync_width;
                            5'd4:  dout <= {1'b0, r4_vtotal};
                            5'd5:  dout <= {3'b000, r5_vadj};
                            5'd6:  dout <= {1'b0, r6_vdisp};
                            5'd7:  dout <= {1'b0, r7_vsync_pos};
                            5'd8:  dout <= {2'b00, r8_mode};
                            5'd9:  dout <= {3'b000, r9_max_scan};
                            5'd10: dout <= {1'b0, r10_cursor_start};
                            5'd11: dout <= {3'b000, r11_cursor_end};
                            5'd12: dout <= {2'b00, r12_start_hi};
                            5'd13: dout <= r13_start_lo;
                            5'd14: dout <= {2'b00, r14_cursor_hi};
                            5'd15: dout <= r15_cursor_lo;
                            5'd16: dout <= {2'b00, r16_lp_hi};
                            5'd17: dout <= r17_lp_lo;
                            default: dout <= 8'hFF;
                        endcase
                    end
                end
            end
        end
    end

    // -----------------------------------------------------------------------
    // Light Pen latch (MA + 2 on LPSTB rising edge)
    // -----------------------------------------------------------------------

    always @(posedge LPSTB or negedge RESET_N) begin
        if (!RESET_N) begin
            lp_addr   <= 14'd0;
            r16_lp_hi <= 6'd0;
            r17_lp_lo <= 8'd0;
        end
        else begin
            lp_addr   = MA + 14'd2;
            r16_lp_hi <= lp_addr[13:8];
            r17_lp_lo <= lp_addr[7:0];
        end
    end

    // -----------------------------------------------------------------------
    // Core timing and address generation
    // -----------------------------------------------------------------------

    always @(posedge CLK or negedge RESET_N) begin
        if (!RESET_N) begin
            h_count       <= 8'd0;
            row_count     <= 8'd0;
            raster_count  <= 5'd0;
            in_vadj       <= 1'b0;
            vadj_count    <= 5'd0;
            ma_line_start <= 14'd0;
            MA            <= 14'd0;
            RA            <= 5'd0;
            DISP_EN       <= 1'b0;
            HSYNC         <= 1'b0;
            VSYNC         <= 1'b0;
            CURSOR        <= 1'b0;
            blink_counter <= 24'd0;
        end
        else begin
            blink_counter <= blink_counter + 24'd1;

            // Default horizontal increment
            MA      <= MA + 14'd1;
            h_count <= h_count + 8'd1;

            // End of scanline?
            if (h_count == r0_htotal) begin
                h_count <= 8'd0;

                if (in_vadj) begin
                    vadj_count   <= vadj_count + 5'd1;
                    raster_count <= 5'd0;

                    if ((vadj_count + 5'd1 >= r5_vadj) || (r5_vadj == 5'd0)) begin
                        in_vadj       <= 1'b0;
                        vadj_count    <= 5'd0;
                        row_count     <= 8'd0;
                        raster_count  <= 5'd0;
                        ma_line_start <= start_addr;
                        MA            <= start_addr;
                    end
                    else begin
                        MA <= ma_line_start;
                    end
                end
                else begin
                    if (raster_count == r9_max_scan) begin
                        raster_count <= 5'd0;

                        if (row_count == {1'b0, r4_vtotal}) begin
                            if (r5_vadj != 5'd0) begin
                                in_vadj    <= 1'b1;
                                vadj_count <= 5'd0;
                                MA         <= ma_line_start;
                            end
                            else begin
                                row_count     <= 8'd0;
                                ma_line_start <= start_addr;
                                MA            <= start_addr;
                            end
                        end
                        else begin
                            row_count     <= row_count + 8'd1;
                            ma_line_start <= ma_line_start + {6'd0, r1_hdisp};
                            MA            <= ma_line_start + {6'd0, r1_hdisp};
                        end
                    end
                    else begin
                        raster_count <= raster_count + 5'd1;
                        MA           <= ma_line_start;
                    end
                end
            end

            // RA output (0 during adjust)
            if (in_vadj) RA <= 5'd0;
            else         RA <= raster_count;

            // Display enable: horiz < R1, row < R6, not in adjust
            DISP_EN <= (!in_vadj) &&
                       (row_count < {1'b0, r6_vdisp}) &&
                       (h_count   < r1_hdisp);

            // HSYNC pulse
            HSYNC <= (h_count >= r2_hsync_pos) &&
                     (h_count <  (r2_hsync_pos + {3'd0, hsw_eff}));

            // VSYNC pulse
            VSYNC <= (!in_vadj) &&
                     (row_count >= {1'b0, r7_vsync_pos}) &&
                     (row_count <  ({1'b0, r7_vsync_pos} + {3'd0, vsw_eff}));

            // Cursor gate
            CURSOR <= blink_gate &&
                      (!in_vadj) &&
                      (row_count < {1'b0, r6_vdisp}) &&
                      (MA == cursor_addr) &&
                      (raster_count >= cursor_start_line) &&
                      (raster_count <= cursor_end_line);
        end
    end

endmodule
