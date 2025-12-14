`timescale 1ns / 1ps

module mc6845 #(
    // ========================================================================
    // CONFIGURATION PARAMETER
    // ========================================================================
    // Change this value to switch startup modes:
    // 0 = 80x25 Text (Standard CGA/MDA)
    // 1 = 40x25 Text (Low Res / TV)
    // 2 = 640x200 Graphics (100 rows of 2 scanlines)
    // 3 = 320x200 Graphics (100 rows of 2 scanlines)
    // 4 = 640x200 "ANSI from Hell" (100 rows of 2 scanlines)
    parameter STARTUP_MODE = 4
)(
    // ========================
    // LEFT SIDE (Pins 1-20)
    // ========================
    
    input wire VSS,     // Pin 1:  Ground
    input wire RESET_n, // Pin 2:  Active Low Reset
    input wire LPSTB,   // Pin 3:  Light Pen Strobe
    
    // Memory Address (MA0-MA13)
    output wire MA0,    // Pin 4
    output wire MA1,    // Pin 5
    output wire MA2,    // Pin 6
    output wire MA3,    // Pin 7
    output wire MA4,    // Pin 8
    output wire MA5,    // Pin 9
    output wire MA6,    // Pin 10
    output wire MA7,    // Pin 11
    output wire MA8,    // Pin 12
    output wire MA9,    // Pin 13
    output wire MA10,   // Pin 14
    output wire MA11,   // Pin 15
    output wire MA12,   // Pin 16
    output wire MA13,   // Pin 17
    
    output wire DE,     // Pin 18: Display Enable
    output wire CURSOR, // Pin 19: Cursor Output
    input wire VCC,     // Pin 20: +5V

    // ========================
    // RIGHT SIDE (Pins 21-40)
    // ========================

    input wire CLK,     // Pin 21: Character Clock
    input wire RW,      // Pin 22: Read/Write (1=Read, 0=Write)
    input wire E,       // Pin 23: Enable (CPU Bus Clock)
    input wire RS,      // Pin 24: Register Select (0=Addr, 1=Data)
    input wire CS_n,    // Pin 25: Chip Select (Active Low)

    // Data Bus (D7-D0)
    inout wire D7,      // Pin 26
    inout wire D6,      // Pin 27
    inout wire D5,      // Pin 28
    inout wire D4,      // Pin 29
    inout wire D3,      // Pin 30
    inout wire D2,      // Pin 31
    inout wire D1,      // Pin 32
    inout wire D0,      // Pin 33

    // Row Address (RA4-RA0)
    output wire RA4,    // Pin 34
    output wire RA3,    // Pin 35
    output wire RA2,    // Pin 36
    output wire RA1,    // Pin 37
    output wire RA0,    // Pin 38

    output wire HS,     // Pin 39: Horizontal Sync
    output wire VS      // Pin 40: Vertical Sync
);

    // ========================================================================
    // Internal Signals
    // ========================================================================
    
    // Internal Data Bus Handling
    wire [7:0] data_bus_in;
    reg  [7:0] data_bus_out;
    wire       drive_data_bus;

    assign data_bus_in = {D7, D6, D5, D4, D3, D2, D1, D0};
    assign drive_data_bus = (!CS_n && E && RW);

    assign D7 = (drive_data_bus) ? data_bus_out[7] : 1'bz;
    assign D6 = (drive_data_bus) ? data_bus_out[6] : 1'bz;
    assign D5 = (drive_data_bus) ? data_bus_out[5] : 1'bz;
    assign D4 = (drive_data_bus) ? data_bus_out[4] : 1'bz;
    assign D3 = (drive_data_bus) ? data_bus_out[3] : 1'bz;
    assign D2 = (drive_data_bus) ? data_bus_out[2] : 1'bz;
    assign D1 = (drive_data_bus) ? data_bus_out[1] : 1'bz;
    assign D0 = (drive_data_bus) ? data_bus_out[0] : 1'bz;

    // Internal Memory Address Register
    reg [13:0] ma_internal; // Current Beam Address (vma)
    
    // Explicit Assignments for MA Pins
    assign MA0  = ma_internal[0];
    assign MA1  = ma_internal[1];
    assign MA2  = ma_internal[2];
    assign MA3  = ma_internal[3];
    assign MA4  = ma_internal[4];
    assign MA5  = ma_internal[5];
    assign MA6  = ma_internal[6];
    assign MA7  = ma_internal[7];
    assign MA8  = ma_internal[8];
    assign MA9  = ma_internal[9];
    assign MA10 = ma_internal[10];
    assign MA11 = ma_internal[11];
    assign MA12 = ma_internal[12];
    assign MA13 = ma_internal[13];

    // Internal Row Address Register
    reg [4:0] ra_internal;
    
    assign RA0 = ra_internal[0];
    assign RA1 = ra_internal[1];
    assign RA2 = ra_internal[2];
    assign RA3 = ra_internal[3];
    assign RA4 = ra_internal[4];

    // Output Registers for Control Signals
    reg hs_reg, vs_reg;
    assign HS = hs_reg;
    assign VS = vs_reg;

    // ========================================================================
    // Internal Registers & Counters
    // ========================================================================
    
    reg [4:0]  addr_ptr; // Address Pointer
    
    // CRTC Registers
    reg [7:0] r0_h_total;
    reg [7:0] r1_h_displayed;
    reg [7:0] r2_h_sync_pos;
    reg [3:0] r3_sync_width;
    reg [6:0] r4_v_total;
    reg [4:0] r5_v_total_adj;
    reg [6:0] r6_v_displayed;
    reg [6:0] r7_v_sync_pos;
    reg [1:0] r8_interlace;
    reg [4:0] r9_max_scan_line;
    reg [6:0] r10_cursor_start;
    reg [4:0] r11_cursor_end;
    reg [5:0] r12_start_addr_h;
    reg [7:0] r13_start_addr_l;
    reg [5:0] r14_cursor_h;
    reg [7:0] r15_cursor_l;
    reg [5:0] r16_light_pen_h;
    reg [7:0] r17_light_pen_l;

    // Timing Counters
    reg [7:0] h_count;
    reg [4:0] sl_count;
    reg [6:0] v_count;
    reg [4:0] vt_adj_count;
    reg [3:0] hs_width_count;
    reg [4:0] vs_width_count;
    reg       v_adjust_active;
    
    // "vma_t" register: Holds the start address for the next row
    reg [13:0] ma_row_start; 

    // Helpers
    wire h_end = (h_count == r0_h_total);
    wire sl_end = (sl_count == r9_max_scan_line);
    wire v_end = (v_count == r4_v_total);
    
    // Helper to determine if we are in the visible screen area
    wire visible_area;
    assign visible_area = !v_adjust_active && (h_count < r1_h_displayed) && (v_count < r6_v_displayed);


    // ========================================================================
    // INITIALIZATION TASK
    // ========================================================================
    // Defines the standard CGA register values for various modes.
    // Called by both initial block and Reset logic.
    task load_default_registers;
    begin
        r12_start_addr_h <= 0;
        r13_start_addr_l <= 0;
        r14_cursor_h     <= 0;
        r15_cursor_l     <= 0;

        case (STARTUP_MODE)
            0: begin // Mode 0: 80x25 Text (Standard)
                r0_h_total       <= 113; // 0x71
                r1_h_displayed   <= 80;  // 0x50
                r2_h_sync_pos    <= 90;  // 0x5A
                r3_sync_width    <= 10;  // 0x0A
                r4_v_total       <= 31;  // 0x1F
                r5_v_total_adj   <= 6;   // 0x06
                r6_v_displayed   <= 25;  // 0x19
                r7_v_sync_pos    <= 28;  // 0x1C
                r8_interlace     <= 2;   // Non-Interlace
                r9_max_scan_line <= 7;   // 8 scanlines per char
                r10_cursor_start <= 6;   
                r11_cursor_end   <= 7;
            end
            
            1: begin // Mode 1: 40x25 Text
                r0_h_total       <= 56;  // 0x38
                r1_h_displayed   <= 40;  // 0x28
                r2_h_sync_pos    <= 45;  // 0x2D
                r3_sync_width    <= 10;  // 0x0A
                r4_v_total       <= 31;  // 0x1F
                r5_v_total_adj   <= 6;   // 0x06
                r6_v_displayed   <= 25;  // 0x19
                r7_v_sync_pos    <= 28;  // 0x1C
                r8_interlace     <= 2;   
                r9_max_scan_line <= 7;   
                r10_cursor_start <= 6;   
                r11_cursor_end   <= 7;
            end

            2: begin // Mode 2: 640x200 Graphics (Black & White)
                // Note: Graphics modes usually define 100 "rows" of characters
                // where each character is 2 scanlines high (R9=1).
                r0_h_total       <= 113; 
                r1_h_displayed   <= 80;  
                r2_h_sync_pos    <= 90;  
                r3_sync_width    <= 10;  
                r4_v_total       <= 127; // 0x7F
                r5_v_total_adj   <= 6;   
                r6_v_displayed   <= 100; // 0x64
                r7_v_sync_pos    <= 112; // 0x70
                r8_interlace     <= 2;   
                r9_max_scan_line <= 1;   // 2 scanlines per "char"
                r10_cursor_start <= 32;  // Cursor usually off
                r11_cursor_end   <= 0;
            end

            3: begin // Mode 3: 320x200 Graphics (Color/BW)
                r0_h_total       <= 56;  // 0x38
                r1_h_displayed   <= 40;  // 0x28
                r2_h_sync_pos    <= 45;  // 0x2D
                r3_sync_width    <= 10;  // 0x0A
                r4_v_total       <= 127; // 0x7F
                r5_v_total_adj   <= 6;   
                r6_v_displayed   <= 100; // 0x64
                r7_v_sync_pos    <= 112; // 0x70
                r8_interlace     <= 2;   
                r9_max_scan_line <= 1;   
                r10_cursor_start <= 32;  
                r11_cursor_end   <= 0;
            end
            
            4: begin // Mode 4: 640x200 ANSI from Hell
                r0_h_total       <= 113; 
                r1_h_displayed   <= 80; 
                r2_h_sync_pos    <= 90; 
                r3_sync_width    <= 15; 
                r4_v_total       <= 127;
                r5_v_total_adj   <= 6;  
                r6_v_displayed   <= 100;
                r7_v_sync_pos    <= 112;
                r8_interlace     <= 2;   
                r9_max_scan_line <= 1;   
                r10_cursor_start <= 32;  
                r11_cursor_end   <= 0;
            end
            
            default: begin // Default to 80x25
                r0_h_total       <= 113; 
                r1_h_displayed   <= 80;  
                r2_h_sync_pos    <= 90;  
                r3_sync_width    <= 10;  
                r4_v_total       <= 31;  
                r5_v_total_adj   <= 6;   
                r6_v_displayed   <= 25;  
                r7_v_sync_pos    <= 28;  
                r8_interlace     <= 2;   
                r9_max_scan_line <= 7;   
                r10_cursor_start <= 6;   
                r11_cursor_end   <= 7;
            end
        endcase
    end
    endtask

    // ========================================================================
    // SIMULATION INITIALIZATION (Time = 0)
    // ========================================================================
    initial begin
        addr_ptr = 0;
        h_count = 0;
        sl_count = 0;
        v_count = 0;
        ma_internal = 0;
        ma_row_start = 0;
        vt_adj_count = 0;
        v_adjust_active = 0;
        
        load_default_registers();
    end

    // ========================================================================
    // CPU Interface Logic (Asynchronous to Video Clock)
    // ========================================================================
    
    // Write Logic
    always @(negedge E or negedge RESET_n) begin
        if (!RESET_n) begin
            addr_ptr <= 0;
            load_default_registers();
        end else if (!CS_n && !RW) begin
            if (RS == 0) begin
                addr_ptr <= data_bus_in[4:0];
            end else begin
                case (addr_ptr)
                    5'h00: r0_h_total       <= data_bus_in;
                    5'h01: r1_h_displayed   <= data_bus_in;
                    5'h02: r2_h_sync_pos    <= data_bus_in;
                    5'h03: r3_sync_width    <= data_bus_in[3:0];
                    5'h04: r4_v_total       <= data_bus_in[6:0];
                    5'h05: r5_v_total_adj   <= data_bus_in[4:0];
                    5'h06: r6_v_displayed   <= data_bus_in[6:0];
                    5'h07: r7_v_sync_pos    <= data_bus_in[6:0];
                    5'h08: r8_interlace     <= data_bus_in[1:0];
                    5'h09: r9_max_scan_line <= data_bus_in[4:0];
                    5'h0A: r10_cursor_start <= data_bus_in[6:0];
                    5'h0B: r11_cursor_end   <= data_bus_in[4:0];
                    5'h0C: r12_start_addr_h <= data_bus_in[5:0];
                    5'h0D: r13_start_addr_l <= data_bus_in;
                    5'h0E: r14_cursor_h     <= data_bus_in[5:0];
                    5'h0F: r15_cursor_l     <= data_bus_in;
                    default: ;
                endcase
            end
        end
    end

    // Read Logic
    always @(*) begin
        data_bus_out = 8'b0;
        if (!CS_n && RW && RS) begin
            case (addr_ptr)
                5'h0C: data_bus_out = {2'b0, r12_start_addr_h};
                5'h0D: data_bus_out = r13_start_addr_l;
                5'h0E: data_bus_out = {2'b0, r14_cursor_h};
                5'h0F: data_bus_out = r15_cursor_l;
                5'h10: data_bus_out = {2'b0, r16_light_pen_h};
                5'h11: data_bus_out = r17_light_pen_l;
                default: data_bus_out = 8'b0;
            endcase
        end
    end

    // ========================================================================
    // Video Timing Logic (Synchronous to CLK)
    // ========================================================================
    
    always @(posedge CLK or negedge RESET_n) begin
        if (!RESET_n) begin
            h_count <= 0;
            sl_count <= 0;
            v_count <= 0;
            vt_adj_count <= 0;
            v_adjust_active <= 0;
            
            hs_reg <= 0;
            vs_reg <= 0;
            hs_width_count <= 0;
            vs_width_count <= 0;
            
            ma_internal <= 0;
            ra_internal <= 0;
            ma_row_start <= 0;
        end else begin
            
            // ----------------------------------------------------------------
            // Horizontal Logic
            // ----------------------------------------------------------------
            if (h_end) 
                h_count <= 0;
            else 
                h_count <= h_count + 1;

            // Horizontal Sync
            if (h_count == r2_h_sync_pos) begin
                hs_reg <= 1;
                hs_width_count <= 0;
            end else if (hs_reg) begin
                if (hs_width_count == r3_sync_width[3:0] - 1)
                    hs_reg <= 0;
                else
                    hs_width_count <= hs_width_count + 1;
            end

            // ----------------------------------------------------------------
            // Memory Address (vma/vma') Logic
            // ----------------------------------------------------------------
            ma_internal <= ma_internal + 1;

            if (h_count == r1_h_displayed) begin
                if (sl_count == r9_max_scan_line) begin
                     ma_row_start <= ma_internal; 
                end
            end

            if (h_end) begin
                ma_internal <= ma_row_start;
            end


            // ----------------------------------------------------------------
            // Vertical Logic (Counters updated at H_End)
            // ----------------------------------------------------------------
            if (h_end) begin
                
                // Vertical Adjust Logic
                if (v_adjust_active) begin
                    if (vt_adj_count == r5_v_total_adj) begin
                        // --- End of Frame (Adjust Complete) ---
                        v_adjust_active <= 0;
                        vt_adj_count <= 0;
                        v_count <= 0;
                        sl_count <= 0;
                        
                        // Reset Addresses
                        ma_row_start <= {r12_start_addr_h, r13_start_addr_l};
                        ma_internal  <= {r12_start_addr_h, r13_start_addr_l};
                    end else begin
                        vt_adj_count <= vt_adj_count + 1;
                        sl_count <= 0;
                    end
                end else begin 
                    // Normal Vertical Counting
                    if (sl_end) begin
                        sl_count <= 0;
                        if (v_end) begin
                            // Reached Vertical Total. Check for Adjust.
                            if (r5_v_total_adj != 0) begin
                                v_adjust_active <= 1;
                                vt_adj_count <= 1; 
                            end else begin
                                // --- End of Frame (No Adjust) ---
                                v_count <= 0;
                                ma_row_start <= {r12_start_addr_h, r13_start_addr_l};
                                ma_internal  <= {r12_start_addr_h, r13_start_addr_l};
                            end
                        end else begin
                            v_count <= v_count + 1;
                        end
                    end else begin
                        sl_count <= sl_count + 1;
                    end
                end
            end

            // Vertical Sync
            if (v_count == r7_v_sync_pos && sl_count == 0 && h_end) begin
                vs_reg <= 1;
                vs_width_count <= 0;
            end else if (vs_reg && h_end) begin
                if (vs_width_count == 15)
                    vs_reg <= 0;
                else
                    vs_width_count <= vs_width_count + 1;
            end

            // Row Address Output
            ra_internal <= sl_count;
        end
    end

    // ========================================================================
    // Output Logic
    // ========================================================================

    assign DE = visible_area;

    // Cursor Blink Field Counter
    reg [4:0] field_count;
    always @(posedge vs_reg) begin
        field_count <= field_count + 1;
    end
    
    wire cursor_blink_en;
    assign cursor_blink_en = (r10_cursor_start[6:5] == 2'b00) ? 1'b1 : 
                             (r10_cursor_start[6:5] == 2'b01) ? 1'b0 : 
                             (r10_cursor_start[6:5] == 2'b10) ? field_count[3] : 
                             field_count[4];

    wire cursor_addr_match = (ma_internal == {r14_cursor_h, r15_cursor_l});
    wire cursor_line_match = (sl_count >= r10_cursor_start[4:0]) && (sl_count <= r11_cursor_end);

    assign CURSOR = (DE && cursor_addr_match && cursor_line_match && cursor_blink_en) ? 1'b1 : 1'b0;

    // ========================================================================
    // Light Pen Latch
    // ========================================================================
    
    reg lpstb_prev;
    always @(posedge CLK) begin
        lpstb_prev <= LPSTB;
        if (LPSTB && !lpstb_prev) begin
            r16_light_pen_h <= ma_internal[13:8];
            r17_light_pen_l <= ma_internal[7:0];
        end
    end

endmodule