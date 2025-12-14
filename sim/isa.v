module isa_8bit_slot (
    // --------------------------------------------------------
    // Power Rails (Pass-through)
    // --------------------------------------------------------
    input  wire        bus_vcc,
    output wire        card_vcc,
    input  wire        bus_gnd,
    output wire        card_gnd,

    // --------------------------------------------------------
    // Motherboard -> Card (Forwarding Inputs to Outputs)
    // --------------------------------------------------------
    input  wire        bus_clk,
    output wire        card_clk,
    input  wire        bus_osc,
    output wire        card_osc,
    input  wire        bus_reset_drv,
    output wire        card_reset_drv,
    
    input  wire [19:0] bus_sa,       // Address
    output wire [19:0] card_sa,
    
    input  wire        bus_ale,
    output wire        card_ale,
    input  wire        bus_aen,
    output wire        card_aen,
    input  wire        bus_ior_n,
    output wire        card_ior_n,
    input  wire        bus_iow_n,
    output wire        card_iow_n,
    input  wire        bus_memr_n,
    output wire        card_memr_n,
    input  wire        bus_memw_n,
    output wire        card_memw_n,
    input  wire [3:1]  bus_dack_n,
    output wire [3:1]  card_dack_n,
    input  wire        bus_tc,
    output wire        card_tc,

    // --------------------------------------------------------
    // Card -> Motherboard (Forwarding Inputs to Outputs)
    // --------------------------------------------------------
    input  wire        card_io_ch_ck_n,
    output wire        bus_io_ch_ck_n,
    input  wire        card_io_ch_rdy,
    output wire        bus_io_ch_rdy,
    input  wire [7:2]  card_irq,     // Interrupts
    output wire [7:2]  bus_irq,
    input  wire [3:1]  card_drq,     // DMA Requests
    output wire [3:1]  bus_drq,

    // --------------------------------------------------------
    // Bidirectional Data (Passive Link)
    // --------------------------------------------------------
    inout  wire [7:0]  bus_sd,
    inout  wire [7:0]  card_sd
);

    // Power pass-through
    assign card_vcc       = bus_vcc;
    assign card_gnd       = bus_gnd;

    // Forwarding logic (Mobo -> Card)
    assign card_clk       = bus_clk;
    assign card_osc       = bus_osc;
    assign card_reset_drv = bus_reset_drv;
    assign card_sa        = bus_sa;
    assign card_ale       = bus_ale;
    assign card_aen       = bus_aen;
    assign card_ior_n     = bus_ior_n;
    assign card_iow_n     = bus_iow_n;
    assign card_memr_n    = bus_memr_n;
    assign card_memw_n    = bus_memw_n;
    assign card_dack_n    = bus_dack_n;
    assign card_tc        = bus_tc;

    // Forwarding logic (Card -> Mobo)
    assign bus_io_ch_ck_n = card_io_ch_ck_n;
    assign bus_io_ch_rdy  = card_io_ch_rdy;
    assign bus_irq        = card_irq;
    assign bus_drq        = card_drq;

    // Bidirectional Link
    tran t0(bus_sd[0], card_sd[0]);
    tran t1(bus_sd[1], card_sd[1]);
    tran t2(bus_sd[2], card_sd[2]);
    tran t3(bus_sd[3], card_sd[3]);
    tran t4(bus_sd[4], card_sd[4]);
    tran t5(bus_sd[5], card_sd[5]);
    tran t6(bus_sd[6], card_sd[6]);
    tran t7(bus_sd[7], card_sd[7]);

endmodule