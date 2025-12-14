// To use in digita:
// label: cga_to_rgb
// inputs: r_in, g_in, b_in, i_in
// outputs: r_out:2, g_out:2, b_out:2

module cga_to_rgb (
    input  wire r_in,
    input  wire g_in,
    input  wire b_in,
    input  wire i_in,
    output wire [1:0] r_out,
    output wire [1:0] g_out,
    output wire [1:0] b_out
);

    // 1. Assign 'is_brown' wire to the "Dark Yellow" (Color 6) input state
    wire is_brown;
    assign is_brown = (r_in && g_in && !b_in && !i_in);

    // 2. Red and Blue are standard (MSB=Color, LSB=Intensity)
    assign r_out = {r_in, i_in};
    assign b_out = {b_in, i_in};

    // 3. Green requires the 5153 brown tweak.
    // If is_brown: MSB becomes 0, LSB becomes 1.
    // If not is_brown: MSB is g_in, LSB is i_in.
    assign g_out[1] = g_in & ~is_brown; // Force MSB low
    assign g_out[0] = i_in |  is_brown; // Force LSB high

endmodule