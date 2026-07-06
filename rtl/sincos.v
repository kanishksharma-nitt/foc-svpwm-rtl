// Shared sine/cosine lookup: 257-entry quarter-wave LUT in Q1.15, indexed by
// the top 10 bits of the 16-bit angle (0..65535 -> 0..2*pi). Entry 256 holds
// sin(pi/2) saturated to +32767. cos(theta) = sin(theta + 90 degrees).
//
// Chosen over CORDIC: at 10-bit phase resolution the worst-case LUT error
// (~0.15%) is far below the current-sensor LSB, and the lookup is a single
// combinational read instead of 12+ iterative cycles.
`default_nettype none

module sincos #(
    parameter LUT_FILE = "../rtl/sin_lut.mem"
) (
    input  wire [15:0]        theta,
    output wire signed [15:0] sin_o,
    output wire signed [15:0] cos_o
);

  reg [15:0] lut [0:256];
  initial $readmemh(LUT_FILE, lut);

  function signed [15:0] lookup(input [15:0] th);
    reg [9:0] ph;
    reg [8:0] idx;
    reg [15:0] val;
    begin
      ph  = th[15:6];
      idx = {1'b0, ph[7:0]};
      if (ph[8]) idx = 9'd256 - idx;    // mirror in quadrants 1 and 3
      val = lut[idx];
      lookup = ph[9] ? -$signed(val) : $signed(val);
    end
  endfunction

  assign sin_o = lookup(theta);
  assign cos_o = lookup(theta + 16'd16384);

endmodule

`default_nettype wire
