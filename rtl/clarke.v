// Clarke transform (two measured line currents, ia + ib + ic = 0):
//   i_alpha = ia
//   i_beta  = (ia + 2*ib) / sqrt(3)
// Q1.15 in/out; 1/sqrt(3) = 18919/32768.
`default_nettype none

module clarke (
    input  wire signed [15:0] ia,
    input  wire signed [15:0] ib,
    output wire signed [15:0] i_alpha,
    output wire signed [15:0] i_beta
);

  wire signed [17:0] sum3 = {{2{ia[15]}}, ia} + {ib[15], ib, 1'b0};
  wire signed [35:0] prod = sum3 * 18'sd18919;
  wire signed [35:0] shf  = prod >>> 15;

  assign i_alpha = ia;
  assign i_beta  = (shf > 36'sd32767)  ? 16'sd32767  :
                   (shf < -36'sd32768) ? -16'sd32768 : shf[15:0];

endmodule

`default_nettype wire
