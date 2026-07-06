// PI controller with anti-windup by integrator clamping.
//   error          : Q1.15 (saturated difference)
//   KP, KI         : Q4.12 parameters
//   accumulator    : Q8.24, clamped to +/-ACC_LIM
//   output         : Q1.15, saturated
//
// The integrator uses the current error (i_next) both for the output and the
// stored state, mirrored exactly by the golden model's PI.step().
// State updates only on the `en` strobe (multi-rate: current loop every N
// cycles, speed loop every 20*N).
`default_nettype none

module pi_ctrl #(
    parameter signed [15:0] KP      = 16'sd4096,       // 1.0 in Q4.12
    parameter signed [15:0] KI      = 16'sd205,        // 0.05 in Q4.12
    parameter signed [31:0] ACC_LIM = 32'sd16777216    // 1.0 in Q8.24
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               en,
    input  wire signed [15:0] ref_i,
    input  wire signed [15:0] fb,
    output wire signed [15:0] u
);

  reg signed [31:0] acc;

  wire signed [16:0] ediff = ref_i - fb;
  wire signed [15:0] e = (ediff > 17'sd32767)  ? 16'sd32767  :
                         (ediff < -17'sd32768) ? -16'sd32768 : ediff[15:0];

  wire signed [31:0] istep  = (KI * e) >>> 3;              // Q5.27 -> Q8.24
  wire signed [32:0] isum   = acc + istep;
  wire signed [32:0] lim    = {ACC_LIM[31], ACC_LIM};
  wire signed [31:0] i_next = (isum > lim)  ? ACC_LIM :
                              (isum < -lim) ? -ACC_LIM :
                              isum[31:0];
  wire signed [31:0] pterm  = (KP * e) >>> 3;              // Q8.24
  wire signed [32:0] usum   = pterm + i_next;
  wire signed [32:0] ushf   = usum >>> 9;                  // Q8.24 -> Q1.15

  assign u = (ushf > 33'sd32767)  ? 16'sd32767  :
             (ushf < -33'sd32768) ? -16'sd32768 : ushf[15:0];

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n)  acc <= 32'sd0;
    else if (en) acc <= i_next;
  end

endmodule

`default_nettype wire
