// Inverse Park transform: rotating d/q voltages -> stationary alpha/beta.
//   v_alpha = vd*cos - vq*sin
//   v_beta  = vd*sin + vq*cos
`default_nettype none

module inv_park (
    input  wire signed [15:0] vd,
    input  wire signed [15:0] vq,
    input  wire signed [15:0] sin_t,
    input  wire signed [15:0] cos_t,
    output wire signed [15:0] v_alpha,
    output wire signed [15:0] v_beta
);

  wire signed [32:0] a_full = vd * cos_t - vq * sin_t;
  wire signed [32:0] b_full = vd * sin_t + vq * cos_t;
  wire signed [32:0] a_shf  = a_full >>> 15;
  wire signed [32:0] b_shf  = b_full >>> 15;

  assign v_alpha = (a_shf > 33'sd32767)  ? 16'sd32767  :
                   (a_shf < -33'sd32768) ? -16'sd32768 : a_shf[15:0];
  assign v_beta  = (b_shf > 33'sd32767)  ? 16'sd32767  :
                   (b_shf < -33'sd32768) ? -16'sd32768 : b_shf[15:0];

endmodule

`default_nettype wire
