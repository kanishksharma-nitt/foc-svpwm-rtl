// Park transform: stationary alpha/beta -> rotating d/q frame.
//   id =  i_alpha*cos + i_beta*sin
//   iq = -i_alpha*sin + i_beta*cos
// Q1.15 in/out, Q2.30 products, arithmetic shift back, saturate.
`default_nettype none

module park (
    input  wire signed [15:0] i_alpha,
    input  wire signed [15:0] i_beta,
    input  wire signed [15:0] sin_t,
    input  wire signed [15:0] cos_t,
    output wire signed [15:0] id,
    output wire signed [15:0] iq
);

  wire signed [32:0] d_full = i_alpha * cos_t + i_beta * sin_t;
  wire signed [32:0] q_full = i_beta * cos_t - i_alpha * sin_t;
  wire signed [32:0] d_shf  = d_full >>> 15;
  wire signed [32:0] q_shf  = q_full >>> 15;

  assign id = (d_shf > 33'sd32767)  ? 16'sd32767  :
              (d_shf < -33'sd32768) ? -16'sd32768 : d_shf[15:0];
  assign iq = (q_shf > 33'sd32767)  ? 16'sd32767  :
              (q_shf < -33'sd32768) ? -16'sd32768 : q_shf[15:0];

endmodule

`default_nettype wire
